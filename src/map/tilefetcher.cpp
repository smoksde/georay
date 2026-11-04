#include "map/tilefetcher.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include "map/tilecache.hpp"

namespace georay {
namespace {

// Identifies georay to whoever is serving the tiles. A server cannot tell a
// simulation from a scraper unless the client says which it is, and several
// of them block unidentified callers outright.
constexpr const char* kUserAgent =
    "georay/1.0 (UAV geolocation simulation; live basemap for a local scene)";

// One curl invocation per batch. Large enough that process start-up is
// irrelevant, small enough to keep the argument list and the burst of
// connections modest.
constexpr size_t kBatchSize = 16;
// How many of a batch curl may have in flight at once.
constexpr const char* kParallelMax = "4";

bool fileExists(const std::string& path) {
    struct stat info;
    return ::stat(path.c_str(), &info) == 0 && info.st_size > 0;
}

}  // namespace

TileFetcher::TileFetcher(std::string directory, int session_limit, double delay_seconds)
    : directory_(std::move(directory)),
      session_limit_(session_limit > 0 ? session_limit : 1),
      delay_seconds_(delay_seconds >= 0.0 ? delay_seconds : 0.0) {}

TileFetcher::~TileFetcher() { stop(); }

bool TileFetcher::curlAvailable() {
    // -V rather than --version so the check costs nothing; the exit status is
    // all that matters.
    return std::system("curl -V >/dev/null 2>&1") == 0;
}

void TileFetcher::start() {
    if (running_.load()) return;
    stopping_.store(false);
    running_.store(true);
    thread_ = std::thread(&TileFetcher::worker, this);
}

void TileFetcher::stop() {
    if (!running_.load()) return;
    stopping_.store(true);
    wake_.notify_all();
    if (thread_.joinable()) thread_.join();
    running_.store(false);
}

int TileFetcher::pending() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return int(queue_.size());
}

std::string TileFetcher::lastError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
}

void TileFetcher::request(const TileSource& source, int zoom, int x, int y) {
    if (limitReached()) return;
    if (zoom < source.min_zoom || zoom > source.max_zoom) return;

    const std::string path = TileCache::pathFor(directory_, source.name, zoom, x, y);
    if (fileExists(path)) return;

    const std::string id = source.name + "/" + std::to_string(zoom) + "/" +
                           std::to_string(x) + "/" + std::to_string(y);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // seen_ covers queued and already-attempted alike: a tile the server
        // does not have must not be asked for again every frame.
        if (!seen_.insert(id).second) return;
        queue_.push_back({source.name, source.urlFor(zoom, x, y), zoom, x, y});
    }
    wake_.notify_one();
}

int TileFetcher::fetchBatch(const std::vector<Request>& batch) {
    if (batch.empty()) return 0;

    // Built as an argv array and handed to execvp - no shell anywhere, so a
    // URL can never be interpreted as a command however it is spelled.
    std::vector<std::string> storage;
    storage.reserve(batch.size() * 2 + 10);
    storage.emplace_back("curl");
    storage.emplace_back("-s");
    // --fail so an HTTP error writes no body. Nothing version-dependent
    // beyond it: --remove-on-error would be tidier but only exists from curl
    // 7.83, and this has to work with whatever curl the machine has. An
    // empty file left behind by a failed request is removed below anyway.
    storage.emplace_back("--fail");
    storage.emplace_back("--max-time");
    storage.emplace_back("30");
    storage.emplace_back("--parallel");
    storage.emplace_back("--parallel-max");
    storage.emplace_back(kParallelMax);
    storage.emplace_back("-A");
    storage.emplace_back(kUserAgent);

    std::vector<std::string> partials;
    partials.reserve(batch.size());
    for (const Request& request : batch) {
        // Downloaded to a temporary name and renamed once curl exits, so the
        // decoder in the render thread can never pick up a half-written tile.
        const std::string path =
            TileCache::pathFor(directory_, request.source, request.zoom, request.x, request.y);
        partials.push_back(path + ".part");
        storage.emplace_back("-o");
        storage.emplace_back(partials.back());
        storage.emplace_back(request.url);
    }

    std::vector<char*> argv;
    argv.reserve(storage.size() + 1);
    for (std::string& item : storage) argv.push_back(&item[0]);
    argv.push_back(nullptr);

    const pid_t pid = ::fork();
    if (pid < 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = "cannot fork to run curl";
        return 0;
    }
    if (pid == 0) {
        // curl's own diagnostics would interleave with georay's output, and
        // this code judges every request by whether its file landed, so they
        // carry no information it acts on.
        if (FILE* null = std::fopen("/dev/null", "w")) {
            ::dup2(::fileno(null), STDERR_FILENO);
        }
        ::execvp("curl", argv.data());
        ::_exit(127);  // only reached if curl is not on the path
    }
    int status = 0;
    ::waitpid(pid, &status, 0);

    // curl's exit status covers the whole batch, so it is not a per-tile
    // answer; what landed on disk is. A partial batch is normal - some tiles
    // simply do not exist - so each is judged by its own file.
    int landed = 0;
    for (size_t i = 0; i < batch.size(); ++i) {
        const std::string path = TileCache::pathFor(directory_, batch[i].source,
                                                    batch[i].zoom, batch[i].x, batch[i].y);
        if (fileExists(partials[i])) {
            ::rename(partials[i].c_str(), path.c_str());
            ++landed;
        } else {
            ::remove(partials[i].c_str());
            failed_.fetch_add(1);
        }
    }
    fetched_.fetch_add(landed);

    if (landed == 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = WIFEXITED(status) && WEXITSTATUS(status) == 127
                          ? "curl is not on the PATH"
                          : "no tiles returned for the last batch";
    }
    return landed;
}

void TileFetcher::worker() {
    while (!stopping_.load()) {
        std::vector<Request> batch;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            wake_.wait(lock, [&] { return stopping_.load() || !queue_.empty(); });
            if (stopping_.load()) return;
            while (!queue_.empty() && batch.size() < kBatchSize) {
                batch.push_back(queue_.front());
                queue_.pop_front();
            }
        }
        if (limitReached()) {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.clear();
            last_error_ = "session tile limit reached";
            continue;
        }

        fetchBatch(batch);

        // A pause between batches, not between tiles: curl already limits
        // concurrency within one, and this is what keeps a long flight from
        // becoming a sustained hammering of somebody's tile server.
        if (delay_seconds_ > 0.0 && !stopping_.load()) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(int(delay_seconds_ * 1000.0)));
        }
    }
}

}  // namespace georay
