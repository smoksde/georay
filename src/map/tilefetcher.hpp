#pragma once

// Fetches missing map tiles while the simulation runs.
//
// georay was deliberately offline - scripts/fetch_map.py did the
// downloading, the binary only read files - for three properties worth
// keeping even once live fetching is allowed:
//
//   * the fetcher lives in the app layer, never in georay_geo, so --selftest
//     is still a pure numeric suite with nothing to reach out to;
//   * it runs on its own thread and the render loop never waits on it;
//   * it writes into the same cache directory and naming the script uses, so
//     neither re-downloads what the other already has;
//   * it is off unless asked for, rate-limited, and capped per session.
//
// Shells out to curl rather than linking libcurl - keeps the build
// dependency-free and gets TLS/redirects/retries/parallelism for free.
// libcurl is the drop-in upgrade if the process boundary ever becomes the
// limiting factor.

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "map/tilesource.hpp"

namespace georay {

class TileFetcher {
public:
    TileFetcher(std::string directory, int session_limit, double delay_seconds);
    ~TileFetcher();

    TileFetcher(const TileFetcher&) = delete;
    TileFetcher& operator=(const TileFetcher&) = delete;

    void start();
    void stop();
    bool running() const { return running_.load(); }

    // Queue a tile. Silently ignored when it is already on disk, already
    // queued, already tried, or the session cap has been reached.
    void request(const TileSource& source, int zoom, int x, int y);

    // Counters for the HUD and for deciding when to re-stitch.
    int fetched() const { return fetched_.load(); }
    int failed() const { return failed_.load(); }
    int pending() const;
    bool limitReached() const { return fetched_.load() + failed_.load() >= session_limit_; }
    std::string lastError() const;

    // Checked once, so a machine without curl says so instead of failing
    // silently per batch.
    static bool curlAvailable();

private:
    struct Request {
        std::string source;
        std::string url;
        int zoom = 0, x = 0, y = 0;
    };

    void worker();
    // Runs one curl invocation over a batch. Returns how many landed.
    int fetchBatch(const std::vector<Request>& batch);

    std::string directory_;
    int session_limit_;
    double delay_seconds_;

    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<Request> queue_;
    std::set<std::string> seen_;  // queued or already attempted, by cache key
    std::string last_error_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopping_{false};
    std::atomic<int> fetched_{0};
    std::atomic<int> failed_{0};
};

}  // namespace georay
