#include "geo/maplibrary.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dirent.h>
#endif

namespace georay {
namespace {

// Lists *.map in a directory. Deliberately hand-rolled rather than
// std::filesystem: that needs -lstdc++fs on the GCC this builds with, and a
// link flag is a worse trade than twenty lines.
std::vector<std::string> listMapFiles(const std::string& directory) {
    std::vector<std::string> names;
#if defined(_WIN32)
    WIN32_FIND_DATAA entry;
    HANDLE handle = FindFirstFileA((directory + "\\*.map").c_str(), &entry);
    if (handle == INVALID_HANDLE_VALUE) return names;
    do {
        names.push_back(entry.cFileName);
    } while (FindNextFileA(handle, &entry));
    FindClose(handle);
#else
    DIR* dir = opendir(directory.c_str());
    if (!dir) return names;
    while (struct dirent* entry = readdir(dir)) {
        const std::string name = entry->d_name;
        if (name.size() > 4 && name.compare(name.size() - 4, 4, ".map") == 0) {
            names.push_back(name);
        }
    }
    closedir(dir);
#endif
    // Sorted so the resident window is deterministic when several overlap
    // equally - a scene that picks a different window each run is a scene
    // whose screenshots cannot be compared.
    std::sort(names.begin(), names.end());
    return names;
}

}  // namespace

bool MapLibrary::open(const std::string& directory, std::string& error) {
    directory_ = directory;
    windows_.clear();
    resident_.reset();
    resident_path_.clear();
    resident_name_.clear();

    const std::vector<std::string> names = listMapFiles(directory);
    if (names.empty()) {
        error = "no .map windows in " + directory;
        directory_.clear();
        return false;
    }

    for (const std::string& name : names) {
        const std::string path = directory + "/" + name;
        // Header only. A region-sized library is dozens of windows of tens
        // of megabytes each; reading them all in full to learn four numbers
        // apiece would be a gigabyte of disk traffic before the first frame.
        MapRasterInfo info;
        std::string load_error;
        if (!MapRaster::readHeader(path, info, load_error)) {
            last_error_ = load_error;
            continue;
        }
        Window window;
        window.path = path;
        window.name = name.substr(0, name.size() - 4);
        window.south = info.southEdge();
        window.north = info.northEdge();
        window.west = info.westEdge();
        window.east = info.eastEdge();
        windows_.push_back(window);
    }

    if (windows_.empty()) {
        error = "no readable .map windows in " + directory +
                (last_error_.empty() ? "" : " (" + last_error_ + ")");
        directory_.clear();
        return false;
    }
    return true;
}

const MapRaster* MapLibrary::windowFor(double lat_deg, double lon_deg) {
    const Window* best = nullptr;
    double best_offset = 0.0;
    for (const Window& window : windows_) {
        if (!window.contains(lat_deg, lon_deg)) continue;
        const double offset = window.centreOffsetSq(lat_deg, lon_deg);
        if (!best || offset < best_offset) {
            best = &window;
            best_offset = offset;
        }
    }
    if (!best) return nullptr;

    // Already resident: nothing to do, which is the common case between
    // re-anchors.
    if (resident_ && resident_path_ == best->path) return resident_.get();

    auto loaded = std::make_unique<MapRaster>();
    std::string error;
    if (!MapRaster::load(best->path, *loaded, error)) {
        last_error_ = error;
        return nullptr;
    }
    resident_ = std::move(loaded);
    resident_path_ = best->path;
    resident_name_ = best->name;
    ++load_count_;
    return resident_.get();
}

}  // namespace georay
