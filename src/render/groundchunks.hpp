#pragma once

// The ground, as a grid of chunks loaded around wherever the camera is
// looking - the same idea as a voxel game streaming terrain in: you can see
// a few km, the world is a few hundred, and only the difference needs to
// exist at any moment.
//
// Chunks are flat: two triangles and one texture, not a heightfield patch
// with its own seams and LOD. Relief is not thrown away - Terrain is still
// what rays are intersected against, and the height-map mode shows it as
// colour and contours - it has only stopped being geometry, since draping a
// map over relief distorts it exactly where it is most worth reading.
//
// A chunk's identity is a block of slippy tiles at a zoom, so it lines up
// with the tile grid it is filled from, and changing zoom just changes
// which chunks exist.

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geo/geodetic.hpp"
#include "geo/mercator.hpp"
#include "geo/dem.hpp"
#include "geo/geoid.hpp"
#include "geo/terrain.hpp"
#include "gl/mesh.hpp"
#include "map/maptexture.hpp"
#include "map/tilecache.hpp"
#include "map/tilefetcher.hpp"
#include "map/tilesource.hpp"
#include "render/viewcamera.hpp"

namespace georay {

// What the ground is skinned with. All three produce one RGB texture per
// chunk, so there is a single path from here on.
enum class GroundMode { Map = 0, Satellite = 1, HeightMap = 2 };
const char* groundModeName(GroundMode mode);

class GroundChunks {
public:
    // Tiles per chunk edge: a 1024-pixel texture at native tile resolution,
    // big enough that per-chunk overhead is irrelevant, small enough that
    // one arriving does not stall a frame.
    static constexpr int kTilesPerChunk = 4;

    GroundChunks();
    ~GroundChunks();

    GroundChunks(const GroundChunks&) = delete;
    GroundChunks& operator=(const GroundChunks&) = delete;

    void setMode(GroundMode mode);
    GroundMode mode() const { return mode_; }

    // Which sources skin the two tile-backed modes.
    void setSources(const TileSource* map_source, const TileSource* satellite_source);
    const TileSource* activeSource() const;

    // Decides what should be resident from where the camera is looking,
    // builds what is missing, drops what is no longer near. Call once a
    // frame; bounded work per call, so a big view change costs several
    // frames rather than one stall. `dem` may be null, in which case the
    // height-map mode falls back to the (bounded) resampled terrain window
    // instead of streaming its own tiles as far as the other two modes do.
    void update(const ViewCamera& camera, const EnuFrame& frame, const Terrain& terrain,
                const DemSource* dem, const GeoidModel& geoid, TileCache& cache,
                TileFetcher* fetcher, double max_ground_radius_m);

    // Everything resident, in one pass. The shader must already be bound.
    void draw(class Shader& shader) const;

    // Thrown away when the frame re-anchors: every chunk's geometry is in
    // ENU, and ENU has just come to mean something else.
    void invalidateGeometry();

    int zoom() const { return zoom_; }
    double metresPerPixel() const { return metres_per_pixel_; }
    size_t residentCount() const { return chunks_.size(); }
    int readyCount() const;
    int pendingCount() const;

private:
    struct Chunk {
        int chunk_x = 0, chunk_y = 0, zoom = 0;
        MercatorBox box;
        GLuint texture = 0;
        SurfaceMesh mesh;
        bool has_texture = false;
        bool complete = false;   // every tile it wanted was present
        double last_used = 0.0;
        // The fetcher's delivered count when this chunk was last skinned. An
        // incomplete chunk is only worth rebuilding once more tiles arrive -
        // retrying every frame starves new-chunk creation, leaving a patch
        // of ground with nothing around it.
        int built_at_delivery = -1;
    };

    std::string key(int chunk_x, int chunk_y, int zoom) const;
    // Builds one chunk's mesh and texture. Returns false when it could not be
    // skinned yet, which for a tile-backed mode means the tiles are missing.
    bool buildChunk(Chunk& chunk, const EnuFrame& frame, const Terrain& terrain,
                    const DemSource* dem, const GeoidModel& geoid, TileCache& cache,
                    TileFetcher* fetcher);
    void buildMesh(Chunk& chunk, const EnuFrame& frame) const;
    bool buildTileTexture(Chunk& chunk, TileCache& cache, TileFetcher* fetcher);
    bool buildHeightTexture(Chunk& chunk, const EnuFrame& frame, const Terrain& terrain,
                            const DemSource* dem, const GeoidModel& geoid);
    void evictFarthest(int chunk_x, int chunk_y);

    GroundMode mode_ = GroundMode::Satellite;
    const TileSource* sources_[2] = {nullptr, nullptr};  // map, satellite

    std::map<std::string, std::unique_ptr<Chunk>> chunks_;
    int zoom_ = 0;
    double metres_per_pixel_ = 0.0;
    double clock_ = 0.0;
    size_t resident_limit_ = 56;
    MapTexture stitcher_;
};

}  // namespace georay
