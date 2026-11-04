#pragma once

// The ground, as a grid of chunks loaded around wherever the camera is
// looking - the same idea as a voxel game streaming terrain in, and for the
// same reason: you can see a few kilometres, the world is a few hundred, and
// only the difference needs to exist at any moment.
//
// Chunks are flat. That is what makes this tractable at all: a chunk is two
// triangles and one texture rather than a patch of heightfield with its own
// seams, normals and level of detail. Relief has not been thrown away - the
// Terrain is still what rays are intersected against, and the height-map mode
// shows it as colour and contour lines - it has only stopped being geometry.
// Draping a map over relief distorts it exactly where it is most worth
// reading, and a map you cannot read is not worth the parallax.
//
// A chunk's identity is a block of slippy tiles at a zoom, so it lines up
// with the tile grid it is filled from, and changing zoom simply changes
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
    // Tiles per chunk edge. Four gives a 1024-pixel texture at native tile
    // resolution: big enough that the per-chunk overhead is irrelevant, small
    // enough that one arriving does not stall a frame.
    static constexpr int kTilesPerChunk = 4;

    GroundChunks();
    ~GroundChunks();

    GroundChunks(const GroundChunks&) = delete;
    GroundChunks& operator=(const GroundChunks&) = delete;

    void setResidentLimit(size_t chunks) { resident_limit_ = chunks > 1 ? chunks : 1; }
    void setMode(GroundMode mode);
    GroundMode mode() const { return mode_; }

    // Which sources skin the two tile-backed modes.
    void setSources(const TileSource* map_source, const TileSource* satellite_source);
    const TileSource* activeSource() const;

    // Decides what should be resident from where the camera is looking,
    // builds what is missing, and drops what is no longer near. Call once a
    // frame; it does a bounded amount of work per call so a big view change
    // costs several frames rather than one long stall.
    // `dem` may be null, in which case the height-map mode falls back to the
    // resampled terrain window - which is bounded, so the ground goes flat
    // outside it. With a DEM it streams its own tiles and the height map
    // reaches as far as the other two modes.
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
        // The fetcher's delivered count when this chunk was last skinned.
        // An incomplete chunk is only worth rebuilding once more tiles have
        // actually arrived; retrying every frame starves the creation of new
        // chunks, which is how a view ends up with a patch of ground in the
        // middle and nothing around it.
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
