#include "render/groundchunks.hpp"

#include <algorithm>
#include <cmath>

#include "gl/shader.hpp"

namespace georay {
namespace {

// How finely a chunk's quad is subdivided. A chunk is a rectangle in
// Mercator and a slightly curved patch in the local tangent plane, so two
// triangles would put the map a metre or two out at the corners. Eight by
// eight costs nothing and removes it.
constexpr int kChunkGrid = 8;

// Texture size for the generated height map. Lower than the tile-backed
// modes because it is sampled from a DEM whose posts are 30 m apart - more
// pixels would only interpolate the same data more smoothly.
constexpr int kHeightTexture = 512;

// At most this many chunks built per update. A large view change otherwise
// decodes several hundred tiles in one frame, which stalls visibly; spread
// over a few frames it is not noticeable.
constexpr int kBuildsPerUpdate = 3;

glm::vec3 toFloat(const glm::dvec3& v) { return glm::vec3(float(v.x), float(v.y), float(v.z)); }

// Hypsometric tint: the map-like elevation ramp, not a photographic one. The
// question this tool answers is "which point on the ground is this", and
// distinguishable bands make a height readable straight off the surface.
void elevationColour(double t, unsigned char* rgb) {
    const double stops[5][3] = {
        {0.21, 0.33, 0.20}, {0.45, 0.47, 0.26}, {0.55, 0.42, 0.28},
        {0.72, 0.62, 0.48}, {0.95, 0.95, 0.97},
    };
    const double scaled = glm::clamp(t, 0.0, 1.0) * 4.0;
    const int band = std::min(int(scaled), 3);
    const double f = scaled - double(band);
    for (int i = 0; i < 3; ++i) {
        const double v = stops[band][i] * (1.0 - f) + stops[band + 1][i] * f;
        rgb[i] = (unsigned char)(glm::clamp(v, 0.0, 1.0) * 255.0);
    }
}

}  // namespace

const char* groundModeName(GroundMode mode) {
    switch (mode) {
        case GroundMode::Map: return "MAP";
        case GroundMode::Satellite: return "SATELLITE";
        case GroundMode::HeightMap: return "HEIGHT MAP";
    }
    return "?";
}

GroundChunks::GroundChunks() = default;

GroundChunks::~GroundChunks() {
    for (auto& entry : chunks_) {
        if (entry.second->texture) glDeleteTextures(1, &entry.second->texture);
    }
}

void GroundChunks::setMode(GroundMode mode) {
    if (mode == mode_) return;
    mode_ = mode;
    // Every chunk's skin is now wrong. The geometry would survive, but the
    // textures are the expensive part anyway and keeping half a chunk alive
    // is not worth the bookkeeping.
    for (auto& entry : chunks_) {
        if (entry.second->texture) glDeleteTextures(1, &entry.second->texture);
    }
    chunks_.clear();
}

void GroundChunks::setSources(const TileSource* map_source, const TileSource* satellite_source) {
    sources_[0] = map_source;
    sources_[1] = satellite_source;
}

const TileSource* GroundChunks::activeSource() const {
    if (mode_ == GroundMode::Map) return sources_[0];
    if (mode_ == GroundMode::Satellite) return sources_[1];
    return nullptr;
}

void GroundChunks::invalidateGeometry() {
    for (auto& entry : chunks_) {
        if (entry.second->texture) glDeleteTextures(1, &entry.second->texture);
    }
    chunks_.clear();
}

std::string GroundChunks::key(int chunk_x, int chunk_y, int zoom) const {
    return std::to_string(zoom) + "/" + std::to_string(chunk_x) + "/" + std::to_string(chunk_y);
}

int GroundChunks::readyCount() const {
    int ready = 0;
    for (const auto& entry : chunks_) {
        if (entry.second->has_texture) ++ready;
    }
    return ready;
}

int GroundChunks::pendingCount() const { return int(chunks_.size()) - readyCount(); }

void GroundChunks::buildMesh(Chunk& chunk, const EnuFrame& frame) const {
    std::vector<SurfaceVertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(size_t(kChunkGrid + 1) * (kChunkGrid + 1));

    for (int iy = 0; iy <= kChunkGrid; ++iy) {
        for (int ix = 0; ix <= kChunkGrid; ++ix) {
            const double u = double(ix) / kChunkGrid;
            const double v = double(iy) / kChunkGrid;
            // Interpolate in Mercator, not in latitude: the two differ by
            // more than a metre across a chunk, and the texture is laid out
            // in the former.
            const double lon = chunk.box.west() + (chunk.box.east() - chunk.box.west()) * u;
            const double north_y = mercatorTile(chunk.box.north(), lon, 20).y;
            const double south_y = mercatorTile(chunk.box.south(), lon, 20).y;
            const double lat = mercatorLatitude(north_y + (south_y - north_y) * v, 20);

            // Flat: the ground plane sits at the frame origin's own
            // elevation, so heights reported in the HUD stay relative to
            // something the viewer can see.
            const glm::dvec3 enu = frame.fromGeodetic(Geodetic(lat, lon, 0.0));
            SurfaceVertex vertex;
            vertex.position = glm::vec3(float(enu.x), float(enu.y), 0.0f);
            vertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
            vertex.map_uv = glm::vec2(float(u), float(v));
            vertices.push_back(vertex);
        }
    }
    for (int iy = 0; iy < kChunkGrid; ++iy) {
        for (int ix = 0; ix < kChunkGrid; ++ix) {
            const unsigned int a = unsigned(iy * (kChunkGrid + 1) + ix);
            const unsigned int b = a + 1;
            const unsigned int c = a + unsigned(kChunkGrid + 1);
            const unsigned int d = c + 1;
            // Counter-clockwise seen from above, which needs the opposite
            // order to the obvious one: the texture's v runs south while
            // ENU's y runs north, so a grid wound in texture order comes out
            // facing downwards and is culled away entirely.
            indices.insert(indices.end(), {a, d, b, a, c, d});
        }
    }
    chunk.mesh.upload(vertices, indices);
}

bool GroundChunks::buildTileTexture(Chunk& chunk, TileCache& cache, TileFetcher* fetcher) {
    const TileSource* source = activeSource();
    if (!source) return false;

    const MapTexture::Result result =
        stitcher_.build(cache, *source, chunk.box, chunk.zoom,
                        kTilesPerChunk * DecodedTile::kSize);

    // Ask for whatever was missing, whether or not anything was drawable:
    // that is how a chunk the camera is looking at gets filled in.
    if (fetcher) {
        for (const auto& tile : result.missing) {
            fetcher->request(*source, result.zoom, tile.first, tile.second);
        }
    }
    if (result.tiles_present == 0) return false;

    chunk.complete = result.tiles_present == result.tiles_wanted;
    if (chunk.texture == 0) glGenTextures(1, &chunk.texture);
    glBindTexture(GL_TEXTURE_2D, chunk.texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, stitcher_.width(), stitcher_.height(), 0, GL_RGB,
                 GL_UNSIGNED_BYTE, stitcher_.pixels().data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (GLEW_EXT_texture_filter_anisotropic) {
        GLfloat max_anisotropy = 1.0f;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max_anisotropy);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT,
                        std::min(max_anisotropy, 8.0f));
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

bool GroundChunks::buildHeightTexture(Chunk& chunk, const EnuFrame& frame,
                                      const Terrain& terrain, const DemSource* dem,
                                      const GeoidModel& geoid) {
    std::vector<unsigned char> rgb(size_t(kHeightTexture) * kHeightTexture * 3, 0);

    // The ramp spans the loaded scene's own relief rather than this chunk's,
    // so neighbouring chunks agree about what a colour means. A chunk outside
    // that range clamps, which reads as "off the top of the scale" and is the
    // honest answer.
    const double low = terrain.minHeight();
    const double span = std::max(terrain.maxHeight() - terrain.minHeight(), 10.0);

    // Contours every 10 m up to 200 m of relief, then coarser, so a flat
    // plain and an alpine valley both end up with a readable number of lines
    // rather than one or a thousand.
    const double interval = span < 200.0 ? 10.0 : (span < 1000.0 ? 50.0 : 200.0);

    // Metres of ground per texel, needed to keep a contour about a pixel wide
    // whatever the slope.
    const double chunk_m =
        (chunk.box.north() - chunk.box.south()) * 110574.0;
    const double metres_per_texel = std::max(chunk_m / kHeightTexture, 0.01);

    // Sampling the DEM directly rather than the resampled terrain window:
    // the window is bounded and the camera is not, so going through it would
    // make the height map go flat exactly where the other modes keep going.
    auto heightAt = [&](double lat, double lon) {
        double orthometric = 0.0;
        if (dem && dem->sampleOrthometric(lat, lon, orthometric)) {
            // Ellipsoidal, to match what the terrain window holds and what
            // the HUD reports.
            return geoid.toEllipsoidal(lat, lon, orthometric) - frame.origin().altitude_m;
        }
        const glm::dvec3 enu = frame.fromGeodetic(Geodetic(lat, lon, 0.0));
        return terrain.sampleHeight(enu.x, enu.y);
    };

    for (int row = 0; row < kHeightTexture; ++row) {
        const double v = (double(row) + 0.5) / kHeightTexture;
        for (int col = 0; col < kHeightTexture; ++col) {
            const double u = (double(col) + 0.5) / kHeightTexture;
            const double lon = chunk.box.west() + (chunk.box.east() - chunk.box.west()) * u;
            const double north_y = mercatorTile(chunk.box.north(), lon, 20).y;
            const double south_y = mercatorTile(chunk.box.south(), lon, 20).y;
            const double lat = mercatorLatitude(north_y + (south_y - north_y) * v, 20);

            const double height = heightAt(lat, lon);
            unsigned char* pixel = rgb.data() + (size_t(row) * kHeightTexture + col) * 3;
            elevationColour((height - low) / span, pixel);

            // A contour where the height is within half a texel's worth of
            // climb of a multiple of the interval. Expressed that way the
            // line stays about one pixel wide on a cliff and on a plain
            // alike; a fixed elevation tolerance would be a hairline on one
            // and a wide band on the other.
            const double step_m = 20.0;
            const double east_step = heightAt(lat, lon + step_m / 74000.0) - height;
            const double north_step = heightAt(lat + step_m / 110574.0, lon) - height;
            const double slope = std::hypot(east_step, north_step) / step_m;
            const double line_half_width = std::max(slope * metres_per_texel, 1e-4);
            const double from_line =
                std::fabs(height / interval - std::round(height / interval)) * interval;
            if (from_line < line_half_width) {
                for (int i = 0; i < 3; ++i) pixel[i] = (unsigned char)(pixel[i] * 0.45);
            }
        }
    }

    chunk.complete = true;
    if (chunk.texture == 0) glGenTextures(1, &chunk.texture);
    glBindTexture(GL_TEXTURE_2D, chunk.texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, kHeightTexture, kHeightTexture, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, rgb.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

bool GroundChunks::buildChunk(Chunk& chunk, const EnuFrame& frame, const Terrain& terrain,
                              const DemSource* dem, const GeoidModel& geoid, TileCache& cache,
                              TileFetcher* fetcher) {
    buildMesh(chunk, frame);
    if (mode_ == GroundMode::HeightMap) {
        return buildHeightTexture(chunk, frame, terrain, dem, geoid);
    }
    return buildTileTexture(chunk, cache, fetcher);
}

void GroundChunks::evictFarthest(int chunk_x, int chunk_y) {
    while (chunks_.size() > resident_limit_) {
        auto farthest = chunks_.end();
        long worst = -1;
        for (auto it = chunks_.begin(); it != chunks_.end(); ++it) {
            const long dx = it->second->chunk_x - chunk_x;
            const long dy = it->second->chunk_y - chunk_y;
            const long distance = dx * dx + dy * dy;
            if (distance > worst) {
                worst = distance;
                farthest = it;
            }
        }
        if (farthest == chunks_.end()) break;
        if (farthest->second->texture) glDeleteTextures(1, &farthest->second->texture);
        chunks_.erase(farthest);
    }
}

void GroundChunks::update(const ViewCamera& camera, const EnuFrame& frame,
                          const Terrain& terrain, const DemSource* dem,
                          const GeoidModel& geoid, TileCache& cache, TileFetcher* fetcher,
                          double max_ground_radius_m) {
    clock_ += 1.0;

    // What the camera can see on the ground, as a radius about where it is
    // looking. The chunks to have resident are the ones inside it - the whole
    // of the streaming decision, in one number.
    //
    // Two and a half times the orbit distance, not one: a camera looking down
    // at a shallow angle sees far more ground than it is standing off by, and
    // sizing the radius to the distance alone leaves the far half of the view
    // empty.
    const double view_radius =
        std::min(std::max(double(camera.distance()) * 2.5, 600.0), max_ground_radius_m);

    const glm::vec3 target = camera.target();
    const Geodetic centre = frame.toGeodetic(glm::dvec3(target.x, target.y, 0.0));

    // Zoom from how much ground one screen pixel covers at the subject, and
    // never finer than a chunk's own texture can hold.
    const TileSource* source = activeSource();
    const int min_zoom = source ? source->min_zoom : 0;
    const int max_zoom = source ? source->max_zoom : 19;
    const double screen_mpp =
        2.0 * double(camera.distance()) * std::tan(glm::radians(55.0) * 0.5) / 900.0;
    zoom_ = zoomNearest(centre.latitude_deg, std::max(screen_mpp, 0.05), min_zoom, max_zoom);
    metres_per_pixel_ = mercatorMetresPerPixel(centre.latitude_deg, zoom_);

    // Chunk coordinates of the view centre, and how many of them the radius
    // reaches. A chunk covers kTilesPerChunk tiles, so its size in metres
    // falls as the zoom rises - which is what keeps the resident count
    // roughly constant however far out the camera is.
    const double chunk_m = metres_per_pixel_ * DecodedTile::kSize * kTilesPerChunk;
    const glm::dvec2 centre_tile =
        mercatorTile(centre.latitude_deg, centre.longitude_deg, zoom_);
    const int centre_cx = int(std::floor(centre_tile.x / kTilesPerChunk));
    const int centre_cy = int(std::floor(centre_tile.y / kTilesPerChunk));
    // Capped at three rings - a seven by seven grid - because the resident
    // count is what costs memory: at 1024 pixels square a chunk is 3 MB, so
    // 49 of them is about 150 MB and 121 would not be affordable. Beyond this
    // the honest answer is coarser chunks further out, which is a quadtree
    // and a different piece of work.
    const int reach = std::min(3, std::max(1, int(std::ceil(view_radius / chunk_m))));

    // Nearest first, so what the viewer is looking at arrives before the rim.
    std::vector<std::pair<long, std::pair<int, int>>> wanted;
    for (int dy = -reach; dy <= reach; ++dy) {
        for (int dx = -reach; dx <= reach; ++dx) {
            wanted.push_back({long(dx) * dx + long(dy) * dy,
                              {centre_cx + dx, centre_cy + dy}});
        }
    }
    std::sort(wanted.begin(), wanted.end());

    const int tiles_total = int(std::pow(2.0, double(zoom_)));
    const int chunks_total = std::max(1, tiles_total / kTilesPerChunk);

    // New chunks and retries get separate budgets. Sharing one lets a view
    // full of half-covered chunks spend every frame retrying them and never
    // create the ones beyond, which leaves a patch of ground surrounded by
    // nothing however long you wait.
    const int delivered = fetcher ? fetcher->fetched() : 0;
    int built = 0, retried = 0;
    for (const auto& entry : wanted) {
        const int cx = entry.second.first, cy = entry.second.second;
        if (cx < 0 || cy < 0 || cx >= chunks_total || cy >= chunks_total) continue;

        const std::string id = key(cx, cy, zoom_);
        const auto found = chunks_.find(id);
        if (found != chunks_.end()) {
            Chunk& chunk = *found->second;
            chunk.last_used = clock_;
            // Only worth rebuilding if tiles have actually arrived since the
            // last attempt. Offline, that is never, and the chunk is left as
            // the partial picture it is.
            const bool worth_retrying =
                !chunk.complete && fetcher && delivered != chunk.built_at_delivery;
            if (worth_retrying && retried < 1) {
                if (buildChunk(chunk, frame, terrain, dem, geoid, cache, fetcher)) {
                    chunk.has_texture = true;
                }
                chunk.built_at_delivery = delivered;
                ++retried;
            }
            continue;
        }
        if (built >= kBuildsPerUpdate) continue;

        auto chunk = std::make_unique<Chunk>();
        chunk->chunk_x = cx;
        chunk->chunk_y = cy;
        chunk->zoom = zoom_;
        chunk->last_used = clock_;
        const double n = std::pow(2.0, double(zoom_));
        chunk->box = MercatorBox::fromBounds(
            mercatorLatitude(double((cy + 1) * kTilesPerChunk), zoom_),
            mercatorLatitude(double(cy * kTilesPerChunk), zoom_),
            mercatorLongitude(double(cx * kTilesPerChunk), zoom_),
            mercatorLongitude(double((cx + 1) * kTilesPerChunk), zoom_));
        (void)n;

        chunk->has_texture = buildChunk(*chunk, frame, terrain, dem, geoid, cache, fetcher);
        chunk->built_at_delivery = delivered;
        ++built;
        chunks_[id] = std::move(chunk);
    }

    // Drop whatever is now furthest from the view, and anything left over at
    // a zoom that is no longer current.
    for (auto it = chunks_.begin(); it != chunks_.end();) {
        if (it->second->zoom != zoom_) {
            if (it->second->texture) glDeleteTextures(1, &it->second->texture);
            it = chunks_.erase(it);
        } else {
            ++it;
        }
    }
    evictFarthest(centre_cx, centre_cy);
}

void GroundChunks::draw(Shader& shader) const {
    for (const auto& entry : chunks_) {
        const Chunk& chunk = *entry.second;
        if (!chunk.has_texture || chunk.texture == 0 || chunk.mesh.empty()) continue;
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, chunk.texture);
        shader.set("u_map", 0);
        chunk.mesh.draw();
    }
}

}  // namespace georay
