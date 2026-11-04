// Real elevation data end to end: the headerless .hgt format (byte order,
// row order, voids, tile selection - exactly where the bugs in that format
// live), the geoid separation that turns orthometric heights into
// ellipsoidal ones, and loading a DEM into the ENU terrain the ray marcher
// actually uses.

#include "selftest_support.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>

#include "geo/dem.hpp"
#include "geo/geoid.hpp"
#include "geo/geolocate.hpp"

namespace georay {

void testHgtTile() {
    std::printf("srtm .hgt tiles\n");

    checkNear(0.0, 0.0, 0.0, "");  // keeps the check counter honest if we bail
    const std::string directory = makeTempDir();
    check(!directory.empty(), "temporary directory for the DEM tests");
    if (directory.empty()) return;

    // Tile names parse, including the hemispheres and the floor behaviour
    // that truncation gets wrong just south of the equator or west of
    // Greenwich.
    check(HgtTile::tileNameFor(48.137, 11.575) == "N48E011", "tile name, northern/eastern");
    check(HgtTile::tileNameFor(-0.5, -0.5) == "S01W001", "tile name floors, not truncates");
    check(HgtTile::tileNameFor(-33.9, 18.4) == "S34E018", "tile name, southern/eastern");
    check(HgtTile::tileNameFor(48.0, 11.0) == "N48E011", "tile name on an exact corner");
    // Garbage in must not produce a name no tile could have, or undefined
    // behaviour from converting a NaN to an int.
    check(HgtTile::tileNameFor(std::nan(""), 11.0) == "N00E011", "a NaN latitude is handled");
    check(HgtTile::tileNameFor(1e9, 1e9) == "N89E179", "an absurd position clamps to a real tile");

    int lat = 0, lon = 0;
    check(HgtTile::parseName("/data/dem/S34E018.hgt", lat, lon) && lat == -34 && lon == 18,
          "parse a path with directories and a suffix");
    check(!HgtTile::parseName("rubbish.hgt", lat, lon), "reject a name that is not a tile");
    check(!HgtTile::parseName("N48E011/x.bin", lat, lon),
          "a directory named like a tile does not make the file one");

    // A 3x3 tile covering N48E011, with a height that encodes its own post
    // indices: h = 1000 + 10*row + col, row 0 being the NORTH edge.
    const int side = 3;
    std::vector<int16_t> values(size_t(side * side));
    for (int r = 0; r < side; ++r) {
        for (int c = 0; c < side; ++c) values[size_t(r * side + c)] = int16_t(1000 + 10 * r + c);
    }
    const std::string path = writeTestHgt(directory, "N48E011", side, values);

    HgtTile tile;
    std::string error;
    check(HgtTile::load(path, tile, error), "load a tile: " + error);
    check(tile.samplesPerSide() == 3, "posting count comes from the file length");
    check(tile.cornerLatitude() == 48 && tile.cornerLongitude() == 11, "corner from the name");

    double h = 0.0;
    // The four corners. These are the assertions that catch a flipped row
    // order, which is otherwise invisible - the terrain still looks like
    // terrain, just mirrored north-south.
    check(tile.sample(49.0, 11.0, h), "sample the north-west corner");
    checkNear(h, 1000.0, 1e-9, "north-west corner is row 0, col 0");
    check(tile.sample(49.0, 12.0, h), "sample the north-east corner");
    checkNear(h, 1002.0, 1e-9, "north-east corner is row 0, last col");
    check(tile.sample(48.0, 11.0, h), "sample the south-west corner");
    checkNear(h, 1020.0, 1e-9, "south-west corner is the LAST row, col 0");
    check(tile.sample(48.0, 12.0, h), "sample the south-east corner");
    checkNear(h, 1022.0, 1e-9, "south-east corner is the last row, last col");

    // Interpolation between them.
    check(tile.sample(48.5, 11.5, h), "sample the tile centre");
    checkNear(h, 1011.0, 1e-9, "centre interpolates to the middle post");
    check(tile.sample(48.75, 11.25, h), "sample a quarter point");
    checkNear(h, 1005.5, 1e-9, "quarter point interpolates linearly");

    check(!tile.sample(47.5, 11.5, h), "a position south of the tile is off it");
    check(!tile.sample(48.5, 13.5, h), "a position east of the tile is off it");

    // Voids: a corner marked -32768 must not drag the surface 32 km down.
    std::vector<int16_t> voided = values;
    voided[0] = HgtTile::kVoid;  // north-west post
    writeTestHgt(directory, "N49E011", side, voided);
    HgtTile void_tile;
    check(HgtTile::load(directory + "/N49E011.hgt", void_tile, error), "load the voided tile");
    check(void_tile.voidCount() == 1, "voids are counted at load");
    check(void_tile.sample(49.5, 11.5, h), "a cell with one void still samples");
    check(h > 900.0 && h < 1100.0,
          "a void is excluded from the interpolation, not read as -32768 metres");

    // An entirely void cell is genuinely no-data and must say so.
    std::vector<int16_t> all_void(size_t(side * side), HgtTile::kVoid);
    writeTestHgt(directory, "N50E011", side, all_void);
    HgtTile empty_tile;
    check(HgtTile::load(directory + "/N50E011.hgt", empty_tile, error), "load an all-void tile");
    check(!empty_tile.sample(50.5, 11.5, h), "an all-void cell reports no data");

    // A file whose length is not a square number of samples is not a tile.
    {
        std::ofstream bad(directory + "/N51E011.hgt", std::ios::binary);
        const char junk[10] = {};
        bad.write(junk, sizeof(junk));
    }
    HgtTile bad_tile;
    check(!HgtTile::load(directory + "/N51E011.hgt", bad_tile, error),
          "reject a file that is not a square array");

    // The source picks the right tile per position, and reports what it could
    // not find rather than treating it as sea level.
    DemSource source(directory);
    check(source.sampleOrthometric(48.5, 11.5, h), "source finds N48E011");
    checkNear(h, 1011.0, 1e-9, "...and samples it correctly");
    check(source.sampleOrthometric(49.5, 11.5, h), "source finds N49E011");
    check(!source.sampleOrthometric(48.5, 20.5, h), "source reports no data where no tile is");
    check(source.loadedTiles().size() == 2, "only the tiles actually needed are loaded");
    check(source.missingTiles().size() == 1, "the absent tile is reported as missing");

    // The cache is bounded, because the aircraft travels. A 26 MB tile per
    // degree means a cache that only grows is not a streaming cache - it is
    // a slow way to load the whole dataset.
    {
        // Four tiles on disk, room for two.
        for (const char* name : {"N60E010", "N60E011", "N61E010", "N61E011"}) {
            writeTestHgt(directory, name, side, values);
        }
        DemSource bounded(directory, 2);
        double h = 0.0;
        check(bounded.sampleOrthometric(60.5, 10.5, h), "tile 1 loads");
        check(bounded.sampleOrthometric(60.5, 11.5, h), "tile 2 loads");
        check(bounded.residentTiles() == 2, "both fit under the limit");

        check(bounded.sampleOrthometric(61.5, 10.5, h), "tile 3 loads");
        check(bounded.residentTiles() == 2, "the limit is respected");
        check(bounded.loadCount() == 3, "three tiles have been read so far");

        // Tile 2 was used more recently than tile 1, so tile 1 is the one
        // that should have gone.
        check(bounded.sampleOrthometric(60.5, 11.5, h), "the recently used tile is still there");
        check(bounded.loadCount() == 3, "...and did not have to be re-read");

        check(bounded.sampleOrthometric(60.5, 10.5, h), "the evicted tile is re-read on demand");
        check(bounded.loadCount() == 4, "...which counts as another load");
        check(bounded.residentTiles() == 2, "and the limit still holds");
    }

    // Neighbouring tiles share their edge posts, so a position on the seam
    // must give the same height whichever side it is sampled from. Getting
    // this wrong puts a one-post cliff along every degree line.
    std::vector<int16_t> south(size_t(side * side));
    for (int r = 0; r < side; ++r) {
        for (int c = 0; c < side; ++c) south[size_t(r * side + c)] = int16_t(2000 + 10 * r + c);
    }
    // Make N47E011's north edge equal N48E011's south edge, as real tiles do.
    for (int c = 0; c < side; ++c) south[size_t(c)] = int16_t(1020 + c);
    writeTestHgt(directory, "N47E011", side, south);

    HgtTile north_tile, south_tile;
    check(HgtTile::load(directory + "/N48E011.hgt", north_tile, error), "load the north tile");
    check(HgtTile::load(directory + "/N47E011.hgt", south_tile, error), "load the south tile");
    double from_north = 0.0, from_south = 0.0;
    check(north_tile.sample(48.0, 11.5, from_north), "seam from the north tile");
    check(south_tile.sample(48.0, 11.5, from_south), "seam from the south tile");
    checkNear(from_north, from_south, 1e-9, "shared edge posts agree across the seam");
}

void testGeoid() {
    std::printf("geoid separation\n");

    const GeoidModel none;
    check(none.isIdentity(), "the default model applies no correction");
    checkNear(none.separationAt(48.0, 11.0), 0.0, 0.0, "...and reports zero separation");

    const GeoidModel flat = GeoidModel::constant(47.3);
    check(!flat.isIdentity(), "a constant model is not the identity");
    checkNear(flat.separationAt(48.0, 11.0), 47.3, 1e-12, "constant separation anywhere");
    checkNear(flat.separationAt(-33.9, 18.4), 47.3, 1e-12, "...really anywhere");
    // The conversion the whole model exists for: orthometric to ellipsoidal.
    checkNear(flat.toEllipsoidal(48.0, 11.0, 515.0), 562.3, 1e-12,
              "orthometric height plus separation is ellipsoidal height");
}

// Loading real elevation data end to end: DEM posts in, ENU heights out.
//
// The hard part to verify is that this cannot be checked by re-running the
// code under test, so every assertion below is against something known
// independently - a closed-form curvature drop, a gradient computed from
// metres-per-degree, an exact identity at the frame origin.
void testDemTerrain() {
    std::printf("dem -> enu terrain\n");

    const std::string directory = makeTempDir();
    check(!directory.empty(), "temporary directory for the terrain tests");
    if (directory.empty()) return;

    const int side = 61;  // 1/60 degree posting, so the values below are integers

    // --- a DEM of constant orthometric height -----------------------------
    //
    // Flat ground on the ellipsoid is NOT flat in a local tangent plane: it
    // curves away below it. That drop is d^2 / 2R, which is independently
    // computable, and it is the one effect that proves the conversion really
    // goes through the ellipsoid rather than just subtracting a number.
    {
        const std::vector<int16_t> flat(size_t(side * side), 500);
        writeTestHgt(directory, "N48E011", side, flat);

        const double separation = 47.0;
        const GeoidModel geoid = GeoidModel::constant(separation);
        // The origin's ellipsoidal height set to match the ground exactly, so
        // the terrain's ENU height at the origin must come out at zero.
        const Geodetic origin(48.5, 11.5, 500.0 + separation);
        const EnuFrame frame(origin);

        DemSource dem(directory);
        Terrain terrain(Terrain::Kind::Flat, 3000.0, 65, 0.0);
        const Terrain::DemLoadReport report = terrain.loadFromDem(dem, geoid, frame);

        check(report.ok, "a covered area loads: " + report.error);
        check(report.posts_from_data == report.posts_total, "every post came from the data");
        check(report.posts_filled == 0, "nothing had to be invented");
        check(report.tiles_used.size() == 1 && report.tiles_used.front() == "N48E011",
              "exactly the one covering tile was loaded");
        checkNear(report.origin_orthometric_m, 500.0, 1e-9, "origin ground height is reported");
        checkNear(report.origin_separation_m, separation, 1e-12, "separation is reported");

        checkNear(terrain.sampleHeight(0.0, 0.0), 0.0, 1e-6,
                  "ground at the frame origin sits at ENU zero");

        // d^2 / 2R, with R the local radius of curvature (~6.37e6 m at this
        // latitude): 3 km out, flat ground falls about 0.71 m below the
        // tangent plane. Zero here would mean the curvature term was dropped.
        const double expected_drop = -3000.0 * 3000.0 / (2.0 * 6.374e6);
        checkNear(terrain.sampleHeight(3000.0, 0.0), expected_drop, 0.02,
                  "flat ground curves away from the tangent plane to the east");
        checkNear(terrain.sampleHeight(0.0, 3000.0), expected_drop, 0.02,
                  "...and to the north by the same amount");
        check(expected_drop < -0.6, "the curvature drop this test checks for is non-trivial");

        // Changing the geoid separation must move the whole surface rigidly.
        Terrain shifted(Terrain::Kind::Flat, 3000.0, 65, 0.0);
        DemSource dem2(directory);
        shifted.loadFromDem(dem2, GeoidModel::constant(separation + 10.0), frame);
        checkNear(shifted.sampleHeight(0.0, 0.0) - terrain.sampleHeight(0.0, 0.0), 10.0, 1e-6,
                  "a 10 m larger separation raises the ground by exactly 10 m");
    }

    // --- a DEM with a known slope ----------------------------------------
    //
    // h = 500 + 1200*(lat - 48) + 600*(lon - 11), chosen so every post is a
    // whole number of metres (the format stores integers) and so the
    // north and east gradients differ enough that swapping them is obvious.
    {
        std::vector<int16_t> sloped(size_t(side * side));
        for (int r = 0; r < side; ++r) {
            for (int c = 0; c < side; ++c) {
                // Row 0 is the north edge, i.e. lat = 49.
                sloped[size_t(r * side + c)] = int16_t(1700 - 20 * r + 10 * c);
            }
        }
        writeTestHgt(directory, "N48E011", side, sloped);

        const Geodetic origin(48.5, 11.5, 1000.0);
        const EnuFrame frame(origin);
        DemSource dem(directory);
        Terrain terrain(Terrain::Kind::Flat, 2000.0, 65, 0.0);
        check(terrain.loadFromDem(dem, GeoidModel::constant(47.0), frame).ok,
              "the sloped DEM loads");

        // Central differences over 2 km, against metres-per-degree at 48.5N.
        // These two numbers differ by a third, so an east/north swap cannot
        // slip through, and a sign error flips them negative.
        const double dz_dn =
            (terrain.sampleHeight(0.0, 1000.0) - terrain.sampleHeight(0.0, -1000.0)) / 2000.0;
        const double dz_de =
            (terrain.sampleHeight(1000.0, 0.0) - terrain.sampleHeight(-1000.0, 0.0)) / 2000.0;

        const double metres_per_degree_lat = 111200.0;  // at 48.5 degrees
        const double metres_per_degree_lon = 73900.0;
        checkNear(dz_dn, 1200.0 / metres_per_degree_lat, 2e-4,
                  "northward slope matches 1200 m per degree of latitude");
        checkNear(dz_de, 600.0 / metres_per_degree_lon, 2e-4,
                  "eastward slope matches 600 m per degree of longitude");
        check(dz_dn > dz_de * 1.2, "north and east gradients are not interchangeable");
    }

    // --- across a tile seam ----------------------------------------------
    //
    // An origin on a degree corner needs four tiles. If tile selection or the
    // shared edge posts were wrong, a one-post cliff would run through the
    // middle of the scene.
    {
        const std::vector<int16_t> flat(size_t(side * side), 800);
        for (const char* name : {"N47E010", "N47E011", "N48E010", "N48E011"}) {
            writeTestHgt(directory, name, side, flat);
        }

        const Geodetic origin(48.0, 11.0, 800.0 + 47.0);
        const EnuFrame frame(origin);
        DemSource dem(directory);
        Terrain terrain(Terrain::Kind::Flat, 2000.0, 65, 0.0);
        const Terrain::DemLoadReport report =
            terrain.loadFromDem(dem, GeoidModel::constant(47.0), frame);

        check(report.ok, "a scene straddling four tiles loads");
        check(report.tiles_used.size() == 4, "all four tiles were used");
        check(report.posts_filled == 0, "no gaps across the seams");

        // Nothing but the smooth curvature drop may appear: step across the
        // seam in 50 m increments and require the surface to stay continuous.
        double worst_step = 0.0;
        double previous = terrain.sampleHeight(-500.0, 0.0);
        for (double e = -450.0; e <= 500.0; e += 50.0) {
            const double here = terrain.sampleHeight(e, 0.0);
            worst_step = std::max(worst_step, std::fabs(here - previous));
            previous = here;
        }
        checkNear(worst_step, 0.0, 0.01, "no cliff where tiles meet (worst step, metres)");
    }

    // --- missing coverage -------------------------------------------------
    {
        const Geodetic origin(0.0, 0.0, 0.0);  // nothing on disk covers this
        const EnuFrame frame(origin);
        DemSource dem(directory);
        Terrain terrain(Terrain::Kind::Hills, 2000.0, 33, 0.0);
        const double before = terrain.sampleHeight(0.0, 0.0);

        const Terrain::DemLoadReport report = terrain.loadFromDem(dem, GeoidModel(), frame);
        check(!report.ok, "a load with no coverage at all fails");
        check(!report.error.empty(), "...and says why");
        checkNear(terrain.sampleHeight(0.0, 0.0), before, 1e-12,
                  "...and leaves the previous terrain alone rather than flattening it");
        check(terrain.sourceName() == "HILLS", "...including what the terrain says it is");
    }

    // --- partial coverage -------------------------------------------------
    //
    // Half a scene of real data is still useful, but the invented half has to
    // be declared. A gap filled silently is a fabricated elevation.
    {
        const std::vector<int16_t> flat(size_t(side * side), 300);
        writeTestHgt(directory, "N20E030", side, flat);
        // The origin sits near the tile's western edge, so the scene runs off
        // it into a neighbour that does not exist.
        const Geodetic origin(20.5, 30.002, 347.0);
        const EnuFrame frame(origin);
        DemSource dem(directory);
        Terrain terrain(Terrain::Kind::Flat, 2000.0, 65, 0.0);
        const Terrain::DemLoadReport report =
            terrain.loadFromDem(dem, GeoidModel::constant(47.0), frame);

        check(report.ok, "a partly covered scene still loads");
        check(report.posts_filled > 0, "the uncovered posts are counted as filled");
        check(report.gapFraction() > 0.0 && report.gapFraction() < 1.0,
              "the gap fraction is reported for the user to judge");
        check(!report.tiles_missing.empty(), "the absent tile is named");
        // Filled posts must be plausible, not zero or a void marker.
        check(terrain.minHeight() > -100.0 && terrain.maxHeight() < 100.0,
              "filled posts take sane values rather than zero or -32768");
    }
}

}  // namespace georay
