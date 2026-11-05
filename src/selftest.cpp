// Numeric verification of the geolocation pipeline, run with --selftest.
//
// This exists because every error in this tool is silent: a wrong sign in a
// rotation, a transposed matrix, a degrees/radians slip - none of them crash
// or even look wrong on screen. They produce a confident latitude and
// longitude that is simply somewhere else. The tests below pin down the
// conventions (which way is yaw, which way is tilt, which axis is north) with
// cases whose answers are known independently of the code under test.
//
// The suite itself is split by subsystem into selftest_*.cpp - geodetic,
// geolocate, dem, streaming, formats - with the shared assertion helpers in
// selftest_support.hpp. This file is just the dispatcher: it calls each test
// function in turn and reports the totals.

#include "selftest.hpp"
#include "selftest_support.hpp"

#include <cstdio>

namespace georay {

void testGeodetic();
void testEnuFrame();
void testCameraModel();
void testDistortion();
void testPose();
void testTerrain();
void testGeolocateFlatClosedForm();
void testGeolocateRoundTrip();
void testGeolocateFailureModes();
void testFootprint();
void testGlProjectionMatchesModel();
void testRenderMatchesGeolocation();
void testHgtTile();
void testGeoid();
void testDemTerrain();
void testMercator();
void testFloatingOrigin();
void testJson();
void testMission();
void testWriteQgcPlan();

int runSelfTest() {
    resetCheckCounts();
    std::printf("georay selftest\n\n");

    testGeodetic();
    testEnuFrame();
    testCameraModel();
    testDistortion();
    testPose();
    testTerrain();
    testGeolocateFlatClosedForm();
    testGeolocateRoundTrip();
    testGeolocateFailureModes();
    testFootprint();
    testGlProjectionMatchesModel();
    testRenderMatchesGeolocation();
    testHgtTile();
    testGeoid();
    testDemTerrain();
    testMercator();
    testFloatingOrigin();
    testJson();
    testMission();
    testWriteQgcPlan();

    std::printf("\n%d checks, %d failures\n", checksRun(), failuresFound());
    return failuresFound() == 0 ? 0 : 1;
}

}  // namespace georay
