#pragma once

// Shared infrastructure for the selftest_*.cpp files: the assertion helpers,
// a reproducible PRNG, and the .hgt-writing helpers several of them need to
// exercise the DEM path without a network. Pulled out of selftest.cpp so
// each test file can be read - and compiled - on its own.

#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace georay {

void check(bool ok, const std::string& what);
void checkNear(double got, double want, double tol, const std::string& what);
void checkVecNear(const glm::dvec3& got, const glm::dvec3& want, double tol,
                  const std::string& what);

// The running totals, reset and read by runSelfTest() in selftest.cpp.
void resetCheckCounts();
int checksRun();
int failuresFound();

// A deterministic pseudo-random sequence, so a failure reported on one
// machine reproduces exactly on another.
struct Rng {
    unsigned int s = 0x9e3779b9u;
    double next() {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return double(s) / double(0xFFFFFFFFu);
    }
    double range(double lo, double hi) { return lo + (hi - lo) * next(); }
};

// Writes a .hgt file whose height at every post is a known function of the
// post indices, so what comes back out can be compared against arithmetic.
//
// Synthetic rather than a downloaded tile on purpose: this has to run in CI
// and on a machine with no network, and a test that needs 7 MB fetched from
// AWS is a test that gets skipped. What it verifies is the format handling -
// byte order, row order, voids, tile selection - which is exactly where the
// bugs in a headerless format live.
std::string writeTestHgt(const std::string& directory, const std::string& name, int side,
                         const std::vector<int16_t>& values);

std::string makeTempDir();

}  // namespace georay
