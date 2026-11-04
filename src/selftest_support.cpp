#include "selftest_support.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>

namespace georay {
namespace {
int g_failures = 0;
int g_checks = 0;
}  // namespace

void check(bool ok, const std::string& what) {
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("  FAIL  %s\n", what.c_str());
    }
}

void checkNear(double got, double want, double tol, const std::string& what) {
    ++g_checks;
    const double err = std::fabs(got - want);
    // NaN fails every comparison, including err <= tol, so it lands here
    // rather than silently passing - which is the point, since a NaN
    // latitude is exactly the kind of failure this tool must not ship.
    if (!(err <= tol)) {
        ++g_failures;
        std::printf("  FAIL  %s: got %.9f want %.9f (err %.3g > %.3g)\n", what.c_str(), got,
                    want, err, tol);
    }
}

void checkVecNear(const glm::dvec3& got, const glm::dvec3& want, double tol,
                  const std::string& what) {
    ++g_checks;
    const double err = glm::length(got - want);
    if (!(err <= tol)) {
        ++g_failures;
        std::printf("  FAIL  %s: got (%.6f %.6f %.6f) want (%.6f %.6f %.6f) err %.3g\n",
                    what.c_str(), got.x, got.y, got.z, want.x, want.y, want.z, err);
    }
}

void resetCheckCounts() {
    g_checks = 0;
    g_failures = 0;
}
int checksRun() { return g_checks; }
int failuresFound() { return g_failures; }

std::string writeTestHgt(const std::string& directory, const std::string& name, int side,
                         const std::vector<int16_t>& values) {
    const std::string path = directory + "/" + name + ".hgt";
    std::ofstream file(path, std::ios::binary);
    for (int i = 0; i < side * side; ++i) {
        const int16_t v = values[size_t(i)];
        // Big-endian, as the format requires. Written byte by byte so the
        // test does not depend on the host's byte order either.
        const unsigned char bytes[2] = {(unsigned char)((uint16_t(v) >> 8) & 0xFF),
                                        (unsigned char)(uint16_t(v) & 0xFF)};
        file.write(reinterpret_cast<const char*>(bytes), 2);
    }
    return path;
}

std::string makeTempDir() {
    // TMPDIR when the environment sets one (the sandbox here does), so the
    // test writes where it is allowed to.
    const char* base = std::getenv("TMPDIR");
    std::string directory = std::string(base && *base ? base : "/tmp") + "/georay_selftest_dem";
    const std::string command = "rm -rf '" + directory + "' && mkdir -p '" + directory + "'";
    if (std::system(command.c_str()) != 0) return {};
    return directory;
}

}  // namespace georay
