#pragma once

namespace georay {
// Runs the numeric verification suite. Returns 0 when everything passed.
// Touches no GL and opens no window, so it runs headless in CI.
int runSelfTest();
}  // namespace georay
