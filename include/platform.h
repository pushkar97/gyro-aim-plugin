// Thin platform abstraction so gyro.c's mapping/calibration logic can be
// compiled unmodified into both the real PS4 plugin and a macOS SDL-based
// tuning harness (see SDL/examples/input/07-gyro-aim-tuner). Only the one
// genuinely PS4-specific side effect (lightbar color) goes through here;
// everything else gyro.c does is plain math on a ScePadData struct.
#pragma once

#include <stdint.h>

// handle is the pad handle from the platform's read call (PS4: the scePad
// handle; harness: unused, pass 0).
void platform_set_lightbar(int32_t handle, uint8_t r, uint8_t g, uint8_t b);
