// Real PS4 implementation of the platform_set_lightbar() hook declared in
// platform.h. Kept in its own translation unit (rather than inline in
// gyro.c) so gyro.c has zero PS4-specific includes and can be compiled
// unmodified into the macOS tuning harness (see
// SDL/examples/input/07-gyro-aim-tuner), which supplies its own
// platform_set_lightbar() instead of linking this file.
#include "pad.h"
#include "platform.h"

void platform_set_lightbar(int32_t handle, uint8_t r, uint8_t g, uint8_t b) {
    ScePadColor color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.reserve = 0;
    scePadSetLightBar(handle, &color);
}
