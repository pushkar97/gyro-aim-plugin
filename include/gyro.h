// Gyro-to-stick mapping, calibration/drift correction, and hotkey/state
// handling for the gyro-aim plugin. See main.c for the scePad hook glue.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pad.h"

// Per-title (or [default]) config, loaded once at plugin_load.
typedef struct GyroProfile {
    bool enabled;
    float sensitivity_h;    // stick units per rad/s, yaw -> stick X
    float sensitivity_v;    // stick units per rad/s, pitch -> stick Y
    float dead_zone;        // rad/s; |corrected gyro| below this is treated as 0
    int dead_zone_bias;     // stick units (0-127); minimum push applied when
                             // gyro contributes non-zero motion, so the game's
                             // own internal stick deadzone doesn't eat it
    int trigger_threshold;  // L2 analogButtons.l2 value (0-255) counted as "held"
    bool invert_x;
    bool invert_y;
    bool yaw_from_z;         // true: angularVelocity.z drives horizontal
                             // instead of .y. DS4 gyro axis conventions
                             // relative to how you physically hold/aim the
                             // controller aren't verified from public
                             // headers (see README's "Known unknowns"); this
                             // is an empirical escape hatch, same idea as
                             // invert_x/invert_y.
} GyroProfile;

// Loads [default] then overlays [<titleid>] (if present) from the given INI
// file path into *out_profile. Missing keys keep whatever hardcoded defaults
// gyro_profile_set_defaults() established. Returns false if the file itself
// could not be opened (profile is left at defaults in that case).
void gyro_profile_set_defaults(GyroProfile* profile);
bool gyro_profile_load(const char* ini_path, const char* title_id, GyroProfile* profile);

// One-time global init (call from plugin_load). Sets up calibration state.
void gyro_state_init(const GyroProfile* profile);

// Updates the active profile's tunable values in place, WITHOUT resetting
// calibration/drift/hotkey state. Used by live-tuning UIs (see the macOS
// SDL harness) so adjusting sensitivity mid-session doesn't force a
// recalibration hold. gyro_state_init() (full reset) is still what
// recalibration hotkeys/plugin_load use.
void gyro_set_profile(const GyroProfile* profile);
GyroProfile gyro_get_profile(void);

// Called from both scePadRead_hook and scePadReadState_hook for every
// individual ScePadData sample, in order, before it's returned to the game.
// `handle` is the pad handle (needed for scePadSetLightBar transitions).
void gyro_process_sample(int32_t handle, ScePadData* pData);
