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
    float curve_power;       // 1.0 = linear, >1.0 = exponential (signed).
                             // Applied to the corrected gyro rate before
                             // multiplying by sensitivity, so small
                             // movements get a smoother ramp-up while
                             // large flicks still max out.
    float curve_min_rate;    // rad/s; below this abs(v) the curve is forced
                             // to pass-through (linear, 1:1 on the already
                             // drift-corrected value), so tiny residual
                             // bias ±<0.15 doesn't get exponentially
                             // amplified into a directional asymmetry.
    bool invert_x;
    bool invert_y;
    bool yaw_from_z;         // true: angularVelocity.z drives horizontal
                             // instead of .y. DS4 gyro axis conventions
                             // relative to how you physically hold/aim the
                             // controller aren't verified from public
                             // headers (see README's "Known unknowns"); this
                             // is an empirical escape hatch, same idea as
                             // invert_x/invert_y.
    float yaw_tilt_weight;   // 0.0 = off (default). Blends in a weighted
                             // contribution from the OTHER horizontal-ish
                             // axis (the one yaw_from_z did NOT pick) into
                             // yaw, before dead zone/curve/sensitivity are
                             // applied. Useful if your natural aiming
                             // motion is a combined rotation+tilt rather
                             // than a pure single-axis yaw -- e.g. 0.3-0.5
                             // adds a noticeable but secondary contribution.
    bool drift_correction_enabled;  // true (default): continuously re-average
                             // gyro bias while the controller is detected as
                             // stationary (via accelerometer magnitude), to
                             // counter slow real-world drift during long aim
                             // holds. Set false to rely solely on the
                             // one-time startup/recalibration-hotkey bias
                             // instead, if continuous correction is ever
                             // fighting a deliberately-held tilt.
    float lowpass_alpha;     // 1.0 = no filtering (default, unchanged
                             // behavior). Exponential moving average
                             // applied to the bias-corrected yaw/pitch
                             // rate before dead zone/curve/sensitivity:
                             // filtered += alpha * (raw - filtered).
                             // Lower alpha = smoother but more lag; useful
                             // to reduce sensor-noise jitter at low
                             // sensitivity/low dead zone settings.
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
