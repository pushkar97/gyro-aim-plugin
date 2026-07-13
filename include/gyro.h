// Gyro-to-stick mapping, calibration/drift correction, and hotkey/state
// handling for the gyro-aim plugin. See main.c for the scePad hook glue.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pad.h"

// Per-title (or [default]) config, loaded once at plugin_load.

#define MAX_GAIN_POINTS 8

typedef struct GyroProfile {
    bool enabled;
    float dead_zone;        // rad/s; |corrected gyro| below this is treated as 0
    int dead_zone_bias;     // stick units (0-127); optional minimum push
                             // applied at the final output stage so tiny
                             // movements still pass a game's internal
                             // stick deadzone. Set to 0 when the gain
                             // curve alone already punches through.
    int trigger_threshold;  // L2 analogButtons.l2 value (0-255) counted as "held"

    // Gain curve (replaces the old CurvePower / CurveMinRate / sensitivity
    // multiply). Each axis gets an independent table of (rate_breakpoint,
    // gain_value) pairs; gain is linearly interpolated between breakpoints.
    // Below the first breakpoint the first gain is used; above the last,
    // the last gain. The output stick value for a given rate is:
    //     raw_stick = corrected_rate × gain_lookup(|rate|)
    // Sensible defaults give high gain (90–25) that tapers down as rate
    // increases, so tiny motions are amplified for precision while large
    // motions don't clip.
    float gain_rates_h[MAX_GAIN_POINTS];
    float gain_values_h[MAX_GAIN_POINTS];
    int gain_count_h;
    float gain_rates_v[MAX_GAIN_POINTS];
    float gain_values_v[MAX_GAIN_POINTS];
    int gain_count_v;

    // EMA applied to the stick output (not the gyro rate). 1.0 = off
    // (default — raw stick value passes through un-smoothed).
    float lowpass_alpha;

    // Multiplicative decay when the gyro rate is inside the deadzone
    // (player is aiming but not actively rotating). Each sample:
    //     float_stick *= damping_factor
    // Default 0.88 gives a rapid but not-jarring stop.
    float damping_factor;

    // Knee for the tanhf-based soft saturation applied just before
    // uint8_t conversion: sat = tanhf(stick / knee) * 128.0f.
    // Lower knee = more aggressive saturation. Default 100.0.
    float saturation_knee;

    bool invert_x;
    bool invert_y;
    bool yaw_from_z;         // true: angularVelocity.z drives horizontal
                              // instead of .y.
    float yaw_tilt_weight;   // 0.0 = off. Blends a weighted contribution
                              // from the axis that yaw_from_z did NOT pick
                              // into yaw, before dead zone/gain/damping.
    bool drift_correction_enabled;  // true (default): continuous bias tracking.
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
