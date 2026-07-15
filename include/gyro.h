// Gyro-to-stick mapping, calibration/drift correction, and hotkey/state
// handling for the gyro-aim plugin. See main.c for the scePad hook glue.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pad.h"

#define MAX_GAIN_POINTS 8

// Per-title (or [default]) config, loaded once at plugin_load.
typedef struct GyroProfile {
    bool enabled;
    float dead_zone;        // rad/s; vector magnitude below this is treated
                             // as zero (deadzone applied to |(yaw,pitch)|,
                             // not per-axis — diagonal movement
                             // consistently deadzones at the same
                             // rotational speed as cardinal).
    int dead_zone_bias;     // stick units (0-127); minimum output vector
                             // MAGNITUDE applied when gyro contributes
                             // non-zero motion, so the game's own internal
                             // stick deadzone doesn't eat it. Direction is
                             // preserved (not a per-axis floor — the
                             // vector preserves the intended angle).
                             // Defaults to 20.
    int trigger_threshold;

    // Gain curve (direction-independent): stick_magnitude = gain(rate_magnitude)
    // * rate_magnitude. Linearly interpolated between (rate, gain) breakpoints.
    // A single curve applies regardless of movement direction — diagonal
    // movements use the same gain as pure horizontal/vertical at the same
    // rotational speed. gain_rates_h/values_h serve as the active curve;
    // gain_rates_v/values_v are stored for backwards compat with existing
    // configs but are DEPRECATED in the response model (only H is used).
    float gain_rates_h[MAX_GAIN_POINTS];
    float gain_values_h[MAX_GAIN_POINTS];
    int gain_count_h;
    float gain_rates_v[MAX_GAIN_POINTS];
    float gain_values_v[MAX_GAIN_POINTS];
    int gain_count_v;

    float lowpass_alpha;    // EMA on the mapped STICK OUTPUT (not the gyro
                             // input), applied while actively moving:
                             // float_stick += alpha * (raw - float_stick).
                             // 1.0 = no smoothing (snaps to raw each
                             // sample); lower = smoother but more lag.
                             // Mutually exclusive with damping_factor: this
                             // runs only while the rate is non-zero
                             // (outside the dead zone); damping runs only
                             // when the rate is exactly zero (inside it).
                             // They never compound.
float damping_factor;   // Interpolation-based damping: the fraction of
                             // the distance to zero covered each sample
                             // while the corrected rate is exactly zero
                             // (inside the dead zone):
                             //   float_stick += (0 - float_stick) * damping.
                             // 0.0 = no damping (sticks at current value);
                             // higher values = faster decay. More
                             // intuitive and less poll-rate-dependent than
                             // the old multiplier-based approach
                             // (task 1 of the refinement plan).
float saturation_strength;  // Soft-saturation applied to the output
                              // vector MAGNITUDE (not per-axis), so the
                              // movement direction is preserved even as
                              // the magnitude is clamped at the edge.
                              // tanhf((mag/128) * strength) * 128.
                              // 2.0 = moderate (default).

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
                             // yaw, before dead zone/gain-curve are
                             // applied. Useful if your natural aiming
                             // motion is a combined rotation+tilt rather
                             // than a pure single-axis yaw -- e.g. 0.3-0.5
                             // adds a noticeable but secondary contribution.
    bool drift_correction_enabled;  // true (default): continuously refine
                             // gyro bias via background EMA while the
                             // player is NOT aiming (L2 released) and the
                             // controller is stationary. Uses idle
                             // gameplay time (cutscenes, menus, running
                             // without aiming) and NEVER runs while
                             // aiming — deliberate motion cannot corrupt
                             // the estimate. Set false to rely solely on
                             // the one-time startup/recalibration bias.
    float bias_alpha;            // EMA blend per accepted sample (0.01
                             // default). Very slow — the bias estimator
                             // requires several seconds of stationary
                             // controller time to produce a noticeable
                             // change. Lower = more conservative.
    float bias_error_threshold;  // rad/s (0.05 default). |raw - bias|
                             // must be below this before bias is updated.
                             // Only nudges the bias when it's already
                             // close — deliberate motion is never
                             // accidentally learned as bias.
    int bias_stationary_samples; // consecutive still samples required
                             // before bias estimation begins (60
                             // default, ~250ms at 250Hz / ~1s at 60Hz).
                             // Prevents transient-motion corruption.
    float sensitivity_h;         // 1.0 = no scaling. Applied to stick_x
                             // AFTER vector processing and BEFORE output
                             // EMA smoothing. Simple output multiplier
                             // independent of deadzone/gain/saturation.
    float sensitivity_v;         // 1.0 = no scaling. Same as above for
                             // stick_y.
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

// Debug state snapshot, updated every gyro_process_sample() call for
// diagnostic rendering (used by the Mac tuner HUD).
typedef struct GyroDebug {
    float gyro[3];    // raw gx, gy, gz
    float bias[3];    // current bias[0], bias[1], bias[2]
    float yaw;        // post-bias, post-deadzone, post-invert yaw
    float pitch;      // post-bias, post-deadzone, post-invert pitch
} GyroDebug;

GyroDebug gyro_get_debug(void);

// Called from both scePadRead_hook and scePadReadState_hook for every
// individual ScePadData sample, in order, before it's returned to the game.
// `handle` is the pad handle (needed for scePadSetLightBar transitions).
void gyro_process_sample(int32_t handle, ScePadData* pData);
