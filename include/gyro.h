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
                             // Defaults to 20. Ramped in, not snapped —
                             // see dead_zone_bias_ramp_scale.
    float dead_zone_bias_ramp_scale; // unitless (4.0 default). The
                             // dead_zone_bias floor ramps linearly from
                             // 0 at the dead-zone boundary up to the
                             // full dead_zone_bias value as the input
                             // magnitude grows from dead_zone to
                             // dead_zone * dead_zone_bias_ramp_scale.
                             // Previously the floor snapped straight to
                             // dead_zone_bias for ANY sample that
                             // cleared the dead zone at all, making the
                             // boundary a binary gate (0 output on one
                             // side, a fixed ~10-20 unit jump on the
                             // other) rather than a true floor — small
                             // enough that a residual bias error of
                             // only 0.001-0.002 rad/s could reliably
                             // decide which side of the gate a
                             // borderline sample landed on, which reads
                             // in-game as one direction being easy to
                             // push and the other feeling dead. Ramping
                             // means a small shift in exactly where the
                             // boundary sits only shifts where the ramp
                             // starts, not the size of a jump. Must be
                             // > 1.0 to have any effect (a value of 1.0
                             // or less falls back to the old snap
                             // behavior).
    int trigger_threshold;

    // Gain curve (direction-independent): stick_magnitude = gain(rate_magnitude)
    // * rate_magnitude. Linearly interpolated between (rate, gain) breakpoints.
    // A single curve applies regardless of movement direction — diagonal
    // movements use the same gain as pure horizontal/vertical at the same
    // rotational speed. This is the only gain curve in the response
    // model: the vector-magnitude pipeline has no separate horizontal/
    // vertical curve to apply (a prior gain_rates_v/gain_values_v pair
    // was removed — it was parsed from config but never read by
    // process_vector; per-axis feel is controlled via sensitivity_h/v
    // below instead).
    float gain_rates_h[MAX_GAIN_POINTS];
    float gain_values_h[MAX_GAIN_POINTS];
    int gain_count_h;

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
    float bias_max_correction_step;  // rad/s (0.05 default). Maximum
                              // per-step correction applied to the bias
                              // estimate during a single stationary
                              // window. Clamps alpha * error so a
                              // badly-off axis converges gradually
                              // instead of jumping.
    int bias_stationary_samples; // consecutive still samples required
                              // before bias estimation begins (60
                              // default, ~250ms at 250Hz / ~1s at 60Hz).
                              // Prevents transient-motion corruption.
    float bias_delta_ema_alpha;  // EMA smoothing factor (0.2 default) for
                              // the per-axis sample-to-sample delta used
                              // to detect stillness. Stillness is judged
                              // from the raw gyro signal's own flatness,
                              // NOT from distance to the current bias
                              // estimate — the latter can deadlock if the
                              // initial calibration missed on an axis,
                              // since being close to a wrong bias would
                              // never happen. Higher = the delta estimate
                              // reacts faster to new motion/stillness but
                              // is noisier.
    float bias_stillness_delta_threshold; // rad/s (0.01 default). The
                             // smoothed per-axis delta must stay below
                             // this for a sample to count as "flat".
                             // This alone only catches abrupt/jittery
                             // motion (it measures sample-to-sample
                             // change, not the reading itself) -- a
                             // smooth, constant-rate rotation such as
                             // gently settling onto a target has almost
                             // no delta and can slip through undetected.
                             // See bias_stillness_magnitude_threshold,
                             // which is required in addition.
    float bias_stillness_magnitude_threshold; // rad/s (0.20 default). The
                             // raw (bias-corrected) reading's magnitude
                             // must ALSO stay below this for a sample to
                             // count as stationary -- this is the check
                             // that catches smooth, low-jerk motion the
                             // delta check alone would miss. Deliberately
                             // compared against a fixed physical bound,
                             // not the current bias estimate, so it can't
                             // reintroduce the old deadlock where a bad
                             // initial calibration prevents itself from
                             // ever being corrected. Should be set well
                             // above plausible DS4 zero-rate offset
                             // (typically ~0.05-0.10 rad/s) and well
                             // below any real intentional movement.
                             // Both this AND the delta check must pass
                             // for a sample to count as still --
                             // deliberately biased toward missing
                              // legitimate stillness (costs convergence
                              // speed only) over misreading real motion
                              // as stillness (corrupts the estimate).
    int bias_settle_samples;     // samples after L2 release before bias
                              // estimation is allowed to resume (50
                              // default, ~200ms at 250Hz). The moments
                              // immediately after releasing the aim
                              // trigger often contain residual hand
                              // settling and micro-adjustments that
                              // look stationary but aren't.
    float bias_drift_accum_threshold; // rad/s (0.01 default). After
                              // the stationary timer elapses, the
                              // per-axis accumulated bias-corrected
                              // error is divided by the window size to
                              // get an average error rate. Sensor noise
                              // cancels out (~0 mean), while sustained
                              // slow motion produces a consistent
                              // non-zero average. If any axis exceeds
                              // this threshold, the window is rejected.
                              // Independent of BiasStationarySamples.
    float bias_max_total_deviation; // rad/s (0.10 default). Hard ceiling
                              // on how far the runtime-estimated bias
                              // is allowed to drift from the value
                              // established at the last (re)calibration,
                              // in either direction, per axis.
                              // bias_max_correction_step alone only
                              // bounds a single update's size -- it
                              // doesn't stop many small same-direction
                              // updates (e.g. a habitual resting-grip
                              // tilt that repeatedly reads as
                              // "stationary") from walking the bias
                              // arbitrarily far over a long session,
                              // which shows up in-game as the crosshair
                              // becoming easy to push one direction and
                              // increasingly dead/stuck in the opposite
                              // direction. This clamp still allows
                              // genuine slow sensor drift to be tracked,
                              // just within a bounded radius.
    float sensitivity_h;         // 1.0 = no scaling. Applied to stick_x
                             // AFTER vector processing and BEFORE output
                             // EMA smoothing. Simple output multiplier
                             // independent of deadzone/gain/saturation.
    float sensitivity_v;         // 1.0 = no scaling. Same as above for
                             // stick_y.

    // Post-flick suppression: a strong flick's deceleration/rebound is a
    // real physical motion, but the gain curve's low-end boost (tuned for
    // precision aim) and dead_zone_bias's guaranteed floor both end up
    // amplifying that small rebound far more than the flick that caused
    // it — the crosshair lands on target and then visibly kicks back.
    // Samples immediately following a fast flick get a temporarily
    // widened dead zone and capped gain for a short cooldown so the
    // rebound doesn't get boosted. Normal precision aim, which never
    // crosses flick_mag_threshold, is unaffected.
    float flick_mag_threshold;   // rad/s (1.00 default). Vector magnitude
                             // at/above which a sample counts as the
                             // flick itself (never suppressed) rather
                             // than a post-flick sample. Matches the
                             // default gain curve's own breakpoint where
                             // gain flattens to its lowest value.
    int flick_suppression_samples;  // cooldown length in samples after a
                             // flick ends (12 default). Starting point —
                             // tune against real flick footage/logs.
    float flick_suppression_deadzone_scale; // dead_zone multiplier during
                             // the cooldown (5.0 default: 0.02 -> 0.10
                             // with default DeadZone). Still far below a
                             // real follow-up flick's magnitude, so an
                             // intentional quick retarget isn't swallowed.
    float flick_suppression_gain_cap; // gain ceiling during the cooldown
                             // (30.0 default) — roughly the flick's own
                             // high-speed gain, so anything that does
                             // clear the widened dead zone isn't boosted
                             // as if it were slow, deliberate aim.
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
    int flick_suppress;  // 0 = inactive, 1 = currently suppressing rebound
    int stillness;       // 0 = not still, 1 = stillness gates passing (flat + near-zero)
    int bias_active;     // 0 = bias estimation idle, 1 = stationary window accumulating
} GyroDebug;

GyroDebug gyro_get_debug(void);

// Called from both scePadRead_hook and scePadReadState_hook for every
// individual ScePadData sample, in order, before it's returned to the game.
// `handle` is the pad handle (needed for scePadSetLightBar transitions).
void gyro_process_sample(int32_t handle, ScePadData* pData);