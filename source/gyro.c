// Gyro-to-stick response model, calibration/drift correction, and
// hotkey/state handling for the gyro-aim plugin.
//
// Response model pipeline (per axis, H=yaw/horizontal, V=pitch/vertical):
//
//   gyro rate (bias-corrected, dead-zoned with hysteresis, inverted)
//     -> gain curve   [stick_raw = rate * gain(|rate|), per-axis, linearly
//                      interpolated lookup table]
//     -> EMA or damping, mutually exclusive [float_stick persists across
//                      samples: EMA blends toward stick_raw while actively
//                      moving; interpolation-based damping eases toward 0
//                      while the rate is inside the dead zone. They never
//                      run in the same sample, so their effects never
//                      compound.]
//     -> soft saturation [tanhf(float_stick / knee) * 128, replacing a
//                      hard clamp with a smooth asymptote]
//     -> SUMMED onto the physical stick's current position (not a
//                      replacement -- see write_stick_uint8) -> uint8
//                      stick value, hard-clamped to the valid range
//
// float_stick resets to 0 whenever aiming stops (L2 released, gyro
// runtime-disabled, or profile disabled) as well as on calibration/
// recalibration, so resuming aiming after a pause never partially blends
// in a stale value from before the pause.
//
// Post-flick suppression: samples immediately following a fast flick
// (magnitude >= GyroProfile.flick_mag_threshold) get a temporarily widened
// dead zone and capped gain for flick_suppression_samples samples, so the small
// physical rebound after a fast stop doesn't get amplified into a
// visible crosshair kick-back. See the FLICK_* constants and
// process_vector() for details.
//
// Deliberate design choices carried over from earlier design/grilling
// sessions:
// - Velocity-based mapping: instantaneous corrected angular velocity feeds
//   the gain curve every sample, with NO integration over time and NO dt
//   term. This matches how native gyro-aim (e.g. Splatoon/JoyShockMapper-
//   style) works: the stick's value each read represents the CURRENT turn
//   rate, same semantics the game already gives a physical stick held at a
//   constant deflection.
// - Calibration/drift timing uses SAMPLE COUNTS, not the ScePadData
//   `timestamp` field, because that field's exact units/epoch could not be
//   verified from public headers. Counting samples avoids depending on an
//   unverified unit assumption; because both scePadRead (averaged) and
//   scePadReadState funnel through gyro_process_sample() per-sample, sample
//   counts scale consistently with real elapsed time regardless of which
//   entry point the game uses.
// - Crash-safety: gx/gy/gz are NaN/Inf-filtered to 0.0 immediately on
//   read (below), and tanhf() in the final saturation step bounds its
//   output to a finite range for any finite input. Combined, this makes a
//   dedicated extra clamp before the final float-to-int cast unnecessary.
#include "gyro.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gyro_common.h"
#include "config.h"
#include "platform.h"

#define CALIB_SAMPLE_COUNT 500        // accepted stationary samples needed
// --- Bias estimation (replaces the old drift-correction system) ----------
// Bias estimation only runs when the player is NOT aiming (L2 released),
// making use of the large amount of idle time that naturally occurs during
// gameplay (cutscenes, menus, running without aiming, etc.). Each axis
// converges independently so a noisy gyro channel never blocks the others.
//
// Pipeline:
//   1. Startup calibration gives a rough initial bias estimate.
//   2. At runtime, whenever NOT aiming AND the controller is stationary
//      (accel near gravity + raw gyro below threshold) for a minimum
//      number of consecutive samples, the bias is slowly nudged via EMA
//      toward the observed reading.
//   3. An additional error check (|raw - bias| < threshold) ensures the
//      bias is only updated when it's already close — deliberate motion
//      is never accidentally learned as bias.
//
// The tunable bias parameters (alpha, error threshold, stationary sample
// count) are now in GyroProfile (see gyro.h) and configurable via
// gyroaim.ini (BiasAlpha, BiasMaxCorrectionStep, BiasStationarySamples,
#define BIAS_LOG_INTERVAL 60           // log every N successful bias updates

// --- Stillness detection ---------------------------------------------
// "Is the controller physically still?" is detected from the RAW gyro
// signal's own sample-to-sample variance, NOT from how close the raw
// reading is to the current bias estimate. The previous (error-based)
// approach created a deadlock: if the initial calibration missed by more
// than the threshold on some axis, that axis's error could never shrink,
// because being close to the current bias was the precondition for
// moving it closer. A flat, low-jitter raw signal is evidence of a
// stationary controller regardless of whether the bias estimate behind
// it is currently correct.
//
// An accelerometer-based check (accel magnitude near gravity) was
// considered but isn't used here: the calibration section below already
// hit unreliable/zeroed accel data on some PS4 firmware variants and
// dropped it for that reason. Same constraint applies here.
// Tunables: GyroProfile.bias_delta_ema_alpha, .bias_stillness_delta_threshold.

#define STICK_CENTER 128
#define STICK_MIN 0
#define STICK_MAX 255

// Dead-zone hysteresis (task 6): enter at a lower threshold, exit at a
// higher one, so the dead-zone boundary is a band rather than a hard line.
// Both are expressed as fractions of the configured dead_zone value so a
// single DeadZone config key still controls the overall feel.
#define DEADZONE_ENTER_FRAC 0.9f   // 0.9 * dead_zone = 0.018 (for default 0.02)
#define DEADZONE_EXIT_FRAC 1.2f    // 1.2 * dead_zone = 0.024

// L3+R3 toggle and touchpad-hold recalibrate hotkeys.
#define RECALIBRATE_HOLD_SAMPLES 150

// Post-flick suppression: a strong flick's deceleration/rebound is a real
// physical motion (wrist/hand relaxing past neutral after a fast stop),
// but the gain curve's low-end boost (tuned for precision aim) and
// DeadZoneBias's guaranteed floor both end up amplifying that small
// rebound far more than the flick that caused it -- the crosshair lands
// on target and then visibly kicks back. When a sample's magnitude
// crosses flick_mag_threshold, a short cooldown is armed for the samples
// that follow: the dead zone is temporarily widened and the gain is
// capped, so a small unintentional rebound right after a flick doesn't
// get boosted. Normal precision aim, which never crosses the flick
// threshold, is completely unaffected.
// Tunables: GyroProfile.flick_mag_threshold, .flick_suppression_samples,
// .flick_suppression_deadzone_scale, .flick_suppression_gain_cap.

typedef enum LightbarState { LB_UNSET = -1, LB_INACTIVE, LB_ACTIVE, LB_CALIBRATING } LightbarState;

static GyroProfile g_profile;

static bool g_calibrated = false;
static int g_calib_count = 0;          // accepted (stationary) samples
static float g_calib_sum[3] = { 0, 0, 0 };
static float g_bias[3] = { 0, 0, 0 };
static float g_calibrated_bias[3] = { 0, 0, 0 };  // bias as of the last
                                     // (re)calibration -- the anchor
                                     // point runtime bias drift is
                                     // clamped around (see
                                     // bias_max_total_deviation)

static bool g_is_stationary = false;  // global: raw gyro signal is flat
static int g_stationary_timer = 0;  // global: consecutive samples the raw
                                     // gyro signal has been flat

static float g_prev_gyro[3] = { 0, 0, 0 };  // previous raw sample, for delta calc
static bool g_prev_gyro_valid = false;
static float g_delta_ema[3] = { 0, 0, 0 };  // smoothed |sample-to-sample delta|, per axis

static bool g_runtime_enabled = true;
static bool g_prev_l3r3_held = false;

static int g_touchpad_hold_count = 0;
static bool g_recal_armed = true;

static LightbarState g_lightbar_state = LB_UNSET;

static int g_bias_log_counter = 0;

static int g_l2_release_samples = 0;  // consecutive samples L2 released
#define L2_RELEASE_RESET_SAMPLES 120   // hard-reset after ~2s of release

static int g_flick_suppress_timer = 0;  // samples remaining in the
                                         // post-flick suppression cooldown
static float g_flick_dir_yaw = 0.0f;   // normalized direction of last
static float g_flick_dir_pitch = 0.0f; // detected flick, for rebound detection
#define FLICK_REVERSAL_DOT -0.5f        // dot product below this = rebound
                                         // (movement >120° away from flick)

// Post-aiming settle delay: prevents post-trigger-release hand settling
// from being learned as bias. Armed when L2 transitions aiming→not aiming.
static int g_settle_timer = 0;

// Windowed drift accumulator: running sum of (raw - bias) per axis during
// stationary evaluation. Resets when stillness breaks or settle timer arms.
// Noise cancels out (sqrt(N)); sustained slow motion accumulates (N).
static float g_drift_acc[3] = { 0, 0, 0 };

// Edge-detection: did the last sample have L2 pressed?
static bool g_was_aiming = false;

// Physical stick drift compensation: DS4/DS5 analog stick potentiometers
// commonly drift from their nominal rest position (128,128) over the life
// of the controller (and can shift further as it warms up mid-session).
// write_stick_uint8 sums gyro output ONTO the physical stick's current
// reading -- if that reading never actually rests at 128 when untouched,
// the drift becomes a constant phantom push in one direction, all the
// time, independent of anything the gyro is doing. g_stick_center tracks
// the LEARNED rest position per axis so physical deflection is measured
// relative to where the stick actually rests, not the nominal constant.
// Deliberately mirrors the (corrected) gyro bias estimator's two-gate
// pattern: flatness (not moving) + a threshold that is a FIXED physical
// bound rather than relative to the current estimate (to avoid the same
// deadlock a bad initial guess could otherwise cause), plus a bounded
// total deviation. Learned only while not aiming (see gyro_process_sample)
// since that's the only time pData->rightStick is guaranteed to still be
// the raw untouched hardware reading, not something write_stick_uint8 has
// already written to.
static float g_stick_center[2] = { (float)STICK_CENTER, (float)STICK_CENTER };
static float g_stick_prev[2] = { 0, 0 };
static bool g_stick_prev_valid = false;
static float g_stick_delta_ema[2] = { 0, 0 };
static int g_stick_stationary_timer = 0;

// Drift-check failure tracking: consecutive windows where the drift
// accumulator rejected the update. Escalates the correction rate after
// repeated failures — a single failure may be transient motion, but
// many in a row indicate a bad initial calibration that must be fixed.
static int g_drift_fail_consecutive = 0;

// Red flash counter: when the drift check blocks a bias update, the
// lightbar turns red for this many samples to indicate the issue.
#define DRIFT_FAIL_FLASH_SAMPLES 60
static int g_drift_fail_flash = 0;

static GyroDebug g_debug;

static float g_float_stick_x = 0.0f;
static float g_float_stick_y = 0.0f;

// Dead-zone hysteresis state (task 6): track whether each axis is
// currently inside the dead zone so we can apply different enter/exit
// thresholds. Reset on calibration.
static bool g_vec_deadzone = false;  // hysteresis state for vector magnitude

// Default gain curve. Tuned down from the previous extremely aggressive
// first breakpoint (200→140) per task 5.
static const float kDefaultGainRates[] = { 0.02f, 0.05f, 0.15f, 0.40f, 1.00f, 100.0f };
static const float kDefaultGainValues[] = { 140.0f, 100.0f, 70.0f, 50.0f, 35.0f, 25.0f };
#define DEFAULT_GAIN_COUNT 6

static void set_default_gain(float* rates, float* values, int* count) {
    memcpy(rates, kDefaultGainRates, sizeof(kDefaultGainRates));
    memcpy(values, kDefaultGainValues, sizeof(kDefaultGainValues));
    *count = DEFAULT_GAIN_COUNT;
}

void gyro_profile_set_defaults(GyroProfile* profile) {
    profile->enabled = true;
    profile->dead_zone = 0.02f;
    profile->dead_zone_bias = 20;
    profile->dead_zone_bias_ramp_scale = 4.0f;
    profile->trigger_threshold = 250;
    set_default_gain(profile->gain_rates_h, profile->gain_values_h, &profile->gain_count_h);
    profile->lowpass_alpha = 0.5f;
    // Task 1: damping is now interpolation-based (fraction moved toward
    // zero per sample, not a multiplier). 0.12 ≈ the old *= 0.88 feel.
    profile->damping_factor = 0.12f;
    profile->saturation_strength = 2.0f;
    profile->invert_x = false;
    profile->invert_y = false;
    profile->yaw_from_z = false;
    profile->yaw_tilt_weight = 0.0f;
    profile->drift_correction_enabled = true;
    profile->bias_alpha = 0.01f;
    profile->bias_max_correction_step = 0.05f;
    profile->bias_stationary_samples = 60;
    profile->bias_delta_ema_alpha = 0.2f;
    profile->bias_stillness_delta_threshold = 0.01f;
    profile->bias_stillness_magnitude_threshold = 0.20f;
    profile->bias_settle_samples = 50;
    profile->bias_drift_accum_threshold = 0.01f;
    profile->bias_max_total_deviation = 0.10f;
    profile->sensitivity_h = 1.0f;
    profile->sensitivity_v = 1.0f;
    profile->flick_mag_threshold = 1.00f;
    profile->flick_suppression_samples = 12;
    profile->flick_suppression_deadzone_scale = 5.0f;
    profile->flick_suppression_gain_cap = 30.0f;
    profile->stick_drift_correction_enabled = true;
    profile->stick_center_alpha = 0.01f;
    profile->stick_center_max_correction_step = 0.5f;
    profile->stick_stationary_samples = 60;
    profile->stick_delta_ema_alpha = 0.2f;
    profile->stick_stillness_delta_threshold = 1.0f;
    profile->stick_max_plausible_drift = 30.0f;
    profile->stick_center_max_deviation = 40.0f;
}

static int parse_float_list(const char* str, float* out, int max_count) {
    int count = 0;
    if (str == NULL || *str == '\0') return 0;
    const char* p = str;
    while (*p && count < max_count) {
        while (*p == ' ') p++;
        if (*p == '\0') break;
        char* end;
        out[count] = strtof(p, &end);
        if (end == p) break;
        count++;
        p = end;
    }
    return count;
}

// Task 7: validate a gain curve after loading. Returns true if the curve
// is usable (non-zero count, ascending rates, no duplicates).
static bool gain_curve_valid(const float* rates, const float* values, int count) {
    if (count <= 0) return false;
    for (int i = 1; i < count; i++) {
        if (rates[i] <= rates[i - 1]) return false;  // not ascending (or duplicate)
    }
    return true;
}

static void load_gain_curve(ini_table_s* table, const char* section, const char* rates_key,
                             const char* values_key, float* rates, float* values, int* count) {
    const char* rates_str = ini_table_get_entry(table, section, rates_key);
    const char* values_str = ini_table_get_entry(table, section, values_key);
    if (rates_str == NULL || values_str == NULL) {
        return;
    }
    float tmp_rates[MAX_GAIN_POINTS];
    float tmp_values[MAX_GAIN_POINTS];
    int rc = parse_float_list(rates_str, tmp_rates, MAX_GAIN_POINTS);
    int vc = parse_float_list(values_str, tmp_values, MAX_GAIN_POINTS);
    if (rc <= 0 || rc != vc) return;  // mismatched or zero entries
    if (!gain_curve_valid(tmp_rates, tmp_values, rc)) return;  // rejected
    memcpy(rates, tmp_rates, rc * sizeof(float));
    memcpy(values, tmp_values, rc * sizeof(float));
    *count = rc;
}

static void load_section(ini_table_s* table, const char* section, GyroProfile* profile) {
    if (_ini_section_find(table, section) == NULL) {
        return;
    }
    ini_table_get_entry_as_bool(table, section, "Enabled", &profile->enabled);
    ini_table_get_entry_as_float(table, section, "DeadZone", &profile->dead_zone);
    ini_table_get_entry_as_int(table, section, "DeadZoneBias", &profile->dead_zone_bias);
    ini_table_get_entry_as_float(table, section, "DeadZoneBiasRampScale", &profile->dead_zone_bias_ramp_scale);
    ini_table_get_entry_as_int(table, section, "TriggerThreshold", &profile->trigger_threshold);

    load_gain_curve(table, section, "GainRates_H", "GainValues_H",
                     profile->gain_rates_h, profile->gain_values_h, &profile->gain_count_h);

    ini_table_get_entry_as_float(table, section, "LowPassAlpha", &profile->lowpass_alpha);
    ini_table_get_entry_as_float(table, section, "DampingFactor", &profile->damping_factor);
    ini_table_get_entry_as_float(table, section, "SaturationStrength", &profile->saturation_strength);

    ini_table_get_entry_as_bool(table, section, "InvertX", &profile->invert_x);
    ini_table_get_entry_as_bool(table, section, "InvertY", &profile->invert_y);
    ini_table_get_entry_as_bool(table, section, "YawFromZ", &profile->yaw_from_z);
    ini_table_get_entry_as_float(table, section, "YawTiltWeight", &profile->yaw_tilt_weight);
    ini_table_get_entry_as_bool(table, section, "DriftCorrectionEnabled", &profile->drift_correction_enabled);
    ini_table_get_entry_as_float(table, section, "BiasAlpha", &profile->bias_alpha);
    ini_table_get_entry_as_float(table, section, "BiasMaxCorrectionStep", &profile->bias_max_correction_step);
    ini_table_get_entry_as_int(table, section, "BiasStationarySamples", &profile->bias_stationary_samples);
    ini_table_get_entry_as_float(table, section, "BiasDeltaEmaAlpha", &profile->bias_delta_ema_alpha);
    ini_table_get_entry_as_float(table, section, "BiasStillnessDeltaThreshold", &profile->bias_stillness_delta_threshold);
    ini_table_get_entry_as_float(table, section, "BiasStillnessMagnitudeThreshold", &profile->bias_stillness_magnitude_threshold);
    ini_table_get_entry_as_int(table, section, "BiasSettleSamples", &profile->bias_settle_samples);
    ini_table_get_entry_as_float(table, section, "BiasDriftAccumThreshold", &profile->bias_drift_accum_threshold);
    ini_table_get_entry_as_float(table, section, "BiasMaxTotalDeviation", &profile->bias_max_total_deviation);
    ini_table_get_entry_as_float(table, section, "SensitivityH", &profile->sensitivity_h);
    ini_table_get_entry_as_float(table, section, "SensitivityV", &profile->sensitivity_v);
    ini_table_get_entry_as_float(table, section, "FlickMagThreshold", &profile->flick_mag_threshold);
    ini_table_get_entry_as_int(table, section, "FlickSuppressionSamples", &profile->flick_suppression_samples);
    ini_table_get_entry_as_float(table, section, "FlickSuppressionDeadzoneScale", &profile->flick_suppression_deadzone_scale);
    ini_table_get_entry_as_float(table, section, "FlickSuppressionGainCap", &profile->flick_suppression_gain_cap);
    ini_table_get_entry_as_bool(table, section, "StickDriftCorrectionEnabled", &profile->stick_drift_correction_enabled);
    ini_table_get_entry_as_float(table, section, "StickCenterAlpha", &profile->stick_center_alpha);
    ini_table_get_entry_as_float(table, section, "StickCenterMaxCorrectionStep", &profile->stick_center_max_correction_step);
    ini_table_get_entry_as_int(table, section, "StickStationarySamples", &profile->stick_stationary_samples);
    ini_table_get_entry_as_float(table, section, "StickDeltaEmaAlpha", &profile->stick_delta_ema_alpha);
    ini_table_get_entry_as_float(table, section, "StickStillnessDeltaThreshold", &profile->stick_stillness_delta_threshold);
    ini_table_get_entry_as_float(table, section, "StickMaxPlausibleDrift", &profile->stick_max_plausible_drift);
    ini_table_get_entry_as_float(table, section, "StickCenterMaxDeviation", &profile->stick_center_max_deviation);
}

bool gyro_profile_load(const char* ini_path, const char* title_id, GyroProfile* profile) {
    ini_table_s* table = ini_table_create();
    if (table == NULL) {
        return false;
    }
    if (!ini_table_read_from_file(table, ini_path)) {
        ini_table_destroy(table);
        return false;
    }

    load_section(table, "default", profile);
    if (title_id != NULL && title_id[0] != '\0') {
        load_section(table, title_id, profile);
    }

    ini_table_destroy(table);
    return true;
}

void gyro_state_init(const GyroProfile* profile) {
    g_profile = *profile;
    g_calibrated = false;
    g_calib_count = 0;
    memset(g_calib_sum, 0, sizeof(g_calib_sum));
    memset(g_bias, 0, sizeof(g_bias));
    memset(g_calibrated_bias, 0, sizeof(g_calibrated_bias));
    g_is_stationary = false;
    g_stationary_timer = 0;
    g_prev_gyro_valid = false;
    memset(g_delta_ema, 0, sizeof(g_delta_ema));
    g_runtime_enabled = true;
    g_prev_l3r3_held = false;
    g_touchpad_hold_count = 0;
    g_recal_armed = true;
    g_lightbar_state = LB_UNSET;
    g_float_stick_x = 0.0f;
    g_float_stick_y = 0.0f;
    g_vec_deadzone = false;
    g_flick_suppress_timer = 0;
    g_flick_dir_yaw = 0.0f;
    g_flick_dir_pitch = 0.0f;
    g_settle_timer = 0;
    memset(g_drift_acc, 0, sizeof(g_drift_acc));
    g_was_aiming = false;
    g_drift_fail_consecutive = 0;
    g_drift_fail_flash = 0;
    // Stick-center drift tracking is intentionally reset ONLY here, not
    // in start_recalibration(). The touchpad-hold hotkey recalibrates
    // gyro RATE bias -- unrelated to the physical stick's learned rest
    // position. Resetting a good stick-center estimate on every manual
    // gyro recalibration would force it to reconverge from nominal 128
    // each time, reintroducing the phantom-push asymmetry for the
    // duration of that reconvergence for no gyro-related reason.
    g_stick_center[0] = (float)STICK_CENTER;
    g_stick_center[1] = (float)STICK_CENTER;
    g_stick_prev_valid = false;
    memset(g_stick_delta_ema, 0, sizeof(g_stick_delta_ema));
    g_stick_stationary_timer = 0;
}

void gyro_set_profile(const GyroProfile* profile) {
    g_profile = *profile;
}

GyroProfile gyro_get_profile(void) {
    return g_profile;
}

GyroDebug gyro_get_debug(void) {
    return g_debug;
}

static void start_recalibration(void) {
    g_calibrated = false;
    g_calib_count = 0;
    memset(g_calib_sum, 0, sizeof(g_calib_sum));
    memset(g_bias, 0, sizeof(g_bias));
    memset(g_calibrated_bias, 0, sizeof(g_calibrated_bias));
    g_is_stationary = false;
    g_stationary_timer = 0;
    g_prev_gyro_valid = false;
    memset(g_delta_ema, 0, sizeof(g_delta_ema));
    g_float_stick_x = 0.0f;
    g_float_stick_y = 0.0f;
    g_vec_deadzone = false;
    g_flick_suppress_timer = 0;
    g_flick_dir_yaw = 0.0f;
    g_flick_dir_pitch = 0.0f;
    g_settle_timer = 0;
    memset(g_drift_acc, 0, sizeof(g_drift_acc));
    g_was_aiming = false;
    g_drift_fail_consecutive = 0;
    g_drift_fail_flash = 0;
}

static void set_lightbar(int32_t handle, LightbarState state) {
    if (state == g_lightbar_state) {
        return;
    }
    g_lightbar_state = state;

    switch (state) {
        case LB_CALIBRATING:
            platform_set_lightbar(handle, 255, 200, 0);   // amber
            break;
        case LB_ACTIVE:
            platform_set_lightbar(handle, 0, 255, 0);      // green
            break;
        case LB_INACTIVE:
        default:
            platform_set_lightbar(handle, 0, 60, 255);     // blue
            break;
    }
}

static int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}



// Vector deadzone with hysteresis (magnitude-based, not per-axis).
// All directions deadzone consistently at the same rotational speed.
// Returns true if the vector should be treated as zero.
static bool vec_deadzone_with_hysteresis(float mag, float dead_zone, bool* alive) {
    if (*alive) {
        if (mag < dead_zone * DEADZONE_ENTER_FRAC) {
            *alive = false;
            return true;
        }
        return false;
    } else {
        if (mag < dead_zone * DEADZONE_EXIT_FRAC) return true;
        *alive = true;
        return false;
    }
}

static float gain_lookup(const float* rates, const float* values, int count, float abs_rate) {
    if (count <= 0) return 0.0f;
    if (abs_rate <= rates[0]) return values[0];
    for (int i = 1; i < count; i++) {
        if (abs_rate <= rates[i]) {
            float t = (abs_rate - rates[i - 1]) / (rates[i] - rates[i - 1]);
            return values[i - 1] + t * (values[i] - values[i - 1]);
        }
    }
    return values[count - 1];
}

// Vector-based response pipeline. Computes stick_x/stick_y from yaw/pitch
// treating them as a 2D rotation vector. Preserves movement direction
// through the entire pipeline: the normalized vector (norm_x, norm_y)
// captures the user's intended aim direction, and only the MAGNITUDE is
// scaled/clamped/saturated.
static void process_vector(float yaw, float pitch,
                            float* float_stick_x, float* float_stick_y,
                            const float* gain_rates, const float* gain_values, int gain_count,
                            float alpha, float damping,
                            float dead_zone, float saturation_strength, int dead_zone_bias) {
    float mag = sqrtf(yaw * yaw + pitch * pitch);

    // Post-flick suppression state machine. A cooldown is armed whenever
    // a sample's magnitude crosses flick_mag_threshold, and every sample
    // during the cooldown gets a widened deadzone + capped gain UNLESS
    // it's classified as a fresh, deliberate flick (see is_flick_sample
    // below).
    //
    // is_reversal (fix for item 1): a fast deceleration snap-back is
    // real physical motion and can easily cross flick_mag_threshold
    // itself -- the same speed the gain curve treats as "a flick". Using
    // magnitude alone to decide "is this a new flick" meant the rebound
    // was misclassified as a brand-new legitimate flick, passed straight
    // through unsuppressed, and re-armed the cooldown pointed the wrong
    // way. Direction fixes this: a sample during an active cooldown that
    // is pointed opposite the flick that's cooling down (dot product
    // below FLICK_REVERSAL_DOT) is a reversal regardless of how fast it
    // is, and stays suppressed rather than bypassing as a new flick.
    bool cooling_down = (g_flick_suppress_timer > 0);
    bool is_reversal = false;
    if (cooling_down && mag > 0.0f && (g_flick_dir_yaw != 0.0f || g_flick_dir_pitch != 0.0f)) {
        float norm_x = yaw / mag;
        float norm_y = pitch / mag;
        float dot = g_flick_dir_yaw * norm_x + g_flick_dir_pitch * norm_y;
        is_reversal = (dot < FLICK_REVERSAL_DOT);
    }

    // A sample only counts as "the flick itself" (bypasses suppression
    // entirely and re-arms the cooldown in its own direction) if its
    // magnitude clears the threshold AND it isn't a reversal of the
    // flick currently on cooldown. A genuine new flick in a *different*
    // (non-reversed) direction during the cooldown still re-arms as
    // before.
    bool is_flick_sample = (mag >= g_profile.flick_mag_threshold) && !is_reversal;

    if (is_flick_sample) {
        g_flick_suppress_timer = g_profile.flick_suppression_samples;
        if (mag > 0.0f) {
            g_flick_dir_yaw = yaw / mag;
            g_flick_dir_pitch = pitch / mag;
        } else {
            g_flick_dir_yaw = 0.0f;
            g_flick_dir_pitch = 0.0f;
        }
    } else if (cooling_down) {
        g_flick_suppress_timer--;
    }

    // Fix for item 2 (off-by-one): use the pre-decrement `cooling_down`
    // snapshot rather than re-reading g_flick_suppress_timer after the
    // decrement above. The old code checked the timer post-decrement, so
    // the sample that entered with timer == 1 decremented to 0 and was
    // never suppressed -- flick_suppression_samples=12 only ever
    // suppressed 11 samples. Checking the pre-decrement state suppresses
    // exactly the configured number of samples.
    bool suppress_this_sample = (!is_flick_sample) && cooling_down;
    g_debug.flick_suppress = suppress_this_sample ? 1 : 0;

    float effective_dead_zone = suppress_this_sample
                                     ? dead_zone * g_profile.flick_suppression_deadzone_scale
                                     : dead_zone;

    // Vector deadzone with hysteresis on magnitude
    if (vec_deadzone_with_hysteresis(mag, effective_dead_zone, &g_vec_deadzone)) {
        mag = 0.0f;
    }

    if (mag > 0.0f) {
        // Normalize the direction vector
        float norm_x = yaw / mag;
        float norm_y = pitch / mag;

        // Gain lookup on vector magnitude
        float gain = gain_lookup(gain_rates, gain_values, gain_count, mag);
        if (suppress_this_sample && gain > g_profile.flick_suppression_gain_cap) {
            gain = g_profile.flick_suppression_gain_cap;
        }
        float output_mag = gain * mag;

        float output_x = norm_x * output_mag;
        float output_y = norm_y * output_mag;

        // Saturate per-axis, not on the combined vector length, so diagonal
        // input can still reach full deflection on each axis independently.
        float raw_x = tanhf((output_x / 128.0f) * saturation_strength) * 128.0f;
        float raw_y = tanhf((output_y / 128.0f) * saturation_strength) * 128.0f;

        // DeadZoneBias: apply the floor to the *post-saturation* vector
        // length, not the pre-saturation gain output. Previously the
        // floor was set on output_mag before the
        // tanh curve, so a configured floor of 20 came out the other side
        // as ~71 (tanhf((20/128)*2)*128) — a 3.5x amplification that
        // collapsed most of the usable range into a narrow top band.
        // Scaling (raw_x, raw_y) as a vector (rather than flooring each
        // axis independently) preserves direction and avoids injecting
        // phantom movement into a near-zero secondary axis.
        //
        // Ramped, not snapped: the floor used to be a snap-to-constant —
        // ANY sample whose natural output fell between 0 and
        // dead_zone_bias got scaled straight up to exactly
        // dead_zone_bias. Since the gain curve's lowest breakpoint sits
        // right at/near dead_zone, gain is already close to max the
        // instant a sample clears the dead zone, so nearly every
        // post-deadzone sample landed in that range — meaning the
        // dead-zone boundary was a binary gate (0 on one side, a fixed
        // ~10-20 unit jump on the other) rather than a true floor. A
        // bias residual as small as 0.001-0.002 rad/s is a meaningful
        // fraction of the dead zone's own width and is enough to
        // consistently decide which side of that gate a borderline
        // sample lands on -- which reads in-game as one direction being
        // "easy" and the other "dead", even though the underlying bias
        // error is tiny. Ramping the floor from 0 at the dead-zone
        // boundary up to dead_zone_bias over
        // [effective_dead_zone, effective_dead_zone * dead_zone_bias_ramp_scale]
        // means a tiny shift in exactly where the boundary sits only
        // shifts where the ramp *starts*, not the size of a jump.
        float sat_vec_mag = sqrtf(raw_x * raw_x + raw_y * raw_y);
        if (dead_zone_bias > 0 && sat_vec_mag > 0.0f) {
            float ramp_top = effective_dead_zone * g_profile.dead_zone_bias_ramp_scale;
            float target_floor;
            if (ramp_top > effective_dead_zone) {
                float t = (mag - effective_dead_zone) / (ramp_top - effective_dead_zone);
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;
                target_floor = (float)dead_zone_bias * t;
            } else {
                // Degenerate ramp width (scale <= 1.0) -- fall back to
                // the old snap behavior rather than divide by ~0.
                target_floor = (float)dead_zone_bias;
            }
            if (sat_vec_mag < target_floor) {
                float scale = target_floor / sat_vec_mag;
                raw_x *= scale;
                raw_y *= scale;
            }
        }

        // Per-axis sensitivity scaling (applied after all vector
        // processing — independent of deadzone/gain/saturation)
        raw_x *= g_profile.sensitivity_h;
        raw_y *= g_profile.sensitivity_v;

        // Per-axis output smoothing (EMA)
        if (alpha < 1.0f) {
            *float_stick_x += alpha * (raw_x - *float_stick_x);
            *float_stick_y += alpha * (raw_y - *float_stick_y);
        } else {
            *float_stick_x = raw_x;
            *float_stick_y = raw_y;
        }
    } else {
        // Damping: interpolation toward zero (per-axis, as before)
        *float_stick_x += (0.0f - *float_stick_x) * damping;
        *float_stick_y += (0.0f - *float_stick_y) * damping;
        // Snap to zero once decay is negligible
        if (fabsf(*float_stick_x) < 0.01f) *float_stick_x = 0.0f;
        if (fabsf(*float_stick_y) < 0.01f) *float_stick_y = 0.0f;
    }
}

// Write a float stick value to uint8, summing with the physical stick
// position. DeadZoneBias is already handled upstream in process_vector()
// — this is just the final conversion + clamp. `center` is the learned
// rest position for this axis (see the stick drift compensation block
// in gyro_process_sample) rather than the nominal STICK_CENTER constant
// -- physical deflection is measured relative to where the stick
// actually rests, so a drifted pot doesn't read as a constant phantom
// input. The OUTPUT is still re-biased around the nominal STICK_CENTER,
// since that's what the game itself expects as center.
static void write_stick_uint8(uint8_t* axis, float stick, float center) {
    int phys = (int)(*axis) - (int)lroundf(center);
    int combined = clamp_int(phys + lroundf(stick), -128, 127);
    *axis = (uint8_t)clamp_int(combined + STICK_CENTER, STICK_MIN, STICK_MAX);
}

void gyro_process_sample(int32_t handle, ScePadData* pData) {
    if (!pData->connected) {
        return;
    }

    // --- Hotkeys -------------------------------------------------------
    bool l3r3_held = (pData->buttons & SCE_PAD_BUTTON_L3) && (pData->buttons & SCE_PAD_BUTTON_R3);
    if (l3r3_held && !g_prev_l3r3_held) {
        g_runtime_enabled = !g_runtime_enabled;
    }
    g_prev_l3r3_held = l3r3_held;

    bool touchpad_held = (pData->buttons & SCE_PAD_BUTTON_TOUCH_PAD) != 0;
    if (touchpad_held) {
        g_touchpad_hold_count++;
        if (g_recal_armed && g_touchpad_hold_count >= RECALIBRATE_HOLD_SAMPLES) {
            start_recalibration();
            g_recal_armed = false;
        }
    } else {
        g_touchpad_hold_count = 0;
        g_recal_armed = true;
    }

    if (!g_profile.enabled || !g_runtime_enabled) {
        set_lightbar(handle, LB_INACTIVE);
        g_float_stick_x = 0.0f;
        g_float_stick_y = 0.0f;
        return;
    }

    float gx = pData->angularVelocity.x;
    float gy = pData->angularVelocity.y;
    float gz = pData->angularVelocity.z;
    if (isnan(gx) || isinf(gx)) gx = 0.0f;
    if (isnan(gy) || isinf(gy)) gy = 0.0f;
    if (isnan(gz) || isinf(gz)) gz = 0.0f;

    g_debug.gyro[0] = gx;
    g_debug.gyro[1] = gy;
    g_debug.gyro[2] = gz;
    g_debug.bias[0] = g_bias[0];
    g_debug.bias[1] = g_bias[1];
    g_debug.bias[2] = g_bias[2];

    // --- Calibration -----------------------------------------------------
    // Accumulates every sample unconditionally — no stationary check.
    // The "reject moving samples" approach (task 2 of the refinement
    // plan) introduced two independent failure points (accel data being
    // zero on some PS4 firmware variants, and a gyro threshold that must
    // clear the DS4's natural resting bias of ~0.05 rad/s on the Z axis)
    // that together prevented calibration from ever completing on real
    // hardware. The original unconditional approach is simpler, known to
    // work, and produces an accurate bias over CALIB_SAMPLE_COUNT
    // samples even if a few are collected while moving.
    if (!g_calibrated) {
        set_lightbar(handle, LB_CALIBRATING);
        g_calib_sum[0] += gx;
        g_calib_sum[1] += gy;
        g_calib_sum[2] += gz;
        g_calib_count++;
        if (g_calib_count >= CALIB_SAMPLE_COUNT) {
            g_bias[0] = g_calib_sum[0] / (float)g_calib_count;
            g_bias[1] = g_calib_sum[1] / (float)g_calib_count;
            g_bias[2] = g_calib_sum[2] / (float)g_calib_count;
            g_calibrated_bias[0] = g_bias[0];
            g_calibrated_bias[1] = g_bias[1];
            g_calibrated_bias[2] = g_bias[2];
            g_calibrated = true;
            log_info("gyro calibration complete: bias=[%f %f %f]\n",
                     g_bias[0], g_bias[1], g_bias[2]);
        }
        return;
    }

    // --- Trigger gate ------------------------------------------------------
    bool aiming = pData->analogButtons.l2 >= g_profile.trigger_threshold;
    set_lightbar(handle, aiming ? LB_ACTIVE : LB_INACTIVE);
    if (!aiming) {
        // --- Post-aiming settle delay ---
        // Arm on the transition from aiming to not-aiming. During the
        // delay, skip ALL bias estimation — the moments immediately
        // after releasing L2 are likely to contain residual hand
        // settling that looks stationary but isn't.
        if (g_was_aiming) {
            g_settle_timer = g_profile.bias_settle_samples;
            memset(g_drift_acc, 0, sizeof(g_drift_acc));
        }
        g_was_aiming = false;
        if (g_settle_timer > 0) {
            g_settle_timer--;
        }

        // Bias estimation: only runs when NOT aiming AND the settle
        // delay has elapsed. Stillness is judged globally across all
        // three axes (both the flat/low-jerk gate and the near-zero/
        // magnitude gate must pass on every axis before ANY axis's
        // bias is updated), then each axis is updated independently.
        // An additional windowed-drift check (accumulated bias-corrected
        // error over the stationary window) rejects sustained slow
        // motion that the instantaneous gates can't distinguish from
        // sensor noise — see the drift accumulator comment below.
        g_debug.stillness = 0;
        g_debug.bias_active = 0;

        if (g_profile.drift_correction_enabled && g_settle_timer == 0) {
            float raw[3] = { gx, gy, gz };

            // Stillness requires two independent signals to agree:
            //  1. Flat (low jerk): the delta check below. Catches
            //     abrupt/jittery motion but, on its own, misses smooth
            //     constant-rate rotation -- e.g. gently settling onto a
            //     target has almost no sample-to-sample delta even
            //     while genuinely moving.
            //  2. Near zero (low magnitude): compared against a fixed
            //     physical bound (bias_stillness_magnitude_threshold),
            //     NOT the current bias estimate -- catches smooth
            //     motion the delta check alone misses, without
            //     reintroducing the old deadlock where a bad bias
            //     estimate could never be corrected because "close to
            //     the current bias" was the precondition for moving it.
            // Both gates must pass. Missing real stillness only costs
            // convergence speed; treating real motion as stillness
            // corrupts the estimate -- this deliberately errs toward
            // the former.
            bool is_flat = g_prev_gyro_valid;
            if (g_prev_gyro_valid) {
                for (int i = 0; i < 3; i++) {
                    float delta = fabsf(raw[i] - g_prev_gyro[i]);
                    g_delta_ema[i] += g_profile.bias_delta_ema_alpha * (delta - g_delta_ema[i]);
                    if (g_delta_ema[i] >= g_profile.bias_stillness_delta_threshold) {
                        is_flat = false;
                    }
                }
            }
            g_prev_gyro[0] = gx;
            g_prev_gyro[1] = gy;
            g_prev_gyro[2] = gz;
            g_prev_gyro_valid = true;

            bool is_near_zero = true;
            for (int i = 0; i < 3; i++) {
                if (fabsf(raw[i]) >= g_profile.bias_stillness_magnitude_threshold) {
                    is_near_zero = false;
                    break;
                }
            }

            bool all_flat = is_flat && is_near_zero;

            g_debug.stillness = all_flat ? 1 : 0;

            if (all_flat) {
                // Accumulate bias-corrected error while the controller
                // appears stationary. Sensor noise has roughly zero mean
                // (random signs → sqrt(N) growth), while sustained slow
                // motion has a consistent sign per axis (→ N growth).
                // This windowed-accumulator check catches slow drift
                // that the instantaneous flat/zero gates alone can't
                // distinguish from sensor noise. The accumulator resets
                // whenever stillness breaks or a window is consumed.
                g_drift_acc[0] += raw[0] - g_bias[0];
                g_drift_acc[1] += raw[1] - g_bias[1];
                g_drift_acc[2] += raw[2] - g_bias[2];

                if (!g_is_stationary) {
                    g_is_stationary = true;
                    g_stationary_timer = 0;
                }
                g_stationary_timer++;
                g_debug.bias_active = g_stationary_timer;
                if (g_stationary_timer >= g_profile.bias_stationary_samples) {
                    // Windowed drift check: normalize accumulated error
                    // by window size so the threshold is independent of
                    // BiasStationarySamples. Noise cancels out over time
                    // (zero-mean → low average); sustained drift does
                    // not (consistent sign → high average).
                    bool drift_ok = true;
                    for (int i = 0; i < 3; i++) {
                        float avg_error = g_drift_acc[i] / (float)g_stationary_timer;
                        if (fabsf(avg_error) >= g_profile.bias_drift_accum_threshold) {
                            drift_ok = false;
                            break;
                        }
                    }
                    if (drift_ok) {
                        g_drift_fail_consecutive = 0;
                        float errors[3];
                        for (int i = 0; i < 3; i++) {
                            errors[i] = raw[i] - g_bias[i];
                            float step = g_profile.bias_alpha * errors[i];
                            if (step > g_profile.bias_max_correction_step) step = g_profile.bias_max_correction_step;
                            if (step < -g_profile.bias_max_correction_step) step = -g_profile.bias_max_correction_step;
                            g_bias[i] += step;
                            // bias_max_correction_step above only bounds
                            // the size of a SINGLE step -- it does not
                            // stop many small steps in the same
                            // direction (e.g. a habitual resting-grip
                            // tilt that repeatedly passes the stillness
                            // gates) from walking g_bias arbitrarily far
                            // from where it was actually calibrated.
                            // Clamp cumulative drift to a fixed radius
                            // around the calibrated anchor so runtime
                            // correction can still track real slow
                            // sensor drift without being able to eat
                            // into one whole direction of aim.
                            float lo = g_calibrated_bias[i] - g_profile.bias_max_total_deviation;
                            float hi = g_calibrated_bias[i] + g_profile.bias_max_total_deviation;
                            if (g_bias[i] < lo) g_bias[i] = lo;
                            if (g_bias[i] > hi) g_bias[i] = hi;
                        }
                        g_bias_log_counter++;
                        if (g_bias_log_counter >= BIAS_LOG_INTERVAL) {
                            g_bias_log_counter = 0;
                            float dyaw  = (g_profile.yaw_from_z ? gz : gy) - g_bias[g_profile.yaw_from_z ? 2 : 1];
                            float dpitch = gx - g_bias[0];
                            float dmag = sqrtf(dyaw * dyaw + dpitch * dpitch);
                            log_info("bias: X raw=%+.4f bias=%+.4f err=%+.4f | Y raw=%+.4f bias=%+.4f err=%+.4f | Z raw=%+.4f bias=%+.4f err=%+.4f | yaw=%+.4f pitch=%+.4f mag=%.4f\n",
                                     gx, g_bias[0], errors[0],
                                     gy, g_bias[1], errors[1],
                                     gz, g_bias[2], errors[2],
                                     dyaw, dpitch, dmag);
                        }
                    } else {
                        g_debug.bias_active = 0;
                        g_drift_fail_consecutive++;
                        g_drift_fail_flash = DRIFT_FAIL_FLASH_SAMPLES;

                        static int drift_fail_counter = 0;
                        drift_fail_counter++;
                        if (drift_fail_counter >= BIAS_LOG_INTERVAL) {
                            drift_fail_counter = 0;
                            float avg[3];
                            avg[0] = g_drift_acc[0] / (float)g_stationary_timer;
                            avg[1] = g_drift_acc[1] / (float)g_stationary_timer;
                            avg[2] = g_drift_acc[2] / (float)g_stationary_timer;
                            log_info("bias: drift check FAILED (avg error X=%+.4f Y=%+.4f Z=%+.4f, threshold=%.4f, consecutive=%d)\n",
                                     avg[0], avg[1], avg[2], g_profile.bias_drift_accum_threshold,
                                     g_drift_fail_consecutive);
                        }

                        float fail_alpha = g_profile.bias_alpha;
                        if (g_drift_fail_consecutive < 3) {
                            fail_alpha *= 0.2f;
                        }
                        for (int i = 0; i < 3; i++) {
                            float step = fail_alpha * (raw[i] - g_bias[i]);
                            if (step > g_profile.bias_max_correction_step)
                                step = g_profile.bias_max_correction_step;
                            if (step < -g_profile.bias_max_correction_step)
                                step = -g_profile.bias_max_correction_step;
                            g_bias[i] += step;
                        }
                    }
                    // Consume the stationary window — reset both the
                    // timer and the accumulator so the next bias update
                    // requires a full fresh window, not just one more
                    // sample.
                    g_stationary_timer = 0;
                    memset(g_drift_acc, 0, sizeof(g_drift_acc));
                }
            } else {
                g_is_stationary = false;
                g_stationary_timer = 0;
                g_debug.bias_active = 0;
                // Stillness broken — reset drift accumulator. Any
                // accumulated error from a partially-still interval
                // is stale the moment the controller moves again.
                memset(g_drift_acc, 0, sizeof(g_drift_acc));
            }
        }

        // --- Physical stick drift compensation ---
        // Independent of the gyro bias block above: different sensor,
        // different failure mode, no reason to share gating state with
        // it. Runs whenever not aiming, using the raw hardware stick
        // reading (pData->rightStick is guaranteed untouched here --
        // write_stick_uint8 is only ever called from the aiming path
        // below, after this block has already returned).
        if (g_profile.stick_drift_correction_enabled) {
            float raw_stick[2] = { (float)pData->rightStick.x, (float)pData->rightStick.y };

            // Flatness gate: same idea as the gyro delta check -- a
            // stick sitting still (whether truly at rest or resting at
            // its drifted position) reads near-constant sample to
            // sample. A stick actively being pushed toward a new
            // position does not.
            bool stick_flat = g_stick_prev_valid;
            if (g_stick_prev_valid) {
                for (int i = 0; i < 2; i++) {
                    float delta = fabsf(raw_stick[i] - g_stick_prev[i]);
                    g_stick_delta_ema[i] += g_profile.stick_delta_ema_alpha * (delta - g_stick_delta_ema[i]);
                    if (g_stick_delta_ema[i] >= g_profile.stick_stillness_delta_threshold) {
                        stick_flat = false;
                    }
                }
            }
            g_stick_prev[0] = raw_stick[0];
            g_stick_prev[1] = raw_stick[1];
            g_stick_prev_valid = true;

            // Plausible-drift gate: measured from the fixed nominal
            // center (128), NOT g_stick_center -- see the GyroProfile
            // struct comment for why using the current estimate here
            // would risk the same deadlock the gyro bias estimator once
            // had. A deliberate held deflection is typically far larger
            // than any real pot drift and is naturally excluded by this
            // bound without needing to be detected directly.
            bool stick_plausible = true;
            for (int i = 0; i < 2; i++) {
                if (fabsf(raw_stick[i] - (float)STICK_CENTER) > g_profile.stick_max_plausible_drift) {
                    stick_plausible = false;
                    break;
                }
            }

            if (stick_flat && stick_plausible) {
                g_stick_stationary_timer++;
                if (g_stick_stationary_timer >= g_profile.stick_stationary_samples) {
                    for (int i = 0; i < 2; i++) {
                        float error = raw_stick[i] - g_stick_center[i];
                        float step = g_profile.stick_center_alpha * error;
                        if (step > g_profile.stick_center_max_correction_step) step = g_profile.stick_center_max_correction_step;
                        if (step < -g_profile.stick_center_max_correction_step) step = -g_profile.stick_center_max_correction_step;
                        g_stick_center[i] += step;
                        // Bound total deviation from the nominal center,
                        // same safety role as bias_max_total_deviation.
                        float lo = (float)STICK_CENTER - g_profile.stick_center_max_deviation;
                        float hi = (float)STICK_CENTER + g_profile.stick_center_max_deviation;
                        if (g_stick_center[i] < lo) g_stick_center[i] = lo;
                        if (g_stick_center[i] > hi) g_stick_center[i] = hi;
                    }
                    // Consume the window, same pattern as the gyro bias
                    // block -- next update needs a fresh full window.
                    g_stick_stationary_timer = 0;
                }
            } else {
                g_stick_stationary_timer = 0;
            }
        }

        // Damped L2 release: decay float_stick toward zero instead of
        // snapping instantly. Feathering L2 then feels smooth — the
        // stick picks up roughly where it left off rather than
        // restarting from zero. After L2_RELEASE_RESET_SAMPLES of
        // continuous release (~2 seconds), hard-reset to zero to prevent
        // stale values from accumulating indefinitely.
        g_float_stick_x += (0.0f - g_float_stick_x) * g_profile.damping_factor;
        g_float_stick_y += (0.0f - g_float_stick_y) * g_profile.damping_factor;
        g_l2_release_samples++;
        if (g_l2_release_samples >= L2_RELEASE_RESET_SAMPLES) {
            g_float_stick_x = 0.0f;
            g_float_stick_y = 0.0f;
            g_l2_release_samples = 0;
        }
        // A flick's cooldown is only meaningful within the aiming session
        // that produced it — clear it immediately on release rather than
        // letting it carry into whatever happens next time L2 is pressed.
        g_flick_suppress_timer = 0;
        g_flick_dir_yaw = 0.0f;
        g_flick_dir_pitch = 0.0f;

        if (g_drift_fail_flash > 0) {
            g_drift_fail_flash--;
            platform_set_lightbar(handle, 255, 30, 30);
            g_lightbar_state = LB_UNSET;
        }
        return;
    }

    // --- Response model: vector-based (gain curve -> saturation -> DeadZoneBias -> EMA) ---
    // L2 is pressed — update edge detector and reset the release counter
    // so the next release-to-damping cycle starts fresh.
    g_was_aiming = true;
    g_l2_release_samples = 0;

    // The yaw/pitch pair is treated as a 2D rotation vector: the magnitude
    // determines speed, the normalized vector preserves the user's intended
    // direction. Diagonal movement now feels identical to pure
    // horizontal/vertical at the same rotational speed.
    float yaw_primary = (g_profile.yaw_from_z ? gz : gy) - g_bias[g_profile.yaw_from_z ? 2 : 1];
    float yaw_secondary = (g_profile.yaw_from_z ? gy : gz) - g_bias[g_profile.yaw_from_z ? 1 : 2];
    float yaw = yaw_primary + yaw_secondary * g_profile.yaw_tilt_weight;
    float pitch = gx - g_bias[0];

    if (g_profile.invert_x) yaw = -yaw;
    if (g_profile.invert_y) pitch = -pitch;

    g_debug.yaw = yaw;
    g_debug.pitch = pitch;

    process_vector(yaw, pitch,
                   &g_float_stick_x, &g_float_stick_y,
                   g_profile.gain_rates_h, g_profile.gain_values_h,
                   g_profile.gain_count_h,
                   g_profile.lowpass_alpha, g_profile.damping_factor,
                   g_profile.dead_zone, g_profile.saturation_strength, g_profile.dead_zone_bias);

    write_stick_uint8(&pData->rightStick.x, g_float_stick_x, g_stick_center[0]);
    write_stick_uint8(&pData->rightStick.y, g_float_stick_y, g_stick_center[1]);
}