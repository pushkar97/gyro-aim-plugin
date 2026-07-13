// Gyro-to-stick response model, calibration/drift correction, and
// hotkey/state handling for the gyro-aim plugin.
//
// Response model pipeline (per axis, H=yaw/horizontal, V=pitch/vertical):
//
//   gyro rate (bias-corrected, dead-zoned, inverted)
//     -> gain curve   [stick_raw = rate * gain(|rate|), per-axis, linearly
//                      interpolated lookup table]
//     -> EMA or damping, mutually exclusive [float_stick persists across
//                      samples: EMA blends toward stick_raw while actively
//                      moving; damping decays float_stick toward 0 while
//                      the rate is inside the dead zone. They never run in
//                      the same sample, so their effects never compound.]
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
//   dedicated extra clamp before the final float-to-int cast unnecessary
//   (an earlier, now-superseded response model's exponential curve could
//   produce huge/non-finite intermediate values from a sensor glitch and
//   needed such a clamp; the gain curve here is bounded by construction --
//   gain() returns at most the largest configured breakpoint value, so
//   stick_raw stays proportional to the (already-finite) rate).
#include "gyro.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gyro_common.h"
#include "config.h"
#include "platform.h"

#define CALIB_SAMPLE_COUNT 500        // ~1-2s at typical pad poll rates
#define STATIONARY_SAMPLE_COUNT 250   // consecutive still samples before drift EMA kicks in
#define DRIFT_ALPHA 0.01f
#define STATIONARY_ACCEL_TOLERANCE 0.5f  // m/s^2 around 9.80665 counted as "still"
// Squared bounds for the stationary check below, avoiding a sqrtf() call
// on every sample (this block runs unconditionally once calibrated,
// regardless of whether the player is aiming, making it the single most
// frequently-executed transcendental-function call site in this file
// before this change). sqrt is monotonic for non-negative inputs, so
// "sqrt(x) in [lo, hi]" <=> "x in [lo^2, hi^2]" for lo, hi >= 0 -- these
// are compile-time constants (simple literal arithmetic, folded
// regardless of optimization level), so this is a pure win with no
// change in behavior.
#define STATIONARY_ACCEL_LOW_SQ \
    ((9.80665f - STATIONARY_ACCEL_TOLERANCE) * (9.80665f - STATIONARY_ACCEL_TOLERANCE))
#define STATIONARY_ACCEL_HIGH_SQ \
    ((9.80665f + STATIONARY_ACCEL_TOLERANCE) * (9.80665f + STATIONARY_ACCEL_TOLERANCE))
#define STICK_CENTER 128
#define STICK_MIN 0
#define STICK_MAX 255

// L3+R3 toggle and touchpad-hold recalibrate hotkeys.
#define RECALIBRATE_HOLD_SAMPLES 150  // consecutive touchpad-held samples to trigger

typedef enum LightbarState { LB_UNSET = -1, LB_INACTIVE, LB_ACTIVE, LB_CALIBRATING } LightbarState;

static GyroProfile g_profile;

static bool g_calibrated = false;
static int g_calib_count = 0;
static float g_calib_sum[3] = { 0, 0, 0 };
static float g_bias[3] = { 0, 0, 0 };

static bool g_is_stationary = false;
static int g_stationary_count = 0;

static bool g_runtime_enabled = true;   // toggled by L3+R3
static bool g_prev_l3r3_held = false;

static int g_touchpad_hold_count = 0;
static bool g_recal_armed = true;  // prevents re-trigger until touchpad released

static LightbarState g_lightbar_state = LB_UNSET;

// Per-axis floating-point stick state, persisted across samples for the
// EMA/damping stages. Reset to 0 whenever aiming stops (see
// gyro_process_sample) or on calibration/recalibration.
static float g_float_stick_x = 0.0f;
static float g_float_stick_y = 0.0f;

static const float kDefaultGainRates[] = { 0.05f, 0.15f, 0.40f, 1.00f, 100.0f };
static const float kDefaultGainValues[] = { 90.0f, 70.0f, 50.0f, 35.0f, 25.0f };
#define DEFAULT_GAIN_COUNT 5

static void set_default_gain(float* rates, float* values, int* count) {
    memcpy(rates, kDefaultGainRates, sizeof(kDefaultGainRates));
    memcpy(values, kDefaultGainValues, sizeof(kDefaultGainValues));
    *count = DEFAULT_GAIN_COUNT;
}

void gyro_profile_set_defaults(GyroProfile* profile) {
    profile->enabled = true;
    profile->dead_zone = 0.02f;
    profile->dead_zone_bias = 0;  // gain curve's boosted low-end response
                                   // should make this unnecessary by default
                                   // (see task 5 of the response-model plan)
    profile->trigger_threshold = 250;
    set_default_gain(profile->gain_rates_h, profile->gain_values_h, &profile->gain_count_h);
    set_default_gain(profile->gain_rates_v, profile->gain_values_v, &profile->gain_count_v);
    profile->lowpass_alpha = 0.5f;
    profile->damping_factor = 0.88f;
    profile->saturation_knee = 100.0f;
    profile->invert_x = false;
    profile->invert_y = false;
    profile->yaw_from_z = false;
    profile->yaw_tilt_weight = 0.0f;
    profile->drift_correction_enabled = true;
}

// Parses a space-separated list of floats from `str` into `out` (max
// `max_count` entries). Returns the number of values actually parsed.
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

static void load_gain_curve(ini_table_s* table, const char* section, const char* rates_key,
                             const char* values_key, float* rates, float* values, int* count) {
    const char* rates_str = ini_table_get_entry(table, section, rates_key);
    const char* values_str = ini_table_get_entry(table, section, values_key);
    if (rates_str == NULL || values_str == NULL) {
        return;
    }
    int rc = parse_float_list(rates_str, rates, MAX_GAIN_POINTS);
    int vc = parse_float_list(values_str, values, MAX_GAIN_POINTS);
    // Mismatched-length pairs are silently skipped (keeps whatever the
    // profile already had -- either the hardcoded default or a prior
    // section's value).
    if (rc > 0 && rc == vc) {
        *count = rc;
    }
}

static void load_section(ini_table_s* table, const char* section, GyroProfile* profile) {
    if (_ini_section_find(table, section) == NULL) {
        return;
    }
    ini_table_get_entry_as_bool(table, section, "Enabled", &profile->enabled);
    ini_table_get_entry_as_float(table, section, "DeadZone", &profile->dead_zone);
    ini_table_get_entry_as_int(table, section, "DeadZoneBias", &profile->dead_zone_bias);
    ini_table_get_entry_as_int(table, section, "TriggerThreshold", &profile->trigger_threshold);

    load_gain_curve(table, section, "GainRates_H", "GainValues_H",
                     profile->gain_rates_h, profile->gain_values_h, &profile->gain_count_h);
    load_gain_curve(table, section, "GainRates_V", "GainValues_V",
                     profile->gain_rates_v, profile->gain_values_v, &profile->gain_count_v);

    ini_table_get_entry_as_float(table, section, "LowPassAlpha", &profile->lowpass_alpha);
    ini_table_get_entry_as_float(table, section, "DampingFactor", &profile->damping_factor);
    ini_table_get_entry_as_float(table, section, "SaturationKnee", &profile->saturation_knee);

    ini_table_get_entry_as_bool(table, section, "InvertX", &profile->invert_x);
    ini_table_get_entry_as_bool(table, section, "InvertY", &profile->invert_y);
    ini_table_get_entry_as_bool(table, section, "YawFromZ", &profile->yaw_from_z);
    ini_table_get_entry_as_float(table, section, "YawTiltWeight", &profile->yaw_tilt_weight);
    ini_table_get_entry_as_bool(table, section, "DriftCorrectionEnabled", &profile->drift_correction_enabled);
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
    g_is_stationary = false;
    g_stationary_count = 0;
    g_runtime_enabled = true;
    g_prev_l3r3_held = false;
    g_touchpad_hold_count = 0;
    g_recal_armed = true;
    g_lightbar_state = LB_UNSET;
    g_float_stick_x = 0.0f;
    g_float_stick_y = 0.0f;
}

void gyro_set_profile(const GyroProfile* profile) {
    g_profile = *profile;
}

GyroProfile gyro_get_profile(void) {
    return g_profile;
}

static void start_recalibration(void) {
    g_calibrated = false;
    g_calib_count = 0;
    memset(g_calib_sum, 0, sizeof(g_calib_sum));
    memset(g_bias, 0, sizeof(g_bias));
    g_is_stationary = false;
    g_stationary_count = 0;
    g_float_stick_x = 0.0f;
    g_float_stick_y = 0.0f;
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

// Linear-interpolation gain lookup from a table of (rate_breakpoint,
// gain_value) pairs. Below the first breakpoint the first gain is
// returned; above the last, the last gain. Between breakpoints gain is
// linearly interpolated (NOT a step function -- a hard jump right at a
// breakpoint would itself feel like a glitch, which this response model
// is trying to eliminate). Rates must be ascending.
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

// Processes one axis: looks up gain for the current rate, then applies
// EMA (while actively moving) or damping (while inside the dead zone) to
// the persistent float_stick state. EMA and damping are mutually
// exclusive per sample -- exactly one runs -- so their effects never
// compound into a double-smoothed/double-lagged feel.
static float process_axis(float rate, float* float_stick,
                           const float* gain_rates, const float* gain_values, int gain_count,
                           float alpha, float damping) {
    if (rate != 0.0f) {
        float raw = rate * gain_lookup(gain_rates, gain_values, gain_count, fabsf(rate));
        if (alpha < 1.0f) {
            *float_stick += alpha * (raw - *float_stick);
        } else {
            *float_stick = raw;
        }
    } else {
        *float_stick *= damping;
    }
    return *float_stick;
}

// Converts the final float_stick value (signed stick-units, nominally
// within about -128..128) to a uint8_t pad report value, applying soft
// saturation and an optional dead-zone-bias floor, then SUMS it onto
// whatever the physical stick's current position already is (read from
// *axis, which holds the raw hardware reading at this point) rather than
// overwriting it -- so gyro contributes ON TOP OF physical stick input
// instead of replacing it while aiming. Soft saturation is applied to the
// gyro component alone (the physical stick is already hardware-bounded to
// -128..127, no need to saturate that separately); the final combined sum
// is hard-clamped to the valid stick range.
static void write_stick_uint8(uint8_t* axis, float stick, float knee, int dbias) {
    float sat = tanhf(stick / knee) * 128.0f;
    if (stick != 0.0f && dbias > 0) {
        int si = (int)sat;
        int mag = si < 0 ? -si : si;
        if (mag < dbias) {
            sat = (sat < 0.0f) ? (float)(-dbias) : (float)dbias;
        }
    }
    int phys = (int)(*axis) - STICK_CENTER;  // current physical stick position, signed
    int combined = clamp_int(phys + (int)sat, -128, 127);
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
        // Not aiming: keep the float stick pinned at 0 so the next aiming
        // session starts fresh rather than partially blending in a stale
        // value via the EMA once motion resumes.
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

    // --- Calibration -----------------------------------------------------
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
            g_calibrated = true;
            log_info("gyro calibration complete: bias=[%f %f %f]\n", g_bias[0], g_bias[1], g_bias[2]);
        }
        return;
    }

    // --- Continuous drift tracking (only while not actively aiming) ------
    if (g_profile.drift_correction_enabled) {
        float ax = pData->acceleration.x;
        float ay = pData->acceleration.y;
        float az = pData->acceleration.z;
        float accel_mag_sq = ax * ax + ay * ay + az * az;
        bool now_stationary =
            accel_mag_sq >= STATIONARY_ACCEL_LOW_SQ && accel_mag_sq <= STATIONARY_ACCEL_HIGH_SQ;

        if (now_stationary) {
            if (!g_is_stationary) {
                g_is_stationary = true;
                g_stationary_count = 0;
            }
            g_stationary_count++;
            if (g_stationary_count >= STATIONARY_SAMPLE_COUNT) {
                g_bias[0] += DRIFT_ALPHA * (gx - g_bias[0]);
                g_bias[1] += DRIFT_ALPHA * (gy - g_bias[1]);
                g_bias[2] += DRIFT_ALPHA * (gz - g_bias[2]);
            }
        } else {
            g_is_stationary = false;
            g_stationary_count = 0;
        }
    }

    // --- Trigger gate ------------------------------------------------------
    bool aiming = pData->analogButtons.l2 >= g_profile.trigger_threshold;
    set_lightbar(handle, aiming ? LB_ACTIVE : LB_INACTIVE);
    if (!aiming) {
        // Same reasoning as the disabled-path reset above: L2 released
        // means no aiming session is active, so the next press should
        // start from a clean float_stick rather than a stale one.
        g_float_stick_x = 0.0f;
        g_float_stick_y = 0.0f;
        return;
    }

    // --- Response model: gain curve -> EMA/damping -> soft saturation ----
    // yaw is primarily driven by whichever axis yaw_from_z selects, with
    // an optional weighted blend of the OTHER axis -- for aiming motions
    // that are naturally a combined rotation+tilt rather than a pure
    // single-axis yaw (both axes' bias is already tracked above
    // regardless of which is "primary", so this is just a sum of two
    // already-bias-corrected rates).
    float yaw_primary = (g_profile.yaw_from_z ? gz : gy) - g_bias[g_profile.yaw_from_z ? 2 : 1];
    float yaw_secondary = (g_profile.yaw_from_z ? gy : gz) - g_bias[g_profile.yaw_from_z ? 1 : 2];
    float yaw = yaw_primary + yaw_secondary * g_profile.yaw_tilt_weight;
    float pitch = gx - g_bias[0];

    // Dead zone — zero out tiny residual motion. This gating also decides
    // whether process_axis() below runs EMA (rate != 0) or damping
    // (rate == 0) for each axis.
    if (fabsf(yaw) < g_profile.dead_zone) yaw = 0.0f;
    if (fabsf(pitch) < g_profile.dead_zone) pitch = 0.0f;

    if (g_profile.invert_x) yaw = -yaw;
    if (g_profile.invert_y) pitch = -pitch;

    float stick_x = process_axis(yaw, &g_float_stick_x,
                                  g_profile.gain_rates_h, g_profile.gain_values_h,
                                  g_profile.gain_count_h,
                                  g_profile.lowpass_alpha, g_profile.damping_factor);
    float stick_y = process_axis(pitch, &g_float_stick_y,
                                  g_profile.gain_rates_v, g_profile.gain_values_v,
                                  g_profile.gain_count_v,
                                  g_profile.lowpass_alpha, g_profile.damping_factor);

    write_stick_uint8(&pData->rightStick.x, stick_x, g_profile.saturation_knee,
                       g_profile.dead_zone_bias);
    write_stick_uint8(&pData->rightStick.y, stick_y, g_profile.saturation_knee,
                       g_profile.dead_zone_bias);
}
