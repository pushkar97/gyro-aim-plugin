// Implementation of gyro-to-stick mapping, calibration and drift correction,
// and hotkey/state handling.
//
// Deliberate design choices carried over from the design/grilling session:
// - Velocity-based mapping: instantaneous corrected angular velocity is
//   scaled directly onto stick deflection every sample, with NO integration
//   over time and NO dt term. This matches how native gyro-aim (e.g.
//   Splatoon/JoyShockMapper-style) works: the stick's value each read
//   represents the CURRENT turn rate, same semantics the game already gives
//   a physical stick held at a constant deflection.
// - Calibration/drift timing uses SAMPLE COUNTS, not the ScePadData
//   `timestamp` field, because that field's exact units/epoch could not be
//   verified from public headers. Counting samples avoids depending on an
//   unverified unit assumption; because both scePadRead (averaged) and
//   scePadReadState funnel through gyro_process_sample() per-sample, sample
//   counts scale consistently with real elapsed time regardless of which
//   entry point the game uses.
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

// Per-axis floating-point stick state. Persists across aim sessions (L2
// feathering), decays via damping when gyro is deadzoning, and is
// smoothed via EMA during active motion. Reset to 0.0 on
// init/recalibration.
static float g_float_stick_x = 0.0f;
static float g_float_stick_y = 0.0f;

static void set_default_gain(float* rates, float* values, int* count,
                               float r1, float v1, float r2, float v2,
                               float r3, float v3, float r4, float v4,
                               float r5, float v5) {
    rates[0] = r1; values[0] = v1;
    rates[1] = r2; values[1] = v2;
    rates[2] = r3; values[2] = v3;
    rates[3] = r4; values[3] = v4;
    rates[4] = r5; values[4] = v5;
    *count = 5;
}

void gyro_profile_set_defaults(GyroProfile* profile) {
    profile->enabled = true;
    profile->dead_zone = 0.02f;
    profile->dead_zone_bias = 0;
    profile->trigger_threshold = 250;
    set_default_gain(profile->gain_rates_h, profile->gain_values_h, &profile->gain_count_h,
                     0.05f, 90.0f, 0.15f, 70.0f, 0.40f, 50.0f, 1.00f, 35.0f, 100.0f, 25.0f);
    set_default_gain(profile->gain_rates_v, profile->gain_values_v, &profile->gain_count_v,
                     0.05f, 90.0f, 0.15f, 70.0f, 0.40f, 50.0f, 1.00f, 35.0f, 100.0f, 25.0f);
    profile->lowpass_alpha = 1.0f;
    profile->damping_factor = 0.88f;
    profile->saturation_knee = 100.0f;
    profile->invert_x = false;
    profile->invert_y = false;
    profile->yaw_from_z = false;
    profile->yaw_tilt_weight = 0.0f;
    profile->drift_correction_enabled = true;
}

// Parse a space-separated list of floats from `str` into `out` (max
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

static void load_section(ini_table_s* table, const char* section, GyroProfile* profile) {
    if (_ini_section_find(table, section) == NULL) {
        return;
    }
    ini_table_get_entry_as_bool(table, section, "Enabled", &profile->enabled);
    ini_table_get_entry_as_float(table, section, "DeadZone", &profile->dead_zone);
    ini_table_get_entry_as_int(table, section, "DeadZoneBias", &profile->dead_zone_bias);
    ini_table_get_entry_as_int(table, section, "TriggerThreshold", &profile->trigger_threshold);

    // Gain curves: space-separated rate/value lists. If only one of the
    // pair is present we ignore it (mismatched arity is silently skipped).
    const char* rates_str = ini_table_get_entry(table, section, "GainRates_H");
    const char* values_str = ini_table_get_entry(table, section, "GainValues_H");
    if (rates_str != NULL && values_str != NULL) {
        int rc = parse_float_list(rates_str, profile->gain_rates_h, MAX_GAIN_POINTS);
        int vc = parse_float_list(values_str, profile->gain_values_h, MAX_GAIN_POINTS);
        if (rc > 0 && rc == vc) profile->gain_count_h = rc;
    }
    rates_str = ini_table_get_entry(table, section, "GainRates_V");
    values_str = ini_table_get_entry(table, section, "GainValues_V");
    if (rates_str != NULL && values_str != NULL) {
        int rc = parse_float_list(rates_str, profile->gain_rates_v, MAX_GAIN_POINTS);
        int vc = parse_float_list(values_str, profile->gain_values_v, MAX_GAIN_POINTS);
        if (rc > 0 && rc == vc) profile->gain_count_v = rc;
    }

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
// linearly interpolated. Rates must be ascending.
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

// Process one axis: apply gain curve, EMA, and damping to produce a new
// float_stick value. Call with the bias-corrected / inverted / dead-zoned
// rate and the persistent float_stick state.
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

// Convert the final float_stick value (in signed stick-units, ~ -128..128
// range) to a uint8_t, applying soft saturation and optional dead-zone
// bias floor.
static void write_stick_uint8(uint8_t* axis, float stick, float knee, int dbias) {
    float sat = tanhf(stick / knee) * 128.0f;
    if (stick != 0.0f && dbias > 0) {
        int si = (int)sat;
        int mag = si < 0 ? -si : si;
        if (mag < dbias) {
            sat = (sat < 0.0f) ? (float)(-dbias) : (float)dbias;
        }
    }
    int val = clamp_int((int)sat, -128, 127) + STICK_CENTER;
    *axis = (uint8_t)clamp_int(val, STICK_MIN, STICK_MAX);
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
        float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
        bool now_stationary = fabsf(accel_mag - 9.80665f) < STATIONARY_ACCEL_TOLERANCE;

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
        return;
    }

    // --- Velocity-based mapping (gain curve + EMA + damping) ----------
    // yaw is primarily driven by whichever axis yaw_from_z selects, with
    // an optional weighted blend of the OTHER axis.
    float yaw_primary = (g_profile.yaw_from_z ? gz : gy) - g_bias[g_profile.yaw_from_z ? 2 : 1];
    float yaw_secondary = (g_profile.yaw_from_z ? gy : gz) - g_bias[g_profile.yaw_from_z ? 1 : 2];
    float yaw = yaw_primary + yaw_secondary * g_profile.yaw_tilt_weight;
    float pitch = gx - g_bias[0];

    // Dead zone — zero out tiny residual motion.
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
