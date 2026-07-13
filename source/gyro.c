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
#define CALIB_GYRO_STILL_THRESHOLD 0.05f  // rad/s; |gx|/|gy|/|gz| must be
                                           // below this for a calibration
                                           // sample to be accepted. Must
                                           // be lenient enough to pass
                                           // with real DS4 gyro rest noise
                                           // (not a lab-grade sensor), but
                                           // tight enough to reject
                                           // deliberate motion.
                                           // Matched to the drift-
                                           // correction threshold for
                                           // consistency.
#define STATIONARY_SAMPLE_COUNT 250   // consecutive still samples before drift EMA kicks in
#define DRIFT_ALPHA 0.01f
#define STATIONARY_ACCEL_TOLERANCE 0.5f  // m/s^2 around 9.80665 counted as "still"
#define STATIONARY_GYRO_THRESHOLD 0.05f  // rad/s; all gyro axes must be below
                                          // this for drift-correction to
                                          // consider the controller stationary
// Squared accel bounds — avoids a sqrtf() call on every sample (see below).
#define STATIONARY_ACCEL_LOW_SQ \
    ((9.80665f - STATIONARY_ACCEL_TOLERANCE) * (9.80665f - STATIONARY_ACCEL_TOLERANCE))
#define STATIONARY_ACCEL_HIGH_SQ \
    ((9.80665f + STATIONARY_ACCEL_TOLERANCE) * (9.80665f + STATIONARY_ACCEL_TOLERANCE))

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

typedef enum LightbarState { LB_UNSET = -1, LB_INACTIVE, LB_ACTIVE, LB_CALIBRATING } LightbarState;

static GyroProfile g_profile;

static bool g_calibrated = false;
static int g_calib_count = 0;          // accepted (stationary) samples
static float g_calib_sum[3] = { 0, 0, 0 };
static float g_bias[3] = { 0, 0, 0 };

static bool g_is_stationary = false;
static int g_stationary_count = 0;

static bool g_runtime_enabled = true;
static bool g_prev_l3r3_held = false;

static int g_touchpad_hold_count = 0;
static bool g_recal_armed = true;

static LightbarState g_lightbar_state = LB_UNSET;

static float g_float_stick_x = 0.0f;
static float g_float_stick_y = 0.0f;

// Dead-zone hysteresis state (task 6): track whether each axis is
// currently inside the dead zone so we can apply different enter/exit
// thresholds. Reset on calibration.
static bool g_yaw_in_dz = false;
static bool g_pitch_in_dz = false;

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
    profile->trigger_threshold = 250;
    set_default_gain(profile->gain_rates_h, profile->gain_values_h, &profile->gain_count_h);
    set_default_gain(profile->gain_rates_v, profile->gain_values_v, &profile->gain_count_v);
    profile->lowpass_alpha = 0.5f;
    // Task 1: damping is now interpolation-based (fraction moved toward
    // zero per sample, not a multiplier). 0.12 ≈ the old *= 0.88 feel.
    profile->damping_factor = 0.12f;
    profile->saturation_knee = 100.0f;
    profile->invert_x = false;
    profile->invert_y = false;
    profile->yaw_from_z = false;
    profile->yaw_tilt_weight = 0.0f;
    profile->drift_correction_enabled = true;
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
    g_yaw_in_dz = false;
    g_pitch_in_dz = false;
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
    g_yaw_in_dz = false;
    g_pitch_in_dz = false;
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

// Task 2: check whether the controller is genuinely stationary (both accel
// and gyro) for calibration-sample admission.
static bool calibration_sample_is_stationary(float gx, float gy, float gz,
                                              float ax, float ay, float az) {
    float accel_mag_sq = ax * ax + ay * ay + az * az;
    // If the accelerometer is returning near-zero (not providing data
    // at all on this firmware / SDK version), skip the accel check and
    // rely solely on gyro stillness below. A zero accel reading is NOT
    // valid stationary data — it's just no data — and would otherwise
    // fail < STATIONARY_ACCEL_LOW_SQ every time, blocking calibration
    // indefinitely.
    if (accel_mag_sq > 1.0f) {
        if (accel_mag_sq < STATIONARY_ACCEL_LOW_SQ || accel_mag_sq > STATIONARY_ACCEL_HIGH_SQ) {
            return false;
        }
    }
    if (fabsf(gx) >= CALIB_GYRO_STILL_THRESHOLD ||
        fabsf(gy) >= CALIB_GYRO_STILL_THRESHOLD ||
        fabsf(gz) >= CALIB_GYRO_STILL_THRESHOLD) {
        return false;
    }
    return true;
}

// Task 3: extended drift-correction stationary check — requires both accel
// AND gyro to be still.
static bool drift_sample_is_stationary(float gx, float gy, float gz,
                                        float ax, float ay, float az) {
    float accel_mag_sq = ax * ax + ay * ay + az * az;
    // Same zero-accel fallback as calibration: if the sensor isn't
    // providing data, skip the accel check and rely on gyro stillness.
    if (accel_mag_sq > 1.0f) {
        if (accel_mag_sq < STATIONARY_ACCEL_LOW_SQ || accel_mag_sq > STATIONARY_ACCEL_HIGH_SQ) {
            return false;
        }
    }
    if (fabsf(gx) >= STATIONARY_GYRO_THRESHOLD ||
        fabsf(gy) >= STATIONARY_GYRO_THRESHOLD ||
        fabsf(gz) >= STATIONARY_GYRO_THRESHOLD) {
        return false;
    }
    return true;
}

// Task 6: apply dead-zone with hysteresis. Returns true if the rate should
// be treated as zero (inside the dead zone).
static bool deadzone_with_hysteresis(float rate, float dead_zone, bool* in_dz) {
    float av = fabsf(rate);
    if (*in_dz) {
        if (av < dead_zone * DEADZONE_EXIT_FRAC) return true;   // stay inside
        *in_dz = false;
        return false;
    } else {
        if (av < dead_zone * DEADZONE_ENTER_FRAC) {
            *in_dz = true;
            return true;  // just entered
        }
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

// Task 1: damping is now interpolation-based (fraction toward zero per
// sample) rather than a multiplier. With the new parameterization,
// damping=0.12 means "move 12% toward zero each sample," equivalent to the
// old damping=0.88 multiplier.
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
        // Interpolation toward zero: fraction 'damping' of the distance
        // to zero is covered each sample. More intuitive tuning and more
        // consistent across polling rates than the old *= multiplier.
        *float_stick += (0.0f - *float_stick) * damping;
    }
    return *float_stick;
}

// Task 4: use lroundf() instead of (int) casts for float→int conversions
// in the stick output path, for symmetry and precision around center.
static void write_stick_uint8(uint8_t* axis, float stick, float knee, int dbias) {
    float sat = tanhf(stick / knee) * 128.0f;
    if (stick != 0.0f && dbias > 0) {
        int si = lroundf(sat);
        int mag = si < 0 ? -si : si;
        if (mag < dbias) {
            sat = (sat < 0.0f) ? (float)(-dbias) : (float)dbias;
        }
    }
    int phys = (int)(*axis) - STICK_CENTER;
    int combined = clamp_int(phys + lroundf(sat), -128, 127);
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

    // --- Calibration (task 2: only accept stationary samples) -------------
    if (!g_calibrated) {
        set_lightbar(handle, LB_CALIBRATING);
        float ax = pData->acceleration.x;
        float ay = pData->acceleration.y;
        float az = pData->acceleration.z;
        if (calibration_sample_is_stationary(gx, gy, gz, ax, ay, az)) {
            g_calib_sum[0] += gx;
            g_calib_sum[1] += gy;
            g_calib_sum[2] += gz;
            g_calib_count++;
            if (g_calib_count >= CALIB_SAMPLE_COUNT) {
                g_bias[0] = g_calib_sum[0] / (float)g_calib_count;
                g_bias[1] = g_calib_sum[1] / (float)g_calib_count;
                g_bias[2] = g_calib_sum[2] / (float)g_calib_count;
                g_calibrated = true;
                log_info("gyro calibration complete: %d accepted samples, bias=[%f %f %f]\n",
                         g_calib_count, g_bias[0], g_bias[1], g_bias[2]);
            }
        }
        return;
    }

    // --- Continuous drift tracking (task 3: accel + gyro stillness) ------
    if (g_profile.drift_correction_enabled) {
        float ax = pData->acceleration.x;
        float ay = pData->acceleration.y;
        float az = pData->acceleration.z;
        bool now_stationary = drift_sample_is_stationary(gx, gy, gz, ax, ay, az);

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
        g_float_stick_x = 0.0f;
        g_float_stick_y = 0.0f;
        return;
    }

    // --- Response model: gain curve -> EMA/damping -> soft saturation ----
    float yaw_primary = (g_profile.yaw_from_z ? gz : gy) - g_bias[g_profile.yaw_from_z ? 2 : 1];
    float yaw_secondary = (g_profile.yaw_from_z ? gy : gz) - g_bias[g_profile.yaw_from_z ? 1 : 2];
    float yaw = yaw_primary + yaw_secondary * g_profile.yaw_tilt_weight;
    float pitch = gx - g_bias[0];

    // Dead zone with hysteresis (task 6): different enter/exit thresholds
    // prevent rapid oscillation between movement and damping near the
    // boundary. Each axis has its own hysteresis state.
    if (deadzone_with_hysteresis(yaw, g_profile.dead_zone, &g_yaw_in_dz)) yaw = 0.0f;
    if (deadzone_with_hysteresis(pitch, g_profile.dead_zone, &g_pitch_in_dz)) pitch = 0.0f;

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