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

void gyro_profile_set_defaults(GyroProfile* profile) {
    profile->enabled = true;
    profile->sensitivity_h = 40.0f;
    profile->sensitivity_v = 40.0f;
    profile->dead_zone = 0.02f;
    profile->dead_zone_bias = 20;
    profile->trigger_threshold = 250;
    profile->invert_x = false;
    profile->invert_y = false;
    profile->yaw_from_z = false;
    profile->curve_power = 2.0f;
    profile->curve_min_rate = 0.15f;
}

static void load_section(ini_table_s* table, const char* section, GyroProfile* profile) {
    if (_ini_section_find(table, section) == NULL) {
        return;
    }
    ini_table_get_entry_as_bool(table, section, "Enabled", &profile->enabled);
    ini_table_get_entry_as_float(table, section, "SensitivityH", &profile->sensitivity_h);
    ini_table_get_entry_as_float(table, section, "SensitivityV", &profile->sensitivity_v);
    ini_table_get_entry_as_float(table, section, "DeadZone", &profile->dead_zone);
    ini_table_get_entry_as_int(table, section, "DeadZoneBias", &profile->dead_zone_bias);
    ini_table_get_entry_as_int(table, section, "TriggerThreshold", &profile->trigger_threshold);
    ini_table_get_entry_as_bool(table, section, "InvertX", &profile->invert_x);
    ini_table_get_entry_as_bool(table, section, "InvertY", &profile->invert_y);
    ini_table_get_entry_as_bool(table, section, "YawFromZ", &profile->yaw_from_z);
    ini_table_get_entry_as_float(table, section, "CurvePower", &profile->curve_power);
    ini_table_get_entry_as_float(table, section, "CurveMinRate", &profile->curve_min_rate);
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

// Applies signed exponential curve to the gyro rate so small movements
// ramp up smoothly while big flicks still reach full speed.  power=1.0
// is linear (no curve); 2.0 (default) gives a gentle s-curve feel.
// Values below `min_rate` are passed through linearly (not curved) so
// tiny residual bias doesn't get exponentially amplified into a
// directional asymmetry — the curve only kicks in above the threshold.
// Input is clamped before powf() so a sensor glitch/spike (the DS4 has
// been observed emitting bad samples in this project before) can't
// produce an enormous or infinite result that later causes undefined
// behavior when cast to int.
#define GYRO_RATE_CLAMP 30.0f  // rad/s; DS4 max spec is far below this
static float signed_pow_curve(float v, float power, float min_rate) {
    float av = fabsf(v);
    if (v == 0.0f || power <= 0.001f || power == 1.0f) return v;
    if (av < min_rate) return v;  // linear shoulder: no curve below threshold
    if (av > GYRO_RATE_CLAMP) av = GYRO_RATE_CLAMP;
    float sign = (v < 0.0f) ? -1.0f : 1.0f;
    return sign * powf(av, power);
}

static void apply_stick_delta(uint8_t* axis, float delta_units, int dead_zone_bias) {
    if (delta_units == 0.0f || isnan(delta_units)) {
        return;
    }

    // The final result is clamped to the -128..127 stick range below
    // regardless, but clamp the FLOAT here too, before the cast to int:
    // casting a float that's outside int range (or +/-Inf) to int is
    // undefined behavior in C, and on x86 can raise an FPU
    // invalid-operation trap depending on exception-mask state -- a real
    // crash risk if any upstream math (e.g. the exponential curve) ever
    // produces an extreme value from a sensor glitch.
    if (delta_units > 100000.0f) delta_units = 100000.0f;
    if (delta_units < -100000.0f) delta_units = -100000.0f;

    int delta = (int)delta_units;
    if (delta == 0) {
        // Rounds to zero in integer space but was non-zero in float space;
        // still apply the minimum deadzone-bias push so tiny gyro motion
        // isn't fully lost, matching the sign of the float delta.
        delta = (delta_units > 0.0f) ? 1 : -1;
    }

    if (dead_zone_bias > 0) {
        int mag = delta < 0 ? -delta : delta;
        if (mag < dead_zone_bias) {
            delta = (delta < 0) ? -dead_zone_bias : dead_zone_bias;
        }
    }

    int current = (int)(*axis) - STICK_CENTER;  // -128..127
    int result = clamp_int(current + delta, -128, 127);
    *axis = (uint8_t)clamp_int(result + STICK_CENTER, STICK_MIN, STICK_MAX);
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

    // --- Trigger gate ------------------------------------------------------
    bool aiming = pData->analogButtons.l2 >= g_profile.trigger_threshold;
    set_lightbar(handle, aiming ? LB_ACTIVE : LB_INACTIVE);
    if (!aiming) {
        return;
    }

    // --- Velocity-based mapping ---------------------------------------
    float yaw = (g_profile.yaw_from_z ? gz : gy) - g_bias[g_profile.yaw_from_z ? 2 : 1];
    float pitch = gx - g_bias[0];

    if (fabsf(yaw) < g_profile.dead_zone) yaw = 0.0f;
    if (fabsf(pitch) < g_profile.dead_zone) pitch = 0.0f;

    if (g_profile.invert_x) yaw = -yaw;
    if (g_profile.invert_y) pitch = -pitch;

    float delta_x = signed_pow_curve(yaw, g_profile.curve_power, g_profile.curve_min_rate) * g_profile.sensitivity_h;
    float delta_y = signed_pow_curve(pitch, g_profile.curve_power, g_profile.curve_min_rate) * g_profile.sensitivity_v;

    apply_stick_delta(&pData->rightStick.x, delta_x, g_profile.dead_zone_bias);
    apply_stick_delta(&pData->rightStick.y, delta_y, g_profile.dead_zone_bias);
}
