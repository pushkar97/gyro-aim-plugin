// GoldHEN gyro-aim plugin.
//
// Hooks scePadRead/scePadReadState (both, since we don't yet know which
// entry point any given target game uses) and injects gyro-derived rotation
// onto the right analog stick while L2 is fully pressed, using the same
// hooking pattern as the official gamepad_helper example plugin
// (GoldHEN_Plugins_Repository/plugin_src/gamepad_helper).
//
// See gyro.c for the calibration/mapping logic itself; this file is just the
// scePad hook glue + config/title lookup + plugin lifecycle.
#include <Common.h>
#include <stdio.h>
#include <sys/stat.h>

#include "common.h"
#include "config.h"
#include "gyro.h"
#include "pad.h"

attr_public const char* g_pluginName = "gyro_aim";
attr_public const char* g_pluginDesc = "Gyro-to-stick aim assist for games without native gyro support";
attr_public const char* g_pluginAuth = "(you)";
attr_public u32 g_pluginVersion = 0x00000100;  // 1.00

HOOK_INIT(scePadRead);
HOOK_INIT(scePadReadState);

#define PLUGIN_CONFIG_PATH GOLDHEN_PATH "/gyroaim.ini"

static bool file_exists(const char* path) {
    struct stat st;
    return stat(path, &st) == 0;
}

int32_t scePadRead_hook(int32_t handle, ScePadData* pData, int32_t num) {
    int ret = HOOK_CONTINUE(scePadRead, int (*)(int32_t, ScePadData*, int32_t), handle, pData, num);
    if (ret <= 0) {
        return ret;
    }
    for (int i = 0; i < ret; i++) {
        gyro_process_sample(handle, &pData[i]);
    }
    return ret;
}

int32_t scePadReadState_hook(int32_t handle, ScePadData* pData) {
    int ret = HOOK_CONTINUE(scePadReadState, int (*)(int32_t, ScePadData*), handle, pData);
    if (ret != 0) {
        return ret;
    }
    gyro_process_sample(handle, pData);
    return ret;
}

s32 attr_public plugin_load(s32 argc, const char* argv[]) {
    log_info("<%s Ver.0x%08x> %s\n", g_pluginName, g_pluginVersion, __func__);
    log_info("Plugin Author(s): %s\n", g_pluginAuth);

    char module[256];
    snprintf(module, sizeof(module), "/%s/common/lib/%s", sceKernelGetFsSandboxRandomWord(),
              "libScePad.sprx");
    int h = 0;
    sys_dynlib_load_prx(module, &h);

    struct proc_info procInfo;
    const char* title_id = "";
    if (sys_sdk_proc_info(&procInfo) == 0) {
        title_id = procInfo.titleid;
        log_info("titleid: %s\n", procInfo.titleid);
    } else {
        log_info("sys_sdk_proc_info failed; proceeding with [default] profile only\n");
    }

    GyroProfile profile;
    gyro_profile_set_defaults(&profile);

    if (file_exists(PLUGIN_CONFIG_PATH)) {
        if (!gyro_profile_load(PLUGIN_CONFIG_PATH, title_id, &profile)) {
            log_info("failed to parse %s; using hardcoded defaults\n", PLUGIN_CONFIG_PATH);
        }
    } else {
        log_info("%s not found; using hardcoded defaults\n", PLUGIN_CONFIG_PATH);
    }

    if (!profile.enabled) {
        log_info("gyro-aim disabled for this title by config\n");
    }

    gyro_state_init(&profile);

    HOOK32(scePadRead);
    HOOK32(scePadReadState);

    return 0;
}

s32 attr_public plugin_unload(s32 argc, const char* argv[]) {
    log_info("<%s Ver.0x%08x> %s\n", g_pluginName, g_pluginVersion, __func__);
    UNHOOK(scePadRead);
    UNHOOK(scePadReadState);
    return 0;
}

s32 attr_module_hidden module_start(s64 argc, const void* args) {
    return 0;
}

s32 attr_module_hidden module_stop(s64 argc, const void* args) {
    return 0;
}
