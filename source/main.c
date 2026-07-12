// GoldHEN gyro-aim plugin.
//
// Hooks scePadRead/scePadReadState... but NOT by calling through to the
// original via HOOK_CONTINUE. Those two functions turned out to be too
// thin/short to safely re-trampoline (this crashed every target game
// immediately on launch in initial testing) -- exactly why the official
// gamepad_helper reference plugin avoids that pattern for these specific
// functions too. Instead, following gamepad_helper's proven approach:
// scePadReadExt/scePadReadStateExt are called directly (no hook needed on
// them at all) after neutralizing an internal privilege/lock guard
// instruction via a raw byte patch (Patcher, not Detour).
#include <Common.h>
#include <Patcher.h>
#include <stdio.h>
#include <sys/stat.h>

#include "gyro_common.h"
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

static Patcher* g_scePadReadExtPatcher;
static Patcher* g_scePadReadStateExtPatcher;

static bool file_exists(const char* path) {
    struct stat st;
    return stat(path, &st) == 0;
}

int32_t scePadRead_hook(int32_t handle, ScePadData* pData, int32_t num) {
    int ret = scePadReadExt(handle, pData, num);
    if (ret <= 0) {
        return ret;
    }
    for (int i = 0; i < ret; i++) {
        gyro_process_sample(handle, &pData[i]);
    }
    return ret;
}

int32_t scePadReadState_hook(int32_t handle, ScePadData* pData) {
    int ret = scePadReadStateExt(handle, pData);
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

    // scePadReadExt/scePadReadStateExt normally refuse to run outside of
    // libScePad's own privileged caller; the reference gamepad_helper plugin
    // defeats that check by NOPing the "xor ecx,ecx"/"xor edx,edx" guard
    // instruction at each function's entry. Same patch, same offsets.
    g_scePadReadExtPatcher = (Patcher*)malloc(sizeof(Patcher));
    g_scePadReadStateExtPatcher = (Patcher*)malloc(sizeof(Patcher));
    Patcher_Construct(g_scePadReadExtPatcher);
    Patcher_Construct(g_scePadReadStateExtPatcher);

    uint8_t xor_ecx_ecx[5] = { 0x31, 0xC9, 0x90, 0x90, 0x90 };
    Patcher_Install_Patch(g_scePadReadExtPatcher, (uint64_t)scePadReadExt, xor_ecx_ecx,
                           sizeof(xor_ecx_ecx));

    uint8_t xor_edx_edx[5] = { 0x31, 0xD2, 0x90, 0x90, 0x90 };
    Patcher_Install_Patch(g_scePadReadStateExtPatcher, (uint64_t)scePadReadStateExt, xor_edx_edx,
                           sizeof(xor_edx_edx));

    HOOK32(scePadRead);
    HOOK32(scePadReadState);

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

    return 0;
}

s32 attr_public plugin_unload(s32 argc, const char* argv[]) {
    log_info("<%s Ver.0x%08x> %s\n", g_pluginName, g_pluginVersion, __func__);
    UNHOOK(scePadRead);
    UNHOOK(scePadReadState);

    Patcher_Destroy(g_scePadReadExtPatcher);
    Patcher_Destroy(g_scePadReadStateExtPatcher);
    free(g_scePadReadExtPatcher);
    free(g_scePadReadStateExtPatcher);

    return 0;
}

s32 attr_module_hidden module_start(s64 argc, const void* args) {
    return 0;
}

s32 attr_module_hidden module_stop(s64 argc, const void* args) {
    return 0;
}

