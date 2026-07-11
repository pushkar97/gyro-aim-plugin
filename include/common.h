// Minimal common definitions for this plugin (no dependency on the upstream
// plugin_common.h, which pulls in git_ver.h/Notify OSD helpers we don't use).
// Logging goes through klog(), declared by the GoldHEN SDK's GoldHEN.h
// (pulled in transitively via <Common.h>).
#pragma once

#include <stdint.h>

#include <Common.h>  // klog() and friends, from the GoldHEN Plugins SDK

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;
typedef float f32;
typedef double f64;

#define attr_module_hidden __attribute__((weak)) __attribute__((visibility("hidden")))
#define attr_public __attribute__((visibility("default")))

#define GOLDHEN_PATH "/data/GoldHEN"

#define LOG_TAG "[gyro_aim]"
#define log_info(a, args...) klog(LOG_TAG " (%s:%d) " a, __FILE__, __LINE__, ##args)
