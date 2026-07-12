// Minimal common definitions shared between the real PS4 plugin and the
// macOS SDL tuning harness (see SDL/examples/input/07-gyro-aim-tuner).
// On PS4 (PLUGIN_PLATFORM_PS4 defined by the plugin's Makefile), logging
// goes through klog() from the GoldHEN SDK. Off-PS4, it falls back to a
// plain printf so gyro.c/config.c compile and log unmodified in the harness.
#pragma once

#include <stdint.h>

#ifdef PLUGIN_PLATFORM_PS4
#include <Common.h>  // klog() and friends, from the GoldHEN Plugins SDK
#else
#include <stdio.h>
#define klog(...) printf(__VA_ARGS__)
#endif

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
