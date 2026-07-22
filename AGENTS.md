# AGENTS.md

This file is for maintainers and coding agents working on this repository. It contains implementation details and build notes that are not necessary for end users of the plugin.

## Build

The plugin is built with the provided Makefile.

### Requirements

- OpenOrbis PS4 toolchain
- GoldHEN Plugins SDK
- LLVM/Clang and lld on macOS
- Rosetta 2 on Apple Silicon if the toolchain's macOS helper binaries are used

### Environment variables

```sh
export OO_PS4_TOOLCHAIN=/path/to/OpenOrbis-PS4-Toolchain
export GOLDHEN_SDK=/path/to/GoldHEN_Plugins_SDK
```

### Build commands

```sh
make
```

Debug build:

```sh
make DEBUG=1
```

The output binaries are written to the build directory, including:

- build/prx_final/gyro_aim.prx
- build/prx_debug/gyro_aim.prx

## Architecture overview

- source/main.c: plugin lifecycle, hook installation, and pad input interception.
- source/gyro.c: gyro processing, calibration, bias correction, response model, drift handling, and hotkey behavior.
- source/config.c and include/config.h: INI parsing and config loading.
- include/pad.h: vendored PS4 pad data structure layout used by the hook implementation.
- include/platform.h and source/platform_ps4.c: platform-specific lightbar and console integration.

## Important implementation notes

- The plugin hooks scePadRead and scePadReadState and injects modified stick values before the game reads them.
- The response model is velocity-based rather than integrating gyro motion into an absolute cursor position.
- The plugin uses a gain curve, dead zone handling, EMA/damping, soft saturation, and post-flick suppression.
- Calibration and drift correction rely on sample counts rather than the ScePadData timestamp field because the timestamp units are not verified from public headers.
- The hook implementation intentionally avoids calling through the original scePadRead/scePadReadState trampoline path because that caused crashes on launch during development.

## Config file behavior

- The config file lives at /data/GoldHEN/gyroaim.ini on the console.
- [default] applies to all games.
- A [TITLE_ID] section overrides the defaults for a specific title.
- The plugin defaults to enabling gyro aim unless explicitly disabled in the config.

## Notes for contributors

- Keep user-facing documentation focused on setup and usage.
- Put implementation history, internal design decisions, and build details in this file rather than the public README.
- When changing config semantics, update both gyroaim.ini.example and the documentation in the public README.
