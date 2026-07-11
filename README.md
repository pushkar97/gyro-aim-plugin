# gyro-aim (GoldHEN plugin)

Adds gyro-to-camera aim assist to PS4 games that have no native gyro
support, by hooking `scePadRead`/`scePadReadState` and injecting
gyro-derived rotation directly onto the right analog stick's X/Y values
before the game reads them.

Built for a jailbroken PS4 running [GoldHEN](https://github.com/GoldHEN/GoldHEN).

## Status

Early scaffold, unverified on real hardware yet. See "Known unknowns" below
before relying on this.

## How it works

- Hooks both `scePadRead` (buffered) and `scePadReadState` (single latest),
  since it's not yet known which entry point any given target game uses.
- Gyro rotation is mapped as a **velocity**, not integrated into an absolute
  position: each sample, `(gyro_rate - bias) * sensitivity` is added directly
  onto the right stick's current X/Y deflection, then clamped. This mirrors
  how native gyro-aim implementations (e.g. Splatoon, JoyShockMapper) work,
  and avoids the drift-accumulation/NaN-propagation issues integration-based
  approaches run into (see the sibling `SDL/` gyro visualizer experiment for
  a worked example of exactly those bugs).
- Gyro only contributes while L2 is fully pressed (`analogButtons.l2 >=
  TriggerThreshold`, default 250/255).
- One-time startup calibration (first ~500 samples) plus continuous
  stillness-based drift correction (via accelerometer magnitude ≈ 9.80665)
  keep the camera from creeping during long aim holds.
- L3+R3 toggles gyro on/off at runtime (for vehicle sections, menus, etc. —
  see the config section below for per-title profiles as the primary
  mechanism, this hotkey is the manual escape hatch).
- Holding the touchpad for ~1s forces recalibration (useful if the pad was
  moving when the plugin loaded).
- The lightbar shows state: amber = calibrating, green = gyro actively
  contributing (L2 held), blue = inactive.

## Known unknowns (please read before trusting this)

These were flagged explicitly during design because they could not be
verified against Sony's own (non-public) headers — only independent
reverse-engineering efforts (OpenOrbis, and GoldHEN's own `gamepad_helper`
plugin) exist, and they disagree on some details:

- **`ScePadData` trailing field layout** — this plugin uses the layout from
  GoldHEN's official `gamepad_helper` example plugin (`include/pad.h`,
  vendored here), since it's proven working in a shipped plugin. OpenOrbis's
  independently reverse-engineered `OrbisPadData` disagrees on some trailing
  fields (extension unit data shape, a few reserved/count fields) — this
  doesn't affect the fields we actually read/write (`buttons`, `leftStick`,
  `rightStick`, `analogButtons`, `angularVelocity`, `acceleration`,
  `connected`), which both sources agree on.
- **Gyro axis sign conventions** — which physical rotation direction
  produces a positive vs. negative `angularVelocity.x/y/z` on the DS4, and
  whether that matches "expected" stick-right/stick-up conventions, is
  untested. `InvertX`/`InvertY` config options exist as an escape hatch;
  expect to flip them empirically on first real test.
- **Whether `HOOK_CONTINUE` cleanly calls through to the real
  `scePadRead`/`scePadReadState`** — the reference `gamepad_helper` plugin
  avoids this specific pattern for these two functions (it instead patches
  and calls `scePadReadExt`/`scePadReadStateExt` directly, for unrelated
  reasons — it wanted extension-unit data). This plugin uses the more
  standard `HOOK_CONTINUE` pattern (as `gamepad_helper` itself does for
  `scePadSetVibration`). If it misbehaves, falling back to the
  Ext-function-patch approach from `gamepad_helper` is the documented
  alternative.
- **`ScePadData.timestamp` units/epoch** — not verified, so calibration and
  drift-correction timing intentionally use sample counts instead of a
  wall-clock duration (see comment at the top of `source/gyro.c`).

## Prerequisites

- [OpenOrbis-PS4-Toolchain](https://github.com/OpenOrbis/OpenOrbis-PS4-Toolchain),
  cloned locally, with `OO_PS4_TOOLCHAIN` pointing at it.
- [GoldHEN_Plugins_SDK](https://github.com/GoldHEN/GoldHEN_Plugins_SDK),
  cloned locally, with `GOLDHEN_SDK` pointing at it.
- clang/lld (on macOS: `brew install llvm`, matching the paths the Makefile
  expects — see `Makefile` if your `llvm` isn't at `/usr/local/opt/llvm`).

```sh
export OO_PS4_TOOLCHAIN=/path/to/OpenOrbis-PS4-Toolchain
export GOLDHEN_SDK=/path/to/GoldHEN_Plugins_SDK
make
```

Produces `build/prx_final/gyro_aim.prx`.

## Installing on console

1. Copy `build/prx_final/gyro_aim.prx` to `/data/GoldHEN/plugins/gyro_aim.prx`
   on the console (via FTP).
2. Add it to `/data/GoldHEN/plugins.ini`:
   ```ini
   [default]
   /data/GoldHEN/plugins/gyro_aim.prx
   ```
3. Copy `gyroaim.ini.example` to `/data/GoldHEN/gyroaim.ini` and edit as
   needed (see below).
4. Debug logs go through `klog`; retrieve via whatever GoldHEN log
   viewing/FTP mechanism you use for other plugins.

## Config (`/data/GoldHEN/gyroaim.ini`)

INI file, `[default]` section applies to every game; a `[TITLE_ID]` section
(e.g. `[CUSA00001]`) overlays/overrides `[default]` for that specific title.
Per the safety decision made during design, an unconfigured game still gets
gyro-aim via `[default]` — restrict testing to single-player/offline titles
until you're confident in the hook's behavior, since anti-cheat in online
titles could flag it.

```ini
[default]
Enabled = true
SensitivityH = 40.0
SensitivityV = 40.0
DeadZone = 0.02
DeadZoneBias = 20
TriggerThreshold = 250
InvertX = false
InvertY = false

[CUSA00001]
; Example: this specific title needs lower sensitivity and gyro disabled
; while driving is handled manually via the L3+R3 toggle hotkey, not here.
SensitivityH = 25.0
SensitivityV = 25.0
```

See `gyroaim.ini.example` for a ready-to-copy version of the above.

## Project layout

```
include/    pad.h (vendored ScePadData), config.h (vendored INI parser),
            common.h (minimal logging/typedefs), gyro.h
source/     main.c (plugin lifecycle + scePad hooks),
            gyro.c (calibration/drift/mapping logic),
            config.c (vendored INI parser implementation)
Makefile    standalone build, see Prerequisites above
```

`pad.h` and `config.c`/`config.h` are vendored (with attribution comments at
the top of each file) from GoldHEN's official `gamepad_helper` example
plugin, since it's a proven, working reference for exactly the struct layout
and config-parsing needs this plugin has.
