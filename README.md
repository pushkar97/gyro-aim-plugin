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
  `scePadRead`/`scePadReadState`** — it does not; see "Fixed: crash on game
  launch" below.
- **`ScePadData.timestamp` units/epoch** — not verified, so calibration and
  drift-correction timing intentionally use sample counts instead of a
  wall-clock duration (see comment at the top of `source/gyro.c`).

## Fixed: crash on game launch (HOOK_CONTINUE)

First real-hardware test crashed every target game immediately on launch.
Root cause: the initial implementation called through to the real
`scePadRead`/`scePadReadState` via `HOOK_CONTINUE` (Detour's trampoline call-
through). `scePadRead`/`scePadReadState` are almost certainly too short/thin
to safely re-trampoline (likely just a stub jumping straight into a syscall
handler) — patching a jump into them and then calling back through corrupts
adjacent code. This is exactly why `gamepad_helper` avoids that specific
pattern for these two functions (its own doc comments don't say why, but the
crash makes it obvious in retrospect).

Fix: still hook `scePadRead`/`scePadReadState` with `HOOK32` (installing the
hook itself is fine — `gamepad_helper` does the same), but the hook bodies no
longer call through to the original. Instead they call
`scePadReadExt`/`scePadReadStateExt` directly (separate, unhooked functions
that return the same data) after neutralizing an internal privilege-check
guard instruction via a raw byte patch (`Patcher_Install_Patch`, not
`Detour`) — identical byte patch and offsets to `gamepad_helper`. See
`source/main.c`.

## Prerequisites

Tested and working end-to-end on macOS (Apple Silicon) as of this writing.

1. **OpenOrbis toolchain — download the release, do NOT `git clone` the repo.**
   The git repo intentionally excludes the full header/library-stub set (to
   avoid bloating it); those only ship in the GitHub Releases tarball.
   ```sh
   curl -sL -o /tmp/oo-toolchain.tar.gz \
     https://github.com/OpenOrbis/OpenOrbis-PS4-Toolchain/releases/latest/download/toolchain-llvm-18.tar.gz
   mkdir /tmp/oo-extract && tar -xzf /tmp/oo-toolchain.tar.gz -C /tmp/oo-extract
   mv /tmp/oo-extract/OpenOrbis/PS4Toolchain /path/to/OpenOrbis-PS4-Toolchain
   export OO_PS4_TOOLCHAIN=/path/to/OpenOrbis-PS4-Toolchain
   ```

2. **GoldHEN_Plugins_SDK — regular git clone is fine here.**
   ```sh
   git clone https://github.com/GoldHEN/GoldHEN_Plugins_SDK.git
   export GOLDHEN_SDK=/path/to/GoldHEN_Plugins_SDK
   ```
   Then build it (produces `libGoldHEN_Hook.a` + `build/crtprx.o`, which this
   plugin's Makefile links against):
   ```sh
   cd $GOLDHEN_SDK && make
   ```

3. **clang + lld, on macOS via Homebrew.** Apple's bundled clang doesn't work
   for this target; you need upstream LLVM. Note `lld` is a *separate*
   Homebrew formula from `llvm` (it doesn't bundle `ld.lld`):
   ```sh
   brew install llvm lld
   ```
   On Apple Silicon, Homebrew's `llvm`/`lld` install under `/opt/homebrew`,
   not the `/usr/local/opt/llvm` path some upstream Makefiles hardcode (an
   Intel-Homebrew-era assumption). This plugin's own `Makefile` auto-detects
   the right prefix via `brew --prefix llvm`/`brew --prefix lld` with
   fallbacks. If you build `GoldHEN_Plugins_SDK` itself and hit
   `/usr/local/opt/llvm/bin/clang: No such file or directory`, override on
   the command line instead of patching its Makefile:
   ```sh
   make CC=$(brew --prefix llvm)/bin/clang \
        CCX=$(brew --prefix llvm)/bin/clang++ \
        LD=$(brew --prefix lld)/bin/ld.lld \
        AR=$(brew --prefix llvm)/bin/llvm-ar
   ```

4. **Rosetta 2, on Apple Silicon.** The toolchain's `create-fself` binary for
   macOS (`bin/macos/create-fself-macos`) is x86_64-only.
   ```sh
   softwareupdate --install-rosetta --agree-to-license
   ```

```sh
export OO_PS4_TOOLCHAIN=/path/to/OpenOrbis-PS4-Toolchain
export GOLDHEN_SDK=/path/to/GoldHEN_Plugins_SDK
make
```

Produces `build/prx_final/gyro_aim.prx`. `make DEBUG=1` builds the
`__FINAL__=0` variant (`build/prx_debug/gyro_aim.prx`) with `klog` chatter
enabled for every code path guarded by non-`final_printf`-style logging.

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
