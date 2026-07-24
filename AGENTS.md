# AGENTS.md

This file is for maintainers and coding agents working on this repository. It contains implementation details, design history, and debugging notes that are not necessary for end users of the plugin. Read this whole file before making changes to `source/gyro.c` or `include/gyro.h` — the bias/stick-drift estimators in particular have already been through several iterations that looked reasonable but were wrong; the "Dead ends already explored" section exists so those aren't repeated.

## Repo relationship: this repo + the SDL tuner

This plugin cannot be tested on real hardware without a PS4 + GoldHEN + FTP round-trip, which is slow. To make iteration fast, `source/gyro.c`, `source/config.c`, `include/gyro.h`, `include/config.h`, `include/pad.h`, and `include/gyro_common.h` are compiled **unmodified** into a second target: a macOS SDL3 GUI harness that lives in a **sibling repo** at `../SDL`, specifically `SDL/examples/input/07-gyro-aim-tuner/`. That harness:

- Reads a real DS4 controller's gyro/accel/buttons over USB/Bluetooth via `SDL_GetGamepadSensorData`/`SDL_GamepadButton` and packs them into a `ScePadData` struct.
- Calls `gyro_process_sample()` — the exact same function the real PS4 hooks call — every frame.
- Renders a live crosshair + on-screen HUD showing gyro/bias/yaw/pitch/flick/stillness state, and exposes every `GyroProfile` field as a keyboard/D-pad hotkey for live tuning.
- Has an `S` hotkey that prints the current profile as ready-to-paste `gyroaim.ini` text (including gain curves, which are NOT live-editable — edit those in the `.ini` and reload).

**Practical workflow: tune in the SDL harness first, copy the resulting `.ini` values to this repo's `gyroaim.ini` (or `gyroaim.ini.example`), then do a confidence-check pass on real PS4 hardware.** When you change a `GyroProfile` field or a bias/stick-drift algorithm in `gyro.c`/`gyro.h`, you almost always need to make matching edits in the tuner (`SDL/examples/input/07-gyro-aim-tuner/gyro-aim-tuner.c`) — new hotkey, new HUD line, new field in the `S`-key ini dump — or the new knob becomes untestable without a PS4. **The SDL repo has a hard rule (its own `AGENTS.md`/CONTRIBUTING): no AI-authored code contributions.** You may read/explore that repo, but when asked to change `gyro-aim-tuner.c`, keep edits minimal, mechanical, and clearly mirroring an existing pattern (hotkey plumbing, HUD line, ini-dump line) — do not use it as a place to redesign algorithms; algorithm changes belong in this repo's `gyro.c`, which the tuner just compiles unmodified.

## Build

The plugin is built with the provided Makefile.

### Requirements

- OpenOrbis PS4 toolchain (download the **release tarball**, not a git clone — the git repo excludes headers/lib stubs)
- GoldHEN Plugins SDK (regular git clone + `make`, produces `libGoldHEN_Hook.a` + `build/crtprx.o`)
- LLVM/Clang and lld on macOS (Apple's bundled clang doesn't work for this cross-compile target; `brew install llvm lld` — note `lld` is a separate formula, doesn't bundle with `llvm`)
- Rosetta 2 on Apple Silicon (the toolchain's `create-fself-macos` helper is x86_64-only)

### Environment variables

```sh
export OO_PS4_TOOLCHAIN=/path/to/OpenOrbis-PS4-Toolchain
export GOLDHEN_SDK=/path/to/GoldHEN_Plugins_SDK
```

### Build commands

```sh
make
```

Debug build (enables `klog` chatter for non-`final_printf`-guarded logging):

```sh
make DEBUG=1
```

Output binaries:
- `build/prx_final/gyro_aim.prx` (release — this is what ships)
- `build/prx_debug/gyro_aim.prx` (debug)

To build the Mac tuner (sibling repo, separate CMake build):

```sh
cmake --build /path/to/SDL/build --target input-gyro-aim-tuner
```

Run it with `./input-gyro-aim-tuner.app/Contents/MacOS/input-gyro-aim-tuner [path-to-ini]` (defaults to `gyroaim.ini` in the CWD).

**Always build BOTH targets after touching `gyro.c`/`gyro.h`/`config.c` and commit only after both succeed.**

## Project layout

```
include/
  pad.h            vendored ScePadData layout (from GoldHEN's gamepad_helper — do not
                    "fix" fields to match OpenOrbis's independently reverse-engineered
                    OrbisPadData; see "Known unknowns" below)
  config.h         vendored INI parser interface
  gyro_common.h    minimal logging/typedefs shared across platform code
  gyro.h           GyroProfile (all tunables), GyroDebug (HUD/diagnostic snapshot),
                    public gyro_* API
  platform.h       platform_set_lightbar() abstraction — the ONLY PS4-specific call
                    gyro.c makes; this is what lets gyro.c compile unmodified into the
                    Mac tuner (platform_mac.c stubs/no-ops it there)
source/
  main.c           plugin lifecycle, scePad hook installation, hook bodies
  gyro.c           ALL response-model / calibration / bias / stick-drift / flick /
                    hotkey logic — the file to read before changing behavior
  config.c         vendored INI parser implementation
  platform_ps4.c   real scePadSetLightBar call
Makefile           standalone cross-compile build; see Build above
gyroaim.ini.example  keep in sync with GyroProfile field additions/renames — every
                    config key in gyro.c's load_section() should appear here with its
                    default value
README.md         user-facing only (install, usage, config key reference).
                    Do NOT put implementation history/design rationale here — put it
                    in this file instead.
```

## Runtime architecture (PS4 side)

- `main.c` hooks `scePadRead`/`scePadReadState` via `HOOK32` from the GoldHEN SDK. **The hook bodies deliberately do NOT call through to the original function via `HOOK_CONTINUE`** — that crashed every target game on launch during initial development (those two functions are apparently too short/thin to safely re-trampoline; GoldHEN's own reference `gamepad_helper` plugin avoids the same pattern for the same reason). Instead, the hooks call `scePadReadExt`/`scePadReadStateExt` directly (separate, unhooked functions returning the same data) after neutralizing an internal privilege-check guard instruction via a raw byte patch (`Patcher_Install_Patch`, not `Detour`) — identical byte patch/offsets to `gamepad_helper`.
- Both hooks funnel every individual `ScePadData` sample (in order) through `gyro_process_sample()`, which mutates `pData->rightStick.x/y` in place before the game ever sees the struct.
- `gyro_process_sample()` is a single big state machine keyed off two orthogonal booleans checked at the top: `!g_calibrated` (still in startup calibration) and `aiming` (L2 held past `trigger_threshold`). There are effectively three code paths per sample: calibrating, aiming, not-aiming-post-calibration. Bias estimation and stick-drift estimation ONLY run in the third path.
- Calibration/settle/stationary/cooldown timing all use **sample counts**, never `ScePadData.timestamp` — that field's units/epoch are unverified from public headers, and both `scePadRead` (buffered/averaged) and `scePadReadState` (single latest) funnel through the same per-sample function, so counts scale consistently with real elapsed time regardless of which entry point a given game uses.

## Response model pipeline (per-sample, while aiming)

```
raw gyro (gx,gy,gz)
  -> NaN/Inf filtered to 0.0
  -> bias-corrected (subtract g_bias[axis], learned — see Bias estimation)
  -> yaw = (YawFromZ ? gz : gy) - bias, plus YawTiltWeight * (other axis - its bias)
     pitch = gx - bias[0]
  -> InvertX/InvertY sign flip
  -> process_vector(yaw, pitch, ...):
       treated as a 2D vector (mag, direction) the WHOLE way through, not per-axis,
       so diagonal movement feels identical to cardinal movement at the same
       rotational speed:
       1. flick suppression state machine (see below) picks effective_dead_zone
       2. vector deadzone with hysteresis (enter at 0.9x, exit at 1.2x dead_zone) on |mag|
       3. gain_lookup(mag) -> linearly-interpolated gain curve (GainRates_H/GainValues_H
          breakpoints) -> output_mag = gain * mag, capped by FlickSuppressionGainCap
          during suppression
       4. per-axis tanhf soft saturation on the OUTPUT vector (not per-axis on the input)
       5. DeadZoneBias ramp (see below) applied to the saturated vector's magnitude
       6. SensitivityH/V per-axis multiplier
       7. EMA (LowPassAlpha) toward this result while mag>0, OR interpolation-based
          damping toward 0 while mag==0 (DampingFactor) -- these are MUTUALLY EXCLUSIVE,
          never run in the same sample, so their effects never compound
  -> write_stick_uint8(): SUMS the result onto the physical stick's CURRENT reading
     (measured relative to the LEARNED stick center, not nominal 128 -- see Stick-center
     drift compensation), re-biased to nominal 128 for output since that's what the game
     expects, then hard-clamped to [0,255]
```

`g_float_stick_x/y` (the persistent EMA/damping state) resets to 0 on: L2 release (via damped decay, not instant), gyro runtime-disabled (L3+R3), profile disabled, and calibration/recalibration. Without this a resumed aim session could partially blend in a stale pre-pause value via the EMA's weighted carryover.

### DeadZoneBias: ramped floor, not a snap (`85c8493`)

`DeadZoneBias` guarantees a minimum output vector magnitude once the input clears the dead zone, so the game's own internal stick deadzone doesn't eat small deliberate movements. **This used to be a hard snap**: any sample whose natural saturated output fell between 0 and `dead_zone_bias` got scaled straight up to exactly `dead_zone_bias`. Because the gain curve's lowest breakpoint sits right at/near `dead_zone`, gain is already near-max the instant a sample clears the dead zone — so nearly every post-deadzone sample landed in that snap range, making the dead-zone boundary a **binary gate**: 0 output on one side, an instant ~10-20 stick-unit jump on the other. A bias residual of only 0.001-0.002 rad/s (tiny, unavoidable) was enough to consistently decide which side of that gate a borderline sample landed on — reading in-game as "one direction is easy to push, the other feels dead," even though the underlying bias error was negligible.

Fix: the floor now **ramps linearly** from 0 at the dead-zone boundary up to the full `dead_zone_bias` value as `mag` grows from `effective_dead_zone` to `effective_dead_zone * DeadZoneBiasRampScale` (default scale 4.0). A tiny shift in exactly where the boundary sits now only shifts where the ramp *starts*, not the size of an instant jump. `DeadZoneBiasRampScale <= 1.0` is degenerate (division by ~0) and falls back to the old snap behavior — don't set it that low.

### Post-flick suppression

A strong flick's deceleration/rebound is real physical motion (wrist/hand relaxing past neutral after a fast stop), but the gain curve's low-end boost and DeadZoneBias's floor both amplify that small rebound far more than the flick itself — the crosshair lands on target and visibly kicks back. State machine (all in `process_vector`):

- `is_flick_sample = (mag >= FlickMagThreshold) && !is_reversal`. A sample only counts as "the flick itself" (bypasses suppression, re-arms the cooldown in ITS direction) if it clears the magnitude threshold AND is not a reversal.
- `is_reversal`: dot product of this sample's normalized direction against the stored flick direction is `< FLICK_REVERSAL_DOT` (-0.5, i.e. >120° away). **Why this exists**: a fast deceleration snap-back can itself cross `FlickMagThreshold` — using magnitude alone to decide "is this a new flick" meant the rebound got misclassified as a brand-new legitimate flick, passed through unsuppressed, and re-armed the cooldown pointing the wrong way (`00baeea`).
- `suppress_this_sample = (!is_flick_sample) && cooling_down`, where `cooling_down` is snapshotted from `g_flick_suppress_timer > 0` **before** it's decremented that same sample. (Reading it post-decrement was an off-by-one that suppressed one fewer sample than `FlickSuppressionSamples` configured — `00baeea`.)
- While suppressing: `effective_dead_zone = dead_zone * FlickSuppressionDeadzoneScale`, and gain is capped at `FlickSuppressionGainCap`.
- `flick_suppress_timer`/`flick_dir_yaw`/`flick_dir_pitch` are cleared immediately on L2 release — a cooldown is only meaningful within the aiming session that produced it.

## Bias estimation (gyro rate bias)

Goal: `g_bias[3]` should track each gyro axis's true zero-rate offset so `raw - bias` is genuinely ~0 when the controller isn't moving. Two-phase:

1. **Startup calibration** (`!g_calibrated`, first `CALIB_SAMPLE_COUNT`=500 samples after plugin load or a manual recalibration): unconditional running average of every sample, no stillness check. `g_bias` AND `g_calibrated_bias` (the anchor — see total-deviation clamp below) are both set from this average. **Deliberately unconditional** — an earlier "reject moving samples" version required accel-near-gravity AND a gyro threshold clearing the DS4's natural ~0.05 rad/s Z-axis resting bias; on real hardware, accel data was sometimes zero (unreliable on some PS4 firmware) and together these two independent failure points meant calibration never completed at all (`6db3fbe`, `65f24c3`, `57980b8`). The simple unconditional average works and is accurate enough even with a few samples collected while moving, since 500 samples heavily dilutes any moving subset.

2. **Runtime bias tracking** (only while NOT aiming, i.e. L2 released, AND `bias_settle_samples` have elapsed since the aim-to-idle transition): a windowed, gated EMA nudge toward the observed reading. Gates, in order:
   - **Settle delay** (`bias_settle_samples`, default 50 ≈ 200ms @ 250Hz): armed on the aiming→not-aiming transition. Skips ALL bias logic during this window — the moments right after releasing L2 often look stationary but contain residual hand settling.
   - **Stillness = flat AND near-zero, both required, both compared against FIXED bounds, never the current bias estimate.** This is the single most important design constraint in this file — see "Dead ends already explored" for why.
     - Flat: per-axis sample-to-sample `|delta|`, smoothed via EMA (`bias_delta_ema_alpha`), must stay below `bias_stillness_delta_threshold`. Catches abrupt/jittery motion; on its own MISSES smooth constant-rate rotation (e.g. gently settling onto a target has almost no delta while genuinely moving).
     - Near-zero: raw `|gyro[axis]|` must stay below `bias_stillness_magnitude_threshold` (default 0.20 — deliberately a fixed physical bound above plausible DS4 zero-rate offset (~0.05-0.10) and below real intentional movement, NOT compared to the current bias). Catches the smooth motion the delta check misses.
     - Both gates must pass on ALL THREE axes before ANY axis accumulates — deliberately biased toward missing legitimate stillness (costs convergence speed only) over misreading real motion as stillness (corrupts the estimate).
   - **Windowed drift accumulator** (`g_drift_acc[3]`, reset whenever stillness breaks or a window is consumed): while stillness holds, accumulate `raw - bias` per axis. After `bias_stationary_samples` (default 60) consecutive still samples, normalize by window size (`avg_error = g_drift_acc[i] / g_stationary_timer`) — sensor noise is zero-mean and cancels out over the window, sustained slow motion does not. If `|avg_error| >= bias_drift_accum_threshold` (default 0.01) on ANY axis, the WHOLE window is rejected (no bias update on any axis) — this is diagnostic-only, see below, it does NOT get overridden by repetition.
   - If the window passes: `step = bias_alpha * (raw - bias)`, clamped to `±bias_max_correction_step` (default 0.05) — bounds a SINGLE step. Then **additionally** clamped so `g_bias[i]` stays within `±bias_max_total_deviation` (default 0.10) of `g_calibrated_bias[i]` — bounds CUMULATIVE drift across many small same-direction steps (e.g. a habitual resting-grip tilt that repeatedly passes the stillness gates) from walking the bias arbitrarily far from where it was actually calibrated over a long session. `g_calibrated_bias` is set once at calibration/recalibration time and is NOT touched by runtime updates — it's the anchor, not a moving target.
   - Window consumption: `g_stationary_timer` and `g_drift_acc` are BOTH reset after every window evaluation (pass or fail) — the next update needs a full fresh window, not just one more sample.

### Diagnostics (drift check failure)

If a window's drift check fails, this is **intentionally left as diagnostic-only** (see "Dead ends already explored" — a prior version applied a reduced/escalating correction anyway, and that was reverted as WRONG). What happens on failure:
- `g_drift_fail_consecutive` increments (reset to 0 on any successful window).
- `g_drift_fail_flash` is set to `DRIFT_FAIL_FLASH_SAMPLES` (60) — the lightbar flashes solid red (`platform_set_lightbar(handle, 255, 30, 30)`, bypassing the normal cached `set_lightbar` state) for that many samples, so a player watching the controller during idle gameplay gets live feedback that drift correction is being blocked.
- Every `BIAS_LOG_INTERVAL` (60) consecutive failures, a `log_info` line reports per-axis `avg_error`, the threshold, and the consecutive-failure count.
- **No bias correction is applied.** A consistently-failing axis means the "stationary-looking" signal has a real, consistent non-zero average — exactly what the check exists to catch. The fix for a persistently-failing axis is to **manually recalibrate in a genuinely neutral hand position** (touchpad-hold hotkey), not to have the code auto-override the check.

## Physical stick drift compensation (`78b7bfd`)

Separate subsystem, same two-gate design pattern as gyro bias, for a different sensor/failure mode: DS4/DS5 analog stick potentiometers commonly rest away from nominal center (128,128), sometimes drifting further as the controller warms up mid-session. `write_stick_uint8()` sums gyro output onto the stick's CURRENT reading — if that reading never actually rests at 128 when untouched, the drift is a constant phantom push in one direction, always, independent of the gyro.

- `g_stick_center[2]` (init to nominal 128,128) is the learned rest position, used by `write_stick_uint8()` as the zero-reference instead of the nominal constant.
- Runs ONLY while not aiming — that's the only time `pData->rightStick` is guaranteed to still be the raw untouched hardware reading (during aiming, `write_stick_uint8` has already overwritten it).
- Gates: flatness (per-axis sample-to-sample delta EMA vs `StickStillnessDeltaThreshold`, default 1.0 stick unit) AND plausibility (`|raw - 128| <= StickMaxPlausibleDrift`, default 30, measured from the FIXED nominal 128, never the current `g_stick_center` estimate — same deadlock-avoidance reasoning as the gyro bias magnitude gate). A deliberate held stick deflection is typically much larger than any real pot drift and is naturally excluded by this bound.
- After `StickStationarySamples` (default 60) consecutive flat+plausible samples: `step = StickCenterAlpha * (raw - center)`, clamped by `StickCenterMaxCorrectionStep` (0.5), then clamped so `g_stick_center` stays within `±StickCenterMaxDeviation` (40.0) of the nominal 128 — same total-deviation safety pattern as the gyro bias clamp.
- **Reset only in `gyro_state_init()` (plugin load), NOT in `start_recalibration()`** (the touchpad-hold hotkey). Rationale: manual recalibration is about gyro RATE bias, unrelated to the physical stick's rest position — resetting a good stick-center estimate on every gyro recalibration would force reconvergence from nominal 128 each time, reintroducing the phantom-push asymmetry for no gyro-related reason.

## Debugging / diagnostics available

- **Lightbar** (`platform_set_lightbar`, real call on PS4 / no-op on Mac tuner): amber = startup calibrating, green = aiming (L2 held past threshold), blue = idle, **red flash (60 samples) = a bias drift-check window was just rejected**. The red flash bypasses the normal cached `LightbarState` machine and force-invalidates it (`g_lightbar_state = LB_UNSET`) afterward so the next non-flash frame reasserts the correct color.
- **`GyroDebug` struct** (`gyro_get_debug()`), rendered live by the Mac tuner HUD:
  - `gyro[3]` — raw gx/gy/gz this sample
  - `bias[3]` — current `g_bias`
  - `yaw`/`pitch` — post-bias, post-invert, pre-deadzone values (only meaningful while aiming)
  - `flick_suppress` — 1 while a sample is being suppressed post-flick
  - `stillness` — 1 if the gyro bias stillness gates (flat+near-zero) passed THIS sample. Explicitly reset to 0 both when not-aiming-but-drift-disabled and when transitioning into aiming — otherwise it would freeze at whatever was true the instant before L2 was pressed and read as "bias tracking active" for the whole aim session (bug fixed in `316a087`).
  - `bias_active` — running count of the current stationary window's sample count (0 when not accumulating) — lets you watch a window fill toward `BiasStationarySamples` live.
- **klog `log_info` lines** (retrieve via GoldHEN's log viewer/FTP on PS4, or stdout on the Mac tuner):
  - `"gyro calibration complete: bias=[...]"` — once, at end of startup calibration.
  - `"bias: X raw=... bias=... err=... | Y ... | Z ... | yaw=... pitch=... mag=..."` — every `BIAS_LOG_INTERVAL` (60) SUCCESSFUL bias updates.
  - `"bias: drift check FAILED (avg error X=... Y=... Z=..., threshold=..., consecutive=...)"` — every 60 consecutive FAILED windows.

If you're debugging an in-game directional-drift complaint, the diagnostic order is: (1) check if red lightbar flashes happen during idle — if never, stillness gates aren't passing at all (check `bias_stillness_magnitude_threshold` against real hand-held tremor, not desk-rest tremor); (2) if flashes happen and don't stop, an axis has a persistent non-zero average that needs a manual recalibration in a neutral position, not a config tweak; (3) if no flashes and drift still happens, suspect the DeadZoneBias ramp/gain curve/stick-center path instead of gyro bias.

## Config keys reference (all loaded in `gyro.c`'s `load_section()`, all defaulted in `gyro_profile_set_defaults()` — keep `gyroaim.ini.example` in sync with both)

| Key | Field | Default | Notes |
|---|---|---|---|
| `Enabled` | `enabled` | true | |
| `DeadZone` | `dead_zone` | 0.02 | rad/s, vector magnitude |
| `DeadZoneBias` | `dead_zone_bias` | 20 | stick units, ramped floor (see above) |
| `DeadZoneBiasRampScale` | `dead_zone_bias_ramp_scale` | 4.0 | must be > 1.0 |
| `TriggerThreshold` | `trigger_threshold` | 250 | L2 analog 0-255 |
| `GainRates_H`/`GainValues_H` | `gain_rates_h[]`/`gain_values_h[]` | see `kDefaultGainRates/Values` | only ONE curve now (no `_V` variant — removed, was parsed but unused) |
| `LowPassAlpha` | `lowpass_alpha` | 0.5 | output EMA while moving |
| `DampingFactor` | `damping_factor` | 0.12 | decay-to-zero while in deadzone, also decay rate on L2 release |
| `SaturationStrength` | `saturation_strength` | 2.0 | tanhf soft-clamp strength |
| `InvertX`/`InvertY` | `invert_x`/`invert_y` | false | |
| `YawFromZ` | `yaw_from_z` | false | swap yaw source .y <-> .z |
| `YawTiltWeight` | `yaw_tilt_weight` | 0.0 | blend in the non-primary axis |
| `DriftCorrectionEnabled` | `drift_correction_enabled` | true | master switch for gyro bias runtime tracking |
| `BiasAlpha` | `bias_alpha` | 0.01 | per-window EMA blend |
| `BiasMaxCorrectionStep` | `bias_max_correction_step` | 0.05 | rad/s, single-step clamp |
| `BiasStationarySamples` | `bias_stationary_samples` | 60 | window size |
| `BiasDeltaEmaAlpha` | `bias_delta_ema_alpha` | 0.2 | flatness-gate smoothing |
| `BiasStillnessDeltaThreshold` | `bias_stillness_delta_threshold` | 0.01 | flatness-gate threshold |
| `BiasStillnessMagnitudeThreshold` | `bias_stillness_magnitude_threshold` | 0.20 | near-zero gate, fixed bound |
| `BiasSettleSamples` | `bias_settle_samples` | 50 | post-L2-release delay |
| `BiasDriftAccumThreshold` | `bias_drift_accum_threshold` | 0.01 | rad/s, normalized window-average gate |
| `BiasMaxTotalDeviation` | `bias_max_total_deviation` | 0.10 | rad/s, cumulative clamp around calibrated anchor |
| `SensitivityH`/`SensitivityV` | `sensitivity_h`/`sensitivity_v` | 1.0 | simple output multiplier |
| `FlickMagThreshold` | `flick_mag_threshold` | 1.00 | rad/s |
| `FlickSuppressionSamples` | `flick_suppression_samples` | 12 | cooldown length |
| `FlickSuppressionDeadzoneScale` | `flick_suppression_deadzone_scale` | 5.0 | |
| `FlickSuppressionGainCap` | `flick_suppression_gain_cap` | 30.0 | |
| `StickDriftCorrectionEnabled` | `stick_drift_correction_enabled` | true | master switch for stick-center tracking |
| `StickCenterAlpha` | `stick_center_alpha` | 0.01 | |
| `StickCenterMaxCorrectionStep` | `stick_center_max_correction_step` | 0.5 | stick units |
| `StickStationarySamples` | `stick_stationary_samples` | 60 | |
| `StickDeltaEmaAlpha` | `stick_delta_ema_alpha` | 0.2 | |
| `StickStillnessDeltaThreshold` | `stick_stillness_delta_threshold` | 1.0 | stick units |
| `StickMaxPlausibleDrift` | `stick_max_plausible_drift` | 30.0 | stick units, fixed bound from nominal 128 |
| `StickCenterMaxDeviation` | `stick_center_max_deviation` | 40.0 | stick units, cumulative clamp |

## Dead ends already explored — do not redo these

These looked like reasonable fixes at the time and were tried, then reverted or replaced. Re-reading git history before "fixing" one of these areas again will save time.

1. **Comparing raw gyro/stick readings to the CURRENT learned bias/center estimate, instead of a fixed physical bound, to decide stillness.** Creates a deadlock: if the initial calibration missed on some axis, that axis's error-from-estimate can never shrink, because being close to a WRONG estimate was the precondition for moving it closer. Both the gyro bias estimator and the stick-center estimator deliberately gate stillness against fixed physical bounds (`bias_stillness_magnitude_threshold`, `stick_max_plausible_drift`) instead (`5d7d71f`, `78b7bfd`).
2. **Rejecting calibration samples based on accel-near-gravity or a gyro stillness threshold.** PS4 accel data was unreliable/zero on some firmware, and the DS4's natural resting gyro bias (~0.05 rad/s on Z) is close enough to reasonable thresholds that both checks together sometimes never converged, hanging calibration forever. Startup calibration is deliberately a simple unconditional 500-sample average (`6db3fbe`, `65f24c3`).
3. **Using magnitude alone to decide "is this sample the flick itself" during a flick-suppression cooldown.** A deceleration snap-back/rebound can itself cross the flick magnitude threshold, so magnitude-only classification let the rebound bypass suppression as a "new flick" and re-arm the cooldown backwards. Fixed by adding a direction-reversal check (dot product vs the armed flick's direction) (`00baeea`).
4. **Reading the flick-suppression cooldown timer AFTER decrementing it, to decide whether to suppress the current sample.** Off-by-one: the last sample of the cooldown window (timer entering at 1, decrementing to 0) was never suppressed, so `FlickSuppressionSamples=N` only ever suppressed N-1 samples. Fixed by snapshotting the pre-decrement state (`00baeea`).
5. **Escalating/overriding the bias drift-accumulator check after repeated consecutive failures** (apply a reduced-alpha correction anyway, then ramp to full alpha after 3+ failures). This was tried (`4a3a7cb`) and explicitly reverted (`316a087`): a consistently-failing window means the axis has a real, consistent non-zero average during "stillness" (e.g. habitual resting-grip tilt) — exactly what the check is designed to catch and refuse. Auto-overriding it after enough failures defeats its purpose and would eventually learn the habitual tilt in at full strength. The correct response to persistent failures is a visible diagnostic (red lightbar flash + log line, both still present) that tells the PLAYER to manually recalibrate in a neutral position — not a silent code-side override.
6. **Snapping DeadZoneBias straight to its configured value for any sample that clears the dead zone.** Created a binary gate at the dead-zone boundary where a tiny (0.001-0.002 rad/s) bias residual reliably decided which side a borderline sample landed on, reading in-game as directional asymmetry even with near-zero actual bias error. Replaced with a linear ramp (`85c8493`).
7. **Calling through to the original `scePadRead`/`scePadReadState` via `HOOK_CONTINUE` after hooking them.** Crashed every target game immediately on launch — these functions are apparently too thin to safely re-trampoline. Call `scePadReadExt`/`scePadReadStateExt` directly instead, after a byte-patch to bypass their internal caller-privilege check (same approach as GoldHEN's reference `gamepad_helper` plugin).
8. **Raising `bias_stillness_magnitude_threshold` from 0.20 to 0.50 to account for hand-held (not desk-rest) tremor** (`e46d96d`). This was tried when directional drift was first reported in-game but NOT reproduced in the tuner even hand-held — meaning the threshold was not actually the root cause. It was reverted back to 0.20 once `BiasMaxTotalDeviation`/`bias_max_total_deviation` (which addresses the REAL cause — small habitual movements walking the bias arbitrarily far, not the stillness gate being too tight) was added in `85c8493`. If directional drift resurfaces, check the total-deviation clamp and the drift-fail lightbar/logs before touching this threshold again.

## Known unknowns (unverified against non-public Sony headers)

- **`ScePadData` trailing field layout**: uses GoldHEN's `gamepad_helper` reference layout (`include/pad.h`), not OpenOrbis's independently reverse-engineered `OrbisPadData` (they disagree on some trailing/reserved fields, but agree on everything this plugin reads/writes: `buttons`, `leftStick`, `rightStick`, `analogButtons`, `angularVelocity`, `acceleration`, `connected`).
- **Gyro axis sign/mapping conventions**: which physical rotation produces positive vs negative `angularVelocity.x/y/z`, and which axis is "yaw" for a given hold/aim style, is empirical/untested against hardware docs. `InvertX`/`InvertY`/`YawFromZ`/`YawTiltWeight` are the escape hatches; pitch is hardcoded to `.x` (no `PitchFromZ` exists yet — same pattern would apply if ever needed).
- **`ScePadData.timestamp` units/epoch**: unverified, hence all timing in `gyro.c` uses sample counts (see "Runtime architecture" above).

## Config file behavior

- Lives at `/data/GoldHEN/gyroaim.ini` on console.
- `[default]` applies to all games; a `[TITLE_ID]` section overlays/overrides `[default]` for that specific title (visible via `sys_sdk_proc_info` at `plugin_load`, logged to klog).
- An unconfigured game still gets gyro-aim via `[default]` (safety decision — restrict testing to single-player/offline titles until confident in the hook, since anti-cheat in online titles could flag it).
- Missing keys keep whatever `gyro_profile_set_defaults()` established — `gyro_profile_load()` returns false only if the file itself couldn't be opened, in which case the profile is left at hardcoded defaults.

## Notes for contributors

- Keep user-facing documentation (`README.md`) focused on setup and usage — no implementation history, internal rationale, or "why we tried X and reverted it." That belongs here.
- When adding/renaming/removing a `GyroProfile` field: update `gyro.h` (struct + field comment), `gyro_profile_set_defaults()`, `load_section()`, `gyroaim.ini.example`, AND the sibling SDL tuner (hotkey, HUD line, `print_profile_as_ini()`) — see "Repo relationship" above. Also update the config table in this file.
- Before "fixing" stillness detection, bias convergence, or dead-zone floor behavior, read "Dead ends already explored" above in full — several plausible-looking approaches in this exact area have already been tried and reverted.
- Build BOTH the PS4 plugin and the Mac tuner after any `gyro.c`/`gyro.h`/`config.c` change, before committing.
