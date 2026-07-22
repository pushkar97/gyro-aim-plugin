# gyro-aim for PS4

gyro-aim is a GoldHEN plugin that adds gyro-assisted camera aiming to PS4 games that do not have native gyro support. It hooks the gamepad input path and translates the DualShock 4 gyroscope into right-stick movement so you can aim by moving the controller.

## What it does

- Gives you gyro aiming in games that only support a normal stick.
- Uses the controller's gyroscope and accelerometer to reduce drift and keep aiming stable.
- Works while L2 is held, and can be disabled temporarily with L3+R3 if needed.
- Supports per-title config overrides so you can tune one game without affecting others.

> This is an experimental plugin. Use it cautiously in online or anti-cheat titles.

## Requirements

- A jailbroken PS4 running GoldHEN
- A DualShock 4 or compatible controller with gyro support
- FTP or another way to copy files to the console

## Installation on a jailbroken PS4

1. Copy the plugin binary to the GoldHEN plugin folder on the console:
   - Source file in this repo: build/prx_final/gyro_aim.prx
   - Destination on the PS4: /data/GoldHEN/plugins/gyro_aim.prx
2. Add the plugin to /data/GoldHEN/plugins.ini:

```ini
[default]
/data/GoldHEN/plugins/gyro_aim.prx
```

3. Copy gyroaim.ini.example to /data/GoldHEN/gyroaim.ini and edit it.
4. Launch a game.

## How to use it

- Hold L2 to activate gyro aim.
- Move the controller to steer the camera.
- Release L2 to stop aiming.
- Press L3+R3 to toggle gyro aiming on or off at runtime.
- Hold the touchpad for about one second to force recalibration if the controller was moved while the plugin loaded.

## Configuration

The config file is an INI file. A [default] section applies to all games. A per-title section such as [CUSA00001] overrides the defaults for that specific game.

A good starting point is the bundled example file, gyroaim.ini.example.

```ini
[default]
Enabled = true
DeadZone = 0.02
DeadZoneBias = 20
TriggerThreshold = 250
GainRates_H = 0.02 0.05 0.15 0.40 1.00 100.0
GainValues_H = 140 100 70 50 35 25
GainRates_V = 0.02 0.05 0.15 0.40 1.00 100.0
GainValues_V = 140 100 70 50 35 25
LowPassAlpha = 0.5
DampingFactor = 0.12
SaturationStrength = 2.0
InvertX = false
InvertY = false
YawFromZ = true
YawTiltWeight = 0.3
DriftCorrectionEnabled = true
BiasAlpha = 0.01
BiasMaxCorrectionStep = 0.05
BiasStationarySamples = 60
BiasDeltaEmaAlpha = 0.2
BiasStillnessDeltaThreshold = 0.01
BiasStillnessMagnitudeThreshold = 0.20
BiasSettleSamples = 50
BiasDriftAccumThreshold = 0.01
SensitivityH = 1.0
SensitivityV = 1.0
FlickMagThreshold = 0.40
FlickSuppressionSamples = 30
FlickSuppressionDeadzoneScale = 5.0
FlickSuppressionGainCap = 30.0
```

### Available knobs

- Enabled: Turns the plugin on or off.
- DeadZone: Minimum angular velocity before the gyro output starts to matter. Increase it if the camera feels too twitchy.
- DeadZoneBias: Adds a small floor so very small motions do not disappear entirely. Raise it if the aim feels too weak near neutral.
- TriggerThreshold: How hard L2 must be pressed before gyro aim engages. The value is from 0 to 255.
- GainRates_H / GainValues_H: Horizontal (yaw) gain curve. The rates are breakpoints and the values are the gain applied at each breakpoint. Higher values mean stronger horizontal aiming.
- GainRates_V / GainValues_V: Same idea as the horizontal curve, but for vertical (pitch) aiming.
- LowPassAlpha: Smoothing strength while the controller is actively moving. Higher values make the response feel smoother.
- DampingFactor: How quickly the output returns to center when the gyro is inside the dead zone. Lower values feel snappier; higher values feel softer.
- SaturationStrength: Controls how quickly the output approaches its maximum. Higher values make the response feel more aggressive at the edges.
- InvertX / InvertY: Reverse the sign of the corresponding axis if aiming feels inverted.
- YawFromZ: Uses the Z gyro axis for horizontal aiming instead of the default axis. Turn this on if horizontal motion feels wrong.
- YawTiltWeight: Blends in some contribution from the non-primary yaw axis. Useful if aiming feels too narrow or too axis-locked.
- DriftCorrectionEnabled: Enables the plugin's automatic drift-correction behavior.
- BiasAlpha: How quickly the plugin learns a steady bias offset. Smaller values change it more gradually.
- BiasMaxCorrectionStep: Caps how much correction can be applied per sample.
- BiasStationarySamples: How many quiet samples are needed before bias correction becomes confident.
- BiasDeltaEmaAlpha: Smoothing for the stillness detector that looks at sample-to-sample gyro change.
- BiasStillnessDeltaThreshold: How much change is considered still enough.
- BiasStillnessMagnitudeThreshold: How close the signal magnitude must be to the expected resting value before the plugin considers the controller still.
- BiasSettleSamples: How long the plugin waits after a transition before it starts applying bias correction aggressively.
- BiasDriftAccumThreshold: Threshold for deciding when a small, sustained drift should be corrected.
- SensitivityH / SensitivityV: Overall multiplier for horizontal and vertical response.
- FlickMagThreshold: Threshold for detecting a fast flick. Once crossed, the plugin briefly reduces rebound.
- FlickSuppressionSamples: How long the post-flick suppression lasts.
- FlickSuppressionDeadzoneScale: Makes the dead zone wider after a flick to reduce kickback.
- FlickSuppressionGainCap: Caps the gain briefly after a flick so the camera does not overshoot.

### Per-title overrides

Create a section with the game's title ID to override the default values for one title only. For example:

```ini
[CUSA00001]
GainRates_H = 0.02 0.05 0.15 0.40 1.00 100.0
GainValues_H = 80 55 40 30 20
GainRates_V = 0.02 0.05 0.15 0.40 1.00 100.0
GainValues_V = 80 55 40 30 20
```

## Troubleshooting

- If aiming feels too sensitive, increase DeadZone or lower the gain values.
- If aiming feels too weak, lower DeadZone or raise the gain values.
- If horizontal movement feels reversed, try InvertX or YawFromZ.
- If the camera drifts, try enabling or tuning the bias-related settings.
- If the plugin does not seem to work, make sure the PRX is loaded by GoldHEN and that the config file is present at /data/GoldHEN/gyroaim.ini.

## For developers

Build and maintenance notes live in AGENTS.md.
