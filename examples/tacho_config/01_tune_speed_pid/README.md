# Speed Loop Tuning Procedure

Interactive calibration and tuning of the speed loop. Upload with your calibrated defines (see the parent README) and open the Serial Monitor at 115200 baud. The **Serial Plotter** shows `MeasuredSpeedRPM` and `DesiredSpeedRPM` in speed mode: you are shaping `MeasuredSpeedRPM` onto `DesiredSpeedRPM`.

## Commands

| Command | Effect |
| --- | --- |
| `SPEED=<rpm>` | Set the desired speed (negative reverses; `SPEED=0` brakes) |
| `PWM=<pwm>` | Open loop: drive at this PWM with the speed loop off, and print the speed in counts/s every 500 ms. `PWM=0` brakes |
| `FFSTATIC=` / `FFKV=` | Feedforward: static PWM, and counts/s per PWM step |
| `TRIM=` | Bound on the correction around the feedforward, in PWM |
| `SPEEDKP=` / `SPEEDKI=` | Speed loop gains: PWM per RPM, and per RPM-second |
| `STOP` | Brake |

## Procedure

1. **Sweep.** With the motor loaded as in use, send `PWM=` at four or five values across the range you'll run at (e.g. 30, 60, 90, 120, 150). Wait for the printed counts/s to settle at each and note it.
2. **Fit.** Fit a line, counts/s = `kv` × (PWM − `pwmStatic`): `kv` is the slope and `pwmStatic` where the line crosses 0 counts/s. Repeat the sweep in reverse (negative PWM) and use the lower `kv` and higher `pwmStatic` of the two directions.
3. **Set** `FFSTATIC=` and `FFKV=`, then `TRIM=20`. Command a mid-range speed, e.g. `SPEED=60`: it should reach the target in well under a second. Err low: feedforward that under-drives only slows the start a little, while over-drive overshoots at start-up.
4. **Gains.** The defaults (Kp 0.6, Ki 3.0) are a starting point. Raise `SPEEDKP` until `MeasuredSpeedRPM` tracks briskly with slight oscillation, then back off; `SPEEDKI` removes the remaining error, and too much shows as slow hunting.
5. **Verify** across the speed range and in both directions; grip the shaft briefly and release to check it returns to speed without a lunge; check `SPEED=0` brakes.
6. Record the results in the parent README's sample-results table and in `baseSpecs.h`.

Without feedforward (`FF_KV 0`, `TRIM_LIMIT 255`) the loop is a plain PI on PWM: it works, but the integral has to build the whole drive, so it is slower to reach speed.
