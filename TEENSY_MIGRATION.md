# Teensy 4.1 Migration Notes

This project now builds around `teensy41` for the active robot entry points that still exist in `src/`.

## What changed

- `platformio.ini` now targets Teensy 4.1 for the active app/test environments.
- Stale PlatformIO environments that referenced deleted source files were removed from the active build matrix.
- `BinaryStreamProcessor` now uses hardware `Serial1` on Teensy instead of incorrectly falling back to `SoftwareSerial`.

## Main environments

- `demoStriker`
- `demoGoalie`
- `teensy41_striker`
- `teensy41_mux_test`
- `avoidLine`
- `IMUtest`
- `PIDStickIntoOneDirection2`

## Build commands

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e demoStriker
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e demoGoalie
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e teensy41_mux_test
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e avoidLine
```


1. Rewire the Teensy 4.1 so the pins in `lib/constants/constants.h` match the physical robot.
2. Confirm the three motor PWM pins (`4`, `5`, `7`) are connected to Teensy PWM-capable outputs on your driver.
3. Confirm the three multiplexers are wired to the signal pins and selector pins listed in `constants.h`.
4. Confirm the IMU is on the main `Wire` bus and the Pixy is still on SPI.
5. Confirm the IR ring / auxiliary sender is connected to Teensy `Serial1`, because that path now assumes hardware UART.


1. Build `teensy41_mux_test` and check that all three multiplexer banks respond.
2. Build `IMUtest` and confirm the yaw is stable.
3. Build `PixyTest` or `demoGoalie` and confirm Pixy detection still works.
4. Build `demoStriker` only after line sensors, IMU, and IR ring are all verified independently.

## Important note

The code now targets Teensy, but the final success still depends on the hardware pin map being correct. If your Teensy wiring does not intentionally mirror the pin numbers in `constants.h`, update that file before uploading to the robot.
