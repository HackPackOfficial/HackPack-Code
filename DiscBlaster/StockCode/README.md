# BLSTR Code Organization

```
StockCode/
├── StockCode.ino      # setup() and loop() only
├── config.h            # User-tunable settings
├── BLSTR.h             # Shared declarations / pin map
├── State.cpp           # Hardware objects, logo bitmap, shared runtime state
├── Power.cpp           # Power button, latch, motor enable/disable
├── Motors.cpp          # Flywheel ramp, RPM measurement, PID, trigger handling
├── Servo.cpp           # Servo pulse generation and firing cycle
├── Shots.cpp           # Shot verification and EEPROM lifetime counter
├── Battery.cpp         # Battery voltage / percentage
├── LEDs.cpp            # NeoPixel animations and USB-safe battery meter
└── Display.cpp         # OLED UI, boot animations, warnings
```

> Functionality and configured values were kept identical to the uploaded `BLSTR_PID(3)` source; this pass only reorganizes the code.
