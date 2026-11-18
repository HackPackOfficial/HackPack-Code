# Selectable Fire Hack

## Controls

| Action | Result |
| --- | --- |
| Single tap (rear power button) | Arm / disarm |
| Double tap | Cycle fire mode: `SINGLE` → `BURST x3` → `AUTO` |
| Long press | Power off |

## Fire Modes

- **Single / Burst**: fire once per trigger press (release to re-arm). A burst completes even if the trigger is released.
- **Auto**: repeats while the trigger is held.
- Burst begins only after spinup.

## Notes

- The reticle is fixed by `RETICLE_STYLE` in `config.h`:

  ```cpp
  #define RETICLE_STYLE 4       // [1 - 5]
  ```

- Existing PID, motor-1 boost, servo timing, battery safety, and EEPROM shot counting retained.
- Removed unused NeoPixel object and startup/shutdown calls to save SRAM.

> **CAUTION:** Test unloaded. A burst cannot be interrupted by disarming until the current blocking servo stroke finishes; power-off is also processed between strokes.
