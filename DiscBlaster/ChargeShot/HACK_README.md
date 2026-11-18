# Charge Shot Hack

## Controls

| Action | Result |
| --- | --- |
| Single tap (rear power button) | Arm / disarm |
| Long press | Power off |
| Hold trigger | Charge |
| Release trigger | Fire one disc |

Double tap no longer changes the reticle.

## Charging

- OLED shows charge progress (full at `CHARGE_FULL_MS`, default 1200 ms).
- Both motors ramp linearly from their normal commanded PWM to PWM 255 over `CHARGE_FULL_MS` while the trigger is held.
- The existing M1-only post-release recovery boost is retained.
- Trigger held before spinup does not automatically fire upon release.
- Very short taps (< 50 ms) are ignored.

> **IMPORTANT:** Charge percentage is a timing/gameplay indicator, NOT measured disc energy or guaranteed RPM.

## Configuration

Set in `config.h`:

```cpp
#define RETICLE_STYLE 4         // [1 - 5] Reticle remains fixed
#define CHARGE_FULL_MS 1200UL   // Hold duration for 100% charge
```

## Notes

- Existing PID, servo timing, battery safety, and EEPROM shot counting retained.
- Removed unused NeoPixel object and startup/shutdown calls to save SRAM.

> **CAUTION:** Test unloaded. The existing servo routine blocks button processing during its ~255 ms firing cycle.
