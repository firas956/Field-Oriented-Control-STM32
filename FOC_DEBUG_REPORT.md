# FOC Debug Report — Problems Found and Fixes Applied

**Date:** 2026-07-15
**Symptoms investigated:** motor sometimes starts on its own, sometimes needs a manual push; once turning it never stops; speed never regulates to target.
**Evidence used:** full source review, `data_logged.csv` (5113 samples, 7.75 s), `motor_alignment_id.md` (bench alignment tests).

---

## Summary

The FOC loop had never actually regulated anything. Two root causes fully explain the symptoms, plus several secondary defects that amplified them:

| # | Problem | Type | Explains |
|---|---------|------|----------|
| 1 | **Leg 1 (RED wire) carries no current — open phase** | Hardware (NOT fixed in code — see action items) | push-to-start, random direction |
| 2 | **Hall angle table mirrored** (and offset, and one typo) | Software (fixed) | "never stops" runaway, no speed regulation |
| 3 | PI controllers never clamped their output | Software (fixed) | total saturation, 789 V demand on a 24 V bus |
| 4 | No initial rotor angle before the first hall edge | Software (fixed) | rotor locking at power-on |
| 5 | Speed loop units/gains nonsensical (0.5 A per RPM) | Software (fixed) | 510 A torque request |
| 6 | Current loops allowed ±24 V (only ±13.9 V exists) | Software (fixed) | permanent overmodulation |
| 7 | Misc: `PWM_ARR` macro, dead duplicate estimators, swapped debug names, stale integrators on state entry | Software (fixed) | latent traps |

---

## Problem 1 — Open phase on Leg 1 (RED wire) — **hardware, still to repair**

### Evidence
- Over the entire log, `i_a` never exceeds **0.36 A** while `i_b` swings **±7 A**. A spinning 3‑phase machine cannot have one phase at zero (Kirchhoff), so the leg monitored by PA0 conducts nothing.
- At inverter power-on (t = 1.765 s) the rotor locked for 17 ms with hall state **4** = `(h1,h2,h3) = (0,0,1)` — *exactly* the reading from the bench test row "Yellow = 24 V, Black = 0 V, **Red floating**" in `motor_alignment_id.md`. The inverter, while commanding all three legs (duties ≈ 0.96 / 1.00 / 0.00), physically reproduced the "Red disconnected" experiment.
- During that lock, `i_b` read **6.1–6.3 A = the entire bench-supply current limit in one phase**. With three conducting legs the current would have split ≈ 3 A / 3 A.

Conclusion: the current sensing is fine — it is a healthy sensor honestly reporting a dead phase. The motor has been running as a **single-phase machine**: pulsating field, zero starting torque when the rotor rests on the field axis (hence the manual push), and sustained rotation in whichever direction it gets kicked.

### Sensor mapping (verified, no code change needed)
- `i_a` = ADC1_IN0 (PA0) = **leg 1 sensor = RED** = phase *a* (TIM1_CH1)
- `i_b` = ADC2_IN1 (PA1) = **leg 2 sensor = YELLOW** = phase *b* (TIM1_CH2)

This matches the SVPWM duty mapping. (Documented in `hw_adc.c`.) The MCU-side PWM config for CH1/CH1N (PA8/PA7) is correct and both channels are started, so the break is downstream of the chip.

### Action items (bench)
1. Scope **PA8** (TIM1_CH1) — confirm PWM leaves the MCU.
2. Follow the signal to the Semikron leg‑1 gate driver input; check the driver's supply and error/enable latch.
3. Check the **RED motor lug/cable** continuity to the leg‑1 output.
4. Once leg 1 conducts, verify the leg‑1 sensor polarity (a real current has never been observed on it).

---

## Problem 2 — Hall angle table mirrored + offset + typo — **fixed**

### Root cause
The table in `hw_hall_sensor.c` was a **mirror image** of the true state→angle mapping (old ≈ 60° − correct), so while the rotor's true electrical angle increased, the firmware's angle decreased. The log shows `speed_measured = −1500 RPM` against a **+1000 RPM** target: the speed loop error grew *with* speed → **structural positive feedback** → the motor could never regulate and never stop. Additionally, entry for state 3 was `2.0f` (raw radians = 114.6°) — a typo missing the `* SIXTY_DEG_RAD` factor.

### Derivation of the correct table
From Test 1 of `motor_alignment_id.md`: each two-wire DC energization creates a stator field at a known electrical angle (in the code's frame, phase *a* = Red): a→c = 30°, b→c = 90°, b→a = 150°, c→a = 210°, c→b = 270°, a→b = 330°. The rotor aligns its d-axis to that field, so each row gives one hall-sector center.

**Convention trap discovered on the way:** the `hall_debug` struct fields were bit-swapped (`h1` actually showed PC8, `h3` showed PA6). The md file's columns follow that (debugger) convention — proven by the power-on lock event matching Test 1 row 5 bit-for-bit. Conversion to the code's `hall_state` (bit2 = PA6, bit1 = PC7, bit0 = PC8) is therefore `state = h1 + 2·h2 + 4·h3` using the md columns:

| Field angle | md (h1,h2,h3) | `hall_state` | Table entry |
|---|---|---|---|
| 30° | 1,0,1 | 5 | 0.5 × 60° |
| 90° | 0,0,1 | 4 | 1.5 × 60° |
| 150° | 0,1,1 | 6 | 2.5 × 60° |
| 210° | 0,1,0 | 2 | 3.5 × 60° |
| 270° | 1,1,0 | 3 | 4.5 × 60° |
| 330° | 1,0,0 | 1 | 5.5 × 60° |

Cross-checks (all pass):
- Test 2 manual-spin sequence converts to states **4→6→2→3→1→5** = +60° steps through the new table.
- The running motor in the CSV produced the *identical* repeating cycle 4→6→2→3→1→5.
- The power-on lock (pure b→c field at 90°, since Red is open) reads state 4, whose new sector is [60°, 120°] — dead center.

### Fix applied (`Core/Src/hw/hw_hall_sensor.c`)
- New table with **sector-center** angles (±30° worst case; the PLL smooths the quantization).
- Direction detector lists swapped: forward (+1) is now the physically-increasing-angle cycle 4→6→2→3→1→5.
- `hall_debug` fields renamed/reassigned to match the pins: `h1` = PA6, `h2` = PC7, `h3` = PC8.

> ⚠️ **Convention change:** from this firmware on, `hall_debug.h1/h2/h3` = PA6/PC7/PC8. Old logs and `motor_alignment_id.md` used the swapped (h1 = PC8) convention. Do not compare them column-for-column.

---

## Problem 3 — PI output not clamped — **fixed**

Only the integrator was clamped; the returned `Kp·error + integrator` was unbounded. Log fingerprint: at t = 0, `iq_target = 510` (0.5 × 1000 RPM error + 10 integrator) and `v_q = 789` (1.5 × 510 + 24). Every stage downstream was saturated; the "current controller" was effectively a full-voltage six-step drive.

**Fix** (`Core/Src/core/pid.c`): output now clamped to `[out_min, out_max]` in addition to the integrator clamp.

## Problem 4 — No initial rotor angle — **fixed**

`HW_Hall_Update_ISR` only runs on hall *edges*; at boot the angle stayed 0.0 regardless of rotor position (the CSV shows hall bits at 0 for the first 1.78 s — the ISR never fired). The FOC therefore applied a frozen full-voltage vector; the rotor snapped into alignment with it and locked (zero torque at equilibrium) until pushed.

**Fix** (`Core/Src/hw/hw_hall_sensor.c`): `HW_Hall_Init()` reads the hall pins once and seeds `base_angle_rad` before enabling the edge interrupt. Additionally, `MotorControl_Reset()` re-seeds the PLL from the current hall state whenever the state machine enters `STATE_RUNNING`.

## Problem 5 — Speed loop units/gains — **fixed**

Old: Kp = 0.5 A/RPM (a 1000 RPM error demanded 500 A), limits ±10 A. New (`Core/Src/app/motor_control.c`): Kp = 0.005 A/RPM, Ki = 0.02, output limited to **±4 A** (`IQ_LIMIT_A`, kept below the 6 A bench-supply limit so the current loops stay linear — this is an actuator limit, not a protection). Starting values; tune on the bench.

## Problem 6 — Current-loop voltage limits — **fixed**

Old limits were ±24 V per axis; the maximum phase voltage in linear SVPWM is **Vdc/√3 ≈ 13.9 V**, shared by d and q. New: both current PIs limited to ±Vdc/√3, plus a **d-priority circle limiter** (`vq ≤ √(v_max² − vd²)`) before the inverse Park.

## Problem 7 — Miscellaneous — **fixed**

- `PWM_ARR` was `4500-1.0f` **without parentheses**, so `duty * PWM_ARR` expanded to `duty*4500 − 1`, and duty = 0 cast a negative float to `uint32_t` (undefined behavior). Now `(4500.0f - 1.0f)`.
- Removed the two duplicate/dead angle estimators: the "strong override" PLL in `hall_pll.c` (`internal_pll`, lazy-init, never called by the control path) and the tick-based interpolation (`omega_rad_per_tick`, `HW_Hall_GetElectricalAngle`, `HW_Hall_GetSpeedRPM`). One source of truth remains: hall table → PLL in `motor_control.c`.
- Added integrator anti-windup to the hall PLL (`PLL_MAX_SPEED_RAD_S`).
- `MotorControl_Reset()` (new) clears all PI integrators + re-seeds the PLL; called by the state machine on the IDLE/CALIBRATION → RUNNING transition, so integrators wound up while gates were off can never drive the first PWM cycles.
- Removed unused `lpf.h` include and stale prototypes (`MotorControl_RunSpeedLoop`); implemented the declared-but-missing `StateMachine_GetState()`.

---

## New commissioning aid: forced-angle test mode

`Core/Src/app/motor_control.c` now has:

```c
#define FORCED_ANGLE_TEST 0   // set to 1 for open-loop commissioning
```

When enabled, the current/speed loops are bypassed and a small fixed vector (`FORCED_ANGLE_VD` = 2 V) rotates at a slow forced ramp (1 electrical rev/s). Expected result with healthy hardware: the motor turns smoothly like a stepper, `hall_debug` marches 4→6→2→3→1→5, and `foc_core.angle_rad` (PLL) tracks the forced angle within ~30°. This single test validates the leg-1 repair, the hall table, and the pole-pair count.

---

## Recommended bring-up order

1. **Repair leg 1** (action items above). Nothing else is testable before this.
2. **Verify pole pairs**: rotate the shaft one full mechanical turn by hand, count hall transitions; `pole_pairs = transitions / 6`. Update `MOTOR_POLE_PAIRS` in `motor_control.c` (currently still the unverified value 2).
3. **Forced-angle test** (`FORCED_ANGLE_TEST 1`): smooth stepper-like rotation, all three phase currents present and balanced, PLL tracks.
4. **Current loop only** (`FORCED_ANGLE_TEST 0`, speed target 0, set `id_target` = 1 A): rotor locks softly, `i_dq.d` tracks 1.0 A. Step 0.5→1.5 A and check response. Measure R and L, then set Kp = L·ω_bw, Ki = R·ω_bw (ω_bw ≈ 2π·500…1000).
5. **Torque mode**: `iq_target` = +0.5 A → motor accelerates smoothly and **`speed_measured` is positive**. (If negative, stop and re-check phase order — but with the corrected table it should be positive.)
6. **Close the speed loop**: start with a low target (e.g. 300 RPM) before 1000 RPM.
7. **Fix the logging**: 500 Hz sampling of ~55 Hz electrical quantities is aliased. For loop tuning, log from the 20 kHz ISR into a RAM ring buffer and dump it afterwards.

## Files changed

| File | Change |
|---|---|
| `Core/Src/hw/hw_hall_sensor.c` | correct table, initial-state seed, direction lists, debug naming, dead code removed |
| `Core/Inc/hw/hw_hall_sensor.h` | pruned dead prototypes, added `HW_Hall_GetDirection` |
| `Core/Src/core/hall_pll.c` | removed duplicate estimator, added integrator clamp |
| `Core/Src/core/pid.c` | output clamping |
| `Core/Src/app/motor_control.c` | voltage limits (Vdc/√3 + circle), speed-loop rescale, `MotorControl_Reset`, forced-angle test mode |
| `Core/Inc/app/motor_control.h` | `MotorControl_Reset` prototype, removed stale one |
| `Core/Src/app/state_machine.c` | reset-on-RUNNING-entry, `StateMachine_GetState` |
| `Core/Src/hw/hw_adc.c` | documented verified sensor mapping |
| `Core/Inc/hw/hw_pwm.h` | `PWM_ARR` parenthesization |

Build verified: `cmake --build build/Debug` → clean link, no warnings introduced (FLASH 4.8 %, RAM 1.9 %).
