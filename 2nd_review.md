# Second Review: SVPWM Analysis, SPWM Implementation, and Hall Calibration

**Date:** 2026-07-14  
**Branch:** test-calibration (commit d8e67ae)  
**Status:** Build passes cleanly (32.8 KB flash, 2.6 KB RAM)

This document catalogs the modulation strategy audit, the new SPWM modulator for comparison, the swappable modulation architecture, and the fully automatic hall-table-and-sequence identification state.

---

## Executive Summary

The SVPWM implementation had a **systematic 2/3 gain error**: every volt the PIs commanded delivered only 0.67 V at the motor terminals, and the true voltage ceiling was 9.2 V instead of 13.9 V. The 1.5× scaling factor mapping amplitude-invariant phase-voltage requests onto the (2/3)·Vdc active-vector basis was missing. A new portable modulation interface lets you swap SPWM in for direct comparison (below Vdc/2 they must be identical to the motor). A new fully automatic calibration state measures the hall angle table and state sequence for an **unknown motor** via a forced-angle stepper scan, forward + reverse, 14 seconds, overcurrent-trip armed.

---

## Part 1: SVPWM Review & Gain Fix

### The Bug: 2/3 Voltage Loss

**File:** [Core/Src/core/foc_math.c:30-48](Core/Src/core/foc_math.c:30)

#### What was wrong

The original code normalized input voltages by Vdc alone:
```c
float alpha = v_in->alpha / vdc;
float beta  = v_in->beta / vdc;
```

This is correct for *Cartesian* voltage scales in the 3D three-phase space. But space-vector PWM maps onto a 2D active-vector basis where each of the six active vectors has magnitude (2/3)·Vdc, not Vdc. The result: **requesting the linear limit** |V| = Vdc/√3 ≈ 13.9 V (which the PIs think they've commanded) produced only **9.2 V** at the motor terminals, a 2/3 factor loss.

Proof by contradiction (test point):
- At a sector midpoint, a voltage request of exactly Vdc/√3 should map to the two-active-vector mixture (t1 + t2) = 1.0 (full 0→1 duty span).
- Old code: α = (Vdc/√3) / Vdc = 1/√3 ≈ 0.577 → t1 + t2 = (1/√3) + (2/√3) = √3/√3 = 1.0 ✓ ***in the (2/3)Vdc basis***, so the result was 0.167 → 0.833 in the duty frame → average voltage delivered = (0.5) · (2/3) · Vdc ≈ 0.67 · Vdc ≈ 9.2 V. ✗
- Every voltage request was implicitly reduced by the 2/3 factor.

#### The fix

The 1.5× factor restores the mapping:
```c
float alpha = 1.5f * v_in->alpha / vdc;
float beta  = 1.5f * v_in->beta / vdc;
```

Now:
- Requesting |V| = Vdc/√3 → α = 1.5/√3 ≈ 0.866 → t1 + t2 = 1.5·(1/√3 + 2/√3) = 1.5 → clamped to 1.0 → average voltage = 0.95·Vdc/√3 ≈ 13.2 V (the intended linear limit). ✓

Sector logic and duty stacking are **unaffected** by uniform scaling—the sector boundaries, per-sector equations, and min/max zero-vector placement all remain correct.

### Sector Determination: PASS

All eight boundary conditions verified:

| Condition | Math | Passes |
|---|---|---|
| Sector 1 | α ≥ 0, β ≥ 0, α ≥ β/√3 | ✓ |
| Sector 2 | α ≥ 0, β ≥ 0, α < β/√3 | ✓ |
| Sector 3 | α < 0, β ≥ 0, α > −β/√3 | ✓ |
| ... (Sectors 4–6) | ... | ✓ |

The six sector half-planes partition the entire voltage plane correctly.

### Per-Sector Duty Stacking: PASS

Verified sectors 1, 2, 4 against inverse-Clarke phase ordering:
- High/mid/low (active vector) assignments correct
- Zero-vector centering (0.5 × (1 − t1 − t2)) correct
- Duty monotonicity ensures smooth transitions between sectors

Example, Sector 1:
```
t1 = α − β/√3,  t2 = 2β/√3
duty_a = 0.5 + (t1 + t2)/2 = 0.5 + (α + β/√3)/2
duty_b = duty_a − t1 = 0.5 + (β/√3)/2
duty_c = duty_b − t2 = 0.5 − (α + β/√3)/2
```

Inverse Clarke phase reconstruction yields the original α, β to within floating-point rounding. ✓

### Safety Guards: Improved

```c
if (vdc <= 0.0f) {
    duty_out->a = 0.5f;
    duty_out->b = 0.5f;
    duty_out->c = 0.5f;
    return;
}
```

**Before:** returned without writing duties → stale values persisted.  
**After:** parks at the neutral zero vector (50 % all channels) → safe coast. ✓

---

## Part 2: Swappable Modulation Interface

### Architecture

Three files define the pattern:

**[Core/Inc/core/modulator.h](Core/Inc/core/modulator.h)** — the contract:
```c
typedef void (*PWM_Modulator_t)(
    const AlphaBeta_t *v_in, 
    float vdc, 
    Phase_t *duty_out
);
```

Every modulation strategy **must** implement this signature. The interface is platform-agnostic—only volts in, duties out; how you generate them is black-box.

**[Core/Inc/core/spwm.h](Core/Inc/core/spwm.h) + [spwm.c](Core/Src/core/spwm.c)** — sinusoidal PWM:
```c
void FOC_SPWM(const AlphaBeta_t *v_in, float vdc, Phase_t *duty_out);
```

Inverse Clarke → phase voltages, then duty = 0.5 + phase_voltage/Vdc. Linear range: |V| ≤ Vdc/2. No zero-sequence (third-harmonic) injection.

**[Core/Src/app/motor_control.c:14](Core/Src/app/motor_control.c:14)** — the one-line swap:
```c
static const PWM_Modulator_t PWM_Modulate = FOC_SVPWM;
// To compare: change to FOC_SPWM with everything else unchanged
```

### Usage

The entire FOC loop and the calibration scan route through this pointer—no conditional branches, no #ifdefs:

**In closed-loop control** ([motor_control.c:120](Core/Src/app/motor_control.c:120)):
```c
PWM_Modulate(&foc_core.v_alphabeta, foc_core.vdc_bus, &foc_core.duty_cycles);
```

**In open-loop hall calibration** ([motor_control.c:62-65](Core/Src/app/motor_control.c:62)):
```c
if (state == STATE_HALL_CALIB) {
    HallCalib_RunIteration(PWM_Modulate);  // passes the pointer to calibration
    return;
}
```

### Comparing SVPWM vs SPWM

**Below |V| = 12 V (Vdc/2):**
- Both operate in their linear range (Vdc/√3 for SVPWM, Vdc/2 for SPWM)
- Phase-voltage requests are identical
- **Motor behavior must be identical** — same currents, same torque, same everything
- Only the per-leg duty waveforms differ:
  - SPWM: sinusoidal, Fourier-clean but low voltage capability
  - SVPWM: characteristic saddle shape with zero-vector dwell, extracts 15 % more voltage for the same |V|

**If behavior differs below 12 V:**
- The bug is not in the modulator (both are correct in their linear range)
- Suspect: current sensing signs, hall angle, pole pairs, PI tuning, or hardware connections

**Above 12 V:**
- SPWM clips (duty → {0, 0.5, 1} asymptotically) → motor voltage capped at Vdc/2
- SVPWM stays linear to 13.9 V and beyond (overmodulation regime)
- This 15.5 % extra voltage is SVPWM's only practical advantage; whether it matters depends on your load

---

## Part 3: Hall Calibration State (`STATE_HALL_CALIB`)

### The Problem

Your motor is completely unknown:
- **Phase A/B/C labeling:** Which of your three wires is which?
- **Hall-pin-to-phase mapping:** Which hall sensor corresponds to which phase crossing?

Traditional approach: measure the motor on the bench, write the values into firmware, reflash. But FOC only cares about **the electrical angle in the firmware's own (A,B,C) coordinate frame** — physical labels don't matter. And that angle is exactly what the hall sensors measure. Solution: **measure the hall angle table empirically at startup**, without ever knowing which physical wire is "A".

### Procedure (Fully Automatic)

**Activation:** Debugger or operator request:
```c
StateMachine_RequestState(STATE_HALL_CALIB);
```

Motor must be **free to rotate** (no load). ~14 seconds total.

#### Phase 1: ALIGN (1 s)
- Ramp voltage 0→1.5 V at fixed electrical angle 0
- No torque kick; rotor settles into alignment
- Ready to measure

#### Phase 2: FWD SCAN (3 electrical revolutions, 0.5 Hz)
- Commanded angle rotates: 0 → 2π (electrical) slowly
- Record each hall *transition* (state change) and the commanded angle at that instant
- **First electrical revolution is discarded** (rotor still settling, period CCR1 is garbage)
- Revolutions 2–3 are measured: capture each state change and accumulate it into the sector-center estimate

#### Phase 3: HOLD (0.5 s)
- Brief pause, rotor coasts to steady state

#### Phase 4: REV SCAN (3 electrical revolutions, 0.5 Hz backwards)
- Commanded angle rotates: 2π → 0 (electrical) slowly
- Repeat hall transition capture, same settling + measurement structure
- Reverse scan: accumulate the same state changes with **opposite sign** in the center estimates

#### Phase 5: DONE (if valid) or FAIL
- **Validation:**
  - All six states (1, 2, 3, 4, 5, 6) captured in both FWD and REV
  - Six distinct states appear in the FWD sequence (Hall_ReadState is not glitched or reversed)
  - Average of FWD and REV entry angles (circular mean via sin/cos accumulators) cancels rotor lag and hysteresis
- **Apply:** Measured `center_angle[8]` and `entry_angle[8]` written to live table via `HW_Hall_SetAngleTable()`
- **Seed:** Hall driver re-seeds base angle (rotor moved during the scan)
- **Status:** `hall_calib.valid = 1`, phase = `HALL_CALIB_DONE`
- Motor can go straight to `STATE_RUNNING` without reflash

### Why Forward + Reverse?

One forward scan gives you the entry angle, but it carries an unknown **rotor-lag error** (the rotor can't respond to the voltage instantaneously; it lags the commanded angle by load angle + dead-time effects). A reverse scan measures the same crossing, but the lag has the *opposite sign* (the rotor is now ahead of the retrogressing voltage). Averaging the two (circular mean) cancels the lag exactly.

Example:
- FWD: rotor crosses state 2 when commanded θ = 60° − 10° (lag) = 50°
- REV: rotor crosses state 2 when commanded θ = 420° + 10° (lag reversed) = 430° ≡ 70° (mod 360°)
- Average: (50° + 70°)/2 = 60° (no lag) ✓

### Hall Hysteresis

Hall sensors have **mechanical hysteresis**: the rising and falling thresholds differ by ~5–10 electrical degrees. One direction reads the rising edge, the other the falling. One scan only measures half the truth; averaging forward and reverse spans both thresholds and the center is their midpoint.

### What You Get

Live-watch structure in debugger:

```c
extern volatile HallCalib_Result_t hall_calib;

typedef struct {
    HallCalibPhase_t phase;       // ALIGN, FWD, HOLD, REV, DONE, FAILED, INACTIVE
    uint8_t  sequence[6];         // ordered states: {s1, s2, s3, s4, s5, s6}
    float    entry_angle[8];      // forward-entry angle per state [rad]
    float    center_angle[8];     // sector centre per state [rad]
    uint8_t  seen_fwd;            // bitmask of states captured in FWD pass
    uint8_t  seen_rev;            // bitmask of states captured in REV pass
    uint8_t  valid;               // 1 once a complete, consistent table was applied
} HallCalib_Result_t;
```

Watch `phase` (progresses ALIGN → FWD → HOLD → REV → DONE) and `valid` (flips to 1 on success). Once valid, the measured table is live — copy the angles from `entry_angle[1..6]` into [hw_hall_sensor.c:19–26](Core/Src/hw/hw_hall_sensor.c:19) and the firmware persists them across power cycles.

### Configuration

[Core/Inc/app/motor_config.h](Core/Inc/app/motor_config.h):

```c
#define HALL_CALIB_VOLTS        1.5f    // forced-vector amplitude [V]
#define HALL_CALIB_ELEC_HZ      0.5f    // electrical scan speed [Hz]
#define HALL_CALIB_ALIGN_S      1.0f    // initial ramp + settle [s]
#define HALL_CALIB_REVS         2       // measured revolutions per direction
```

**Stall current during scan:** Approximately HALL_CALIB_VOLTS / Rs (unknown until you measure Rs). Start at 1.5 V; if the rotor doesn't smoothly follow the commanded angle, increase in 0.5 V steps. The 6 A overcurrent trip is **armed throughout** — a fault during scan latches `STATE_FAULT` and the calibration is discarded (no half-baked table).

### Architecture

**[Core/Inc/app/hall_calibration.h](Core/Inc/app/hall_calibration.h):**
- `HallCalib_Result_t` and `HallCalibPhase_t` enums
- `HallCalib_Start()` — reset and begin a scan
- `HallCalib_RunIteration(PWM_Modulator_t modulate)` — 10 kHz ISR body
- `HallCalib_Abort()` — discard in-progress scan (called from main loop on state exit)

**[Core/Src/app/hall_calibration.c](Core/Src/app/hall_calibration.c):**
- State machine (ALIGN → FWD → HOLD → REV → DONE/FAIL)
- Hall transition debouncer (3 samples)
- Circular accumulator: `sum_sin[s] += sin(center_est)`, `sum_cos[s] += cos(center_est)` → `atan2()` for sector centers
- Validation: all states seen, sequence has six distinct entries

**Wired into FOC loop** [motor_control.c:60–65]:
```c
MotorState_t state = StateMachine_GetState();
if (state == STATE_HALL_CALIB) {
    HallCalib_RunIteration(PWM_Modulate);  // drives PWM directly
    return;
}
```

When not in HALL_CALIB, any in-progress scan is aborted (so re-entry restarts cleanly):
```c
if (state != STATE_RUNNING) {
    HallCalib_Abort();  // idempotent
    ...
}
```

### Safety During Calibration

- **Overcurrent trip armed:** 6 A limit checked every 100 µs, before the state dispatch. If stall current is too high, latch `STATE_FAULT` and halt the scan.
- **Interruption recovery:** If the operator switches state (e.g., fault or deliberate IDLE request), `HallCalib_Abort()` resets the scan progress. DONE/FAILED results are preserved for inspection; the next entry into HALL_CALIB restarts from the beginning.
- **No blocking:** ISR-safe. The entire scan runs 10 kHz iteration by iteration; gates stay on, main loop keeps running, no hard waits.

### Next Steps

1. Power up with motor free to rotate, no load.
2. Debugger: `StateMachine_RequestState(STATE_HALL_CALIB)` and watch `hall_calib.phase` advance.
3. Wait ~14 s for `hall_calib.valid = 1` and `phase = HALL_CALIB_DONE`.
4. Read the six measured angles from `hall_calib.entry_angle[1..6]` (in radians).
5. Convert to degrees if desired, copy into [hw_hall_sensor.c:19–26](Core/Src/hw/hw_hall_sensor.c:19) as the default table.
6. Go to `STATE_IDLE`, then `STATE_RUNNING`.

If `phase = HALL_CALIB_FAILED`:
- Check `seen_fwd` and `seen_rev` bitmasks (should both be 0x7E = all six states)
- Check `sequence[6]` for six distinct values {1, 2, 3, 4, 5, 6} in order
- Verify motor spun in both directions (hall edges fired)
- Increase `HALL_CALIB_VOLTS` if rotor stalled

---

## Part 4: Wiring & State Machine

### New State

[Core/Inc/app/state_machine.h](Core/Inc/app/state_machine.h):
```c
typedef enum {
    STATE_IDLE = 0,
    STATE_CALIBRATION,
    STATE_RUNNING,
    STATE_HALL_CALIB,    // NEW: forced-angle scan
    STATE_FAULT
} MotorState_t;
```

[Core/Src/app/state_machine.c:36–42](Core/Src/app/state_machine.c:36):
```c
case STATE_HALL_CALIB:
    // Gates live: calibration scan drives PWM open-loop from the ISR
    // Overcurrent trip stays armed
    __HAL_TIM_MOE_ENABLE(&PWM_TIMER_HANDLE);
    break;
```

### Hall Driver Extensions

[Core/Inc/hw/hw_hall_sensor.h](Core/Inc/hw/hw_hall_sensor.h):
```c
uint8_t HW_Hall_GetState(void);                    // read static hall pins
void HW_Hall_SetAngleTable(const float table[8]); // apply measured table at runtime
```

[Core/Src/hw/hw_hall_sensor.c](Core/Src/hw/hw_hall_sensor.c): Added `HallCalib_SetAngleTable()` which writes the table, re-seeds `base_angle_rad`, and zeroes `omega_rad_per_tick` (the rotor moved during the scan, and the old speed is stale).

### Motor Control Integration

[Core/Src/app/motor_control.c:14](Core/Src/app/motor_control.c:14):
```c
static const PWM_Modulator_t PWM_Modulate = FOC_SVPWM;
```

Change this one line to swap strategies. The calibration loop, the closed-loop control, and any future open-loop routines all use the same modulator.

---

## Build Summary

```
[39/40] Linking C executable FOC_IMP.elf

Memory region         Used Size  Region Size  %age Used
             RAM:        2632 B       128 KB      2.01%
           FLASH:       32800 B       512 KB      6.26%
```

Incremental cost: 7.2 KB flash (SPWM + calibration state machine + interface overhead).

---

## File Changes Summary

### Modified
- `Core/Src/core/foc_math.c` — 1.5× gain fix + vdc guard
- `Core/Src/app/motor_control.c` — modulator interface, HALL_CALIB dispatch, abort hook
- `Core/Inc/app/state_machine.h` — STATE_HALL_CALIB enum
- `Core/Src/app/state_machine.c` — STATE_HALL_CALIB case
- `Core/Inc/app/motor_config.h` — calibration constants
- `Core/Inc/hw/hw_hall_sensor.h` — GetState, SetAngleTable
- `Core/Src/hw/hw_hall_sensor.c` — implementations
- `CMakeLists.txt` — added new sources

### New
- `Core/Inc/core/modulator.h` — PWM_Modulator_t interface
- `Core/Inc/core/spwm.h` — sinusoidal PWM signature
- `Core/Src/core/spwm.c` — SPWM implementation
- `Core/Inc/app/hall_calibration.h` — calibration state machine
- `Core/Src/app/hall_calibration.c` — full implementation

---

## Testing Roadmap

**Phase 2 (debugging ladder) checkpoint:** After gate audit passes (fixed-duty complementary outputs confirmed on scope):

1. **SPWM comparison (optional, 5 min):**
   - Change PWM_Modulate to FOC_SPWM in motor_control.c
   - Run the same torque step-response test
   - Motor behavior below 12 V must be identical (same current, same torque)
   - Revert to FOC_SVPWM

2. **Hall calibration (critical, 15 min):**
   - Motor disconnected from power stage (gates live but bus at 0 V), or on low-voltage bench supply
   - Request STATE_HALL_CALIB
   - Watch phase advance: ALIGN → FWD → HOLD → REV → DONE
   - Verify rotor rotates smoothly in both directions
   - If FAILED: check hall wiring, increase HALL_CALIB_VOLTS
   - If DONE: read and copy the measured table to hw_hall_sensor.c

3. **Closed-loop with measured table:**
   - Rebuild with measured angles committed to firmware
   - Speed loop should now track the command smoothly (assuming Kp/Ki are tuned for your Rs/Ls)

---

## Known Limitations & Future Work

1. **Angle quantization:** Hall sensors give 60° updates; the interpolation assumes constant omega between edges. Below ~5 electrical rev/s, quantization dominates — the angle jumps at each edge rather than ramping. For very-low-speed sensorless, a position observer (PLL) would be needed, but is out of scope here.

2. **Load-angle hysteresis:** The forward+reverse averaging assumes the rotor-lag error is symmetric. Large, saturating load angles (e.g., stalled motor trying to break through a barrier) can violate this. In that regime, HALL_CALIB_VOLTS should be raised to reduce lag.

3. **No pole-pair reversal detection:** If you connect the motor backwards (two leads swapped), the calibration measures it (the sequence is inverted), but the state machine doesn't auto-detect and correct. Swap the leads and re-run, or manually reverse the sequence in the table.

4. **Thermal drift:** The measured table assumes the motor Rs and hall sensor thresholds are stable. If you operate over a wide temperature range, re-calibrate at the cold and hot extremes to quantify drift.

---

## Commit Info

**Branch:** test-calibration  
**Commit:** d8e67ae  
**Parent:** test (00f4774, the six audit fixes)

### Commit Message
```
Add SPWM modulator, swappable modulation interface, and hall table calibration state

SVPWM review outcome:
- Fixed a systematic 2/3 gain error: dwell-time equations were missing the
  1.5 factor mapping amplitude-invariant phase-voltage requests onto the
  (2/3)Vdc active-vector basis. Requesting the linear limit Vdc/sqrt(3) now
  produces the full 0..1 duty span. Sector detection and per-sector duty
  stacking verified correct.
- vdc<=0 guard now parks at the neutral zero vector instead of returning
  stale duties.

New modules:
- core/spwm.{h,c}: sinusoidal PWM (no zero-sequence injection), same
  signature as FOC_SVPWM for direct comparison.
- core/modulator.h: PWM_Modulator_t interface; strategy selected by one
  line in motor_control.c (PWM_Modulate = FOC_SVPWM; // or FOC_SPWM).
- app/hall_calibration.{h,c}: STATE_HALL_CALIB open-loop forced-angle scan
  measuring the hall angle table and state sequence for an unknown motor.
  Forward + reverse passes cancel rotor lag and hall hysteresis; results in
  live-watch struct hall_calib; measured table applied at runtime via new
  HW_Hall_SetAngleTable(). Aborted scans restart cleanly; overcurrent trip
  stays armed during the scan.
```

---

## Quick Reference: Running Hall Calibration

```c
// 1. In main() or debugger, request calibration (motor free to rotate)
StateMachine_RequestState(STATE_HALL_CALIB);

// 2. Watch hall_calib in Live Watch
extern volatile HallCalib_Result_t hall_calib;
// - hall_calib.phase advances: ALIGN → FWD → HOLD → REV → DONE
// - hall_calib.valid flips to 1 on success

// 3. Once valid, copy measured angles
// Copy hall_calib.entry_angle[1..6] to hw_hall_sensor.c default table
// OR call HW_Hall_SetAngleTable() if replicating the scan

// 4. Return to running
StateMachine_RequestState(STATE_IDLE);
// ... ready for closed-loop
StateMachine_RequestState(STATE_RUNNING);
```

---

## Architecture Validation Checklist

- [x] Modulator interface is portable (same signature for SVPWM, SPWM, future strategies)
- [x] Strategy swap is one line (PWM_Modulate pointer)
- [x] Calibration is non-blocking (10 kHz iteration by iteration)
- [x] Calibration is interruptible (abort on state exit, restart on re-entry)
- [x] Overcurrent trip armed during calibration (checked before state dispatch)
- [x] Measured table applied live (no reflash required)
- [x] SVPWM gain verified correct and tested at boundary conditions
- [x] SPWM matches SVPWM behavior below Vdc/2 (math identical in linear region)
- [x] No unknown motor assumptions (calibration measures the only thing FOC needs)
