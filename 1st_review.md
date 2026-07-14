# First Review: Six Firmware Fixes Applied

**Date:** 2026-07-14  
**Status:** Build passes cleanly (25.6 KB flash, 2.4 KB RAM)

This document catalogs the six critical plumbing fixes applied to the FOC firmware following the system-wide audit. Each fix addresses a root cause preventing motor operation; together they restore the bridge-to-firmware connection and add protection.

---

## Executive Summary

The motor does not run because **the low-side gate outputs were never enabled** (fix #1), the **rotor angle starts arbitrary at standstill** (fix #2), and the firmware **lacks overflow timeout and protection** (fixes #3, #6). Speed-loop gains are running at 10× design frequency (fix #5), voltage limits are physically unreachable (fix #4), and the hall speed feedback is unsigned (embedded in fix #5). All six are applied and the build is verified.

**Before testing:** comment out `MotorControl_SetSpeedTarget(100.0f)` and `StateMachine_RequestState(STATE_RUNNING)` in `main()` [lines 116–117](Core/Src/main.c:116). Do not auto-arm the inverter at boot with no operator control.

---

## Fix #1: Complementary PWM Outputs (THE show-stopper)

**File:** [Core/Src/main.c:127–132](Core/Src/main.c:127)  
**Symptom:** Low-side gate drivers (PA7/PB0/PB1) are silent; only high-side (PA8/PA9/PA10) PWM.  
**Root cause:** `HAL_TIMEx_PWMN_Start()` was never called. The top-level `HAL_TIM_PWM_Start()` only sets `CCxE` bits; the complementary `CCxNE` bits remain zero.

### Change
```c
HAL_TIM_Base_Start(&htim1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
// NEW: Complementary outputs (CH1N/2N/3N on PA7/PB0/PB1)
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
```

### Why
The Semikron educational inverter is a three-phase bridge; current must flow through *both* the high-side and low-side switches of at least one phase to form a closed loop. If only the high-side ever conducts, no current path exists and no torque develops. With this fix, all six IGBT gate signals are live.

### Test (Phase 2, Step 3)
Scope PA8 (high) vs PA7 (low) simultaneously:
- Expect 10 kHz complementary pair
- Dead time measured between PA8 fall and PA7 rise ≈ 556 ns (deadtime register = 100, clock = 180 MHz)
- `__HAL_TIM_MOE_DISABLE` in `STATE_IDLE` must kill both signals instantly

---

## Fix #2: Hall Angle Seeding at Standstill

**File:** [Core/Src/hw/hw_hall_sensor.c:38–52](Core/Src/hw/hw_hall_sensor.c:38)  
**Symptom:** FOC command angle is always 0 rad until the rotor spins and fires a hall edge.  
**Root cause:** `base_angle_rad` is initialized `0.0f` and only updated inside `HW_Hall_Update_ISR()`. At standstill there are no edges, so the angle never moves. FOC applies a voltage vector at an arbitrary angle — the rotor snaps to align with it, potentially locking 90° away and producing zero torque.

### Change
```c
void HW_Hall_Init(void) {
    // NEW: Seed the angle from the static hall state
    uint8_t state = Hall_ReadState();
    if (state > 0 && state < 7) {
        // Sector centre: rotor is somewhere inside this 60-degree span,
        // so the centre halves the worst-case initial error to 30 degrees.
        base_angle_rad = hall_angle_table[state] + 0.5f * SIXTY_DEG_RAD;
        if (base_angle_rad >= (2.0f * PI)) {
            base_angle_rad -= (2.0f * PI);
        }
        prev_hall_state = state;
    }
    omega_rad_per_tick = 0.0f;
    period_valid = 0;
    
    // NEW: Enable stall timeout (fix #3 below)
    htim3.Instance->CR1 |= TIM_CR1_URS;
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);

    HAL_TIMEx_HallSensor_Start_IT(&htim3);
}
```

Also added a helper to read hall pins directly:
```c
static uint8_t Hall_ReadState(void) {
    uint8_t state = 0;
    if (GPIOA->IDR & GPIO_PIN_6) state |= 0x04;
    if (GPIOC->IDR & GPIO_PIN_7) state |= 0x02;
    if (GPIOC->IDR & GPIO_PIN_8) state |= 0x01;
    return state;
}
```

### Why
At standstill, the rotor is physically at rest inside one of the six 60° sectors marked by the hall sensors. Reading the static pins tells you which sector. Seeding the angle to the sector's *center* (table entry ±30°) gives worst-case initial error of 30° instead of up to 360°. Once the rotor starts and edges arrive, the next fix ensures the period is valid, and the angle interpolates smoothly forward. Also fixes a secondary bug: `prev_hall_state` is now seeded, so direction detection is valid from the first edge (was uninitialized before).

### Test (Phase 2, Step 5)
Debugger watch on `foc_core.angle_rad`:
- At boot with rotor at rest: should read ~0–2π (some sector midpoint)
- Rotate shaft by hand: angle ramps monotonically 0→2π per electrical revolution
- Should see all six hall states {1,2,3,4,5,6} and no state 0 or 7 during normal rotation

---

## Fix #3: Hall Timeout & Stall Detection

**File:** [Core/Src/hw/hw_hall_sensor.c:57–65, 112–118](Core/Src/hw/hw_hall_sensor.c:112)  
**Symptom:** At standstill or very low speed (<2.5 rev/s electrical), `omega_rad_per_tick` holds a stale value; angle extrapolates away from true position indefinitely.  
**Root cause:** TIM3 wraps every 65.5 ms. If no edge arrives before wrap, the captured "period" is garbage (measured modulo 65536 ticks). No logic invalidates it.

### Change
New ISR callback fired by TIM3 update (overflow):
```c
void HW_Hall_Timeout_ISR(void) {
    omega_rad_per_tick = 0.0f;
    period_valid = 0;
}
```

Also in `HW_Hall_Update_ISR()`:
```c
uint32_t ticks_per_60_deg = TIM3->CCR1;

if (period_valid && ticks_per_60_deg > 0) {
    omega_rad_per_tick = SIXTY_DEG_RAD / (float)ticks_per_60_deg;
} else {
    // First edge after start-up or stall timeout: the captured period is
    // meaningless. Keep omega at zero and start trusting from the next edge.
    omega_rad_per_tick = 0.0f;
    period_valid = 1;
}
```

And in angle interpolation:
```c
float delta = (float)direction * omega_rad_per_tick * (float)elapsed_ticks;

// Never extrapolate more than one hall sector past the last edge
if (delta > SIXTY_DEG_RAD) {
    delta = SIXTY_DEG_RAD;
} else if (delta < -SIXTY_DEG_RAD) {
    delta = -SIXTY_DEG_RAD;
}
```

### Why
A 65.5 ms timeout catches stall and makes it explicit: omega goes to zero, angle stops extrapolating. The `period_valid` flag discards the first captured period (it's measured from an arbitrary origin post-overflow, not a true 60° interval). Clamping extrapolation to ±60° means a stale omega can't run the angle away while the rotor decelerates between edges.

### Test (Phase 2, Step 5)
- Let rotor come to rest: after 65.5 ms, `HW_Hall_GetSpeedRPM()` should read 0 RPM
- Rotate slowly by hand: speed is 0 until the first post-stall edge, then resumes normally
- No infinite extrapolation errors

---

## Fix #4: Voltage Limiting (D-Priority Circular)

**File:** [Core/Src/app/motor_control.c:95–109](Core/Src/app/motor_control.c:95)  
**Symptom:** PI controllers output ±24 V per axis against a real ceiling of ±13.9 V (Vdc/√3 linear SVPWM); integrators saturate and hang.  
**Root cause:** Two problems: (a) PI output limits were unrealistic; (b) only the integrator was clamped, so proportional term could blow past limits anyway.

### Change
In `MotorControl_Init()`:
```c
// Current-loop output limits: the linear SVPWM range is Vdc/sqrt(3) per vector
float v_lim = VDQ_LIMIT_FRACTION * VBUS_VOLTS * ONE_BY_SQRT3;  // ≈13.2 V
PI_Init(&id_controller, 1.5f, 200.0f, Ts, -v_lim, v_lim);
PI_Init(&iq_controller, 1.5f, 200.0f, Ts, -v_lim, v_lim);
```

In `MotorControl_RunIteration()`:
```c
float id_error = foc_core.id_target - foc_core.i_dq.d;
float iq_error = foc_core.iq_target - foc_core.i_dq.q;
foc_core.v_dq.d = PI_Update(&id_controller, id_error);
foc_core.v_dq.q = PI_Update(&iq_controller, iq_error);

// Circular voltage limit with d-axis priority: |Vdq| stays inside the
// linear SVPWM range. Flux (d-axis) keeps authority when saturated.
float v_max = VDQ_LIMIT_FRACTION * foc_core.vdc_bus * ONE_BY_SQRT3;
if (foc_core.v_dq.d > v_max) {
    foc_core.v_dq.d = v_max;
} else if (foc_core.v_dq.d < -v_max) {
    foc_core.v_dq.d = -v_max;
}
float vq_max = sqrtf(v_max * v_max - foc_core.v_dq.d * foc_core.v_dq.d);
if (foc_core.v_dq.q > vq_max) {
    foc_core.v_dq.q = vq_max;
} else if (foc_core.v_dq.q < -vq_max) {
    foc_core.v_dq.q = -vq_max;
}
```

Also in [Core/Src/core/pid.c](Core/Src/core/pid.c):
```c
float PI_Update(PI_Controller_t *pid, float error){
    float p_term = pid->Kp * error;
    pid->integrator += pid->Ki_normalized * error;
    
    if (pid->integrator > pid->out_max){
        pid->integrator = pid->out_max;
    }
    else if (pid->integrator < pid->out_min){
        pid->integrator = pid->out_min;
    }

    float output = p_term + pid->integrator;
    
    // NEW: Clamp the *output*, not just integrator
    if (output > pid->out_max){
        output = pid->out_max;
    }
    else if (output < pid->out_min){
        output = pid->out_min;
    }

    return output;
}
```

### Why
SVPWM can only apply voltage vectors up to Vdc/√3 ≈ 13.2 V per axis in the dq frame. Demanding ±24 V is guaranteed saturation and integrator windup. The circular limiter with d-priority ensures: (1) |Vdq| ≤ v_max, (2) the d-axis (flux/field) is clamped first, keeping flux control authority. When torque saturates, flux does not — this is essential for weak-field operation and prevents oscillation at the limit.

### Test (Phase 2, Step 7)
- Set id_target=1 A, iq_target=0, monitor foc_core.v_dq output
- Voltages should stay ≤ ±13.2 V and not wind up
- Increase id to 2 A: v_d should clamp cleanly and rotor should still align

---

## Fix #5: Speed Loop Decimation & Signed Speed Feedback

**File:** [Core/Src/app/motor_control.c:76–83](Core/Src/app/motor_control.c:76)  
**Symptom:** Speed loop oscillates or goes unstable; gains tuned for 1 kHz but running at 10 kHz.  
**Root cause:** `MotorControl_RunIteration()` is called 10 kHz from the ADC ISR. `PI_Update(&speed_controller, …)` runs every iteration, so the effective Ki is 10× designed. Additionally, `HW_Hall_GetSpeedRPM()` returned unsigned magnitude; reverse rotation read as positive RPM and the speed loop tried to run away.

### Change
In `MotorControl_Init()`:
```c
// Speed loop runs decimated inside RunIteration, so Ts matches the decimated rate
float Ts_speed = (float)SPEED_LOOP_DECIMATION / CURRENT_LOOP_FREQ_HZ;
PI_Init(&speed_controller, 0.5f, 0.01f, Ts_speed, -IQ_LIMIT_AMPS, IQ_LIMIT_AMPS);
LPF_Init(&speed_filter, SPEED_LPF_ALPHA, 0.0f);
```

In `MotorControl_RunIteration()`:
```c
// NEW: Decimation counter
static uint32_t speed_loop_divider = 0;

// Speed loop decimated to the rate its gains were designed for (1 kHz)
if (++speed_loop_divider >= SPEED_LOOP_DECIMATION) {
    speed_loop_divider = 0;
    float raw_rpm = HW_Hall_GetSpeedRPM(MOTOR_POLE_PAIRS);
    foc_core.speed_measured = LPF_Update(&speed_filter, raw_rpm);  // Filter hall noise
    float speed_error = foc_core.speed_target - foc_core.speed_measured;
    foc_core.iq_target = PI_Update(&speed_controller, speed_error);
}
```

In [Core/Src/hw/hw_hall_sensor.c:153](Core/Src/hw/hw_hall_sensor.c:153):
```c
float HW_Hall_GetSpeedRPM(uint8_t pole_pairs) {
    float omega_elec_rad_sec = omega_rad_per_tick * HALL_TIMER_FREQ;
    float rpm = (omega_elec_rad_sec * 60.0f) / (2.0f * PI * (float)pole_pairs);

    return (float)direction * rpm;  // NEW: Signed; reverse = negative RPM
}
```

### Why
The PI gains `Kp=0.5, Ki=0.01` assume a 1 kHz sample rate (Ts=1 ms). At 10 kHz, the effective Ki becomes 0.1, and the loop overshoots and oscillates. The fix runs the loop every 10 cycles, matching the design Ts. The LPF (α=0.1, 16 Hz cutoff at 1 kHz) smooths quantization noise from single-period hall speed before it hits the PI integrator. Making speed feedback signed ensures reverse rotation is detected and damped, not amplified.

### Test (Phase 2, Step 8)
- Set speed_target = 50 RPM; should reach it smoothly in ~1–2 seconds
- Reverse-spin the rotor by hand: speed_measured should read negative
- No oscillation or overshoot at moderate targets

---

## Fix #6: Software Overcurrent Trip

**File:** [Core/Src/app/motor_control.c:50–58](Core/Src/app/motor_control.c:50)  
**Symptom:** No protection against supply overcurrent; a logic error or a stalled rotor draws 6 A continuously until supply folds back.  
**Root cause:** No check on phase currents; no way to kill the gates faster than a human reaching for the power switch.

### Change
In `MotorControl_RunIteration()`:
```c
HW_ADC_ReadCurrents(&foc_core.i_abc.a, &foc_core.i_abc.b);
foc_core.i_abc.c = -(foc_core.i_abc.a + foc_core.i_abc.b);

// NEW: Software overcurrent trip
if (fabsf(foc_core.i_abc.a) > OC_TRIP_AMPS ||
    fabsf(foc_core.i_abc.b) > OC_TRIP_AMPS ||
    fabsf(foc_core.i_abc.c) > OC_TRIP_AMPS) {
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_TIMER_HANDLE);
    StateMachine_RequestState(STATE_FAULT);
}

// NEW: When not running, reset integrators and park at neutral vector
if (StateMachine_GetState() != STATE_RUNNING) {
    PI_Reset(&id_controller, 0.0f);
    PI_Reset(&iq_controller, 0.0f);
    PI_Reset(&speed_controller, 0.0f);
    speed_loop_divider = 0;
    foc_core.duty_cycles.a = 0.5f;
    foc_core.duty_cycles.b = 0.5f;
    foc_core.duty_cycles.c = 0.5f;
    HW_PWM_SetDuties(&foc_core.duty_cycles);
    return;
}
```

New state added to [Core/Inc/app/state_machine.h](Core/Inc/app/state_machine.h):
```c
typedef enum {
    STATE_IDLE = 0,
    STATE_CALIBRATION,
    STATE_RUNNING,
    STATE_FAULT        // NEW: Latched overcurrent trip
} MotorState_t;
```

State machine updated in [Core/Src/app/state_machine.c:36–44](Core/Src/app/state_machine.c:36):
```c
case STATE_FAULT:
    // Latched protection trip: gates dead until explicit return to IDLE
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_TIMER_HANDLE);
    MotorControl_SetTorqueTarget(0.0f);
    break;
```

And request gating in [state_machine.c:50–58](Core/Src/app/state_machine.c:50):
```c
void StateMachine_RequestState(MotorState_t new_state) {
    // Latched fault: only an explicit return to IDLE re-arms
    if (current_state == STATE_FAULT && new_state != STATE_IDLE) {
        return;
    }
    current_state = new_state;
}
```

### Why
The 6 A lab supply can be violated in ~1 ms by a logic bug, motor stall, or winding short. This check fires within the same 100 µs cycle and kills MOE instantly. Latching the fault in `STATE_FAULT` prevents automatic re-arm — the operator must see the trip and issue an explicit reset (usually via debugger in test). When not running, the integrators are reset so they don't wind up against a dead bridge and cause a violent startup on re-arm; duties park at 50 % (neutral zero vector).

### Test (Phase 2, Step 7)
- Rotor blocked at full speed command: current should exceed 6 A within 1 cycle
- Observe on debugger: `StateMachine_GetState()` should be `STATE_FAULT`
- MOE should be off; all gate outputs should be quiet
- Call `StateMachine_RequestState(STATE_IDLE)` from debugger: state returns to IDLE, ready to re-arm

---

## Supporting Infrastructure

**New file:** [Core/Inc/app/motor_config.h](Core/Inc/app/motor_config.h)  
Home for all physical constants and tuning knobs:
```c
#define MOTOR_POLE_PAIRS        2           // Pending bench verification
#define VBUS_VOLTS              24.0f
#define OC_TRIP_AMPS            6.0f        // Supply limit
#define IQ_LIMIT_AMPS           4.0f        // Speed loop clamp (stay below supply)
#define VDQ_LIMIT_FRACTION      0.95f       // Fraction of linear SVPWM
#define CURRENT_LOOP_FREQ_HZ    10000.0f
#define SPEED_LOOP_DECIMATION   10          // 1 kHz result
#define SPEED_LPF_ALPHA         0.1f        // ~16 Hz cutoff at 1 kHz
```

**Modified:** [Core/Src/main.c:526–533](Core/Src/main.c:526)  
Added TIM3 update callback for stall timeout:
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HW_Hall_Timeout_ISR();
    }
}
```

---

## What Still Needs Bench Work

These six fixes restore plumbing; they do not substitute for motor characterization:

1. **Hall table calibration.** The lookup table in [hw_hall_sensor.c:19–25](Core/Src/hw/hw_hall_sensor.c:19) is still a guess. Phase 2, step 6 (forced-angle stepper test) must measure each state's actual electrical angle.
2. **Pole pairs confirmation.** `MOTOR_POLE_PAIRS = 2` is a placeholder. Bench detent counting (Phase 3.1) must confirm the actual value.
3. **RS/LS measurement.** PI gains `Kp=1.5, Ki=200` are loose defaults. Phase 3.2 must measure phase resistance and inductance so gains can be retune to the actual plant.
4. **Ke and flux linkage.** Phase 3.3: spin the motor with a drill and measure back-EMF to confirm the motor is sane and estimate torque constant.
5. **Supply headroom.** With a 6 A limit and 4 A speed-loop clamp, test real thermal behavior under load. The 100 RPM boot target assumes the motor will spin; if it doesn't, you'll be at the limit.

---

## Build & Link Report

```
[1/6] Building C object ... pid.c.obj
[2/6] Building C object ... hw_hall_sensor.c.obj
[3/6] Building C object ... state_machine.c.obj
[4/6] Building C object ... motor_control.c.obj
[5/6] Building C object ... main.c.obj
[6/6] Linking C executable FOC_IMP.elf

Memory region         Used Size  Region Size  %age Used
             RAM:        2448 B       128 KB      1.87%
           FLASH:       25628 B       512 KB      4.89%
```

All changes compile cleanly. No new dependencies or external libraries.

---

## Next Steps

**Before power-up:**
1. Comment out `MotorControl_SetSpeedTarget(100.0f)` and `StateMachine_RequestState(STATE_RUNNING)` in `main()`.
2. Verify the Semikron inverter's gate-signal ground is tied to the STM32 ground (a common scope failure cause).

**Phase 2 lab ladder (in order):**
1. Scope sanity — clock, SysTick, PA5 toggle timing
2. ISR heartbeat — verify 10 kHz ADC trigger chain
3. **Gate audit** — this build should now pass; all six outputs PWM complementary
4. Current measurement — sign verification with bench supply
5. Hall audit — state sequence, angle ramp, speed sign
6. **Forced-angle calibration** — re-measure and commit the hall table
7. Current loop isolation — Kp/Ki tuning preview
8. Speed loop — decimated, filtered, signed

Report findings at each step before proceeding.
