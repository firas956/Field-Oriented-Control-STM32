#include "app/hall_calibration.h"
#include "app/motor_config.h"
#include "app/state_machine.h"
#include "hw/hw_hall_sensor.h"
#include "hw/hw_pwm.h"
#include <math.h>

volatile HallCalib_Result_t hall_calib = {0};

#define TWO_PI              (2.0f * PI)
#define THIRTY_DEG_RAD      (PI / 6.0f)
#define CALIB_DTHETA        (TWO_PI * HALL_CALIB_ELEC_HZ / CURRENT_LOOP_FREQ_HZ)
#define CALIB_ALIGN_TICKS   ((uint32_t)(HALL_CALIB_ALIGN_S * CURRENT_LOOP_FREQ_HZ))
#define CALIB_HOLD_TICKS    ((uint32_t)(0.5f * CURRENT_LOOP_FREQ_HZ))
#define CALIB_TRAVEL_SKIP   TWO_PI                              /* settling rev */
#define CALIB_TRAVEL_END    (TWO_PI * (1.0f + (float)HALL_CALIB_REVS))
#define CALIB_DEBOUNCE      3                                   /* samples */

// Scan progress (ISR context only)
static uint32_t tick_counter;
static float    theta;              // commanded electrical angle
static float    travel;             // angle travelled in the current pass
static uint8_t  last_state;
static uint8_t  candidate_state;
static uint8_t  candidate_count;
static uint8_t  seq_idx;

// Per-state circular accumulation of the sector-centre estimate. Forward
// entries contribute (theta + 30 deg), reverse entries (theta - 30 deg):
// the rotor-lag error has opposite sign in the two passes and cancels.
static float sum_sin[8];
static float sum_cos[8];

static float Wrap2Pi(float a) {
    if (a >= TWO_PI) a -= TWO_PI;
    else if (a < 0.0f) a += TWO_PI;
    return a;
}

void HallCalib_Start(void)
{
    tick_counter = 0;
    theta = 0.0f;
    travel = 0.0f;
    candidate_count = 0;
    seq_idx = 0;
    for (int i = 0; i < 8; i++) {
        sum_sin[i] = 0.0f;
        sum_cos[i] = 0.0f;
        hall_calib.entry_angle[i] = 0.0f;
        hall_calib.center_angle[i] = 0.0f;
    }
    for (int i = 0; i < 6; i++) {
        hall_calib.sequence[i] = 0;
    }
    hall_calib.seen_fwd = 0;
    hall_calib.seen_rev = 0;
    hall_calib.valid = 0;
    hall_calib.phase = HALL_CALIB_ALIGN;
}

void HallCalib_Abort(void)
{
    // Called whenever the drive is not in STATE_HALL_CALIB: a scan that was
    // interrupted (fault, operator request) must restart from scratch, but
    // completed DONE/FAILED results stay readable for the debugger.
    if (hall_calib.phase == HALL_CALIB_ALIGN ||
        hall_calib.phase == HALL_CALIB_FWD  ||
        hall_calib.phase == HALL_CALIB_HOLD ||
        hall_calib.phase == HALL_CALIB_REV) {
        hall_calib.valid = 0;
        hall_calib.phase = HALL_CALIB_INACTIVE;
    }
}

static void HallCalib_Fail(void)
{
    hall_calib.valid = 0;
    hall_calib.phase = HALL_CALIB_FAILED;
    StateMachine_RequestState(STATE_IDLE);
}

static void HallCalib_Finish(void)
{
    const uint8_t all_states = 0x7E; // states 1..6

    // Every state must have been captured in both directions, and the forward
    // sequence must contain six distinct states
    uint8_t seq_mask = 0;
    for (int i = 0; i < 6; i++) {
        seq_mask |= (uint8_t)(1u << hall_calib.sequence[i]);
    }
    if (hall_calib.seen_fwd != all_states ||
        hall_calib.seen_rev != all_states ||
        seq_mask != all_states) {
        HallCalib_Fail();
        return;
    }

    float table[8] = {0};
    for (int s = 1; s <= 6; s++) {
        float center = atan2f(sum_sin[s], sum_cos[s]);
        if (center < 0.0f) center += TWO_PI;
        float entry = Wrap2Pi(center - THIRTY_DEG_RAD);
        hall_calib.center_angle[s] = center;
        hall_calib.entry_angle[s] = entry;
        table[s] = entry; // forward-entry convention used by hw_hall_sensor.c
    }

    HW_Hall_SetAngleTable(table);
    hall_calib.valid = 1;
    hall_calib.phase = HALL_CALIB_DONE;
    StateMachine_RequestState(STATE_IDLE);
}

// Debounced hall sampling; records a transition's centre estimate once the
// settling revolution of the pass is over. Returns 0 on a wiring fault.
static uint8_t HallCalib_SampleHalls(float dir)
{
    uint8_t sample = HW_Hall_GetState();
    if (sample == 0 || sample == 7) {
        return 0; // impossible state: disconnected or shorted hall line
    }

    if (sample == last_state) {
        candidate_count = 0;
        return 1;
    }
    if (sample != candidate_state) {
        candidate_state = sample;
        candidate_count = 1;
        return 1;
    }
    if (++candidate_count < CALIB_DEBOUNCE) {
        return 1;
    }

    // Accepted transition into candidate_state
    last_state = sample;
    candidate_count = 0;

    if (travel > CALIB_TRAVEL_SKIP) {
        float center_est = Wrap2Pi(theta + dir * THIRTY_DEG_RAD);
        sum_sin[sample] += sinf(center_est);
        sum_cos[sample] += cosf(center_est);
        if (dir > 0.0f) {
            hall_calib.seen_fwd |= (uint8_t)(1u << sample);
            if (seq_idx < 6) {
                hall_calib.sequence[seq_idx++] = sample;
            }
        } else {
            hall_calib.seen_rev |= (uint8_t)(1u << sample);
        }
    }
    return 1;
}

void HallCalib_RunIteration(PWM_Modulator_t modulate)
{
    if (hall_calib.phase == HALL_CALIB_INACTIVE ||
        hall_calib.phase == HALL_CALIB_DONE ||
        hall_calib.phase == HALL_CALIB_FAILED) {
        HallCalib_Start();
    }

    float v_amp = HALL_CALIB_VOLTS;

    switch (hall_calib.phase) {

        case HALL_CALIB_ALIGN:
            // Ramp the voltage in at a fixed angle so the rotor pulls into
            // alignment without a torque kick
            if (tick_counter < (CALIB_ALIGN_TICKS / 2u)) {
                v_amp = HALL_CALIB_VOLTS * ((float)tick_counter / (float)(CALIB_ALIGN_TICKS / 2u));
            }
            if (++tick_counter >= CALIB_ALIGN_TICKS) {
                tick_counter = 0;
                travel = 0.0f;
                last_state = HW_Hall_GetState();
                candidate_state = last_state;
                candidate_count = 0;
                hall_calib.phase = HALL_CALIB_FWD;
            }
            break;

        case HALL_CALIB_FWD:
            theta = Wrap2Pi(theta + CALIB_DTHETA);
            travel += CALIB_DTHETA;
            if (!HallCalib_SampleHalls(1.0f)) {
                HallCalib_Fail();
                return;
            }
            if (travel >= CALIB_TRAVEL_END) {
                tick_counter = 0;
                hall_calib.phase = HALL_CALIB_HOLD;
            }
            break;

        case HALL_CALIB_HOLD:
            // Let the rotor settle before reversing the scan direction
            if (++tick_counter >= CALIB_HOLD_TICKS) {
                tick_counter = 0;
                travel = 0.0f;
                candidate_count = 0;
                hall_calib.phase = HALL_CALIB_REV;
            }
            break;

        case HALL_CALIB_REV:
            theta = Wrap2Pi(theta - CALIB_DTHETA);
            travel += CALIB_DTHETA;
            if (!HallCalib_SampleHalls(-1.0f)) {
                HallCalib_Fail();
                return;
            }
            if (travel >= CALIB_TRAVEL_END) {
                HallCalib_Finish();
                return;
            }
            break;

        default:
            return;
    }

    // Apply the commanded vector through the selected modulator, so the scan
    // exercises exactly the same output path as closed-loop operation
    AlphaBeta_t v;
    v.alpha = v_amp * arm_cos_f32(theta);
    v.beta  = v_amp * arm_sin_f32(theta);

    Phase_t duty;
    modulate(&v, VBUS_VOLTS, &duty);
    HW_PWM_SetDuties(&duty);
}
