#include "hw/hw_hall_sensor.h"

typedef struct {
    uint8_t h1;
    uint8_t h2;
    uint8_t h3;
    uint8_t combined_state;
} Hall_Debug_t;
volatile Hall_Debug_t hall_debug;

extern TIM_HandleTypeDef htim3;
static float base_angle_rad = 0.0f;
static float omega_rad_per_tick = 0.0f;
static int8_t direction = 1;
static uint8_t prev_hall_state = 0;
// CCR1 only holds a true 60-degree period once two consecutive edges have been
// seen without a counter overflow in between. Cleared at init and on timeout.
static volatile uint8_t period_valid = 0;

// Forward-entry electrical angle per hall state. These defaults are a GUESS
// for an unknown motor: run STATE_HALL_CALIB to measure and overwrite them
// at runtime (HW_Hall_SetAngleTable), then persist the measured values here.
static float hall_angle_table[8] = {
    0.0f,                 // 0: Invalid
    1.0f * SIXTY_DEG_RAD, // 1
    5.0f * SIXTY_DEG_RAD, // 2
    0.0f,                 // 3
    3.0f * SIXTY_DEG_RAD, // 4
    2.0f * SIXTY_DEG_RAD, // 5
    4.0f * SIXTY_DEG_RAD, // 6
    0.0f                  // 7: Invalid
};

static uint8_t Hall_ReadState(void) {
    uint8_t state = 0;
    if (GPIOA->IDR & GPIO_PIN_6) state |= 0x04; // Hall 1
    if (GPIOC->IDR & GPIO_PIN_7) state |= 0x02; // Hall 2
    if (GPIOC->IDR & GPIO_PIN_8) state |= 0x01; // Hall 3
    return state;
}

uint8_t HW_Hall_GetState(void) {
    return Hall_ReadState();
}

void HW_Hall_SetAngleTable(const float table[8]) {
    for (int i = 0; i < 8; i++) {
        hall_angle_table[i] = table[i];
    }
    // Re-seed: the rotor moved while the table was being measured, so the old
    // base angle is stale. Same sector-centre seeding as HW_Hall_Init.
    uint8_t state = Hall_ReadState();
    if (state > 0 && state < 7) {
        float angle = hall_angle_table[state] + 0.5f * SIXTY_DEG_RAD;
        if (angle >= (2.0f * PI)) {
            angle -= (2.0f * PI);
        }
        base_angle_rad = angle;
        prev_hall_state = state;
    }
    omega_rad_per_tick = 0.0f;
    period_valid = 0;
}

void HW_Hall_Init(void) {
    // Seed the angle from the static hall state so FOC does not start from an
    // arbitrary angle at standstill (no edges fire until the rotor moves).
    uint8_t state = Hall_ReadState();
    if (state > 0 && state < 7) {
        // Sector centre: the rotor is somewhere inside this 60-degree span,
        // so the centre halves the worst-case initial error to 30 degrees.
        base_angle_rad = hall_angle_table[state] + 0.5f * SIXTY_DEG_RAD;
        if (base_angle_rad >= (2.0f * PI)) {
            base_angle_rad -= (2.0f * PI);
        }
        prev_hall_state = state;
    }
    omega_rad_per_tick = 0.0f;
    period_valid = 0;

    // Stall timeout: use the counter overflow (65.5 ms without an edge) as a
    // "rotor stopped" signal. URS=1 so the per-edge slave-mode reset does not
    // also raise the update interrupt.
    htim3.Instance->CR1 |= TIM_CR1_URS;
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);

    // Start the input capture interrupt for CH1
    HAL_TIMEx_HallSensor_Start_IT(&htim3);
}

void HW_Hall_Update_ISR(void) {
    // 1. Read the exact pins mapped in your hardware
    uint8_t hall_state = Hall_ReadState();

    hall_debug.h1 = (hall_state & 0x04) ? 1 : 0;
    hall_debug.h2 = (hall_state & 0x02) ? 1 : 0;
    hall_debug.h3 = (hall_state & 0x01) ? 1 : 0;
    hall_debug.combined_state = hall_state;

    // States 0 and 7 mean a disconnected sensor or a glitch: keep the previous
    // angle and period rather than slamming the angle to the table's 0 entry.
    if (hall_state == 0 || hall_state == 7) {
        return;
    }

    // 2. Update base angle from lookup table
    base_angle_rad = hall_angle_table[hall_state];

    // 3. Evaluate direction on edge transition
    if (prev_hall_state != 0 && prev_hall_state != hall_state) {
        // Validating state transitions to determine direction
        if ((hall_state == 2 && prev_hall_state == 3) ||
            (hall_state == 6 && prev_hall_state == 2) ||
            (hall_state == 4 && prev_hall_state == 6) ||
            (hall_state == 5 && prev_hall_state == 4) ||
            (hall_state == 1 && prev_hall_state == 5) ||
            (hall_state == 3 && prev_hall_state == 1)) {
            direction = 1;  // Forward Rotation
        }
        else if ((hall_state == 3 && prev_hall_state == 2) ||
                 (hall_state == 2 && prev_hall_state == 6) ||
                 (hall_state == 6 && prev_hall_state == 4) ||
                 (hall_state == 4 && prev_hall_state == 5) ||
                 (hall_state == 5 && prev_hall_state == 1) ||
                 (hall_state == 1 && prev_hall_state == 3)) {
            direction = -1; // Reverse Rotation
        }
    }
    prev_hall_state = hall_state;

    // 4. Grab the capture ticks from the hardware timer register
    uint32_t ticks_per_60_deg = TIM3->CCR1;

    if (period_valid && ticks_per_60_deg > 0) {
        // Omega = (PI/3) / ticks
        omega_rad_per_tick = SIXTY_DEG_RAD / (float)ticks_per_60_deg;
    } else {
        // First edge after start-up or a stall timeout: the captured period is
        // meaningless (measured from an arbitrary origin, possibly across a
        // wrap). Keep omega at zero and start trusting from the next edge.
        omega_rad_per_tick = 0.0f;
        period_valid = 1;
    }
}

/**
 * @brief Called from the TIM3 update (overflow) interrupt: 65.5 ms elapsed
 *        without a hall edge, so the rotor is stopped or nearly so.
 */
void HW_Hall_Timeout_ISR(void) {
    omega_rad_per_tick = 0.0f;
    period_valid = 0;
}

/**
 * @brief Returns the interpolated high-resolution angle to motor_control.c
 * @note Kept lightweight and fast for 10 kHz execution context.
 */
float HW_Hall_GetElectricalAngle(void) {
    // Read the time elapsed since the last Hall edge directly from the counter
    uint32_t elapsed_ticks = TIM3->CNT;

    // Linearly extrapolate the angle forward from our last known base angle
    float delta = (float)direction * omega_rad_per_tick * (float)elapsed_ticks;

    // Never extrapolate more than one hall sector past the last edge: if the
    // rotor decelerates, the stale omega must not run the angle away.
    if (delta > SIXTY_DEG_RAD) {
        delta = SIXTY_DEG_RAD;
    } else if (delta < -SIXTY_DEG_RAD) {
        delta = -SIXTY_DEG_RAD;
    }

    float interpolated_angle = base_angle_rad + delta;

    // Keep the angle bounded safely between 0 and 2*PI
    if (interpolated_angle >= (2.0f * PI)) {
        interpolated_angle -= (2.0f * PI);
    } else if (interpolated_angle < 0.0f) {
        interpolated_angle += (2.0f * PI);
    }

    return interpolated_angle;
}

float HW_Hall_GetSpeedRPM(uint8_t pole_pairs) {
    // Convert radians/tick to electrical radians/sec, then to mechanical RPM.
    // Signed: reverse rotation must read negative or the speed loop fights it.
    float omega_elec_rad_sec = omega_rad_per_tick * HALL_TIMER_FREQ;
    float rpm = (omega_elec_rad_sec * 60.0f) / (2.0f * PI * (float)pole_pairs);

    return (float)direction * rpm;
}
