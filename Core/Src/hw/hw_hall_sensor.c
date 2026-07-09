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

static const float hall_angle_table[8] = {
    0.0f,                 // 0: Invalid
    5.0f * SIXTY_DEG_RAD, // 1: 300 deg
    1.0f * SIXTY_DEG_RAD, // 2: 60 deg
    0.0f,                 // 3: 0 deg
    3.0f * SIXTY_DEG_RAD, // 4: 180 deg
    4.0f * SIXTY_DEG_RAD, // 5: 240 deg
    2.0f * SIXTY_DEG_RAD, // 6: 120 deg
    0.0f                  // 7: Invalid
};

void HW_Hall_Init(void) {
    // Start the input capture interrupt for CH1
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
}

void HW_Hall_Update_ISR(void) {
    uint8_t hall_state = 0;
    static uint8_t prev_hall_state = 0;

    // 1. Read the exact pins mapped in your hardware
    if (GPIOA->IDR & GPIO_PIN_6) hall_state |= 0x01; // Hall 1
    if (GPIOC->IDR & GPIO_PIN_7) hall_state |= 0x02; // Hall 2
    if (GPIOC->IDR & GPIO_PIN_8) hall_state |= 0x04; // Hall 3
    hall_debug.h1 = (hall_state & 0x01) ? 1 : 0;
    hall_debug.h2 = (hall_state & 0x02) ? 1 : 0;
    hall_debug.h3 = (hall_state & 0x04) ? 1 : 0;
    hall_debug.combined_state = hall_state;

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

    if (ticks_per_60_deg > 0) {
        // Omega = (PI/3) / ticks
        omega_rad_per_tick = SIXTY_DEG_RAD / (float)ticks_per_60_deg;
    } else {
        omega_rad_per_tick = 0.0f;
    }
}

/**
 * @brief Returns the interpolated high-resolution angle to motor_control.c
 * @note Kept lightweight and fast for 10 kHz execution context.
 */
float HW_Hall_GetElectricalAngle(void) {
    // Read the time elapsed since the last Hall edge directly from the counter
    uint32_t elapsed_ticks = TIM3->CNT;

    // Linearly extrapolate the angle forward from our last known base angle
    float interpolated_angle = base_angle_rad + ((float)direction * omega_rad_per_tick * (float)elapsed_ticks);

    // Keep the angle bounded safely between 0 and 2*PI
    if (interpolated_angle >= (2.0f * PI)) {
        interpolated_angle -= (2.0f * PI);
    } else if (interpolated_angle < 0.0f) {
        interpolated_angle += (2.0f * PI);
    }

    return interpolated_angle;
}

float HW_Hall_GetSpeedRPM(uint8_t pole_pairs) {
    // Convert radians/tick to electrical radians/sec, then to mechanical RPM
    float omega_elec_rad_sec = omega_rad_per_tick * HALL_TIMER_FREQ;
    float rpm = (omega_elec_rad_sec * 60.0f) / (2.0f * PI * (float)pole_pairs);
    
    return rpm;
}