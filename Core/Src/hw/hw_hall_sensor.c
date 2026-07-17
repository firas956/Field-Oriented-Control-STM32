#include "hw/hw_hall_sensor.h"

typedef struct {
    uint8_t h1;             // PA6  (TIM3_CH1) -> hall_state bit 2
    uint8_t h2;             // PC7  (TIM3_CH2) -> hall_state bit 1
    uint8_t h3;             // PC8  (TIM3_CH3) -> hall_state bit 0
    uint8_t combined_state;
} Hall_Debug_t;
volatile Hall_Debug_t hall_debug;

extern TIM_HandleTypeDef htim3;
static float base_angle_rad = 0.0f;
static int8_t direction = 1;

/*
 * Electrical angle of each hall state, identified experimentally
 * states 4 -> 6 -> 2 -> 3 -> 1 -> 5 -> 4.
 */
static const float hall_angle_table[8] = {
    0.0f,                  // 0: invalid
    3.0f * SIXTY_DEG_RAD,  // 1
    5.0f * SIXTY_DEG_RAD,  // 2
    4.0f * SIXTY_DEG_RAD,  // 3
    1.0f * SIXTY_DEG_RAD,  // 4
    2.0f * SIXTY_DEG_RAD,  // 5
    0.0f * SIXTY_DEG_RAD ,  // 6
    0.0f                   // 7: invalid
};

static uint8_t HW_Hall_ReadPins(void) {
    uint8_t hall_state = 0;
    if (GPIOA->IDR & GPIO_PIN_6) hall_state |= 0x04; // Hall 1
    if (GPIOC->IDR & GPIO_PIN_7) hall_state |= 0x02; // Hall 2
    if (GPIOC->IDR & GPIO_PIN_8) hall_state |= 0x01; // Hall 3
    return hall_state;
}

void HW_Hall_Init(void) {
    /* Seed the angle from the current rotor position BEFORE any edge occurs.
     * Without this the angle stays 0 until the first hall transition, so a
     * stationary rotor would be driven with a frozen, arbitrary vector. */
    uint8_t hall_state = HW_Hall_ReadPins();
    base_angle_rad = hall_angle_table[hall_state];

    hall_debug.h1 = (hall_state & 0x04) ? 1 : 0;
    hall_debug.h2 = (hall_state & 0x02) ? 1 : 0;
    hall_debug.h3 = (hall_state & 0x01) ? 1 : 0;
    hall_debug.combined_state = hall_state;

    // Start the input capture interrupt for CH1 (fires on every hall edge)
    HAL_TIMEx_HallSensor_Start_IT(&htim3);
}

void HW_Hall_Update_ISR(void) {
    static uint8_t prev_hall_state = 0;
    uint8_t hall_state = HW_Hall_ReadPins();

    hall_debug.h1 = (hall_state & 0x04) ? 1 : 0; // PA6
    hall_debug.h2 = (hall_state & 0x02) ? 1 : 0; // PC7
    hall_debug.h3 = (hall_state & 0x01) ? 1 : 0; // PC8
    hall_debug.combined_state = hall_state;

    // Update base angle from lookup table
    base_angle_rad = hall_angle_table[hall_state];

    // Evaluate direction on edge transition.
    // Forward (+1) = increasing electrical angle = 4->6->2->3->1->5->4
    if (prev_hall_state != 0 && prev_hall_state != hall_state) {
        if ((hall_state == 6 && prev_hall_state == 4) ||
            (hall_state == 2 && prev_hall_state == 6) ||
            (hall_state == 3 && prev_hall_state == 2) ||
            (hall_state == 1 && prev_hall_state == 3) ||
            (hall_state == 5 && prev_hall_state == 1) ||
            (hall_state == 4 && prev_hall_state == 5)) {
            direction = 1;  // Forward rotation
        }
        else if ((hall_state == 4 && prev_hall_state == 6) ||
                 (hall_state == 6 && prev_hall_state == 2) ||
                 (hall_state == 2 && prev_hall_state == 3) ||
                 (hall_state == 3 && prev_hall_state == 1) ||
                 (hall_state == 1 && prev_hall_state == 5) ||
                 (hall_state == 5 && prev_hall_state == 4)) {
            direction = -1; // Reverse rotation
        }
    }
    prev_hall_state = hall_state;
}

float HW_Hall_GetBaseAngle(void) {
    return base_angle_rad;
}

int8_t HW_Hall_GetDirection(void) {
    return direction;
}
