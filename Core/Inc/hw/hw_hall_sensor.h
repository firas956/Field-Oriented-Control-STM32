#ifndef HW_HALL_SENSOR_H
#define HW_HALL_SENSOR_H

#include "main.h"

// Define PI and 60 degrees in radians
#ifndef PI
#define PI 3.14159265358979f
#endif
#define SIXTY_DEG_RAD    1.04719755f // PI / 3

// Assuming TIM3 is your Hall timer running at 1 MHz (1 microsecond per tick)
#define HALL_TIMER_FREQ  1000000.0f  

void HW_Hall_Init(void);
void HW_Hall_Update_ISR(void);
void HW_Hall_Timeout_ISR(void);
float HW_Hall_GetElectricalAngle(void);
float HW_Hall_GetSpeedRPM(uint8_t pole_pairs);

#endif