#ifndef HW_HALL_SENSOR_H
#define HW_HALL_SENSOR_H

#include "main.h"

// Define PI and 60 degrees in radians
#ifndef PI
#define PI 3.14159265358979f
#endif
#define SIXTY_DEG_RAD    1.04719755f // PI / 3

void HW_Hall_Init(void);
void HW_Hall_Update_ISR(void);

float  HW_Hall_GetBaseAngle(void);
int8_t HW_Hall_GetDirection(void);

#endif
