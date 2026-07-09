#ifndef HW_ADC_H
#define HW_ADC_H

#include "main.h"

// Pre-calculated constant: (3.3V / 4095) / 0.125 V/A
#define ADC_RAW_TO_AMPS  0.00644688f 

void HW_ADC_Init(void);
void HW_ADC_ReadCurrents(float *i_a, float *i_b);

#endif