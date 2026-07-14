#ifndef LPF_H
#define LPF_H

#include <stdint.h>

/**
 * @brief Low Pass Filter (First-Order IIR / EMA) State Structure
 */
typedef struct {
    float alpha; // Smoothing factor (0.0 to 1.0)
    float out;   // Current filter output (y[n])
} LPC_Filter_t;

/**
 * @brief  Initializes the Low Pass Filter.
 * @param  filter: Pointer to the filter structure.
 * @param  alpha: Smoothing factor (0.0 < alpha <= 1.0). Lower = more filtering.
 * @param  initial_value: Starting value (usually 0.0, or the first ADC read).
 */
void LPF_Init(LPC_Filter_t *filter, float alpha, float initial_value);

/**
 * @brief  Updates the filter with a new raw input sample.
 * @param  filter: Pointer to the filter structure.
 * @param  input: The new raw data point (x[n]).
 * @retval The newly calculated filtered output.
 */
float LPF_Update(LPC_Filter_t *filter, float input);

#endif // LPC_H