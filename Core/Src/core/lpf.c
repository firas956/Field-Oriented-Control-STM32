#include "core/lpf.h"

void LPF_Init(LPC_Filter_t *filter, float alpha, float initial_value) {
    // Constrain alpha to safe bounds to prevent math errors or instability
    if (alpha <= 0.0f) {
        filter->alpha = 0.001f; // Minimum practical alpha
    } else if (alpha > 1.0f) {
        filter->alpha = 1.0f;   // Maximum alpha (no filtering)
    } else {
        filter->alpha = alpha;
    }

    // Set the initial state of the filter
    filter->out = initial_value;
}

float LPF_Update(LPC_Filter_t *filter, float input) {
    // Apply the first-order IIR formula: y[n] = (alpha * x[n]) + ((1 - alpha) * y[n-1])
    filter->out = (filter->alpha * input) + ((1.0f - filter->alpha) * filter->out);
    
    return filter->out;
}