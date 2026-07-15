#ifndef SVPWM_H
#define SVPWM_H

#include "core/foc_math.h"

/**
 * Space Vector PWM: min/max-centered modulation built from the two adjacent
 * active vectors plus symmetric zero-vector dwell.
 *
 * Linear range: |V| <= Vdc/sqrt(3), i.e. 15.5% more than SPWM's Vdc/2 —
 * the zero-sequence (common-mode) component shifts each leg without
 * affecting the line-to-line voltages the motor sees.
 *
 * Same signature as FOC_SPWM so the two are drop-in interchangeable
 * (see PWM_Modulator_t in modulator.h).
 */
void FOC_SVPWM(const AlphaBeta_t *v_in, float vdc, Phase_t *duty_out);

#endif /* SVPWM_H */
