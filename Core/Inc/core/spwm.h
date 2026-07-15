#ifndef SPWM_H
#define SPWM_H

#include "core/foc_math.h"

/**
 * Sinusoidal PWM: plain inverse-Clarke phase voltages referenced to Vdc/2,
 * with no zero-sequence (third harmonic) injection.
 *
 * Linear range: |V| <= Vdc/2, i.e. 86.6% of SVPWM's Vdc/sqrt(3). Comparing
 * this against FOC_SVPWM isolates exactly the zero-sequence contribution:
 * below Vdc/2 both must produce identical line-to-line voltages, so the motor
 * must behave identically; only the per-leg duty waveforms differ.
 *
 * Same signature as FOC_SVPWM so the two are drop-in interchangeable
 * (see PWM_Modulator_t in modulator.h).
 */
void FOC_SPWM(const AlphaBeta_t *v_in, float vdc, Phase_t *duty_out);

#endif /* SPWM_H */
