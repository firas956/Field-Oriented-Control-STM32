#ifndef MODULATOR_H
#define MODULATOR_H

#include "core/foc_math.h"
#include "core/svpwm.h"
#include "core/spwm.h"

/**
 * Common signature every modulation strategy implements:
 *   v_in     alpha/beta voltage request [V], amplitude-invariant frame
 *   vdc      DC bus voltage [V]
 *   duty_out per-phase duty cycles in [0..1]
 *
 * Available strategies:
 *   FOC_SVPWM (svpwm.h) - space vector, linear range |V| <= Vdc/sqrt(3)
 *   FOC_SPWM  (spwm.h)  - sinusoidal,   linear range |V| <= Vdc/2
 *
 * Select the strategy with one line in motor_control.c:
 *   static const PWM_Modulator_t PWM_Modulate = FOC_SVPWM;  // or FOC_SPWM
 */
typedef void (*PWM_Modulator_t)(const AlphaBeta_t *v_in, float vdc, Phase_t *duty_out);

#endif /* MODULATOR_H */
