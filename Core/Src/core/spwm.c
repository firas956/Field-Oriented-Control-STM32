#include "core/spwm.h"

void FOC_SPWM(const AlphaBeta_t *v_in, float vdc, Phase_t *duty_out)
{
    // Protect against division by zero: park at the neutral zero vector
    if (vdc <= 0.0f) {
        duty_out->a = 0.5f;
        duty_out->b = 0.5f;
        duty_out->c = 0.5f;
        return;
    }

    // Phase voltage targets in the amplitude-invariant frame
    Phase_t v;
    FOC_InverseClarke(v_in, &v);

    // Each half-bridge swings +/- Vdc/2 around the bus midpoint, so the
    // average leg voltage is (duty - 0.5) * Vdc
    float inv_vdc = 1.0f / vdc;
    duty_out->a = 0.5f + v.a * inv_vdc;
    duty_out->b = 0.5f + v.b * inv_vdc;
    duty_out->c = 0.5f + v.c * inv_vdc;

    // Explicit clamping to safe boundaries [0.0, 1.0]
    if (duty_out->a > 1.0f) duty_out->a = 1.0f; else if (duty_out->a < 0.0f) duty_out->a = 0.0f;
    if (duty_out->b > 1.0f) duty_out->b = 1.0f; else if (duty_out->b < 0.0f) duty_out->b = 0.0f;
    if (duty_out->c > 1.0f) duty_out->c = 1.0f; else if (duty_out->c < 0.0f) duty_out->c = 0.0f;
}
