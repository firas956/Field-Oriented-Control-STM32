#include "core/smc.h"

void SMC_Init(SMC_Controller_t *s,
              float J, float F, float Kt, float Ts,
              float c1, float c2, float eta, float k, float phi,
              float out_min, float out_max)
{
    s->J = J; s->F = F; s->Kt = Kt; s->Ts = Ts;
    s->c1 = c1; s->c2 = c2; s->eta = eta; s->k = k; s->phi = phi;
    s->out_min = out_min; s->out_max = out_max;
    s->Tr_hat = 0.0f;
    s->eps = 0.0f;
    s->e_prev = 0.0f;
}

void SMC_Reset(SMC_Controller_t *s)
{
    s->eps = 0.0f;
    s->e_prev = 0.0f;
    s->Tr_hat = 0.0f;
}

float SMC_Update(SMC_Controller_t *s, float omega_ref, float omega_meas)
{
    float e = omega_ref - omega_meas;               /* (27) speed error       */
    float eps = s->eps + s->e_prev;                 /* (28) Euler integral    */
    float S = s->c1 * e + s->c2 * eps;              /* (26) sliding surface   */

    /* boundary-layer saturation sat(S/phi) in [-1,1] (suppresses chattering) */
    float sat;
    if (s->phi > 0.0f) {
        sat = S / s->phi;
        if      (sat >  1.0f) sat =  1.0f;
        else if (sat < -1.0f) sat = -1.0f;
    } else {
        sat = (S > 0.0f) ? 1.0f : ((S < 0.0f) ? -1.0f : 0.0f);
    }

    float b = s->Ts * s->Kt / s->J;                 /* control gain           */
    float f = s->c1 * (e + (s->Ts * s->F / s->J) * omega_meas);
    float d = s->c1 * s->Ts / s->J;

    float iq = ( f + d * s->Tr_hat                  /* (32) control law       */
                 + s->c2 * (eps + e)
                 - s->eta * S
                 + s->k * s->Ts * sat ) / (s->c1 * b);

    /* output saturation + conditional-integration anti-windup */
    if      (iq > s->out_max) iq = s->out_max;      /* saturated: freeze eps  */
    else if (iq < s->out_min) iq = s->out_min;      /* saturated: freeze eps  */
    else                      s->eps = eps;         /* in range: commit eps   */

    s->e_prev = e;
    return iq;
}
