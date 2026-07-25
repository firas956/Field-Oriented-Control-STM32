#include "core/smc_current.h"

void CSMC_Init(CSMC_Controller_t *s,
               float L, float R, float psi_r, float Ts,
               float c1, float c2, float eta, float k, float phi,
               float v_max)
{
    s->L = L; s->R = R; s->psi_r = psi_r; s->Ts = Ts;
    s->c1 = c1; s->c2 = c2; s->eta = eta; s->k = k; s->phi = phi;
    s->v_max = v_max;
    s->eps_d = 0.0f; s->eps_q = 0.0f;
    s->ed_prev = 0.0f; s->eq_prev = 0.0f;
}

void CSMC_Reset(CSMC_Controller_t *s)
{
    s->eps_d = 0.0f; s->eps_q = 0.0f;
    s->ed_prev = 0.0f; s->eq_prev = 0.0f;
}

/* One axis of the DISMCC. E is the matched disturbance for that axis. */
static float CSMC_Axis(CSMC_Controller_t *s, float ref, float meas, float E,
                       float *eps_state, float *e_prev)
{
    float e   = ref - meas;                     /* (27) current error     */
    float eps = *eps_state + *e_prev;           /* (28) Euler integral    */
    float S   = s->c1 * e + s->c2 * eps;        /* (26) sliding surface   */

    /* boundary-layer saturation sat(S/phi) in [-1,1] (suppresses chattering) */
    float sat;
    if (s->phi > 0.0f) {
        sat = S / s->phi;
        if      (sat >  1.0f) sat =  1.0f;
        else if (sat < -1.0f) sat = -1.0f;
    } else {
        sat = (S > 0.0f) ? 1.0f : ((S < 0.0f) ? -1.0f : 0.0f);
    }

    float b = s->Ts / s->L;                     /* control gain           */
    float f = s->c1 * (e + (s->Ts * s->R / s->L) * meas);
    float d = s->c1 * s->Ts / s->L;

    float v = ( f + d * E                       /* (32) control law       */
                + s->c2 * (eps + e)
                - s->eta * S
                + s->k * s->Ts * sat ) / (s->c1 * b);

    /* output saturation + conditional-integration anti-windup */
    if      (v >  s->v_max) v =  s->v_max;      /* saturated: freeze eps  */
    else if (v < -s->v_max) v = -s->v_max;      /* saturated: freeze eps  */
    else                    *eps_state = eps;   /* in range: commit eps   */

    *e_prev = e;
    return v;
}

void CSMC_Update(CSMC_Controller_t *s, const DQ_t *i_ref, const DQ_t *i_meas,
                 float omega_elec, DQ_t *v_dq)
{
    /* Cross-coupling and back-EMF: the matched disturbance E of each axis.
     * Known analytically, so it is fed forward exactly (gain 1 in the law). */
    float Ed = -omega_elec * s->L * i_meas->q;
    float Eq =  omega_elec * (s->L * i_meas->d + s->psi_r);

    v_dq->d = CSMC_Axis(s, i_ref->d, i_meas->d, Ed, &s->eps_d, &s->ed_prev);
    v_dq->q = CSMC_Axis(s, i_ref->q, i_meas->q, Eq, &s->eps_q, &s->eq_prev);
}
