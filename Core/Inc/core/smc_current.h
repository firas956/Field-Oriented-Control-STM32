#ifndef SMC_CURRENT_H
#define SMC_CURRENT_H

#include "core/foc_math.h"

/*
 * Discrete Integral Sliding-Mode Current Controller (DISMCC) for the PMSM
 * d/q current loops. Drop-in alternative to the id/iq PIs: takes the dq
 * current references and measurements, returns the dq voltage commands.
 *
 * Same design as the speed DISMSC (smc.h) - the current loop has the same
 * first-order form, with J->L, F->R, Kt->1 and the load torque replaced by
 * the back-EMF / cross-coupling term E:
 *
 *   Plant (forward-Euler): i_{k+1} = i_k + (Ts/L)(v_k - R*i_k - E)
 *                          E_d = -we*L*i_q ,  E_q = we*(L*i_d + psi_r)
 *   Surface (integral)   : S_k = c1*e_k + c2*eps_k,  eps_k = eps_{k-1}+e_{k-1}
 *   Reaching law (Gao)   : S_{k+1} = eta*S_k - k*Ts*sat(S_k),  eta = 1 - q*Ts
 *   Control law          : v = (f + d*E + c2*(eps+e) - eta*S
 *                               + k*Ts*sat(S/phi)) / (c1*b)
 *                          b = Ts/L, f = c1*(e+(Ts*R/L)*i), d = c1*Ts/L
 *
 * Expanded, the law is  v = (q*L + R)*e + (c2/c1)*q*L*eps + R*i + E
 *                           + (k*L/c1)*sat(S/phi)
 * so with q = 2*pi*BW and c2 = c1*R*Ts/L the linear part reproduces the
 * tuned PI exactly (same wn and zeta); E is the exact decoupling feed-forward
 * and the sat() term adds +/-(k*L/c1) volts of robustness (dead-time, model
 * error). Unlike the speed loop's Tr, E is computed analytically here - no
 * observer needed.
 */
typedef struct {
    /* plant constants */
    float L;        /* phase inductance            [H]      */
    float R;        /* phase resistance            [ohm]    */
    float psi_r;    /* PM flux linkage             [Wb]     */
    float Ts;       /* current-loop sample time    [s]      */
    /* surface + reaching-law gains (shared by both axes) */
    float c1;       /* surface weight on error    */
    float c2;       /* surface weight on integral */
    float eta;      /* reaching pole = 1 - q*Ts   (0..1) */
    float k;        /* switching gain  (k*Ts < 1) */
    float phi;      /* boundary-layer half-width for sat() [A] */
    /* per-axis output clamp (same limit the PI used) */
    float v_max;
    /* per-axis state */
    float eps_d, eps_q;     /* integral of error */
    float ed_prev, eq_prev; /* previous error    */
} CSMC_Controller_t;

void CSMC_Init(CSMC_Controller_t *s,
               float L, float R, float psi_r, float Ts,
               float c1, float c2, float eta, float k, float phi,
               float v_max);
void CSMC_Reset(CSMC_Controller_t *s);
/* i_ref, i_meas [A]; omega_elec [rad/s electrical] -> v_dq [V] */
void CSMC_Update(CSMC_Controller_t *s, const DQ_t *i_ref, const DQ_t *i_meas,
                 float omega_elec, DQ_t *v_dq);

#endif /* SMC_CURRENT_H */
