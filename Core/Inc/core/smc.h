#ifndef SMC_H
#define SMC_H
/*
 * Discrete Integral Sliding-Mode Speed Controller (DISMSC) for the PMSM speed
 * loop. Drop-in alternative to the speed PI: takes mechanical speed reference
 * and measurement [rad/s], returns the q-axis current reference iq* [A].
 *
 * Plant (forward-Euler): w_{k+1} = w_k + (Ts/J)(Kt*iq_k - F*w_k - Tr)
 * Surface (integral)   : S_k = c1*e_k + c2*eps_k,  eps_k = eps_{k-1}+e_{k-1}
 * Reaching law (Gao)   : S_{k+1} = eta*S_k - k*Ts*sat(S_k),  eta = 1 - q*Ts
 * Control law          : iq* = (f + d*Tr_hat + c2*(eps+e) - eta*S
 *                                + k*Ts*sat(S/phi)) / (c1*b)
 *                        b = Ts*Kt/J, f = c1*(e+(Ts*F/J)*w), d = c1*Ts/J
 *
 * Tr_hat is a load-torque feed-forward (0 unless a DESMO observer sets it);
 * the integral + switching terms reject a constant load on their own.
 */
typedef struct {
    /* plant constants */
    float J;        /* rotor+load inertia           [kg.m^2]    */
    float F;        /* viscous friction             [N.m.s/rad] */
    float Kt;       /* torque constant 1.5*p*psi_r  [N.m/A]     */
    float Ts;       /* speed-loop sample time       [s]         */
    /* surface + reaching-law gains */
    float c1;       /* surface weight on error    */
    float c2;       /* surface weight on integral */
    float eta;      /* reaching pole = 1 - q*Ts   (0..1) */
    float k;        /* switching gain  (k*Ts < 1) */
    float phi;      /* boundary-layer half-width for sat() [rad/s] */
    /* output limits (iq reference) */
    float out_min;
    float out_max;
    /* feed-forward + state */
    float Tr_hat;   /* load-torque estimate [N.m], 0 without observer */
    float eps;      /* integral of error */
    float e_prev;   /* previous error    */
} SMC_Controller_t;

void  SMC_Init(SMC_Controller_t *s,
               float J, float F, float Kt, float Ts,
               float c1, float c2, float eta, float k, float phi,
               float out_min, float out_max);
void  SMC_Reset(SMC_Controller_t *s);
float SMC_Update(SMC_Controller_t *s, float omega_ref, float omega_meas); /* rad/s -> iq[A] */

#endif /* SMC_H */
