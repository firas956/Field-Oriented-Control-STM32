#ifndef SPEED_REF_H
#define SPEED_REF_H

/*
 * Smooth speed-reference generator: a chain of THREE integrators driven by a
 * bounded constant (the "snap", d3w/dt3). The output w_ref is therefore the
 * triple integral of a constant - C2 continuous with a bounded third
 * derivative. That is what the sliding-mode law needs: its control law uses
 * w*_{k+1}, and a raw step reference makes that term jump, which is what
 * drives the torque spike and the overshoot on every step.
 *
 *   a_des = sat( kv*(w_tgt - w), a_max )
 *   j_des = sat( ka*(a_des - a), j_max )
 *   s     = sat( kj*(j_des - j), s_max )
 *   j += s*Ts ;  a += j*Ts ;  w += a*Ts        (three cascaded integrators)
 *
 * With kj = 3p, ka = p, kv = p/3 the unsaturated closed form is
 *
 *      W(s) / W_tgt(s) = p^3 / (s + p)^3
 *
 * i.e. a TRIPLE REAL POLE at -p: monotonic, exactly zero overshoot, and the
 * transition takes roughly 6/p seconds. The saturations never act on a normal
 * step - they are safety clamps on acceleration / jerk / snap.
 *
 * Unit-agnostic: drive it in RPM and the limits are rpm/s, rpm/s^2, rpm/s^3.
 * The derivatives are kept as states, so w_ref (r->w), dw/dt (r->a) and
 * d2w/dt2 (r->j) can be fed forward to the controller.
 */
typedef struct {
    float Ts;              /* update period [s]                       */
    float kv, ka, kj;      /* cascade gains, derived from the pole p  */
    float a_max;           /* |dw/dt|   clamp                         */
    float j_max;           /* |d2w/dt2| clamp                         */
    float s_max;           /* |d3w/dt3| clamp                         */
    /* integrator chain: s -> j -> a -> w */
    float w;               /* shaped reference   (output)             */
    float a;               /* dw/dt                                   */
    float j;               /* d2w/dt2                                 */
} SpeedRef_t;

void  SpeedRef_Init(SpeedRef_t *r, float Ts, float pole,
                    float a_max, float j_max, float s_max);
void  SpeedRef_Reset(SpeedRef_t *r, float w0);   /* bumpless: start from w0 */
float SpeedRef_Update(SpeedRef_t *r, float w_target);

#endif /* SPEED_REF_H */
