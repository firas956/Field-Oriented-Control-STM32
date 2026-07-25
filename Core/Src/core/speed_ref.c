#include "core/speed_ref.h"

static float clampf(float x, float lim)
{
    if (x >  lim) return  lim;
    if (x < -lim) return -lim;
    return x;
}

void SpeedRef_Init(SpeedRef_t *r, float Ts, float pole,
                   float a_max, float j_max, float s_max)
{
    r->Ts = Ts;
    /* Triple real pole at -pole:
     * s^3 + kj*s^2 + kj*ka*s + kj*ka*kv == (s + p)^3  with kj=3p, ka=p, kv=p/3 */
    r->kj = 3.0f * pole;
    r->ka = pole;
    r->kv = pole / 3.0f;

    r->a_max = a_max;
    r->j_max = j_max;
    r->s_max = s_max;

    r->w = 0.0f;
    r->a = 0.0f;
    r->j = 0.0f;
}

void SpeedRef_Reset(SpeedRef_t *r, float w0)
{
    r->w = w0;      /* start from where the motor already is -> bumpless */
    r->a = 0.0f;
    r->j = 0.0f;
}

float SpeedRef_Update(SpeedRef_t *r, float w_target)
{
    float a_des = clampf(r->kv * (w_target - r->w), r->a_max);
    float j_des = clampf(r->ka * (a_des    - r->a), r->j_max);
    float s     = clampf(r->kj * (j_des    - r->j), r->s_max);

    /* Three cascaded integrators, semi-implicit Euler (newest state first):
     * the constant s is integrated into j, j into a, a into w.            */
    r->j = clampf(r->j + s    * r->Ts, r->j_max);
    r->a = clampf(r->a + r->j * r->Ts, r->a_max);
    r->w =        r->w + r->a * r->Ts;

    return r->w;
}
