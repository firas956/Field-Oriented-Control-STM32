#include "core/offset_compensator.h"
#include <math.h>

#define PI_F    3.14159265359f
#define TWO_PI  6.28318530718f

static void ResetWindow(OffsetComp_t *o, float theta)
{
    o->sum_a      = 0.0f;
    o->sum_b      = 0.0f;
    o->theta_acc  = 0.0f;
    o->prev_theta = theta;
    o->n          = 0u;
}

void OffsetComp_Init(OffsetComp_t *o, float min_we, float alpha,
                     float clamp, uint32_t max_samples)
{
    o->min_we      = min_we;
    o->alpha       = alpha;
    o->clamp       = clamp;
    o->max_samples = max_samples;

    o->off_a = 0.0f;
    o->off_b = 0.0f;
    ResetWindow(o, 0.0f);
}

void OffsetComp_Reset(OffsetComp_t *o, float theta)
{
    ResetWindow(o, theta);
}

void OffsetComp_Apply(const OffsetComp_t *o, float *ia, float *ib)
{
    *ia -= o->off_a;
    *ib -= o->off_b;
}

void OffsetComp_Update(OffsetComp_t *o, float ia, float ib,
                       float theta, float we, int settled)
{
    if (!settled || fabsf(we) < o->min_we) {
        ResetWindow(o, theta);           /* hold the estimate, drop the window */
        return;
    }

    /* Unwrapped angle swept since the window opened (direction agnostic) */
    float dth = theta - o->prev_theta;
    if (dth >  PI_F) dth -= TWO_PI;
    if (dth < -PI_F) dth += TWO_PI;
    o->prev_theta = theta;
    o->theta_acc += dth;

    o->sum_a += ia;
    o->sum_b += ib;
    o->n++;

    if (o->n > o->max_samples)          { ResetWindow(o, theta); return; }
    if (fabsf(o->theta_acc) < TWO_PI)   { return; }   /* window still open */

    /* One full electrical period accumulated: the mean is the residual */
    float inv_n = 1.0f / (float)o->n;
    o->off_a += o->alpha * (o->sum_a * inv_n);
    o->off_b += o->alpha * (o->sum_b * inv_n);

    if (o->off_a >  o->clamp) o->off_a =  o->clamp;
    if (o->off_a < -o->clamp) o->off_a = -o->clamp;
    if (o->off_b >  o->clamp) o->off_b =  o->clamp;
    if (o->off_b < -o->clamp) o->off_b = -o->clamp;

    ResetWindow(o, theta);
}
