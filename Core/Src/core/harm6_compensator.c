#include "core/harm6_compensator.h"
#include <math.h>

#define PI_F    3.14159265359f
#define TWO_PI  6.28318530718f

static void ResetWindow(Harm6Comp_t *h, float theta)
{
    h->sum_dc     = 0.0f;
    h->sum_ds     = 0.0f;
    h->sum_qc     = 0.0f;
    h->sum_qs     = 0.0f;
    h->theta_acc  = 0.0f;
    h->prev_theta = theta;
    h->n          = 0u;
}

static void ClampCoeffs(Harm6Comp_t *h)
{
    const float c = h->clamp;
    if (h->ad >  c) h->ad =  c;  else if (h->ad < -c) h->ad = -c;
    if (h->bd >  c) h->bd =  c;  else if (h->bd < -c) h->bd = -c;
    if (h->aq >  c) h->aq =  c;  else if (h->aq < -c) h->aq = -c;
    if (h->bq >  c) h->bq =  c;  else if (h->bq < -c) h->bq = -c;
}

void Harm6Comp_Init(Harm6Comp_t *h, float min_we, float alpha,
                    float clamp, uint32_t max_samples)
{
    h->min_we      = min_we;
    h->alpha       = alpha;
    h->clamp       = clamp;
    h->max_samples = max_samples;

    h->ad = 0.0f;  h->bd = 0.0f;
    h->aq = 0.0f;  h->bq = 0.0f;

    h->res_ad = 0.0f;  h->res_bd = 0.0f;
    h->res_aq = 0.0f;  h->res_bq = 0.0f;
    h->windows = 0u;

    h->cos6th = 1.0f;
    h->sin6th = 0.0f;

    ResetWindow(h, 0.0f);
}

void Harm6Comp_Reset(Harm6Comp_t *h, float theta)
{
    ResetWindow(h, theta);
}

void Harm6Comp_Apply(Harm6Comp_t *h, DQ_t *i_dq, float theta)
{
    /* arm_sin/cos_f32 wrap the argument internally, so 6*theta needs no
     * range reduction here. Cached for Update: two lookups per tick, not four. */
    const float th6 = 6.0f * theta;
    h->cos6th = arm_cos_f32(th6);
    h->sin6th = arm_sin_f32(th6);

    i_dq->d -= h->ad * h->cos6th + h->bd * h->sin6th;
    i_dq->q -= h->aq * h->cos6th + h->bq * h->sin6th;
}

void Harm6Comp_Update(Harm6Comp_t *h, const DQ_t *i_dq, float theta,
                      float we, int settled)
{
    if (!settled || fabsf(we) < h->min_we) {
        ResetWindow(h, theta);           /* hold the estimate, drop the window */
        return;
    }

    /* Unwrapped angle swept since the window opened (direction agnostic) */
    float dth = theta - h->prev_theta;
    if (dth >  PI_F) dth -= TWO_PI;
    if (dth < -PI_F) dth += TWO_PI;
    h->prev_theta = theta;
    h->theta_acc += dth;

    h->sum_dc += i_dq->d * h->cos6th;
    h->sum_ds += i_dq->d * h->sin6th;
    h->sum_qc += i_dq->q * h->cos6th;
    h->sum_qs += i_dq->q * h->sin6th;
    h->n++;

    if (h->n > h->max_samples)          { ResetWindow(h, theta); return; }
    if (fabsf(h->theta_acc) < TWO_PI)   { return; }   /* window still open */

    /* x2: the projection of a real sinusoid returns half its amplitude */
    const float g = 2.0f / (float)h->n;

    h->res_ad = h->sum_dc * g;
    h->res_bd = h->sum_ds * g;
    h->res_aq = h->sum_qc * g;
    h->res_bq = h->sum_qs * g;
    h->windows++;

    h->ad += h->alpha * h->res_ad;
    h->bd += h->alpha * h->res_bd;
    h->aq += h->alpha * h->res_aq;
    h->bq += h->alpha * h->res_bq;

    ClampCoeffs(h);
    ResetWindow(h, theta);
}
