#include "core/harm2_compensator.h"
#include <math.h>

#define PI_F    3.14159265359f
#define TWO_PI  6.28318530718f

static void ResetWindow(Harm2Comp_t *h, float theta)
{
    h->sum_dc     = 0.0f;
    h->sum_ds     = 0.0f;
    h->sum_qc     = 0.0f;
    h->sum_qs     = 0.0f;
    h->sum_mag    = 0.0f;
    h->theta_acc  = 0.0f;
    h->prev_theta = theta;
    h->n          = 0u;
}

static void ClampCoeffs(Harm2Comp_t *h)
{
    const float c = h->clamp;
    if (h->ad >  c) h->ad =  c;  else if (h->ad < -c) h->ad = -c;
    if (h->bd >  c) h->bd =  c;  else if (h->bd < -c) h->bd = -c;
    if (h->aq >  c) h->aq =  c;  else if (h->aq < -c) h->aq = -c;
    if (h->bq >  c) h->bq =  c;  else if (h->bq < -c) h->bq = -c;
}

void Harm2Comp_Init(Harm2Comp_t *h, float min_we, float min_mag, float alpha,
                    float clamp, float mag_beta, uint32_t max_samples)
{
    h->min_we      = min_we;
    h->min_mag     = min_mag;
    h->alpha       = alpha;
    h->clamp       = clamp;
    h->mag_beta    = mag_beta;
    h->max_samples = max_samples;

    h->ad = 0.0f;  h->bd = 0.0f;
    h->aq = 0.0f;  h->bq = 0.0f;

    h->cos2th = 1.0f;
    h->sin2th = 0.0f;
    h->mag    = 0.0f;

    ResetWindow(h, 0.0f);
}

void Harm2Comp_Reset(Harm2Comp_t *h, float theta)
{
    /* The shape survives -- it is a sensor/hall property, not an operating
     * point. The MAGNITUDE does not: re-seed it from zero so a stale |idq|
     * from before the stop cannot scale the correction on the first ticks. */
    h->mag = 0.0f;
    ResetWindow(h, theta);
}

void Harm2Comp_Apply(Harm2Comp_t *h, DQ_t *i_dq, float theta)
{
    /* arm_sin/cos_f32 wrap the argument internally, so 2*theta needs no
     * range reduction here. Cached for Update: two lookups per tick, not four. */
    const float two_th = 2.0f * theta;
    h->cos2th = arm_cos_f32(two_th);
    h->sin2th = arm_sin_f32(two_th);

    /* Fundamental magnitude, tracked well below 2*we so the ripple itself
     * cannot modulate the scaling (that would fold a 4*we term back in). */
    const float m = sqrtf(i_dq->d * i_dq->d + i_dq->q * i_dq->q);
    h->mag += h->mag_beta * (m - h->mag);

    i_dq->d -= h->mag * (h->ad * h->cos2th + h->bd * h->sin2th);
    i_dq->q -= h->mag * (h->aq * h->cos2th + h->bq * h->sin2th);
}

void Harm2Comp_Update(Harm2Comp_t *h, const DQ_t *i_dq, float theta,
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

    h->sum_dc  += i_dq->d * h->cos2th;
    h->sum_ds  += i_dq->d * h->sin2th;
    h->sum_qc  += i_dq->q * h->cos2th;
    h->sum_qs  += i_dq->q * h->sin2th;
    h->sum_mag += h->mag;
    h->n++;

    if (h->n > h->max_samples)          { ResetWindow(h, theta); return; }
    if (fabsf(h->theta_acc) < TWO_PI)   { return; }   /* window still open */

    const float inv_n    = 1.0f / (float)h->n;
    const float mean_mag = h->sum_mag * inv_n;

    /* Too little fundamental to normalize against: the window carries no
     * usable shape, only ADC noise. Drop it, keep the last estimate. */
    if (mean_mag < h->min_mag) { ResetWindow(h, theta); return; }

    /* x2: the projection of a real sinusoid returns half its amplitude.
     * /mean_mag: store the shape per amp, not in amps. */
    const float g = 2.0f * inv_n / mean_mag;

    h->ad += h->alpha * (h->sum_dc * g);
    h->bd += h->alpha * (h->sum_ds * g);
    h->aq += h->alpha * (h->sum_qc * g);
    h->bq += h->alpha * (h->sum_qs * g);

    ClampCoeffs(h);
    ResetWindow(h, theta);
}
