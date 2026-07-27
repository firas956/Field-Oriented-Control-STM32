#ifndef HARM2_COMPENSATOR_H
#define HARM2_COMPENSATOR_H

#include <stdint.h>
#include "core/foc_math.h"

/*
 * Second-harmonic (2*theta_e) compensator, dq frame.
 *
 * Same period-synchronous projection as the phase-offset compensator, one
 * harmonic up and one frame over: instead of averaging ia/ib to pull out
 * their DC, we project id/iq onto cos(2*theta) and sin(2*theta) and average
 * over one exact electrical period. Over that window the dq DC, the 1st
 * harmonic and the 6th all integrate to zero, so what survives is purely the
 * 2*we content -- orthogonal to whatever the offset compensator is doing in
 * abc, so the two run side by side without fighting. The window must be the
 * FULL 2*pi: half a period would still resolve the 2nd harmonic but would let
 * the 1st leak straight into the estimate.
 *
 * Projecting a real sinusoid onto cos/sin returns HALF its amplitude, hence
 * the factor 2 on the coefficients -- the one place the recipe differs from
 * the DC/offset case.
 *
 * The 2nd harmonic comes from a phase-to-phase GAIN mismatch (negative
 * sequence) or hall placement asymmetry, so unlike the DC offset its
 * amplitude is PROPORTIONAL to the fundamental, not absolute. The
 * coefficients are therefore stored NORMALIZED (per amp of |idq|) and
 * rescaled by a slow |idq| tracker on the way out. That makes them a property
 * of the hardware, valid at any load -- and safe to HOLD across a restart,
 * exactly like the offset estimate.
 *
 * Call order, once per ISR tick, both with the SAME theta:
 *     Harm2Comp_Apply(raw i_dq)  ->  Harm2Comp_Update(corrected i_dq)
 * Apply caches cos/sin(2*theta) and advances the |idq| tracker that Update
 * consumes, so it must come first. Feeding Update the CORRECTED currents is
 * what closes the loop: the estimate converges instead of ramping away.
 */
typedef struct {
    /* ---- configuration ---- */
    float    min_we;        /* below |we| the window is dropped    [rad/s]  */
    float    min_mag;       /* below |idq| the window is dropped   [A]      */
    float    alpha;         /* per-period correction fraction               */
    float    clamp;         /* |coefficient| sanity clamp    [A per A]      */
    float    mag_beta;      /* |idq| tracker pole; must be << 2*we_min      */
    uint32_t max_samples;   /* window guard: aborts a window that runs long */

    /* ---- estimate: 2nd-harmonic shape, normalized by |idq| ---- */
    float ad, bd;           /* id ripple = |idq|*(ad*cos2th + bd*sin2th)    */
    float aq, bq;           /* iq ripple = |idq|*(aq*cos2th + bq*sin2th)    */

    /* ---- per-tick cache: written by Apply, read by Update ---- */
    float cos2th;
    float sin2th;
    float mag;              /* slow |idq| the correction scales with  [A]   */

    /* ---- averaging-window state ---- */
    float    sum_dc, sum_ds;
    float    sum_qc, sum_qs;
    float    sum_mag;
    float    theta_acc;     /* electrical angle swept since window opened   */
    float    prev_theta;
    uint32_t n;
} Harm2Comp_t;

/* Cold start: sets the tuning and zeroes BOTH the window and the estimate. */
void Harm2Comp_Init(Harm2Comp_t *h, float min_we, float min_mag, float alpha,
                    float clamp, float mag_beta, uint32_t max_samples);

/* Drop the window and re-seed the |idq| tracker, but KEEP the shape
 * coefficients (warm restart). */
void Harm2Comp_Reset(Harm2Comp_t *h, float theta);

/* Subtract the estimated 2*we ripple from the freshly Park-ed currents. */
void Harm2Comp_Apply(Harm2Comp_t *h, DQ_t *i_dq, float theta);

/* Accumulate one sample. 'settled' is the caller's static-regime flag:
 * false drops the window and holds the estimate.                          */
void Harm2Comp_Update(Harm2Comp_t *h, const DQ_t *i_dq, float theta,
                      float we, int settled);

#endif /* HARM2_COMPENSATOR_H */
