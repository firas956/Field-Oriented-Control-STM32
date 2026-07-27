#ifndef HARM6_COMPENSATOR_H
#define HARM6_COMPENSATOR_H

#include <stdint.h>
#include "core/foc_math.h"

/*
 * Sixth-harmonic (6*theta_e) compensator, dq frame.
 *
 * Same period-synchronous projection as the 2nd-harmonic compensator, four
 * harmonics up: project id/iq onto cos(6*theta) and sin(6*theta) and average
 * over one exact electrical period. That window holds exactly six cycles of
 * the 6th, and every other harmonic -- the dq DC, the 1st the offset trim
 * leaves behind, the 2nd -- integrates to zero across it. So this runs
 * alongside the other two trims without any of them seeing each other.
 *
 * Projecting a real sinusoid onto cos/sin returns HALF its amplitude, hence
 * the factor 2 on the coefficients.
 *
 * WHERE IT COMES FROM, and why this file is not just harm2 with a 6 in it:
 * the 6*we ripple is inverter dead-time. In abc that distortion is 5th and
 * 7th harmonic; the 5th is negative sequence and the 7th positive, so both
 * land on 6*we once Park has rotated the frame. Its amplitude is set by
 * Vdc*td*fsw -- roughly CONSTANT in volts, flipping with the SIGN of the
 * phase current rather than scaling with its size. So unlike the 2nd
 * harmonic (a gain mismatch, proportional to the fundamental) the
 * coefficients here are stored in RAW AMPS and applied as-is. Normalizing
 * them by |idq| would under-correct at high load and over-correct at low.
 * That also means no magnitude tracker and no sqrtf in the hot path.
 *
 * The estimate is a property of the inverter, so it is HELD across a restart
 * exactly like the offset estimate.
 *
 * Call order, once per ISR tick, both with the SAME theta:
 *     Harm6Comp_Apply(raw i_dq)  ->  Harm6Comp_Update(corrected i_dq)
 * Apply caches cos/sin(6*theta) for Update, so it must come first. Feeding
 * Update the CORRECTED currents closes the loop: the estimate converges
 * instead of ramping away.
 */
typedef struct {
    /* ---- configuration ---- */
    float    min_we;        /* below |we| the window is dropped    [rad/s]  */
    float    alpha;         /* per-period correction fraction               */
    float    clamp;         /* |coefficient| sanity clamp          [A]      */
    uint32_t max_samples;   /* window guard: aborts a window that runs long */

    /* ---- estimate: subtracted from the dq currents [A] ---- */
    float ad, bd;           /* id ripple = ad*cos6th + bd*sin6th            */
    float aq, bq;           /* iq ripple = aq*cos6th + bq*sin6th            */

    /* ---- diagnostics: residual of the last CLOSED window (goes to 0) ---- */
    float    res_ad, res_bd;
    float    res_aq, res_bq;
    uint32_t windows;       /* completed windows; stalls if the gate is shut */

    /* ---- per-tick cache: written by Apply, read by Update ---- */
    float cos6th;
    float sin6th;

    /* ---- averaging-window state ---- */
    float    sum_dc, sum_ds;
    float    sum_qc, sum_qs;
    float    theta_acc;     /* electrical angle swept since window opened   */
    float    prev_theta;
    uint32_t n;
} Harm6Comp_t;

/* Cold start: sets the tuning and zeroes BOTH the window and the estimate. */
void Harm6Comp_Init(Harm6Comp_t *h, float min_we, float alpha,
                    float clamp, uint32_t max_samples);

/* Drop the averaging window but KEEP the estimate (warm restart). */
void Harm6Comp_Reset(Harm6Comp_t *h, float theta);

/* Subtract the estimated 6*we ripple from the freshly Park-ed currents. */
void Harm6Comp_Apply(Harm6Comp_t *h, DQ_t *i_dq, float theta);

/* Accumulate one sample. 'settled' is the caller's static-regime flag:
 * false drops the window and holds the estimate.                          */
void Harm6Comp_Update(Harm6Comp_t *h, const DQ_t *i_dq, float theta,
                      float we, int settled);

#endif /* HARM6_COMPENSATOR_H */
