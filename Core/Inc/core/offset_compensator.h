#ifndef OFFSET_COMPENSATOR_H
#define OFFSET_COMPENSATOR_H

#include <stdint.h>

/*
 * Phase-current offset compensator (period-synchronous averaging).
 *
 * Over one exact electrical period the fundamental -- and every harmonic of
 * it -- integrates to zero, so the mean of ia over that window IS the
 * residual offset left over by the standstill ADC calibration. The window
 * closes on the ROTOR ANGLE, not on a timer, so it stays exactly Te at any
 * speed. It is armed only in the static regime; outside it the last estimate
 * is HELD, never zeroed -- the drift is thermal, i.e. orders of magnitude
 * slower than the transient we are stepping around.
 *
 * Feed Update() the CORRECTED currents (raw minus the current estimate, i.e.
 * the output of Apply()): that closes the loop, so the estimate converges on
 * the true offset instead of ramping away.
 */
typedef struct {
    /* ---- configuration ---- */
    float    min_we;        /* below |we| the window is dropped   [rad/s]   */
    float    alpha;         /* per-period correction fraction               */
    float    clamp;         /* |offset| sanity clamp              [A]       */
    uint32_t max_samples;   /* window guard: aborts a window that runs long */

    /* ---- estimate: subtracted from the raw phase currents [A] ---- */
    float off_a;
    float off_b;

    /* ---- averaging-window state ---- */
    float    sum_a;
    float    sum_b;
    float    theta_acc;     /* electrical angle swept since window opened   */
    float    prev_theta;
    uint32_t n;
} OffsetComp_t;

/* Cold start: sets the tuning and zeroes BOTH the window and the estimate. */
void OffsetComp_Init(OffsetComp_t *o, float min_we, float alpha,
                     float clamp, uint32_t max_samples);

/* Drop the averaging window but KEEP the estimate (warm restart). */
void OffsetComp_Reset(OffsetComp_t *o, float theta);

/* Subtract the current estimate from the freshly sampled phase currents. */
void OffsetComp_Apply(const OffsetComp_t *o, float *ia, float *ib);

/* Accumulate one sample. 'settled' is the caller's static-regime flag:
 * false drops the window and holds the estimate.                          */
void OffsetComp_Update(OffsetComp_t *o, float ia, float ib,
                       float theta, float we, int settled);

#endif /* OFFSET_COMPENSATOR_H */
