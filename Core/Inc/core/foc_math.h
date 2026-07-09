#ifndef FOC_MATH_H
#define FOC_MATH_H

#include "arm_math.h"

#define SQRT3 1.73205081f
#define ONE_BY_SQRT3 0.57735027f
#define TWO_BY_SQRT3     1.15470054f
// 1. The 3-Phase Domain (a, b, c)
typedef struct {
    float a;
    float b;
    float c;
} Phase_t;

// 2. The Stationary Domain (alpha, beta)
typedef struct {
    float alpha;
    float beta;
} AlphaBeta_t;

// 3. The Rotating Domain (d, q)
typedef struct {
    float d;
    float q;
} DQ_t;

// Function Prototypes (Note the addition of 'const' for input safety)
void FOC_Clark(const Phase_t *in, AlphaBeta_t *out);
void FOC_Park(const AlphaBeta_t *in, DQ_t *out, float angle_rad);
void FOC_InversePark(const DQ_t *in, AlphaBeta_t *out, float angle_rad);
void FOC_InverseClarke(const AlphaBeta_t *in, Phase_t *out);
void FOC_SVPWM(const AlphaBeta_t *v_in, float vdc, Phase_t *duty_out);
#endif