#include "core/foc_math.h"

void FOC_Clark(const Phase_t *in, AlphaBeta_t *out){
    out->alpha = in->a;
    out->beta = (in->a + 2.0f * in->b) * ONE_BY_SQRT3;
}

void FOC_Park(const AlphaBeta_t *in, DQ_t *out, float angle_rad){
    float sin_theta = arm_sin_f32(angle_rad);
    float cos_theta = arm_cos_f32(angle_rad);
    
    out->d =  in->alpha * cos_theta + in->beta * sin_theta;
    out->q = -in->alpha * sin_theta + in->beta * cos_theta;
}

void FOC_InversePark(const DQ_t *in, AlphaBeta_t *out, float angle_rad){
    float sin_theta = arm_sin_f32(angle_rad);
    float cos_theta = arm_cos_f32(angle_rad);
    
    out->alpha = in->d * cos_theta - in->q * sin_theta;
    out->beta  = in->d * sin_theta + in->q * cos_theta;
}

void FOC_InverseClarke(const AlphaBeta_t *in, Phase_t *out){
    out->a =  in->alpha;
    // Note: Multiplying by 0.5f is slightly faster than dividing by 2.0f
    out->b =  (-in->alpha + in->beta * SQRT3) * 0.5f; 
    out->c = -(out->a + out->b);
}
void FOC_SVPWM(const AlphaBeta_t *v_in, float vdc, Phase_t *duty_out)
{
    // Protect against division by zero: park at the neutral zero vector
    // instead of returning with stale duties in duty_out
    if (vdc <= 0.0f) {
        duty_out->a = 0.5f;
        duty_out->b = 0.5f;
        duty_out->c = 0.5f;
        return;
    }

    // Normalize input voltages by Vdc.
    // The 1.5 factor maps an amplitude-invariant phase-voltage request onto
    // the (2/3)Vdc active-vector basis so the average phase voltage delivered
    // equals the request. Without it the modulator has a fixed 2/3 gain:
    // requesting the linear limit Vdc/sqrt(3) at a sector midpoint must give
    // a full 0..1 duty span (t1 + t2 = 1).
    float alpha = 1.5f * v_in->alpha / vdc;
    float beta  = 1.5f * v_in->beta / vdc;
    
    float t1 = 0.0f;
    float t2 = 0.0f;
    int sector = 1;

    // 1. Sector Determination
    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            sector = (ONE_BY_SQRT3 * beta > alpha) ? 2 : 1;
        } else {
            sector = (-ONE_BY_SQRT3 * beta > alpha) ? 3 : 2;
        }
    } else {
        if (alpha >= 0.0f) {
            sector = (-ONE_BY_SQRT3 * beta > alpha) ? 5 : 6;
        } else {
            sector = (ONE_BY_SQRT3 * beta > alpha) ? 4 : 5;
        }
    }

    // 2. Compute Normalized Conduction Times (0.0 to 1.0 scale)
    switch(sector) {
        case 1:
            t1 = alpha - ONE_BY_SQRT3 * beta;
            t2 = TWO_BY_SQRT3 * beta;
            duty_out->a = (1.0f + t1 + t2) * 0.5f;
            duty_out->b = duty_out->a - t1;
            duty_out->c = duty_out->b - t2;
            break;
        case 2:
            t1 = alpha + ONE_BY_SQRT3 * beta;
            t2 = -alpha + ONE_BY_SQRT3 * beta;
            duty_out->b = (1.0f + t1 + t2) * 0.5f;
            duty_out->a = duty_out->b - t2;
            duty_out->c = duty_out->a - t1;
            break;
        case 3:
            t1 = TWO_BY_SQRT3 * beta;
            t2 = -alpha - ONE_BY_SQRT3 * beta;
            duty_out->b = (1.0f + t1 + t2) * 0.5f;
            duty_out->c = duty_out->b - t1;
            duty_out->a = duty_out->c - t2;
            break;
        case 4:
            t1 = -alpha + ONE_BY_SQRT3 * beta;
            t2 = -TWO_BY_SQRT3 * beta;
            duty_out->c = (1.0f + t1 + t2) * 0.5f;
            duty_out->b = duty_out->c - t2;
            duty_out->a = duty_out->b - t1;
            break;
        case 5:
            t1 = -alpha - ONE_BY_SQRT3 * beta;
            t2 = alpha - ONE_BY_SQRT3 * beta;
            duty_out->c = (1.0f + t1 + t2) * 0.5f;
            duty_out->a = duty_out->c - t1;
            duty_out->b = duty_out->a - t2;
            break;
        case 6:
            t1 = -TWO_BY_SQRT3 * beta;
            t2 = alpha + ONE_BY_SQRT3 * beta;
            duty_out->a = (1.0f + t1 + t2) * 0.5f;
            duty_out->c = duty_out->a - t2;
            duty_out->b = duty_out->c - t1;
            break;
        default:
            break;
    }

    // 3. Explicit Clamping to safe boundaries [0.0, 1.0]
    if (duty_out->a > 1.0f) duty_out->a = 1.0f; else if (duty_out->a < 0.0f) duty_out->a = 0.0f;
    if (duty_out->b > 1.0f) duty_out->b = 1.0f; else if (duty_out->b < 0.0f) duty_out->b = 0.0f;
    if (duty_out->c > 1.0f) duty_out->c = 1.0f; else if (duty_out->c < 0.0f) duty_out->c = 0.0f;
}