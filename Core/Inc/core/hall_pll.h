#ifndef HALL_PLL_H
#define HALL_PLL_H

#include <stdint.h>

/* * PI-based Phase-Locked Loop structure for Hall Sensor interpolation
 */
typedef struct {
    // Controller Gains
    float kp;               // Proportional gain
    float ki;               // Integral gain
    
    // System variables
    float dt;               // Sample time in seconds (e.g., 0.001 for 1kHz)
    
    // State variables
    float integral_term;    // Accumulated integral of the error
    
    // Outputs
    float est_angle;        // Estimated electrical angle in radians [0, 2*PI]
    float est_speed;        // Full PI output, drives the angle integrator
    float est_speed_lpf;    // Integral path only: use this for the speed loop

} HallPLL_t;

/* Function Prototypes */

/**
 * @brief Initializes the PLL parameters and state variables
 */
void HallPLL_Init(HallPLL_t *pll, float kp, float ki, float dt);

/**
 * @brief Updates the PLL. Call this at a fixed frequency (e.g., in your FOC loop).
 * @param pll Pointer to the PLL instance
 * @param measured_angle The current coarse angle from the Hall sensors (in radians)
 */
void HallPLL_Update(HallPLL_t *pll, float measured_angle);

/**
 * @brief Resets the PLL state (useful when motor stops or faults)
 */
void HallPLL_Reset(HallPLL_t *pll, float current_angle);

#endif // HALL_PLL_H