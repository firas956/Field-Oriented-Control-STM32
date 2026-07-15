#include "app/motor_control.h"
#include "hw/hw_adc.h"
#include "hw/hw_hall_sensor.h"
#include "hw/hw_pwm.h"
#include "core/lpf.h"
#include "core/hall_pll.h"  // PLL Header already included

#define MOTOR_POLE_PAIRS 2
#define TWO_PI           6.28318530718f

FOC_Controller_t foc_core;
static PI_Controller_t id_controller;
static PI_Controller_t iq_controller;
static PI_Controller_t speed_controller;

// 1. Instance of the Tracking PLL
static HallPLL_t hall_pll;

// 2. Decimation counter to downsample the speed loop from 20kHz to 1kHz
static uint16_t speed_loop_counter = 0;

void MotorControl_Init(void) {
    // Zero out telemetry structure state
    foc_core.id_target = 0.0f; 
    foc_core.iq_target = 0.0f; 
    foc_core.vdc_bus   = 24.0f; 
    foc_core.speed_target = 0.0f;   
    foc_core.speed_measured = 0.0f; 

    // Current Loop Time Step (20 kHz)
    float Ts = 1.0f / 20000.0f; 
    
    // Initialize Current PI Loops
    PI_Init(&id_controller, 1.5f, 200.0f, Ts, -24.0f, 24.0f);
    PI_Init(&iq_controller, 1.5f, 200.0f, Ts, -24.0f, 24.0f);

    // Speed Loop Time Step (1 kHz)
    float Ts_speed = 1.0f / 1000.0f;
    PI_Init(&speed_controller, 0.5f, 0.01f, Ts_speed, -10.0f, 10.0f);
    
    // 3. Initialize the PLL structure with the 20 kHz time step (Ts)
    // Tune Kp and Ki based on your motor's behavior (Start with Kp=100, Ki=4000)
    HallPLL_Init(&hall_pll, 100.0f, 4000.0f, Ts);
    speed_loop_counter = 0;
}

void MotorControl_RunIteration(void) {
    
    // 4. Update the Tracking PLL
    // Fetch the coarse 60-degree discrete step angle from the hardware layer
    float coarse_hall_angle = HW_Hall_GetBaseAngle(); 
    HallPLL_Update(&hall_pll, coarse_hall_angle);
    
    // 5. Extract high-resolution angle and speed from the PLL
    foc_core.angle_rad = hall_pll.est_angle;
    
    // Convert electrical rad/s from PLL into mechanical RPM
    float omega_elec = hall_pll.est_speed;
    foc_core.speed_measured = (omega_elec * 60.0f) / (TWO_PI * (float)MOTOR_POLE_PAIRS);
    
    // 6. Execute Speed Control Loop at 1 kHz (Every 20th iteration of the 20kHz loop)
    speed_loop_counter++;
    if (speed_loop_counter >= 20) {
        speed_loop_counter = 0; // Reset counter
        
        float speed_error = foc_core.speed_target - foc_core.speed_measured;
        foc_core.iq_target = PI_Update(&speed_controller, speed_error);
    } 
    
    // 7. Core FOC Current Loop Execution (Runs at full 20 kHz)
    HW_ADC_ReadCurrents(&foc_core.i_abc.a, &foc_core.i_abc.b);
    foc_core.i_abc.c = -(foc_core.i_abc.a + foc_core.i_abc.b);
    
    // Phase transformations use the beautifully smoothed PLL angle
    FOC_Clark(&foc_core.i_abc, &foc_core.i_alphabeta);
    FOC_Park(&foc_core.i_alphabeta, &foc_core.i_dq, foc_core.angle_rad);

    // Current PI Controllers
    float id_error = foc_core.id_target - foc_core.i_dq.d;
    float iq_error = foc_core.iq_target - foc_core.i_dq.q;
    foc_core.v_dq.d = PI_Update(&id_controller, id_error);
    foc_core.v_dq.q = PI_Update(&iq_controller, iq_error);
    
    // Inverse transformations and Space Vector Modulation output
    FOC_InversePark(&foc_core.v_dq, &foc_core.v_alphabeta, foc_core.angle_rad);
    FOC_SVPWM(&foc_core.v_alphabeta, foc_core.vdc_bus, &foc_core.duty_cycles);
    HW_PWM_SetDuties(&foc_core.duty_cycles);
}

void MotorControl_SetTorqueTarget(float iq_amps) {
    foc_core.iq_target = iq_amps;
}

void MotorControl_SetSpeedTarget(float speed_rpm) {
    foc_core.speed_target = speed_rpm;
}