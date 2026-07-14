#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* ---- Motor physical parameters ------------------------------------------
 * Placeholder values until the bench characterization is done.
 * MOTOR_POLE_PAIRS must be confirmed by detent counting / hall-edge counting
 * before the speed readout or the speed loop can be trusted.
 */
#define MOTOR_POLE_PAIRS        2

/* ---- Power stage / protection ------------------------------------------- */
#define VBUS_VOLTS              24.0f
#define OC_TRIP_AMPS            6.0f    /* software overcurrent trip; bench supply limit */
#define IQ_LIMIT_AMPS           4.0f    /* speed-loop torque clamp, keep below supply limit */
#define VDQ_LIMIT_FRACTION      0.95f   /* fraction of the Vdc/sqrt(3) linear SVPWM range */

/* ---- Loop scheduling ------------------------------------------------------ */
#define CURRENT_LOOP_FREQ_HZ    10000.0f
#define SPEED_LOOP_DECIMATION   10      /* 10 kHz / 10 = 1 kHz speed loop */
#define SPEED_LPF_ALPHA         0.1f    /* first-order IIR on raw hall speed, at 1 kHz */

#endif /* MOTOR_CONFIG_H */
