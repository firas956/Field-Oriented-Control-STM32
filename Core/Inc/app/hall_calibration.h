#ifndef HALL_CALIBRATION_H
#define HALL_CALIBRATION_H

#include <stdint.h>
#include "core/modulator.h"

/**
 * Hall table / sequence identification ("voltage stepper" scan).
 *
 * Because the motor is unknown (phase A/B/C labelling and hall-pin-to-phase
 * mapping both unknown), this state measures the only thing FOC actually
 * needs: the electrical angle IN THE FIRMWARE'S OWN (A,B,C) FRAME at which
 * each hall state begins. Physical phase labels never need to be identified;
 * if the calibrated motor then spins opposite to your desired positive
 * direction, swap any two motor leads and re-run the calibration.
 *
 * Procedure (fully automatic, ~14 s):
 *   1. ALIGN: ramp a fixed voltage vector at angle 0 so the rotor settles.
 *   2. FWD:   rotate the commanded angle slowly forward; after one discarded
 *             settling revolution, record each hall transition angle and the
 *             state sequence.
 *   3. HOLD:  brief pause, then
 *   4. REV:   same scan backwards. Averaging forward and reverse entry
 *             angles cancels the rotor lag (load angle) and hall hysteresis.
 *   5. DONE:  per-state sector centres and forward-entry angles are computed,
 *             the live table in hw_hall_sensor.c is updated, and the state
 *             machine returns to IDLE. On any inconsistency: FAILED (table
 *             untouched).
 *
 * Usage: StateMachine_RequestState(STATE_HALL_CALIB) with the motor free to
 * rotate (no load). Watch `hall_calib` in the debugger. The overcurrent trip
 * stays armed during the scan.
 */

typedef enum {
    HALL_CALIB_INACTIVE = 0,
    HALL_CALIB_ALIGN,
    HALL_CALIB_FWD,
    HALL_CALIB_HOLD,
    HALL_CALIB_REV,
    HALL_CALIB_DONE,
    HALL_CALIB_FAILED
} HallCalibPhase_t;

typedef struct {
    HallCalibPhase_t phase;
    uint8_t  sequence[6];      // hall states in forward rotation order
    float    entry_angle[8];   // forward-entry electrical angle per state [rad]
    float    center_angle[8];  // sector centre per state [rad]
    uint8_t  seen_fwd;         // bitmask of states captured in the forward pass
    uint8_t  seen_rev;         // bitmask of states captured in the reverse pass
    uint8_t  valid;            // 1 once a complete, consistent table was applied
} HallCalib_Result_t;

// Live-watch friendly result/progress structure
extern volatile HallCalib_Result_t hall_calib;

// Reset and start a new scan (called automatically on the first iteration
// in STATE_HALL_CALIB; may also be called explicitly)
void HallCalib_Start(void);

// Discard an in-progress scan (interrupted by a fault or operator request)
// so the next entry into STATE_HALL_CALIB restarts cleanly. DONE/FAILED
// results are preserved for inspection.
void HallCalib_Abort(void);

// One 10 kHz iteration of the scan. Drives the PWM directly through the
// supplied modulator; call only while the state machine is in STATE_HALL_CALIB.
void HallCalib_RunIteration(PWM_Modulator_t modulate);

#endif /* HALL_CALIBRATION_H */
