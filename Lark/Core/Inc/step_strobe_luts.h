/*
 * luts.h
 *
 *  Created on: Aug 30, 2025
 *      Author: HamSlices
 */

#ifndef INC_STEP_STROBE_LUTS_H_
#define INC_STEP_STROBE_LUTS_H_

// 1. Define the structure that the tables use.
//    (Assuming the types for demonstration)
typedef struct {
    unsigned int strobe_mask;
    int microstep_value;
} IO_State;

// 2. Declare the lookup tables as 'extern const'.
//    This makes them globally accessible in a read-only manner.
extern const IO_State io_state_lut_full[];
extern const IO_State io_state_lut_half[];
extern const IO_State io_state_lut_quarter[];
extern const IO_State io_state_lut_eighth[];

#endif /* INC_STEP_STROBE_LUTS_H_ */
