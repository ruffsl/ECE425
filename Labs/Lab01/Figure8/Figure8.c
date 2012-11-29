// EXAMPLE 4:
#include "capi324v221.h"

void CBOT_main( void )
{
STEPPER_open(); // Open STEPPER module for use. 
// Move BOTH wheels forward and
// make the robot move in a Figure 8.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 3341.106, 334.1106, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 2305.88, 230.588, 400, STEPPER_BRK_OFF ); // Right

STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 2305.88, 230.588, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 3341.106, 334.1106, 400, STEPPER_BRK_OFF ); // Right

// Infinite loop!
while( 1 ); 
}
