// EXAMPLE 2:
#include "capi324v221.h"

// Make some CONSTANTS
#define c_both 0.936935742
#define c_left 1.002734732
#define c_right 0.997265268



void CBOT_main( void )
{
STEPPER_open(); // Open STEPPER module for use. 
/*
//
//CW Square
//
//START 1st Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right
// Then TURN RIGHT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 150*c_left, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_REV, 150*c_right, 200, 400, STEPPER_BRK_OFF ); // Right
//
//START 2nd Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right
// Then TURN RIGHT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 150*c_left, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_REV, 150*c_right, 200, 400, STEPPER_BRK_OFF ); // Right
//
//START 3rd Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right
// Then TURN RIGHT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 150*c_left, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_REV, 150*c_right, 200, 400, STEPPER_BRK_OFF ); // Right
//
//START 4th Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right
//// Then TURN RIGHT (~90-degrees)...
//STEPPER_move_stwt( STEPPER_BOTH, 
//STEPPER_FWD, 150*c_left, 200, 400, STEPPER_BRK_OFF, // Left
//STEPPER_REV, 150*c_right, 200, 400, STEPPER_BRK_OFF ); // Right
//
//Done
//
*/
//
//CCW Square
//
//START 1st Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right
// Then TURN LEFT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_REV, 150*c_left, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 150*c_left, 200, 400, STEPPER_BRK_OFF ); // Right
//
//START 2nd Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right
// Then TURN LEFT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_REV, 150*c_left, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 150*c_left, 200, 400, STEPPER_BRK_OFF ); // Right
//
//START 3rd Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right
// Then TURN LEFT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_REV, 150*c_left, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 150*c_left, 200, 400, STEPPER_BRK_OFF ); // Right
//
//START 4th Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right

// Infinite loop!
while( 1 ); 
}

