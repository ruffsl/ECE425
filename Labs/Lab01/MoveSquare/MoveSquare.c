// EXAMPLE 2:
#include "capi324v221.h"

// Make some CONSTANTS
#define c_both 0.936935742
#define c_left 1.002734732
#define c_right 0.997265268
#define sideLength 750
#define turnLength 125



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
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON ); // Right
// Then TURN RIGHT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, turnLength*c_left, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_REV, turnLength*c_right, 200, 400, STEPPER_BRK_ON ); // Right
//
//START 2nd Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON ); // Right
// Then TURN RIGHT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, turnLength*c_left, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_REV, turnLength*c_right, 200, 400, STEPPER_BRK_ON ); // Right
//
//START 3rd Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON ); // Right
// Then TURN RIGHT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, turnLength*c_left, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_REV, turnLength*c_right, 200, 400, STEPPER_BRK_ON ); // Right
//
//START 4th Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON ); // Right
//// Then TURN RIGHT (~90-degrees)...
//STEPPER_move_stwt( STEPPER_BOTH, 
//STEPPER_FWD, turnLength*c_left, 200, 400, STEPPER_BRK_ON, // Left
//STEPPER_REV, turnLength*c_right, 200, 400, STEPPER_BRK_ON ); // Right
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
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON ); // Right
// Then TURN LEFT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_REV, turnLength*c_left, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_FWD, turnLength*c_left, 200, 400, STEPPER_BRK_ON ); // Right
//
//START 2nd Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON ); // Right
// Then TURN LEFT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_REV, turnLength*c_left, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_FWD, turnLength*c_left, 200, 400, STEPPER_BRK_ON ); // Right
//
//START 3rd Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON ); // Right
// Then TURN LEFT (~90-degrees)...
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_REV, turnLength*c_left, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_FWD, turnLength*c_left, 200, 400, STEPPER_BRK_ON ); // Right
//
//START 4th Leg
//
// Move BOTH wheels forward.
STEPPER_move_stwt( STEPPER_BOTH, 
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON, // Left
STEPPER_FWD, sideLength*c_both, 200, 400, STEPPER_BRK_ON ); // Right

// Infinite loop!
while( 1 ); 
}

