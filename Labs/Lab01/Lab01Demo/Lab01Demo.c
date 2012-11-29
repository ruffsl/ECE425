/***************************************************************************************************
LAB01: Locomotion Demonstration

CodeSource: 'Lab01Demo.c'

Authors: Ander Solorzano & Ruffin White

Class: ECE425 - Mobile Robotics

Date: 11/29/2013

Code Description: 

This code makes the robot perform 3 different movements based on the pushbutton that is pressed.
The robot is able to move in a square, circle, and figure8. When in iddle, the robot displays "Hello Dolly."



*****************************************************************************************************/

// Header files go here
#include "capi324v221.h"
#include "stdio.h"

// # DEFINE constants go here
#define c_both 0.9369357426
#define c_left 1.142900308
#define c_right 0.8570996920

// list all function prototypes 
void performAction( void ); 

// open status variables
SUBSYS_OPENSTAT ATopstat;//ATTINY open status
SUBSYS_OPENSTAT LCopstat;//LCD open status

// list any global variables

// enter MAIN function 
void CBOT_main( void )
{
	// list any local variables

	// Open modules to be used
	ATopstat = ATTINY_open();//open the tiny microcontroller
	LCopstat = LCD_open(); //open the LCD module

	
	//Infinite Loop
	while(1)
	{
	//Print "Hello Dolly" when NO button is pressed
	LCD_printf( "Hello Dolly\n");
	
	performAction();
	}
}

/**********************************************************************************************/

//Make any subfunctions

	//Perform Action (type of robot movement) based on the pushbutton pressed 
	void performAction(void)
	{
        BOOL btnState1, btnState2, btnState3;//local variables - button states
//		int rtnValue=0;//return the button value

		if((ATopstat.state=SUBSYS_OPEN))
		{
            // Get switch states.
			btnState1 = ATTINY_get_SW_state( ATTINY_SW3 );
			btnState2 = ATTINY_get_SW_state( ATTINY_SW4 );
			btnState3 = ATTINY_get_SW_state( ATTINY_SW5 );
			//LCD_printf("btnStates: %d %d %d \n", btnState1, btnState2, btnState3);

			if( btnState1 == TRUE ) 
			{
				LCD_printf( "Make a SQUARE\n");
				TMRSRVC_delay(1000);//wait 1 s
				
				// MAKE A SQUARE

				STEPPER_open(); // Open STEPPER module for use. 
				//START 1st Leg
				//
				// Move BOTH wheels forward.
				STEPPER_move_stwt( STEPPER_BOTH, 
				STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
				STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right
				// Then TURN LEFT (~90-degrees)...
				STEPPER_move_stwt( STEPPER_BOTH, 
				STEPPER_REV, 110*c_left, 200, 400, STEPPER_BRK_OFF, // Left
				STEPPER_FWD, 110*c_left, 200, 400, STEPPER_BRK_OFF ); // Right
				//
				//START 2nd Leg
				//
				// Move BOTH wheels forward.
				STEPPER_move_stwt( STEPPER_BOTH, 
				STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
				STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right
				// Then TURN LEFT (~90-degrees)...
				STEPPER_move_stwt( STEPPER_BOTH, 
				STEPPER_REV, 110*c_left, 200, 400, STEPPER_BRK_OFF, // Left
				STEPPER_FWD, 110*c_left, 200, 400, STEPPER_BRK_OFF ); // Right
				//
				//START 3rd Leg
				//
				// Move BOTH wheels forward.
				STEPPER_move_stwt( STEPPER_BOTH, 
				STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
				STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right
				// Then TURN LEFT (~90-degrees)...
				STEPPER_move_stwt( STEPPER_BOTH, 
				STEPPER_REV, 110*c_left, 200, 400, STEPPER_BRK_OFF, // Left
				STEPPER_FWD, 110*c_left, 200, 400, STEPPER_BRK_OFF ); // Right
				//
				//START 4th Leg
				//
				// Move BOTH wheels forward.
				STEPPER_move_stwt( STEPPER_BOTH, 
				STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF, // Left
				STEPPER_FWD, 750*c_both, 200, 400, STEPPER_BRK_OFF ); // Right


			}//end if button 1 state open

			if( btnState2 == TRUE ) 
			{
				LCD_printf( "Make a CIRCLE\n");
				TMRSRVC_delay(1000);//wait 1 s
				
				// MAKE A CIRCLE

  				STEPPER_open(); // Open STEPPER module for use. 
				// Move BOTH wheels forward and
				// make the robot move in a circle.
				STEPPER_move_stwt( STEPPER_BOTH, 
				STEPPER_FWD, 3341.106, 334.1106, 400, STEPPER_BRK_OFF, // Left
				STEPPER_FWD, 2305.88, 230.588, 400, STEPPER_BRK_OFF ); // Right

			}//end if btn 2 open

			if ( btnState3 == TRUE ) 
			{
				LCD_printf( "Make a FIGURE8\n");
				TMRSRVC_delay(1000);//wait 1 s

				// MAKE A FIGURE8

				STEPPER_open(); // Open STEPPER module for use. 
				// Move BOTH wheels forward and
				// make the robot move in a Figure 8.
				STEPPER_move_stwt( STEPPER_BOTH, 
				STEPPER_FWD, 3341.106, 334.1106, 400, STEPPER_BRK_OFF, // Left
				STEPPER_FWD, 2305.88, 230.588, 400, STEPPER_BRK_OFF ); // Right

				STEPPER_move_stwt( STEPPER_BOTH, 
				STEPPER_FWD, 2305.88, 230.588, 400, STEPPER_BRK_OFF, // Left
				STEPPER_FWD, 3341.106, 334.1106, 400, STEPPER_BRK_OFF ); // Right

	
			}//end if btn 3 open
            LCD_clear();
//			return rtnValue;
		}//end AT while

	}//end the performAction() subfunction declaration
