/*******************************************************************
* FileName:        (change filename of template).c
* Processor:       ATmega324P
* Compiler:        
*
* Code Description:
*
*                                                                     
*
* Creation and Revisions:
*
*      AUTHOR               DATE			COMMENTS
*      Ander Solorzano
*      &
*      Ruffin White   
********************************************************************/

/** Header Files ***************************************************/     
#include "capi324v221.h"
#include "stdio.h"
#include "CEEN_Interfaces.h"

/** Define Constants Here ******************************************/
#define C_B 1
#define C_L 1
#define C_R 1
#define D_STEP 0.1335176878

#define IR_OBST_F_THRESH 15
#define IR_OBST_R_THRESH 10
#define IR_OBST_L_THRESH 10
#define IR_OBST_B_THRESH 15

#define IR_WALL_F_THRESH 25
#define IR_WALL_R_THRESH 20
#define IR_WALL_L_THRESH 20
#define IR_WALL_B_THRESH 25


/** Local Function Prototypes **************************************/
void moveWall(void);
void checkIR(void);
char moveWander(void);
char moveAway(void);
char check_threshhold(float, float, float, float);


/** Global Variables ***********************************************/
int sampleVariable = 0;
float	ltIR = 0;//left IR sensor
float	rtIR = 0;//right IR sensor
float	ftIR = 0;//front IR sensor
float 	bkIR = 0;//back IR sensor
	
/*******************************************************************
* Function:        void CBOT_main( void )
********************************************************************/

void CBOT_main( void )
{
	// Initialize variables
	int btnValue=0;//value of button pushed

	ATopstat = ATTINY_open();//open the tiny microcontroller
	LEopstat = LED_open(); //open the LED module
	LCopstat = LCD_open(); //open the LCD module
	SPKR_open(SPKR_BEEP_MODE);//open the speaker in beep mode
	
	LED_open();
	I2C_open();
	ADC_open();//open the ADC module
 	ADC_set_VREF( ADC_VREF_AVCC );// Set the Voltage Reference first so VREF=5V.


	// Infinite loop
	while (1)
    {
		checkIR();
	//	moveWall();
		moveAway();
		
    }
}// end the CBOT_main()

/*******************************************************************
* Additional Helper Functions
********************************************************************/


/*******************************************************************
* Function:			void moveWall(void)
* Input Variables:	void
* Output Return:	void
* Overview:			This moves the robot in any arc length
********************************************************************/
void moveWall( void )
{
	
	moveWander();
}

/*******************************************************************
* Function:			void moveWander(void)
* Input Variables:	none
* Output Return:	none
* Overview:			Use a comment block like this before functions
********************************************************************/
char moveWander ( void )
{
	// Check moveAway() (shy kid) program
	char isShy = moveAway();
	char objectSeen = 0;
	
	// Check to see if robot sees a wall. If YES go track the wall else randomly WANDER.
	if (isShy)
	{
		return objectSeen = 1;
	}
	
	char irBool = check_threshhold(IR_WALL_F_THRESH,IR_WALL_B_THRESH,IR_WALL_L_THRESH,IR_WALL_R_THRESH);
	if (irBool)
	{	
		return objectSeen = 0;
	}
	else
	{
		STEPPER_STEPS curr_steps = STEPPER_get_nSteps();
		
		// IF moveAway() returns zero (NOT shy) and my motion is complete do random motion
		if ((isShy == 0)&(curr_steps.left == 0)&(curr_steps.right == 0))
		{
			// create random values for wheel position and wheel speed
			float moveRandR = abs(rand()*200);
			float moveRandL = abs(rand()*200);
			float turnRandR = abs(100+rand()*100);
			float turnRandL = abs(100+rand()*100);
			
			
			// Move Randomly
			STEPPER_move_stnb( STEPPER_BOTH, 
			STEPPER_FWD, moveRandL, turnRandL, 450, STEPPER_BRK_OFF, // Left
			STEPPER_FWD, moveRandR, turnRandR, 450, STEPPER_BRK_OFF ); // Right
			
		}
	}
	return objectSeen = 1;
}

/*******************************************************************
* Function:			void moveAway(void)
* Input Variables:	none
* Output Return:	char
* Overview:			Use a comment block like this before functions
********************************************************************/
char moveAway ( void )
{	
	// check the IR sensors
	// checkIR();
	
	// return this value to the high-level subroutine to inform the robot of the last behavior
	char shyRobot = 0;
	
	// determinde which IR sensor detects an obstacle
	float moveY = ftIR - bkIR;
	float moveX = rtIR - ltIR;
	
	//check front and back sensors
	//if ((ftIR < IR_OBST_F_THRESH)|(bkIR < IR_OBST_FB_THRESH))
	
	char irBool = check_threshhold(IR_OBST_F_THRESH,IR_OBST_B_THRESH,IR_OBST_L_THRESH,IR_OBST_R_THRESH);
	
	if ((irBool&&0b0011))
	{
			BOOL moveForward = moveY <= 0;
			
			// Move.
			STEPPER_move_stnb( STEPPER_BOTH, 
			moveForward, 50, abs(moveY)+moveX, 450, STEPPER_BRK_OFF, // Left
			moveForward, 50, abs(moveY)-moveX, 450, STEPPER_BRK_OFF ); // Right
			
			shyRobot = 1;
	}
	// check left and right sensors
	// else if ((rtIR < IR_OBST_R_THRESH)|(ltIR < IR_OBST_L_THRESH))
	else if ((irBool&&0b1100))
	{
			BOOL moveForwardR = moveX <= 0;
			BOOL moveForwardL = moveX > 0;
			
			// Move.
			STEPPER_move_stnb( STEPPER_BOTH, 
			moveForwardL, 200, abs(moveX), 450, STEPPER_BRK_OFF, // Left
			moveForwardR, 200, abs(moveX), 450, STEPPER_BRK_OFF ); // Right
			
			shyRobot = 1;
	}
	// else
	// {
		
	// //	STOP.
		// STEPPER_move_stnb( STEPPER_BOTH, 
		// STEPPER_FWD, 0, 0, 0, STEPPER_BRK_OFF, // Left
		// STEPPER_FWD, 0, 0, 0, STEPPER_BRK_OFF ); // Right
	// }
	
	return shyRobot;
}


/*******************************************************************
* Function:			char check_threshhold(float F, float B, float L, float R)
* Input Variables:	float F, float B, float L, float R
* Output Return:	char
* Overview:			This check the IR values to thresholds
********************************************************************/

char check_threshhold(float F, float B, float L, float R)
{
	char check = 0;
	check += 1*(ftIR < F);
	check += 2*(bkIR < B);
	check += 4*(ltIR < L);
	check += 8*(rtIR < R);
	return check;	
}

/*******************************************************************
* Function:			void checkIR(void)
* Input Variables:	none
* Output Return:	none
* Overview:			Use a comment block like this before functions
********************************************************************/
void checkIR( void )
{
	// Update all IR values
	ftIR = getFrontIR();
	bkIR = getBackIR();
	ltIR = getLeftIR();
	rtIR = getRightIR();

}

/*******************************************************************
* Function:			void move_arc_stwt(void)
* Input Variables:	void
* Output Return:	void
* Overview:			This moves the robot in any arc length
********************************************************************/

void move_arc_stwt(int arcRadius, int arcLength, int arcSpeed, int arcAccel, BOOL arcBrk){
	
	// BOOL step_Fwd_L;
	// BOOL step_Fwd_R;
	// int step_Num_L;
	// int step_Num_R;
	// int step_Speed_L;
	// int step_Speed_R;
	// int step_Accel_L;
	// int step_Accel_R;

	// if(arcRadius){

	// }
	// else{
		// stepper_NUM = arcLength/D_STEP;
		// bool stepper_FWD = arcLength >= 0;
		// STEPPER_move_stwt( STEPPER_BOTH, 
		// stepper_FWD, stepper_NUM, arcSpeed, arcAccel, arcBrk, // Left
		// stepper_FWD, stepper_NUM, arcSpeed, arcAccel, arcBrk); // Right
	// }
}
