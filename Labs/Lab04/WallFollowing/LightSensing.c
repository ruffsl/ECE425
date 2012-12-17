/*******************************************************************
* FileName:        Light Sensing.c
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

// Obstacle Avoidance Threshold
#define IR_OBST_F_THRESH 1
#define IR_OBST_R_THRESH 1
#define IR_OBST_L_THRESH 1
#define IR_OBST_B_THRESH 1

// Wall Following Threshold
#define IR_WALL_F_THRESH 20
#define IR_WALL_R_THRESH 20
#define IR_WALL_L_THRESH 20
#define IR_WALL_B_THRESH 25

// PID Control Gains
// Note: these current values are adjusted for control cycles times
// including LCD debug print statements, but not for prefilter
#define KP 2
#define KI 0.001
#define KD 5

#define IR_B_KICK 1

// Maximum Wall Following Speed
#define MAX_SPEED 200

// Maximum Magnitude Control Effort
#define MAX_EFFORT 100
#define PREFILTER_SIZE 30


/** Local Function Prototypes **************************************/
void checkIR(void);
char moveWall(void);
char moveWander(void);
char moveAway(void);
char check_threshhold(float, float, float, float);
float pidController(float ,char);
void prefilter(char);


/** Global Variables ***********************************************/
// Integrator Error
float Ierror = 0;
// Previous Error Value
float error_old = 0;

// Infrared Rangefinding Sensor Values
float	ltIR = 0;//left IR sensor
float	rtIR = 0;//right IR sensor
float	ftIR = 0;//front IR sensor
float 	bkIR = 0;//back IR sensor

// Range Sensor Value Arrays For Prefilter
float	ltIR_old[PREFILTER_SIZE];//left IR sensor
float	rtIR_old[PREFILTER_SIZE];//right IR sensor
float	ftIR_old[PREFILTER_SIZE];//front IR sensor
float 	bkIR_old[PREFILTER_SIZE];//back IR sensor
	
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
	STEPPER_open(); // Open STEPPER module for use
	SPKR_open(SPKR_BEEP_MODE);//open the speaker in beep mode
	
	LED_open();
	I2C_open();
	ADC_open();//open the ADC module
 	ADC_set_VREF( ADC_VREF_AVCC );// Set the Voltage Reference first so VREF=5V.

	// Initialize IR Values and Reset Prefilter
	checkIR();
	prefilter(1);

	// Infinite loop
	while (1)
    {
		float voltageR = getRightLight();
		float voltageL = getLeftLight();
		LCD_printf("R Voltage: %3.2f\nL Voltage: %3.2f\n\n\n", voltageR, voltageL);
		TMRSRVC_delay(2000);//wait 2 seconds
		LCD_clear;
    }
}// end the CBOT_main()

/*******************************************************************
* Additional Helper Functions
********************************************************************/

/*******************************************************************
* Function:			void prefilter(char reset)
* Input Variables:	char reset
* Output Return:	void
* Overview:			This appies a prefilter to the IR data by
*					storing the history of measurements within the
*					an arrayand calculating a running average
********************************************************************/
void prefilter(char reset)
{	
	int i;
	// if reset is true, 
	// set all the values within the array to the current measurement
	if(reset)
	{
		for (i = 0; i < PREFILTER_SIZE; i++)
		{
			ltIR_old[i] = ltIR;
			rtIR_old[i] = rtIR;
			ftIR_old[i] = ftIR;
			bkIR_old[i] = bkIR;
		}
	}
	int j;
	float ltIR_new = 0;
	float rtIR_new = 0;
	float ftIR_new = 0;
	float bkIR_new = 0;
	
	// Loop through the entire array and shift each element forward
	// and calculate a running sum simultaneously
	for (i = PREFILTER_SIZE-1; i >= 0 ; i--)
	{
		j = i - 1;
		ltIR_old[i] = ltIR_old[j];
		rtIR_old[i] = rtIR_old[j];
		ftIR_old[i] = ftIR_old[j];
		bkIR_old[i] = bkIR_old[j];
		ltIR_new += ltIR_old[i];
		rtIR_new += rtIR_old[i];
		ftIR_new += ftIR_old[i];
		bkIR_new += bkIR_old[i];
	}
	
	// Set the current reading to the first element 
	// within the historical array
	ltIR_old[0] = ltIR;
	rtIR_old[0] = rtIR;
	ftIR_old[0] = ftIR;
	bkIR_old[0] = bkIR;
	
	// Calculate the average value within the array
	// and said that as the new current measurement
	ltIR = ltIR_new/PREFILTER_SIZE;
	rtIR = rtIR_new/PREFILTER_SIZE;
	ftIR = ftIR_new/PREFILTER_SIZE;
	bkIR = bkIR_new/PREFILTER_SIZE;
}

/*******************************************************************
* Function:			float pidController(float error, char reset)
* Input Variables:	float error, char reset
* Output Return:	float
* Overview:			This computes the control effort using a PID 
*					controller approach
********************************************************************/
float pidController(float error, char reset )
{	
	// Reset the integrator error if specified
	if(reset){
		Ierror = 0;
	}
	// At the current error to the running sum
	Ierror += error;
	
	// Use the PID controller approach to calculate the effort
	float effort = (KP*error) + (KD*(error-error_old)) + (KI*Ierror);
	
	return effort;	
}

/*******************************************************************
* Function:			char moveWall(void)
* Input Variables:	void
* Output Return:	char
* Overview:			This function searches for walls and adjust the 
*					robots differential steering to attempts to
*					follow them
********************************************************************/
char moveWall( void )
{	
	// Check to see if a wall has encountered the wall thresholds
	// or that any other lower-level function requires execution
	char isWander = moveWander();
	if(isWander){
		return isWander;
	}
	
	// A variable that contains the logic of which wall is imaginary
	BOOL isLEFT;
	
	// If there is no wall on our right side
	// place an imaginary wall just within the threshold
	if(rtIR>IR_WALL_R_THRESH){
		rtIR = IR_WALL_R_THRESH-15;
		isLEFT = 0;
	}
	// If there is no wall on our left side
	// place an imaginary wall just within the threshold
	if(ltIR>IR_WALL_L_THRESH){
		ltIR = IR_WALL_L_THRESH-15;
		isLEFT = 1;
	}
	
	float error;
	
	// Check to see if the wall exists in front of the robot
	if(bkIR < IR_WALL_B_THRESH)
	{
		// if the imaginary wall was on the left side
		// then biased the error so that when the robot encounters
		// an upcoming corner, the robot will turn away from both walls
		if (isLEFT)
		{
			error = rtIR - (ltIR + bkIR*bkIR);
		}
		// biased the error appropriately for the inverse situation
		else 
		{
			error = rtIR - (ltIR - bkIR*bkIR);
		}
	}
	
	// If no front facing walls detected
	// the air is simply the right distance minus the left left distance
	// this ensures symmetry that the robot will follow in between the two walls
	// either one real and one imaginary, or both real
	else 
	{
		error = rtIR - ltIR;
	}

	// Use the PID controller function to calculate error
	float effort = pidController(error, 0);
	
	// Limit the control effort to the max allowable effort
	if((abs(effort) > MAX_EFFORT)&(effort!=0)){
		effort = MAX_EFFORT*(effort/abs(effort));
	}
	
	// Calculate the stepper speeds for each wheel using a ratio
	float stepper_speed_L = MAX_SPEED/2 + (MAX_SPEED/2)*(effort/MAX_EFFORT);
	float stepper_speed_R = MAX_SPEED/2 - (MAX_SPEED/2)*(effort/MAX_EFFORT);
	
	// Move with wall
	STEPPER_move_stnb( STEPPER_BOTH, 
	STEPPER_REV, 50, stepper_speed_L, 450, STEPPER_BRK_OFF, // Left
	STEPPER_REV, 50, stepper_speed_R, 450, STEPPER_BRK_OFF ); // Right
	
	// debug LCP print statement
	LCD_clear();
	LCD_printf("bkIR: %3.2f\nmoveWall\nError: %3f\nEffort: %3f\n", bkIR, error, effort);
	
}

/*******************************************************************
* Function:			void moveWander(void)
* Input Variables:	none
* Output Return:	none
* Overview:			This function checks for walls and moves the 
*					robot randomly if walls are not detected
********************************************************************/
char moveWander ( void )
{
	// Check to see if a object has encountered the object thresholds
	// or that any other lower-level function requires execution
	char isShy = moveAway();
	if (isShy)
	{
		return isShy;
	}
	
	char isWander = 0;
	
	// Check for walls before attempting to wonder
	// if the wall is detected return without wandering
	if ((ftIR < IR_WALL_F_THRESH)|(bkIR < IR_WALL_B_THRESH)|(rtIR < IR_WALL_R_THRESH)|(ltIR < IR_WALL_L_THRESH))
	{	
		return isWander = 0;
	}
	
	else
	{
		// reset Ierror if we are now wandering
		Ierror = 0;
		
		// if we are wondering
		// first check the current progress of our wondering
		STEPPER_STEPS curr_steps = STEPPER_get_nSteps();
		
		
		// IF my motion is complete do another random motion
		if ((curr_steps.left == 0)&(curr_steps.right == 0))
		{
			// create random values for wheel position and wheel speed
			int moveRand = rand()%400+400;
			float turnRandR = rand()%200+200;
			float turnRandL = rand()%200+200;
			
			// Weight the chance that we will go forward slightly more
			// so that the robot may possibly traverse farther
			BOOL direction = ~((rand()%10)>7);
					
			// Move.
			STEPPER_move_stnb( STEPPER_BOTH, 
			direction, moveRand, turnRandL, 450, STEPPER_BRK_OFF, // Left
			direction, moveRand, turnRandR, 450, STEPPER_BRK_OFF ); // Right
			
			// debug LCP print statement
			LCD_clear();
			LCD_printf("moveWander\nmoveRand: %3d\nturnRandR: %3d\nturnRandL: %3d\n",moveRand,turnRandR,turnRandL);
			
		}
		// If we have wondered
		// notify that we have
		isWander = 1;
	}
	return isWander;
}

/*******************************************************************
* Function:			void moveAway(void)
* Input Variables:	none
* Output Return:	char
* Overview:			Use a comment block like this before functions
********************************************************************/
char moveAway ( void )
{	
	char shyRobot = 0;
	
	// Use the differences between the front and back
	// left and right distances to calculate a force vector
	float moveY = ftIR - bkIR;
	float moveX = rtIR - ltIR;
	
	// if the object is in front of us are behind us
	// move appropriately in the Y direction
	if ((ftIR < IR_OBST_F_THRESH)|(bkIR < IR_OBST_B_THRESH))
	{
			BOOL moveForward = ~(moveY <= 0);
			
			// Move.
			STEPPER_move_stnb( STEPPER_BOTH, 
			moveForward, 50, abs(moveY)+moveX, 450, STEPPER_BRK_OFF, // Left
			moveForward, 50, abs(moveY)-moveX, 450, STEPPER_BRK_OFF ); // Right
			
			// debug LCP print statement
			LCD_clear();
			LCD_printf("moveAwayF\n\n\n\n");
			
			// if the robot was shy
			// state that fact
			shyRobot = 1;
	}
	
	// if the object is on either side of the robot
	// rotate the robot appropriately
	else if ((rtIR < IR_OBST_R_THRESH)|(ltIR < IR_OBST_L_THRESH))
	{
			BOOL moveForwardR = ~(moveX <= 0);
			BOOL moveForwardL = ~(moveX > 0);
			
			// Move.
			STEPPER_move_stnb( STEPPER_BOTH, 
			moveForwardL, 200, abs(moveX), 450, STEPPER_BRK_OFF, // Left
			moveForwardR, 200, abs(moveX), 450, STEPPER_BRK_OFF ); // Right
			
			// debug LCP print statement
			LCD_clear();
			LCD_printf("moveAwayS\n\n\n\n");
			
			// if the robot was shy
			// state that fact
			shyRobot = 1;
	}
	
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
	check += 0b00000001*(ftIR < F);
	check += 0b00000010*(bkIR < B);
	check += 0b00000100*(ltIR < L);
	check += 0b00001000*(rtIR < R);
	return check;	
}

/*******************************************************************
* Function:			void checkIR(void)
* Input Variables:	none
* Output Return:	none
* Overview:			This function update all of the infrared range
*					finding sensor values sequentially
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
