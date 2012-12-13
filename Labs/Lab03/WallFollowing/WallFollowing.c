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

#define IR_OBST_F_THRESH 1
#define IR_OBST_R_THRESH 1
#define IR_OBST_L_THRESH 1
#define IR_OBST_B_THRESH 1

#define IR_WALL_F_THRESH 20
#define IR_WALL_R_THRESH 20
#define IR_WALL_L_THRESH 20
#define IR_WALL_B_THRESH 40


#define KP 7
#define KI 0
#define KD 7


#define MAX_SPEED 300
#define MAX_EFFORT 100
#define PREFILTER_SIZE 10


/** Local Function Prototypes **************************************/
void checkIR(void);
char moveWall(void);
char moveWander(void);
char moveAway(void);
char check_threshhold(float, float, float, float);
float pidController(float ,char);
void prefilter(char);


/** Global Variables ***********************************************/
float Ierror = 0;
float error_old = 0;

float	ltIR = 0;//left IR sensor
float	rtIR = 0;//right IR sensor
float	ftIR = 0;//front IR sensor
float 	bkIR = 0;//back IR sensor

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


	checkIR();
	prefilter(1);

	// Infinite loop
	while (1)
    {
		checkIR();
		// prefilter(0);
		moveWall();
		
    }
}// end the CBOT_main()

/*******************************************************************
* Additional Helper Functions
********************************************************************/

/*******************************************************************
* Function:			void prefilter(char reset)
* Input Variables:	char reset
* Output Return:	void
* Overview:			This appies a prefilter to the IR data
********************************************************************/
void prefilter(char reset)
{	
	int i;
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
	
	ltIR_old[0] = ltIR;
	rtIR_old[0] = rtIR;
	ftIR_old[0] = ftIR;
	bkIR_old[0] = bkIR;
	
	ltIR = ltIR_new/PREFILTER_SIZE;
	rtIR = rtIR_new/PREFILTER_SIZE;
	ftIR = ftIR_new/PREFILTER_SIZE;
	bkIR = bkIR_new/PREFILTER_SIZE;
}

/*******************************************************************
* Function:			float pidController(float error, char reset)
* Input Variables:	float error, char reset
* Output Return:	float
* Overview:			This computes the control effort
********************************************************************/
float pidController(float error, char reset )
{	
	if(reset){
		Ierror = 0;
	}
	Ierror += error;
	
	float effort = (KP*error) + (KD*(error-error_old)) + (KI*Ierror);
	
	return effort;	
}

/*******************************************************************
* Function:			char moveWall(void)
* Input Variables:	void
* Output Return:	char
* Overview:			This moves the robot in any arc length
********************************************************************/
char moveWall( void )
{	
	char isWander = moveWander();
	if(isWander){
		return isWander;
	}
	
	BOOL isLEFT;
	
	if(rtIR>IR_WALL_R_THRESH){
		rtIR = IR_WALL_R_THRESH-15;
		isLEFT = 0;
	}
	if(ltIR>IR_WALL_L_THRESH){
		ltIR = IR_WALL_L_THRESH-15;
		isLEFT = 1;
	}
	
	float error;
	if(bkIR < IR_WALL_B_THRESH)
	{
		if (isLEFT)
		{
			error = rtIR - (ltIR + bkIR);
		}
		else 
		{
			error = rtIR - (ltIR - bkIR);
		}
	}
	else 
	{
		error = rtIR - ltIR;
	}

	
	float effort = pidController(error, 0);
	if((abs(effort) > MAX_EFFORT)&(effort!=0)){
		effort = MAX_EFFORT*(effort/abs(effort));
	}
	
	float stepper_speed_L = MAX_SPEED/2 + (MAX_SPEED/2)*(effort/MAX_EFFORT);
	float stepper_speed_R = MAX_SPEED/2 - (MAX_SPEED/2)*(effort/MAX_EFFORT);
	
	// Move with wall
	STEPPER_move_stnb( STEPPER_BOTH, 
	STEPPER_REV, 50, stepper_speed_L, 450, STEPPER_BRK_OFF, // Left
	STEPPER_REV, 50, stepper_speed_R, 450, STEPPER_BRK_OFF ); // Right
	
	LCD_clear();
	LCD_printf("moveWall\nError: %3f\nEffort: %3f\n\n", error, effort);
	
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
	char isWander = 0;
	
	// Check to see if robot sees a wall. If YES go track the wall else randomly WANDER.
	if (isShy)
	{
		return isShy;
	}
	
	// char irBool = check_threshhold(IR_WALL_F_THRESH,IR_WALL_B_THRESH,IR_WALL_L_THRESH,IR_WALL_R_THRESH);
	if ((ftIR < IR_WALL_F_THRESH)|(bkIR < IR_WALL_B_THRESH)|(rtIR < IR_WALL_R_THRESH)|(ltIR < IR_WALL_L_THRESH))
	{	
		return isWander = 0;
	}
	else
	{
		STEPPER_STEPS curr_steps = STEPPER_get_nSteps();
		
		// reset Ierror if we are now wandering
		Ierror = 0;
		
		// IF moveAway() returns zero (NOT shy) and my motion is complete do random motion
		if ((curr_steps.left == 0)&(curr_steps.right == 0))
		{
			// create random values for wheel position and wheel speed
			int moveRand = rand()%400+400;
			float turnRandR = rand()%200+200;
			float turnRandL = rand()%200+200;
			
			BOOL direction = ~((rand()%10)>7);
					
			// Move.
			STEPPER_move_stnb( STEPPER_BOTH, 
			direction, moveRand, turnRandL, 450, STEPPER_BRK_OFF, // Left
			direction, moveRand, turnRandR, 450, STEPPER_BRK_OFF ); // Right
			LCD_clear();
			LCD_printf("moveWander\nmoveRand: %3d\nturnRandR: %3d\nturnRandL: %3d\n",moveRand,turnRandR,turnRandL);
			
		}
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
	
	float moveY = ftIR - bkIR;
	float moveX = rtIR - ltIR;
	
	
	if ((ftIR < IR_OBST_F_THRESH)|(bkIR < IR_OBST_B_THRESH))
	{
			BOOL moveForward = ~(moveY <= 0);
			
			// Move.
			STEPPER_move_stnb( STEPPER_BOTH, 
			moveForward, 50, abs(moveY)+moveX, 450, STEPPER_BRK_OFF, // Left
			moveForward, 50, abs(moveY)-moveX, 450, STEPPER_BRK_OFF ); // Right
			LCD_clear();
			LCD_printf("moveAwayF\n\n\n\n");
			
			shyRobot = 1;
	}
	else if ((rtIR < IR_OBST_R_THRESH)|(ltIR < IR_OBST_L_THRESH))
	{
			BOOL moveForwardR = ~(moveX <= 0);
			BOOL moveForwardL = ~(moveX > 0);
			
			// Move.
			STEPPER_move_stnb( STEPPER_BOTH, 
			moveForwardL, 200, abs(moveX), 450, STEPPER_BRK_OFF, // Left
			moveForwardR, 200, abs(moveX), 450, STEPPER_BRK_OFF ); // Right
			LCD_clear();
			LCD_printf("moveAwayS\n\n\n\n");
			
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
