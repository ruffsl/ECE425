/*******************************************************************
* FileName:        PathPlanning.c
* Processor:       ATmega324P
* Compiler:        
*
* Code Description: This code is designed to allow the robot to 
* navigate through world with a predefined map from a starting location 
* to a specified goal point
*		
*                                                                     
*
* Creation and Revisions:
*
*      AUTHOR               DATE	1/22/2013		COMMENTS
*      Ander Solorzano
*      &
*      Ruffin White   
********************************************************************/

/** Header Files ***************************************************/     
#include "capi324v221.h"
#include "stdio.h"
#include "CEEN_Interfaces.h"

/** Define Constants Here ******************************************/

#define SUCCESS 1;
#define FAIL 0;

// Arc Function Constants
#define C_B 1
#define C_L 1
#define C_R 1
#define D_STEP 0.1335176878
#define WHEEL_BASE 21.3
#define POINT_TURN 0
#define MOVE_LEFT 1
#define MOVE_FORWARD 2
#define MOVE_RIGHT 3
#define MOVE_STOP 4
#define NO_TURN 2147483647
#define RIGHT_TURN 18.00
#define LEFT_TURN -18.00

// Obstacle Avoidance Threshold
#define IR_OBST_F_THRESH 7
#define IR_OBST_R_THRESH 10
#define IR_OBST_L_THRESH 10
#define IR_OBST_B_THRESH 7

// Wall Following Threshold
#define IR_WALL_F_THRESH 0
#define IR_WALL_R_THRESH 10
#define IR_WALL_L_THRESH 10
#define IR_WALL_B_THRESH 15

// Gateway Thresholds
#define FT_GATEWAY 35
#define BK_GATEWAY 35
#define LT_GATEWAY 35
#define RT_GATEWAY 35

// Orientation constants
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// Light Sensor Threshold values based on ambient light
#define LIGHT_R_THRESH	4.19
#define LIGHT_L_THRESH	4.19
#define LIGHT_R_MAX 4.7
#define LIGHT_L_MAX 4.7

// PID Control Gains
// Note: these current values are adjusted for control cycles times
// including LCD debug print statements, but not for prefilter
#define KP 1
#define KI 0
#define KD 0

#define IR_B_KICK 1

// Maximum Wall Following Speed
#define MAX_SPEED_LIGHT 100
#define MAX_SPEED 200

// Maximum Magnitude Control Effort
#define MAX_EFFORT 100
#define PREFILTER_SIZE 30

// Light behavior selection
#define LIGHT_WANDER 0
#define LIGHT_LOVER  1
#define LIGHT_AGGRO  2
#define	LIGHT_SHY	 3

// Maximum number of User Moves
#define MAX_MOVE_SIZE 12

// World Size
#define WORLD_ROW_SIZE 4
#define WORLD_COLUMN_SIZE 4
#define WORLD_RESOLUTION_SIZE 45.72

// Button States
#define PRESSED 0
#define UNPRESSED 1

// Grid Resolution
#define GRIDRES 45


/** Local Function Prototypes **************************************/
void checkIR(void);
void checkLightSensor(void);
void checkContactIR(void);
char moveWall(void);
char moveWander(void);
char moveAway(void);
char check_threshhold(float, float, float, float);
float pidController(float ,char);
void prefilter(char);
void movesInput(void);
void worldInput(void);
char moveBehavior(int);
char moveWorld(void);
char moveMetric(void);
char move_arc_stwt(float, float, float, float, BOOL);
char move_arc_stnb(float, float, float, float, BOOL);
char checkOdometry(void);
void checkWorld(void);
void getGateways(void);
unsigned char rotateCell(unsigned char,unsigned char);
void orientationInput(void);

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

// Light Range Sensor global variables
float	rightLightVolt	= 0;//right light sensors
float	leftLightVolt	= 0;//left light sensors

// Range Sensor Value Arrays For Prefilter
float	ltIR_old[PREFILTER_SIZE];//left IR sensor
float	rtIR_old[PREFILTER_SIZE];//right IR sensor
float	ftIR_old[PREFILTER_SIZE];//front IR sensor
float 	bkIR_old[PREFILTER_SIZE];//back IR sensor

// Contact IR sensors
BOOL rightContact;
BOOL leftContact;

// moveBehavior Global Flag Variables
char moveWallFlagStatus = 0;
char currentMove = 0;
char oldMove = 0;

// Create an array for button value commands
unsigned char moveCommands[MAX_MOVE_SIZE];
unsigned char moveGateways[MAX_MOVE_SIZE];
unsigned char currentMoveWorld = 0;
unsigned char currentCellWorld = 0;
unsigned char currentOrientation = 0b00;
unsigned char currentGateway = 0;
unsigned char nextGateway = 0;

#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"
#define BYTETOBINARY(byte)  \
  (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0) 


// Create the Robot World
static unsigned char ROBOT_WORLD[WORLD_ROW_SIZE][WORLD_COLUMN_SIZE] = 	{
																{0b1101,0b1111,0b1111,0b1101},
																{0b0001,0b1010,0b1010,0b0100},
																{0b0101,0b1111,0b1111,0b0101},
																{0b0111,0b1111,0b1011,0b0110}
																};
							
// Create an shift matrix for rotating cells							
static unsigned char shifted[16] = {0b0000, 0b0010, 0b0100, 0b0110, 
									0b1000, 0b1010, 0b1100, 0b1110, 
									0b0001, 0b0011, 0b0101, 0b0111, 
									0b1001, 0b1011, 0b1101, 0b1111};
	
/*******************************************************************
* Function:        void CBOT_main( void )
********************************************************************/

void CBOT_main( void )
{
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
	
	//unsigned char i = 0;
	// unsigned char orent = 0b0001;
	// while(1){
		// LCD_clear();
		// LCD_printf("Nume: %i\nOrent\n"BYTETOBINARYPATTERN,i,BYTETOBINARY(orent));
		// TMRSRVC_delay(2000);//wait 1 seconds
		// orent  = rotateCell (orent, 0b01);
		// i++;
	// }
	
	
	// Enter the robot's current (starting) position
	LCD_printf("START location\n\n\n\n");	
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();
	worldInput();
	TMRSRVC_delay(1000);//wait 3 seconds
	LCD_clear();
	
	// Enter the robot's current (starting) orientation
	LCD_printf("START orientation\n\n\n\n");	
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();
	orientationInput();
	TMRSRVC_delay(1000);//wait 3 seconds
	LCD_clear();
	
	// Enter the robot topological commands
	LCD_printf("ENTER move commands\n\n\n\n");
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();
	movesInput();
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();
	
	// Print the robot gateways
	LCD_printf("Robot Gateways:\n\n\n\n");
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();
	getGateways();
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();
			
	

	// Infinite loop
	while (1)
    {
		checkIR();	
		checkWorld();
		moveWorld();	
		// moveMetric();	
		
		// // Test arc function
		// LCD_printf("Move Arc\n\n\n\n");
		// TMRSRVC_delay(1000);//wait 1 seconds
		// move_arc_stwt(POINT_TURN, RIGHT_TURN, 10, 10, 0);
			
		// //Test contact Sensors
		// LCD_printf("Right Contact: %i\nLeft Contact: %i\n\n\n",rightContact,leftContact);
		// TMRSRVC_delay(1000);//wait 1 seconds
		
		// // Test IR Sensors
		// LCD_clear();
		// LCD_printf("FrontIR = %3.2f\nBackIR = %3.2f\nLeftIR = %3.2f\nRightIR = %3.2f\n", ftIR,bkIR,ltIR,rtIR);
		// TMRSRVC_delay(1000);//wait 1 seconds
    }
}// end the CBOT_main()



/*******************************************************************
* Additional Helper Functions
********************************************************************/

/*******************************************************************
* Function:			char getGateways(void)
* Input Variables:	void
* Output Return:	void
* Overview:		    Interpolates the list of gateways in the path by
					using the map and initial conditions 
********************************************************************/
void getGateways(void)
{
	// Get the start location of the robot
	unsigned char curRow = (currentCellWorld>>2) & 0b1100;
	unsigned char curCol = currentCellWorld & 0b0011;
	
	// Git the start orientation of the robot
	unsigned char curOrient = currentOrientation;
	
	// This will be the gatway the robot will look for
	unsigned char curCell;
		
	// This will be the move the robot will preform
	unsigned char curMove;
	
	// This is the index of the move we are looking at
	unsigned char j;
	
	for (j = 0; j<=MAX_MOVE_SIZE; j++)
	{	// Get the current move
		curMove = moveCommands[j];
		
		// Get the current cell
		curCell = ROBOT_WORLD[curRow][curCol];
		
		// Rotate the cell with reference to the robot
		curCell = rotateCell(curCell,curOrient);
		
		// Store the cell as a searchable gateway
		moveGateways[j] = curCell;
				
		// If we are moving forward
		// move to the next cell with respect to our orientation
		if (curMove == MOVE_FORWARD){
			switch(curOrient){
				case NORTH:
					curRow -= 1;
					break;
				case EAST:
					curCol += 1;
					break;
				case SOUTH:
					curRow += 1;					
					break;
				case WEST:
					curCol -= 1;					
					break;
				default:
					break;
			}			
		}
		//	If we are turning right
		// then rotate our map orientation appropriately
		else if (curMove == MOVE_RIGHT){
			curOrient++;
			curOrient = curOrient&0b11;
			// LCD_clear();
			// LCD_printf("Num:\n%i\curOrient:\n"BYTETOBINARYPATTERN,j,BYTETOBINARY(curOrient));
			// TMRSRVC_delay(500);//wait 1/2 seconds
		}
		//	If we are turning left
		// then rotate our map orientation appropriately
		else if (curMove == MOVE_LEFT){
			// if(curOrient == 0){
				// curOrient = 0b0011;
			// }
			curOrient--;
			curOrient = curOrient&0b11;
		}			
	}
	
	for (j = 0; j<=MAX_MOVE_SIZE; j++)
	{
		curCell = moveGateways[j];
		LCD_clear();
		LCD_printf("Num:\n%i\nCurCell:\n"BYTETOBINARYPATTERN,j,BYTETOBINARY(curCell));
		TMRSRVC_delay(500);//wait 1/2 seconds
	}
}

/*******************************************************************
* Function:			void rotateCell(unsigned char,char)
* Input Variables:	unsigned char, unsigned char
* Output Return:	unsigned char
* Overview:		    Rotates the current cell to match the robot's
					orientation, so the robot can recognize gateways
********************************************************************/
unsigned char rotateCell (unsigned char worldCell, unsigned char orientation)
{
	//	a temp for orientation
	unsigned char orient = orientation;
	
	//	a temp for world cell
	unsigned char cell = worldCell;
	
	// Use the given orientation as a counter 
	// to know the number of rotations to correct the map
	// for the robots current orientation
	while (orient!=0){
		// use the shift array to rotate
		cell = shifted[cell];
		orient--;
	}
	return cell;
}

/*******************************************************************
* Function:			void checkWorld(void)
* Input Variables:	void
* Output Return:	void
* Overview:		    Checks the cell of the robot using IR sensors
********************************************************************/
void checkWorld( void )
{	
	currentGateway = 0;
	
	// Acquire current gateway description
	currentGateway += (ftIR<FT_GATEWAY)<<3;
	currentGateway += (ltIR<LT_GATEWAY)<<2;
	currentGateway += (bkIR<BK_GATEWAY)<<1;
	currentGateway += (rtIR<RT_GATEWAY)<<0;
	nextGateway = moveGateways[currentMoveWorld+1];
	
	// Check to see if the robot has entered the next cell of the robot world
	if(currentGateway == nextGateway){
		currentMoveWorld += 1;
	}
}

/*******************************************************************
* Function:			void worldInput(void)
* Input Variables:	void
* Output Return:	void
* Overview:			Allows the user to initialize the location of
*					the robot 
*						NORTH = 0b00
*						EAST = 0b01
*						SOUTH = 0b10
*						WEST = 0b11
********************************************************************/
void worldInput( void )
{
	// Initialize a button holder
	unsigned char btnHolder = UNPRESSED;
	// unsigned char btnHolderOld = UNPRESSED;
	unsigned char i = 0;
	
	while (i < WORLD_ROW_SIZE){
		btnHolder = EnterTopoCommand();

		if (btnHolder == MOVE_LEFT){
			currentCellWorld = currentCellWorld << 1;
			currentCellWorld += 0;
			i++;
		}
		else if (btnHolder == MOVE_FORWARD){
			currentCellWorld = currentCellWorld << 1;
			currentCellWorld += 1;
			i++;
		}

		// if (btnHolder != 0){
			LCD_clear();
			LCD_printf("Current World Cell:\n%i\nCommand Num: %i\n",currentCellWorld,i);
		// }
		TMRSRVC_delay(500);	//wait 0.5 seconds
	}
}

/*******************************************************************
* Function:			void orientationInput(void)
* Input Variables:	void
* Output Return:	void
* Overview:		    Enter the starting orientation of the robot
*						NORTH = 0b00
*						EAST = 0b01
*						SOUTH = 0b10
*						WEST = 0b11
********************************************************************/
void orientationInput(void)
{
	// Initialize a button holder
	unsigned char btnHolder = UNPRESSED;
	// unsigned char btnHolderOld = UNPRESSED;
	unsigned char i = 0;
	
	while (i < 2){
		btnHolder = EnterTopoCommand();

		if (btnHolder == MOVE_LEFT){
			currentOrientation = currentOrientation << 1;
			currentOrientation += 0;
			i++;
		}
		else if (btnHolder == MOVE_FORWARD){
			currentOrientation = currentOrientation << 1;
			currentOrientation += 1;
			i++;
		}

		if (btnHolder != 0){
			LCD_clear();
			LCD_printf("Current World Orientation:\n%i\nCommand Num: %i\n",currentOrientation,i);	
		}
		TMRSRVC_delay(500);	//wait 0.5 seconds
	}
	LCD_clear();
	switch(currentOrientation){
		case NORTH:
			LCD_printf("Current World Orientation:\nNORTH\n\n");
			break;
		case EAST:
			LCD_printf("Current World Orientation:\nEAST\n\n");
			break;
		case SOUTH:
			LCD_printf("Current World Orientation:\nSOUTH\n\n");
			break;
		case WEST:
			LCD_printf("Current World Orientation:\nWEST\n\n");
			break;
		default:
			break;
	}
	TMRSRVC_delay(500);	//wait 0.5 seconds
}

/*******************************************************************
* Function:			void movesInput(void)
* Input Variables:	void
* Output Return:	void
* Overview:			allows the user to specify a desired path
********************************************************************/
void movesInput( void )
{
	// Initialize a button holder
	unsigned char btnHolder = UNPRESSED;
	unsigned char btnHolderOld = UNPRESSED;
	unsigned char i = 0;
	
	while (i < (MAX_MOVE_SIZE-1)){
		btnHolder = EnterTopoCommand();
	
		if (btnHolder == MOVE_LEFT){
			moveCommands[i] = MOVE_LEFT;
			i++;
		}
		else if (btnHolder == MOVE_FORWARD){
			moveCommands[i] = MOVE_FORWARD;
			i++;
		}
		else if (btnHolder == MOVE_RIGHT){
			moveCommands[i] = MOVE_RIGHT;
			i++;
		}

		if (btnHolder != 0){
			LCD_clear();
			LCD_printf("Old Command: %i\nNew Command: %i\nCommand Num %i\n\n",btnHolderOld,btnHolder,i);
			btnHolderOld = btnHolder;
		}
		TMRSRVC_delay(500);	//wait 0.5 seconds
	}
	i++;
	moveCommands[i] = MOVE_STOP;
}

/*******************************************************************
* Function:			char moveWorld(void)
* Input Variables:	void
* Output Return:	char
* Overview:		    Iteratively moves robot through the world
********************************************************************/
char moveWorld( void )
{	
	LCD_clear();
	// LCD_printf("Current Move:\n%i\n",currentMove);
	
	currentMove = moveCommands[currentMoveWorld];
	// if(currentMove != oldMove){
		// move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE, 10, 10, 0);
	// }
	// LCD_clear();
	
	if(((currentMove == MOVE_LEFT)|(currentMove == MOVE_RIGHT))&(oldMove == MOVE_FORWARD))
	{
		move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE*(3.0/5.0), 10, 10, 0);		
	}
	
	// if(((oldMove == MOVE_LEFT)|(oldMove == MOVE_RIGHT))&(currentMove == MOVE_FORWARD))
	// {
		// move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE*(3.0/5.0), 10, 10, 0);		
	// }
	
	switch(currentMove){
		case MOVE_LEFT:
			LCD_printf("Left\nCurMove:%i\nGateway:%i\nNextGateway:%i\n",currentMoveWorld,currentGateway,nextGateway);
			// TMRSRVC_delay(1000);//wait 1 seconds
			move_arc_stwt(POINT_TURN, LEFT_TURN, 10, 10, 0);
			break;
		case MOVE_FORWARD:
			LCD_printf("Forward\nCurMove:%i\nGateway:%i\nNextGateway:%i\n",currentMoveWorld,currentGateway,nextGateway);
			// TMRSRVC_delay(1000);//wait 1 seconds
			moveWall();
			// move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE, 10, 10, 0);
			break;
		case MOVE_RIGHT:
			LCD_printf("Right\nCurMove:%i\nGateway:%i\nNextGateway:%i\n",currentMoveWorld,currentGateway,nextGateway);
			// TMRSRVC_delay(1000);//wait 1 seconds
			move_arc_stwt(POINT_TURN, RIGHT_TURN, 10, 10, 0);
			break;
		default:
			LCD_printf("What?!");
			STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_OFF);
			while(1);
			break;
	}
	// TMRSRVC_delay(1000);//wait 1 seconds
	oldMove = currentMove;
	return 1;
}

/*******************************************************************
* Function:			char moveMetric(void)
* Input Variables:	void
* Output Return:	char
* Overview:		    Moves robot through the world
********************************************************************/
char moveMetric( void )
{	
	LCD_clear();
	// LCD_printf("Current Move:\n%i\n",currentMove);
	
	currentMove = moveCommands[currentMoveWorld];
	// if(currentMove != oldMove){
		// move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE, 10, 10, 0);
	// }
	// LCD_clear();
	
	switch(currentMove){
		case MOVE_LEFT:
			LCD_printf("Left\nCurMove:%i\nGateway:%i\nNextGateway:%i\n",currentMoveWorld,currentGateway,nextGateway);
			// TMRSRVC_delay(1000);//wait 1 seconds
			move_arc_stwt(POINT_TURN, LEFT_TURN, 10, 10, 0);
			break;
		case MOVE_FORWARD:
			LCD_printf("Forward\nCurMove:%i\nGateway:%i\nNextGateway:%i\n",currentMoveWorld,currentGateway,nextGateway);
			// TMRSRVC_delay(1000);//wait 1 seconds
			move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE*(12/10), 10, 10, 0);
			break;
		case MOVE_RIGHT:
			LCD_printf("Right\nCurMove:%i\nGateway:%i\nNextGateway:%i\n",currentMoveWorld,currentGateway,nextGateway);
			// TMRSRVC_delay(1000);//wait 1 seconds
			move_arc_stwt(POINT_TURN, RIGHT_TURN, 10, 10, 0);
			break;
		default:
			LCD_printf("What?!");
			STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_OFF);
			while(1);
			break;
	}
	// TMRSRVC_delay(1000);//wait 1 seconds
	oldMove = currentMove;
	currentMoveWorld++;
	return 1;
}

/*******************************************************************
* Function:			char moveBehavior (int)
* Input Variables:	int
* Output Return:	char
* Overview:		    This is the flow for the behavior of the robot
********************************************************************/
char moveBehavior( int behavior)
{
	// Check the moveAway behavior for obstacles
	// if(moveAway()){
		// Ierror = 0;
		// return 1; 
	// }
	
	if(moveWorld()){
		Ierror = 0;
		return 1; 
	}
	
	// if(moveWallFlagStatus){
		// // Run the moveWall behavior
		// if(moveWall()){
			// Ierror = 0;
			// return 3;
		// }
	// }
	return 0;	
}

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
	// Check for walls
	BOOL isWall = (ftIR < IR_WALL_F_THRESH)|(bkIR < IR_WALL_B_THRESH)|(rtIR < IR_WALL_R_THRESH)|(ltIR < IR_WALL_L_THRESH);
	if(!isWall){	
		move_arc_stnb(NO_TURN, 10, 10, 10, 0);
		return isWall;
	}
		
	// A variable that contains the logic of which wall is imaginary
	BOOL isLEFT;
	
	// If there is no wall on our right side
	// place an imaginary wall just within the threshold
	if(rtIR>IR_WALL_R_THRESH){
		rtIR = IR_WALL_R_THRESH-18;
		isLEFT = 0;
	}
	// If there is no wall on our left side
	// place an imaginary wall just within the threshold
	if(ltIR>IR_WALL_L_THRESH){
		ltIR = IR_WALL_L_THRESH-18;
		isLEFT = 1;
	}
	
	float error;
	
	// Check to see if the wall exists in front of the robot
	if(ftIR < IR_WALL_F_THRESH)
	{
		// if the imaginary wall was on the left side
		// then biased the error so that when the robot encounters
		// an upcoming corner, the robot will turn away from both walls
		if (isLEFT)
		{
			error = rtIR - (ltIR + (1000/ftIR));
		}
		// biased the error appropriately for the inverse situation
		else 
		{
			error = rtIR - (ltIR - (1000/ftIR));
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
	float effort = pidController(-error, 0);
	
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
	// LCD_clear();
	// LCD_printf("bkIR: %3.2f\nmoveWall\nError: %3f\nEffort: %3f\n", bkIR, error, effort);
	return isWall;
	
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
	// If we have wondered
	// notify that we have
	char isWander = 1;
	
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
		// LCD_clear();
		// LCD_printf("moveWander\nmoveRand: %3d\nturnRandR: %3d\nturnRandL: %3d\n",moveRand,turnRandR,turnRandL);
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
			BOOL moveForward = (moveY >= 0);
			
			// Move.
			STEPPER_move_stnb( STEPPER_BOTH, 
			moveForward, 50, abs(moveY)+moveX, 450, STEPPER_BRK_OFF, // Left
			moveForward, 50, abs(moveY)-moveX, 450, STEPPER_BRK_OFF ); // Right
			
			// debug LCP print statement
			// LCD_clear();
			// LCD_printf("moveAwayF\n\n\n\n");
			
			// if the robot was shy
			// state that fact
			shyRobot = 1;
	}
	
	// if the object is on either side of the robot
	// rotate the robot appropriately
	else if ((rtIR < IR_OBST_R_THRESH))
	{
			// BOOL moveForwardR = ~(moveX <= 0);
			// BOOL moveForwardL = ~(moveX > 0);
			
			// Move.
			STEPPER_move_stnb( STEPPER_BOTH, 
			0, 200, abs(moveX), 450, STEPPER_BRK_OFF, // Left
			1, 200, abs(moveX), 450, STEPPER_BRK_OFF ); // Right
			
			// debug LCP print statement
			// LCD_clear();
			// LCD_printf("moveAwayS\n\n\n\n");
			
			// if the robot was shy
			// state that fact
			shyRobot = 1;
	}
	else if ((ltIR < IR_OBST_L_THRESH))
	{
			// BOOL moveForwardR = ~(moveX <= 0);
			// BOOL moveForwardL = ~(moveX > 0);
			
			// Move.
			STEPPER_move_stnb( STEPPER_BOTH, 
			1, 200, abs(moveX), 450, STEPPER_BRK_OFF, // Left
			0, 200, abs(moveX), 450, STEPPER_BRK_OFF ); // Right
			
			// debug LCP print statement
			// LCD_clear();
			// LCD_printf("moveAwayS\n\n\n\n");
			
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
* Function:			void checkLightSensor(void)
* Input Variables:	none
* Output Return:	none
* Overview:			This function update all of the light range
*					sensors values sequentially
********************************************************************/
void checkLightSensor( void )
{
	// Update both light sensor variables
	rightLightVolt = getRightLight();
	leftLightVolt = getLeftLight();
	// LCD_printf("R Voltage: %3.2f\nL Voltage: %3.2f\n\n\n", voltageR, voltageL);
	// TMRSRVC_delay(2000);//wait 2 seconds
	// LCD_clear();
}

/*******************************************************************
* Function:			unsigned char getContactIR(void)
* Input Variables:	none
* Output Return:	none
* Overview:			Acquires status of contact sensors
********************************************************************/
void checkContactIR(void)
{
	unsigned char sensors = ATTINY_get_sensors();
	rightContact = 0b00000001 & sensors;
	leftContact =  (0b00000010 & sensors)>>1;
	
	
}

/*******************************************************************
* Function:			char move_arc_stwt(float, float, float, float, BOOL)
* Input Variables:	char
* Output Return:	float, float, float, float, BOOL
* Overview:			This moves the robot in any arc length
********************************************************************/
char move_arc_stwt(float arc_Radius, float arc_Length, float arc_Speed, float arc_Accel, BOOL arc_Brk)
{
	
	BOOL step_Fwd_L = (arc_Length>0);
	BOOL step_Fwd_R = (arc_Length>0);
	float step_Num = abs(arc_Length/D_STEP);
	float step_Speed = abs(arc_Speed/D_STEP);
	float step_Accel = abs(arc_Accel/D_STEP);


	if(arc_Radius == NO_TURN){
		STEPPER_move_stwt( STEPPER_BOTH, 
		step_Fwd_L, step_Num, step_Speed, step_Accel, arc_Brk, // Left
		step_Fwd_R, step_Num, step_Speed, step_Accel, arc_Brk); // Right
		return SUCCESS;
	}
	
	if(arc_Radius == POINT_TURN){				
		STEPPER_move_stwt( STEPPER_BOTH, 
	   !step_Fwd_L, step_Num, step_Speed, step_Accel, arc_Brk, // Left
		step_Fwd_R, step_Num, step_Speed, step_Accel, arc_Brk); // Right
		return SUCCESS;
	}
		
	float arc_Length_L;
	float arc_Length_R;	
	float arc_Speed_L;
	float arc_Speed_R;	
	float arc_Accel_L;
	float arc_Accel_R;
		
	float step_Num_L;
	float step_Num_R;	
	float step_Speed_L;
	float step_Speed_R;	
	float step_Accel_L;
	float step_Accel_R;
	
	if(arc_Radius > 0){
		arc_Length_L = arc_Length * (1 - WHEEL_BASE/arc_Radius);
		arc_Length_R = arc_Length * (1 + WHEEL_BASE/arc_Radius);
		step_Num_L = arc_Length_L/D_STEP;
		step_Num_R = arc_Length_R/D_STEP;
		
		
		arc_Speed_L = arc_Speed * (1 - WHEEL_BASE/arc_Radius);
		arc_Speed_R = arc_Speed * (1 + WHEEL_BASE/arc_Radius);
		step_Speed_L = arc_Speed_L/D_STEP;
		step_Speed_R = arc_Speed_R/D_STEP;
		
		
		arc_Accel_L = arc_Accel * (1 - WHEEL_BASE/arc_Radius);
		arc_Accel_R = arc_Accel * (1 + WHEEL_BASE/arc_Radius);
		step_Accel_L = arc_Accel_L/D_STEP;
		step_Accel_R = arc_Accel_R/D_STEP;
		
		STEPPER_move_stwt( STEPPER_BOTH, 
		step_Fwd_L, step_Num_L, step_Speed_L, step_Accel_L, arc_Brk, // Left
		step_Fwd_R, step_Num_R, step_Speed_R, step_Accel_R, arc_Brk); // Right
		return SUCCESS;
	}	
	
	if(arc_Radius < 0){
		arc_Length_L = arc_Length * (1 + WHEEL_BASE/arc_Radius);
		arc_Length_R = arc_Length * (1 - WHEEL_BASE/arc_Radius);
		step_Num_L = arc_Length_L/D_STEP;
		step_Num_R = arc_Length_R/D_STEP;
		
		
		arc_Speed_L = arc_Speed * (1 + WHEEL_BASE/arc_Radius);
		arc_Speed_R = arc_Speed * (1 - WHEEL_BASE/arc_Radius);
		step_Speed_L = arc_Speed_L/D_STEP;
		step_Speed_R = arc_Speed_R/D_STEP;
		
		
		arc_Accel_L = arc_Accel * (1 + WHEEL_BASE/arc_Radius);
		arc_Accel_R = arc_Accel * (1 - WHEEL_BASE/arc_Radius);
		step_Accel_L = arc_Accel_L/D_STEP;
		step_Accel_R = arc_Accel_R/D_STEP;
		
		STEPPER_move_stwt( STEPPER_BOTH, 
		step_Fwd_L, step_Num_L, step_Speed_L, step_Accel_L, arc_Brk, // Left
		step_Fwd_R, step_Num_R, step_Speed_R, step_Accel_R, arc_Brk); // Right
		return SUCCESS;
	}	
	return FAIL;
}

/*******************************************************************
* Function:			char move_arc_stnb(float, float, float, float, BOOL)
* Input Variables:	char
* Output Return:	float, float, float, float, BOOL
* Overview:			This moves the robot in any arc length
********************************************************************/
char move_arc_stnb(float arc_Radius, float arc_Length, float arc_Speed, float arc_Accel, BOOL arc_Brk)
{
	
	BOOL step_Fwd_L = (arc_Length>0);
	BOOL step_Fwd_R = (arc_Length>0);
	float step_Num = abs(arc_Length/D_STEP);
	float step_Speed = abs(arc_Speed/D_STEP);
	float step_Accel = abs(arc_Accel/D_STEP);


	if(arc_Radius == NO_TURN){
		STEPPER_move_stnb( STEPPER_BOTH, 
		step_Fwd_L, step_Num, step_Speed, step_Accel, arc_Brk, // Left
		step_Fwd_R, step_Num, step_Speed, step_Accel, arc_Brk); // Right
		return SUCCESS;
	}
	
	if(arc_Radius == POINT_TURN){				
		STEPPER_move_stnb( STEPPER_BOTH, 
	   !step_Fwd_L, step_Num, step_Speed, step_Accel, arc_Brk, // Left
		step_Fwd_R, step_Num, step_Speed, step_Accel, arc_Brk); // Right
		return SUCCESS;
	}
		
	float arc_Length_L;
	float arc_Length_R;	
	float arc_Speed_L;
	float arc_Speed_R;	
	float arc_Accel_L;
	float arc_Accel_R;
		
	float step_Num_L;
	float step_Num_R;	
	float step_Speed_L;
	float step_Speed_R;	
	float step_Accel_L;
	float step_Accel_R;
	
	if(arc_Radius > 0){
		arc_Length_L = arc_Length * (1 - WHEEL_BASE/arc_Radius);
		arc_Length_R = arc_Length * (1 + WHEEL_BASE/arc_Radius);
		step_Num_L = arc_Length_L/D_STEP;
		step_Num_R = arc_Length_R/D_STEP;
		
		
		arc_Speed_L = arc_Speed * (1 - WHEEL_BASE/arc_Radius);
		arc_Speed_R = arc_Speed * (1 + WHEEL_BASE/arc_Radius);
		step_Speed_L = arc_Speed_L/D_STEP;
		step_Speed_R = arc_Speed_R/D_STEP;
		
		
		arc_Accel_L = arc_Accel * (1 - WHEEL_BASE/arc_Radius);
		arc_Accel_R = arc_Accel * (1 + WHEEL_BASE/arc_Radius);
		step_Accel_L = arc_Accel_L/D_STEP;
		step_Accel_R = arc_Accel_R/D_STEP;
		
		STEPPER_move_stnb( STEPPER_BOTH, 
		step_Fwd_L, step_Num_L, step_Speed_L, step_Accel_L, arc_Brk, // Left
		step_Fwd_R, step_Num_R, step_Speed_R, step_Accel_R, arc_Brk); // Right
		return SUCCESS;
	}	
	
	if(arc_Radius < 0){
		arc_Length_L = arc_Length * (1 + WHEEL_BASE/arc_Radius);
		arc_Length_R = arc_Length * (1 - WHEEL_BASE/arc_Radius);
		step_Num_L = arc_Length_L/D_STEP;
		step_Num_R = arc_Length_R/D_STEP;
		
		
		arc_Speed_L = arc_Speed * (1 + WHEEL_BASE/arc_Radius);
		arc_Speed_R = arc_Speed * (1 - WHEEL_BASE/arc_Radius);
		step_Speed_L = arc_Speed_L/D_STEP;
		step_Speed_R = arc_Speed_R/D_STEP;
		
		
		arc_Accel_L = arc_Accel * (1 + WHEEL_BASE/arc_Radius);
		arc_Accel_R = arc_Accel * (1 - WHEEL_BASE/arc_Radius);
		step_Accel_L = arc_Accel_L/D_STEP;
		step_Accel_R = arc_Accel_R/D_STEP;
		
		STEPPER_move_stnb( STEPPER_BOTH, 
		step_Fwd_L, step_Num_L, step_Speed_L, step_Accel_L, arc_Brk, // Left
		step_Fwd_R, step_Num_R, step_Speed_R, step_Accel_R, arc_Brk); // Right
		return SUCCESS;
	}	
	return FAIL;
}