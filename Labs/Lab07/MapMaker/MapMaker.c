/*******************************************************************
* FileName:        MapeMaker.c
* Processor:       ATmega324P
* Compiler:        GCC
*
* Code Description: Maps the world for later pathplaning
*
* AUTHORS: Ander Solorzano & Ruffin White   
********************************************************************/

/** Header Files ***************************************************/     
#include "capi324v221.h"
#include "stdio.h"
#include "CEEN_Interfaces.h"
#include "Custom_Support.h"

/** Define Constants Here ******************************************/

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
#define MAX_SPEED 200
#define WALL_STEP 50

// Gateway Thresholds
#define FT_GATEWAY 13
#define BK_GATEWAY 35
#define LT_GATEWAY 30
#define RT_GATEWAY 30


// Maximum number of User Moves
#define MAX_MOVE_SIZE 12

// World Size
#define WORLD_ROW_SIZE 4
#define WORLD_COLUMN_SIZE 4
#define WORLD_RESOLUTION_SIZE 45.72


/** Local Function Prototypes **************************************/
char moveWall(void);
char moveWander(void);
char moveAway(void);
void movesInput(void);
void worldInput(void);
char moveBehavior(int);
char moveWorld(void);
char mapWorld(void);
void checkWorld(void);
void getGateways(void);
void setGateways(void);
void orientationInput(void);
void checkOdometry(unsigned char);
void printMap(void);
void printCell(unsigned char, unsigned char, unsigned char, BOOL);

/** Global Variables ***********************************************/

// moveBehavior Global Flag Variables
char currentMove;
char oldMove;
char isMapping;

// Create an array for button value commands
unsigned char moveCommands[MAX_MOVE_SIZE];
unsigned char moveGateways[MAX_MOVE_SIZE];
unsigned char currentMoveWorld;
unsigned char currentCellWorld;
unsigned char currentCellWorldStart;
unsigned char currentOrientation;
unsigned char currentOrientationStart;
unsigned char currentGateway;
unsigned char nextGateway;

// odometry values
unsigned char odometryStepL;
unsigned char odometryStepR;
unsigned char odometryTrigger;
unsigned char odometryFlag;
STEPPER_STEPS curr_step;



// Map of the Robot World
unsigned char ROBOT_WORLD[WORLD_ROW_SIZE][WORLD_COLUMN_SIZE];

																
/*******************************************************************
* Function:        void CBOT_main( void )
********************************************************************/

void CBOT_main( void )
{
	// initialize the robot
	initializeRobot();
	
	// Enter the robot's current (starting) position
	LCD_printf("START Map/nlocation\n\n\n");	
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();
	worldInput();
	TMRSRVC_delay(1000);//wait 3 seconds
	LCD_clear();
	
	// Enter the robot's current (starting) orientation
	LCD_printf("START Map/norientation\n\n\n");	
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();
	orientationInput();
	TMRSRVC_delay(1000);//wait 3 seconds
	LCD_clear();
	
	odometryTrigger = WORLD_RESOLUTION_SIZE;
	isMapping = 1;
	
	while(isMapping)
	{
		checkIR();	
		checkWorld();
		checkOdometry(0);
		mapWorld();
		isMapping = !((currentCellWorldStart == currentCellWorld)&(currentOrientationStart == currentOrientation));
	}
	
	// Enter the robot's current (starting) position
	LCD_printf("START Path\nlocation\n\n\n");	
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();
	worldInput();
	TMRSRVC_delay(1000);//wait 3 seconds
	LCD_clear();
	
	// Enter the robot's current (starting) orientation
	LCD_printf("START Path\norientation\n\n\n");
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();
	orientationInput();
	TMRSRVC_delay(1000);//wait 3 seconds
	LCD_clear();
	
	// Enter the robot topological commands
	LCD_printf("ENTER Path\ncommands\n\n\n");
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
	
	// unsigned char i = 0;
	// unsigned char orent = 0b0001;
	// while(1){
		// LCD_clear();
		// LCD_printf("Nume: %i\nOrent\n"BYTETOBINARYPATTERN,i,BYTETOBINARY(orent));
		// TMRSRVC_delay(2000);//wait 1 seconds
		// orent  = rotateCell (orent, 0b01);
		// i++;
	// }
		
		
	// Infinite loop
	while (1)
    {
		checkIR();	
		// checkWorld();
		// moveWorld();	
		
		//Test arc function
		// LCD_printf("Move Arc\n\n\n\n");
		// TMRSRVC_delay(1000);//wait 1 seconds
		// move_arc_stwt(POINT_TURN, LEFT_TURN, 10, 10, 0);
			
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
* Function:			void printMap(void)
* Input Variables:	void
* Output Return:	void
* Overview:		    Print the map
********************************************************************/
void printMap(void)
{
	unsigned char r;
	unsigned char c;
	unsigned char cell;
	
	unsigned char curRow = currentCellWorld >> 2;
	unsigned char curCol = currentCellWorld & 0b0011;
	
	BOOL isrobot;
	for(r = 0; r < WORLD_ROW_SIZE; r++){
		for(c = 0; c < WORLD_COL_SIZE; c++){
			cell = ROBOT_WORLD[r][c];
			isrobot = (r == curRow)&&(c == curCol);
			printCell(cell, r, c, isrobot);
		}	
	}
}

/*******************************************************************
* Function:			void printCell(unsigned char, unsigned char, unsigned char)
* Input Variables:	void
* Output Return:	unsigned char, unsigned char, unsigned char
* Overview:		    Prints the cell
********************************************************************/
void printCell(unsigned char cell, unsigned char r, unsigned char c, BOOL isrobot){

	r = r*8;
	c = c*8;
	
	LCD_set_pixel(r,   c,   1);
	LCD_set_pixel(r+7, c,   1);
	LCD_set_pixel(r,   c+7, 1);
	LCD_set_pixel(r+7, c+7, 1);
	
	if(cell&0b1000){
		LCD_set_pixel(r, c+1, 1);
		LCD_set_pixel(r, c+2, 1);
		LCD_set_pixel(r, c+3, 1);
		LCD_set_pixel(r, c+4, 1);
		LCD_set_pixel(r, c+5, 1);		
		LCD_set_pixel(r, c+6, 1);		
	}
	if(cell&0b0100){
		LCD_set_pixel(r+1, c+7, 1);
		LCD_set_pixel(r+2, c+7, 1);
		LCD_set_pixel(r+3, c+7, 1);
		LCD_set_pixel(r+4, c+7, 1);
		LCD_set_pixel(r+5, c+7, 1);		
		LCD_set_pixel(r+6, c+7, 1);			
	}
	if(cell&0b0010){
		LCD_set_pixel(r+7, c+1, 1);
		LCD_set_pixel(r+7, c+2, 1);
		LCD_set_pixel(r+7, c+3, 1);
		LCD_set_pixel(r+7, c+4, 1);
		LCD_set_pixel(r+7, c+5, 1);		
		LCD_set_pixel(r+7, c+6, 1);		
	}
	if(cell&0b0001){
		LCD_set_pixel(r+1, c, 1);
		LCD_set_pixel(r+2, c, 1);
		LCD_set_pixel(r+3, c, 1);
		LCD_set_pixel(r+4, c, 1);
		LCD_set_pixel(r+5, c, 1);		
		LCD_set_pixel(r+6, c, 1);		
	}	
	if(isrobot){
		LCD_set_pixel(r+3, c+3, isrobot);
		LCD_set_pixel(r+4, c+3, isrobot);
		LCD_set_pixel(r+3, c+4, isrobot);
		LCD_set_pixel(r+4, c+4, isrobot);
	}
}

/*******************************************************************
* Function:			void checkOdometry(unsigned char)
* Input Variables:	void
* Output Return:	unsigned char reset resets the odometry
* Overview:		    Checks the current odometry to the trigger and
*					sets the flag whe appropriate
********************************************************************/
void checkOdometry( unsigned char reset )
{	
	// Update the avrage 
	unsigned char odometry = ((odometryStepL + odometryStepR)/2)*D_STEP;
	// check to see if we have traveresed the trigger distance
	// or that a reset has been called
	if((odometry > odometryTrigger)||(reset))
	{
		odometryFlag = 1;
		odometryStepL = 0;
		odometryStepR = 0;
	}
}

/*******************************************************************
* Function:			char mapWorld(void)
* Input Variables:	void
* Output Return:	char
* Overview:		    maps the world as it moves through it
********************************************************************/
char mapWorld( void )
{	
	if(!(currentGateway&0b0001)){	
		// If we can make a left turn,
		// then turn left
		currentMove = MOVE_LEFT;
		// Reset Odometry
		checkOdometry(1);
	}
	else if(!(currentGateway&0b1000)){
		// If we can't make a left turn,
		// but we can go forward,
		// go forward
		currentMove = MOVE_FORWARD;
	}
	else {
		// If we can't turn left or go forward
		// spin right
		currentMove = MOVE_RIGHT;
		// Reset Odometry
		checkOdometry(1);
	}
	
	if(odometryFlag)
	{
		// Only update the map if we are done moving
		setGateways();
	}
	
	LCD_clear();
	switch(currentMove){
		case MOVE_LEFT:
			LCD_printf("Left\nCurMove:%i\nGateway:%i\nNextGateway:%i\n",currentMoveWorld,currentGateway,nextGateway);
			// TMRSRVC_delay(1000);//wait 1 seconds
			move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE*(2.8/5.0), 10, 10, 0);
			move_arc_stwt(POINT_TURN, LEFT_TURN, 10, 10, 0);
			move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE, 10, 10, 0);
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
			break;
	}
	
	// TMRSRVC_delay(1000);//wait 1 seconds
	return 1;
}

/*******************************************************************
* Function:			char setGateways(void)
* Input Variables:	void
* Output Return:	void
* Overview:		    Interpolates the map using the current 
*					orientation and reading
********************************************************************/
void setGateways(void)
{
	// This will be the gatway the robot will look for
	unsigned char curCell = currentGateway;
		
	// Get the start location of the robot
	unsigned char curRow = currentCellWorld >> 2;
	unsigned char curCol = currentCellWorld & 0b0011;
		
	// Git the start orientation of the robot
	unsigned char curOrient = currentOrientation;
		
	// Rotate the cell with reference to the robot
	curCell = rotateCell(curCell,curOrient,0);
	
	// Set the current cell
	ROBOT_WORLD[curRow][curCol] = curCell;
		
		
	switch(currentMove){
		case MOVE_LEFT:
			curOrient--;
			curOrient = curOrient&0b11;
			break;
		case MOVE_FORWARD:
			break;
		case MOVE_RIGHT:		
			curOrient++;
			curOrient = curOrient&0b11;
			break;
		default:
			LCD_printf("Whatz?!");
			break;
	}
	
	// move to the next cell with respect to our orientation
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
	currentCellWorld = (curRow << 2) + curCol;
}

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
	// unsigned char curRow = (currentCellWorld>>2) & 0b1100;
	// unsigned char curCol = currentCellWorld & 0b0011;
	
	unsigned char curRow = currentCellWorld >> 2;
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
		curCell = rotateCell(curCell,curOrient,1);
		
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
		currentCellWorldStart = currentCellWorld;
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
* Overview:			Stores the button values pressed by user into an
*					array of max size 32.
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
* Overview:		    Moves robot through the world
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
	
	if(((oldMove == MOVE_LEFT)|(oldMove == MOVE_RIGHT))&(currentMove == MOVE_FORWARD))
	{
		move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE*(3.0/5.0), 10, 10, 0);		
	}
	
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
	return 0;	
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
	
	// Update odometry
	curr_step = STEPPER_get_nSteps();
	odometryStepL += WALL_STEP - curr_step.left;
	odometryStepR += WALL_STEP - curr_step.right;
	
	// Move with wall
	STEPPER_move_stnb( STEPPER_BOTH, 
	STEPPER_REV, WALL_STEP, stepper_speed_L, 450, STEPPER_BRK_OFF, // Left
	STEPPER_REV, WALL_STEP, stepper_speed_R, 450, STEPPER_BRK_OFF ); // Right
	
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
