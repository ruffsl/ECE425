/*******************************************************************
* FileName:        WallFollow_LightTrack.c
* Processor:       ATmega324P
* Compiler:        
*
* Code Description:
*		This code runs a wall following behavior and tracks a light source 
*		when it detects a light beacon. It will then proceed to dock in front of the light,
*		retreat away from the light, and then continue tracking the wall from the same spot
*		it left the wall.
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

// Flags
#define SUCCESS 1;
#define FAIL 0;

// Arc Function Constants
#define C_B 1
#define C_L 1
#define C_R 1
#define D_STEP 0.1335176878
#define WHEEL_BASE 21.3
#define POINT_TURN 0
#define NO_TURN 2147483647
#define RIGHT_TURN 17.50
#define LEFT_TURN -17.50

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// World Constants
#define WORLD_ROW_SIZE 4
#define WORLD_COLUMN_SIZE 4

// Movement Commands for pathplanning
#define MOVE_LEFT 1
#define MOVE_FORWARD 2
#define MOVE_RIGHT 3
#define MOVE_STOP 4




/** Local Function Prototypes **************************************/
void printMetricMap(void);
void wavefrontMake(void);
unsigned char fourNeighborSearch(unsigned char);
void metricMove(void);
char move_arc_stwt(float, float, float, float, BOOL);
unsigned char shiftMap(unsigned char, unsigned char, unsigned char);
void moveMap(void);

/** Global Variables ***********************************************/
unsigned char minNeighbor;
unsigned char nextMetricCell = 0b0000;
unsigned char currentGoalWorld;
unsigned char nextOrientation;
unsigned char currentMove;
unsigned char currentOrientation;
unsigned char currentCellWorld;
unsigned char reachedEnd;

// Create the Robot World and Metric World
static unsigned char ROBOT_WORLD[WORLD_ROW_SIZE][WORLD_COLUMN_SIZE] = 	
																{
																{0b1101,0b1111,0b1111,0b1101},
																{0b0101,0b1111,0b1111,0b0101},
																{0b0001,0b1010,0b1010,0b0100},
																{0b0111,0b1111,0b1111,0b0111}
																};
																
static unsigned char WORLD_CELL[WORLD_ROW_SIZE][WORLD_COLUMN_SIZE] = 	
																{
																{0b0000,0b0001,0b0010,0b0011},
																{0b0100,0b0101,0b0110,0b0111},
																{0b1000,0b1001,0b1010,0b1011},
																{0b1100,0b1101,0b1110,0b1111}
																};
																
unsigned char ROBOT_METRIC_WORLD[WORLD_ROW_SIZE][WORLD_COLUMN_SIZE] =	
																{
																{0,0,0,0},
																{0,0,0,0},
																{0,0,0,0},
																{0,0,0,0}
																};


	
/*******************************************************************
* Function:        void CBOT_main( void )
********************************************************************/

void CBOT_main( void )
{
	// Initialize Robot
	ATopstat = ATTINY_open();//open the tiny microcontroller
	LEopstat = LED_open(); //open the LED module
	LCopstat = LCD_open(); //open the LCD module
	STEPPER_open(); // Open STEPPER module for use
	SPKR_open(SPKR_BEEP_MODE);//open the speaker in beep mode
	
	LED_open();
	I2C_open();
	ADC_open();//open the ADC module
 	ADC_set_VREF( ADC_VREF_AVCC );// Set the Voltage Reference first so VREF=5V.

	// // Print a debug statement
	// LCD_printf("It's ALIVE\n\n\n\n");
	// TMRSRVC_delay(3000);//wait 3 seconds
	// LCD_clear();
	
	// Print the metric Map
	// printMetricMap();
	// LCD_printf("%i %i %i %i \n%i %i %i %i \n%i %i %i %i \n%i %i %i %i \n",ROBOT_METRIC_WORLD[0][0],ROBOT_METRIC_WORLD[0][1],ROBOT_METRIC_WORLD[0][2],ROBOT_METRIC_WORLD[0][3],ROBOT_METRIC_WORLD[1][0],ROBOT_METRIC_WORLD[1][1],ROBOT_METRIC_WORLD[1][2],ROBOT_METRIC_WORLD[1][3],ROBOT_METRIC_WORLD[2][0],ROBOT_METRIC_WORLD[2][1],ROBOT_METRIC_WORLD[2][2],ROBOT_METRIC_WORLD[2][3],ROBOT_METRIC_WORLD[3][0],ROBOT_METRIC_WORLD[3][1],ROBOT_METRIC_WORLD[3][2],ROBOT_METRIC_WORLD[3][3]);
	// TMRSRVC_delay(10000);//wait 10 seconds
	// wavefrontMake();
	// LCD_printf("%i %i %i %i \n%i %i %i %i \n%i %i %i %i \n%i %i %i %i \n",ROBOT_METRIC_WORLD[0][0],ROBOT_METRIC_WORLD[0][1],ROBOT_METRIC_WORLD[0][2],ROBOT_METRIC_WORLD[0][3],ROBOT_METRIC_WORLD[1][0],ROBOT_METRIC_WORLD[1][1],ROBOT_METRIC_WORLD[1][2],ROBOT_METRIC_WORLD[1][3],ROBOT_METRIC_WORLD[2][0],ROBOT_METRIC_WORLD[2][1],ROBOT_METRIC_WORLD[2][2],ROBOT_METRIC_WORLD[2][3],ROBOT_METRIC_WORLD[3][0],ROBOT_METRIC_WORLD[3][1],ROBOT_METRIC_WORLD[3][2],ROBOT_METRIC_WORLD[3][3]);
	// TMRSRVC_delay(10000);//wait 10 seconds
	// LCD_clear();
	
	// Test the 4-Neighbor search
	
	// LCD_printf("%i %i %i %i \n%i %i %i %i \n%i %i %i %i \n%i %i %i %i \n",ROBOT_METRIC_WORLD[0][0],ROBOT_METRIC_WORLD[0][1],ROBOT_METRIC_WORLD[0][2],ROBOT_METRIC_WORLD[0][3],ROBOT_METRIC_WORLD[1][0],ROBOT_METRIC_WORLD[1][1],ROBOT_METRIC_WORLD[1][2],ROBOT_METRIC_WORLD[1][3],ROBOT_METRIC_WORLD[2][0],ROBOT_METRIC_WORLD[2][1],ROBOT_METRIC_WORLD[2][2],ROBOT_METRIC_WORLD[2][3],ROBOT_METRIC_WORLD[3][0],ROBOT_METRIC_WORLD[3][1],ROBOT_METRIC_WORLD[3][2],ROBOT_METRIC_WORLD[3][3]);
	// TMRSRVC_delay(5000);
	// LCD_clear();
	
	currentCellWorld = WORLD_CELL[0][0];
	wavefrontMake();
	
	while(currentCellWorld!=reachedEnd){
		nextOrientation = fourNeighborSearch(currentCellWorld);
		metricMove();
		moveMap();
		currentCellWorld = shiftMap(currentCellWorld, currentMove, currentOrientation);
	}
	LCD_printf("LOOOOOOOOOOOOLZ");
	TMRSRVC_delay(5000);
	LCD_clear();
	
	// Unit test the function
	
	// sqrt function
	// int xDelta, yDelta;
	// float distance;
	// xDelta = abs(-3);
	// yDelta = abs(0);
	// distance = sqrt(((xDelta*xDelta)+(yDelta*yDelta)));
	// LCD_printf("xDel = %d\nyDel = %d\ndist = %f\n\n",xDelta,yDelta,distance);
	// TMRSRVC_delay(5000);
	// LCD_clear();
	
	// cell values
	
	
	
}// end the CBOT_main()

/*******************************************************************
* Additional Helper Functions
********************************************************************/

/*******************************************************************
* Function:                                         void moveMap(void)
* Input Variables:            void
* Output Return:             void
* Overview:                           moves the robot through the map
********************************************************************/
void moveMap( void )
{              
	switch(currentMove){
		case MOVE_LEFT:
			move_arc_stwt(POINT_TURN, LEFT_TURN, 10, 10, 0);
			break;
		case MOVE_FORWARD:
			// checkOdometry(1);
			// while(!odometryFlag){
							// moveWall();
							// checkOdometry(0);
			// }
			
			move_arc_stwt(NO_TURN, 45, 10, 10, 0);
			break;
		case MOVE_RIGHT:
			move_arc_stwt(POINT_TURN, RIGHT_TURN, 10, 10, 0);
			break;
		default:
			LCD_printf("Whatz2?!");
			break;
	}
}

/*******************************************************************
* Function:                                         unsigned char shiftMap(unsigned char currentCell, unsigned char curMove, unsigned char curOrient)
* Input Variables:            unsigned char, unsigned char, unsigned char
* Output Return:             unsigned char
* Overview:                           shifts the map after robot moves
********************************************************************/
unsigned char shiftMap( unsigned char currentCell, unsigned char curMove, unsigned char curOrient)
{                              
                // Get the currrent location of the robot
                unsigned char curRow = currentCell >> 2;
                unsigned char curCol = currentCell & 0b0011;
                                
                // // Git the currrent orientation of the robot
                // unsigned char curOrient = currentOrientation;
                                                
                                
                switch(curMove){
                                case MOVE_LEFT:
                                                //If we move left
                                                // shift our oriention CCW
                                                curOrient--;
                                                curOrient = curOrient&0b11;
                                                break;
                                case MOVE_FORWARD:
                                                //If we move forward
                                                // then shift to the next cell
                                                // with repect to our curent oriention
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
                                                break;
                                case MOVE_RIGHT:
                                                //If we move right
                                                // shift our oriention CW
                                                curOrient++;
                                                curOrient = curOrient&0b11;
                                                break;
                                default:
                                                LCD_printf("Whatz3?!");
                                                break;
                }
                
                // Set the new cell of the robot
                currentCell = (curRow << 2) + curCol;
                // Set the new orientation of the robot
                currentOrientation = curOrient;
                return currentCell;
}

/*******************************************************************
* Function:			void metricMove(void)
* Input Variables:	none
* Output Return:	none
* Overview:			Moves around the map using metric navigation
********************************************************************/
void metricMove (void)
{
	switch(currentOrientation){
		case NORTH:
			switch(nextOrientation){
				case NORTH:
					currentMove = MOVE_FORWARD; break;
				case EAST:
					currentMove = MOVE_RIGHT; break;
				case SOUTH:
					currentMove = MOVE_LEFT; break;
				case WEST:
					currentMove = MOVE_LEFT; break;
				default:
					LCD_printf("Whatz5?!"); break;
			}
			break;
		case EAST:
			switch(nextOrientation){
				case NORTH:
					currentMove = MOVE_LEFT; break;
				case EAST:
					currentMove = MOVE_FORWARD; break;
				case SOUTH:
					currentMove = MOVE_RIGHT; break;
				case WEST:
					currentMove = MOVE_LEFT; break;
				default:
					LCD_printf("Whatz5?!"); break;
			}
			break;
		case SOUTH:
			switch(nextOrientation){
				case NORTH:
					currentMove = MOVE_LEFT; break;
				case EAST:
					currentMove = MOVE_LEFT; break;
				case SOUTH:
					currentMove = MOVE_FORWARD; break;
				case WEST:
					currentMove = MOVE_RIGHT; break;
				default:
					LCD_printf("Whatz5?!"); break;
			}
			break;
		case WEST:
			switch(nextOrientation){
				case NORTH:
					currentMove = MOVE_RIGHT; break;
				case EAST:
					currentMove = MOVE_LEFT; break;
				case SOUTH:
					currentMove = MOVE_LEFT; break;
				case WEST:
					currentMove = MOVE_FORWARD; break;
				default:
					LCD_printf("Whatz5?!"); break;
			}
			break;
		default:
			LCD_printf("Whatz5?!"); break;
	}
}

/*******************************************************************
* Function:			void wavefrontMake(unsigned char)
* Input Variables:	currentGoalWorld
* Output Return:	none
* Overview:			Makes the wavefront metric map to goal location from current location 
********************************************************************/
void wavefrontMake(void)
{
	// Enter the goal location here
	unsigned char goalLocation = WORLD_CELL[3][3];
	reachedEnd = goalLocation;
	
	// User-defined goal location
	// unsigned char goalLocation = currentGoalWorld;
	// Extract x and y goal location
	int xGoal = (goalLocation>>2);
	int yGoal = (goalLocation&0b0011);
	// Declare some variables and initialize a distance 
	unsigned int xDelta, yDelta;
	int row, col;
	int distance = 0;
	
	// search every cell in the world
	for(row = 0; row < WORLD_ROW_SIZE; row++)
	{
		for(col = 0; col < WORLD_COLUMN_SIZE; col++)
		{
			// for cells with 4 walls, set metric map vaule to 99
			if(ROBOT_WORLD[row][col] == 0b1111){
				ROBOT_METRIC_WORLD[row][col] = 99;
			}
			// for all other cells compute the distance
			else{
				// compute the differences in rows and columns
				xDelta = abs((WORLD_CELL[row][col]>>2) - xGoal);
				yDelta = abs((WORLD_CELL[row][col]&0b0011) - yGoal);
				// compute the distance without using sqrt
				distance = ((xDelta*xDelta)+(yDelta*yDelta));
				// overwrite the cells in the metric map to the actual distance values
				ROBOT_METRIC_WORLD[row][col] = distance;
			}
		}
	}
}
/*******************************************************************
* Function:			void fourNeighborSearch(void)
* Input Variables:	none
* Output Return:	unsigned char
* Overview:			searches the four-neighboring cells in the wavefront map
********************************************************************/
unsigned char fourNeighborSearch(unsigned char curCellWorld)
{
	// Enter the robot's current position
	// unsigned char curCell = WORLD_CELL[2][2];
	
	// Make a current position
	// Enter the goal location here
	unsigned char curCell = curCellWorld;
	unsigned char nextGoCell;
	
	// Make some initial variables
	unsigned char minVal;
	unsigned char topVal,leftVal,botVal,rightVal;
	unsigned char minInd;
	
	// Get the cell current row and column
	unsigned char curRow = (curCell>>2);
	unsigned char curCol = (curCell&0b0011);
	
	// Reset minInd value and minNeighbor
	minVal = 0;
	minInd = 0;
	nextMetricCell = 0b0000;
	
	// Perform a 4-neighbor search and store the lowest value
	// Robot in four-corners (2-neighbors to worry about)
	
	// Top-left corner
	if(curCell==0b0000){
		rightVal = ROBOT_METRIC_WORLD[curRow][curCol+1];
		botVal = ROBOT_METRIC_WORLD[curRow+1][curCol];
		if(rightVal<botVal){
			minVal=rightVal;
			nextGoCell = EAST;
			return nextGoCell;
		}
		else{
			minVal=botVal;
			nextGoCell = SOUTH;
			return nextGoCell;
		}
	}
	
	// Top-right corner
	if(curCell==0b0011){
		leftVal = ROBOT_METRIC_WORLD[curRow][curCol-1];
		botVal = ROBOT_METRIC_WORLD[curRow+1][curCol];
		if(leftVal<botVal){
			minVal=leftVal;
			nextGoCell = WEST;
			return nextGoCell;
		}
		else{
			minVal=botVal;
			nextGoCell = SOUTH;
			return nextGoCell;
		}
	}
	
	// Bottom-left corner
	if(curCell==0b1100){
		rightVal = ROBOT_METRIC_WORLD[curRow][curCol+1];
		topVal = ROBOT_METRIC_WORLD[curRow-1][curCol];
		if(rightVal<topVal){
			minVal=rightVal;
			nextGoCell = EAST;
			return nextGoCell;
		}
		else{
			minVal=topVal;
			nextGoCell = NORTH;
			return nextGoCell;
		}
	}
	
	// Bottom-right corner
	if(curCell==0b1111){
		leftVal = ROBOT_METRIC_WORLD[curRow][curCol-1];
		topVal = ROBOT_METRIC_WORLD[curRow-1][curCol];
		if(leftVal<topVal){
			minVal=leftVal;
			nextGoCell = WEST;
			return nextGoCell;
		}
		else{
			minVal=topVal;
			nextGoCell = NORTH;
			return nextGoCell;
		}
	}
	
	// Robot on top boundary of world (row = 0)
	else if(curRow == 0){
		leftVal = ROBOT_METRIC_WORLD[curRow][curCol-1];
		rightVal = ROBOT_METRIC_WORLD[curRow][curCol+1];
		botVal = ROBOT_METRIC_WORLD[curRow+1][curCol];
		if((leftVal<rightVal)&&(leftVal<botVal)){
			minVal = leftVal;
			nextGoCell = WEST;
			return nextGoCell;
		}
		if((rightVal<leftVal)&&(rightVal<botVal)){
			minVal = rightVal;
			nextGoCell = EAST;
			return nextGoCell;
		}
		else{
			minVal = botVal;
			nextGoCell = SOUTH;
			return nextGoCell;
		}
		
	}
	
	// Robot on left boundary of world (col = 0)
	else if(curCol == 0){
		rightVal = ROBOT_METRIC_WORLD[curRow][curCol+1];
		botVal = ROBOT_METRIC_WORLD[curRow+1][curCol];
		topVal = ROBOT_METRIC_WORLD[curRow-1][curCol];
		if((rightVal<botVal)&&(rightVal<topVal)){
			minVal = rightVal;
			nextGoCell = EAST;
			return nextGoCell;
		}
		if((botVal<rightVal)&&(botVal<topVal)){
			minVal = botVal;
			nextGoCell = SOUTH;
			return nextGoCell;
		}
		else{
			minVal = topVal;
			nextGoCell = NORTH;
			return nextGoCell;
		}
	}
	
	// Robot on bottom boundary of world (row = 3)
	else if(curRow == 3){
		leftVal = ROBOT_METRIC_WORLD[curRow][curCol-1];
		rightVal = ROBOT_METRIC_WORLD[curRow][curCol+1];
		topVal = ROBOT_METRIC_WORLD[curRow-1][curCol];
		if((leftVal<rightVal)&&(leftVal<topVal)){
			minVal = leftVal;
			return minInd=WORLD_CELL[curRow][curCol-1];
		}
		if((rightVal<leftVal)&&(rightVal<topVal)){
			minVal = rightVal;
			return minInd=WORLD_CELL[curRow][curCol+1];
		}
		else{
			minVal = topVal;
			return minInd=WORLD_CELL[curRow-1][curCol]; 
		}
	}
	
	// Robot on right boundary of world (col = 3)
	else if(curCol == 3){
		leftVal = ROBOT_METRIC_WORLD[curRow][curCol-1];
		topVal = ROBOT_METRIC_WORLD[curRow-1][curCol];
		botVal = ROBOT_METRIC_WORLD[curRow+1][curCol];
		if((leftVal<topVal)&&(leftVal<botVal)){
			minVal = leftVal;
			nextGoCell = WEST;
			return nextGoCell;
		}
		if((topVal<leftVal)&&(topVal<botVal)){
			minVal = topVal;
			nextGoCell = NORTH;
			return nextGoCell;
		}
		else{
			minVal = botVal;
			nextGoCell = SOUTH;
			return nextGoCell;
		}
	}
	
	// // Else the robot is inside the world with four-neighboring cells
	else{
		topVal = ROBOT_METRIC_WORLD[curRow-1][curCol];
		leftVal = ROBOT_METRIC_WORLD[curRow][curCol-1];
		botVal = ROBOT_METRIC_WORLD[curRow+1][curCol];
		rightVal = ROBOT_METRIC_WORLD[curRow][curCol+1];
		if((topVal<leftVal)&&(topVal<botVal)&&(topVal<rightVal)){
			minVal = topVal;
			nextGoCell = NORTH;
			return nextGoCell;
		}
		if((leftVal<topVal)&&(leftVal<botVal)&&(leftVal<rightVal)){
			minVal = leftVal;
			nextGoCell = WEST;
			return nextGoCell;
		}
		if((botVal<topVal)&&(botVal<leftVal)&&(botVal<rightVal)){
			minVal = botVal;
			nextGoCell = SOUTH;
			return nextGoCell;
		}
		else{
			minVal = rightVal;
			nextGoCell = EAST;
			return nextGoCell;
		}
	}

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
* Function:			void printMetricMap(void)
* Input Variables:	none
* Output Return:	none
* Overview:			print the map in metric
********************************************************************/
void printMetricMap(void)
{
	int row,col;
	
	for (row = 0; row < WORLD_ROW_SIZE; row++)
	{
		for (col = 0; col < WORLD_ROW_SIZE; col++)
		{
			if(ROBOT_WORLD[row][col] == 0b1111){
				ROBOT_METRIC_WORLD[row][col] = 99;
			}
			else{
			ROBOT_METRIC_WORLD[row][col] = 1;
			}
		}
	}
}
