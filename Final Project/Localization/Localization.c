/*******************************************************************
* FileName:        Localization.c
* Processor:       ATmega324P
* Compiler:        GCC
*
* Code Description: Localizes its self in the world
*
* AUTHORS: Ander Solorzano & Ruffin White   
********************************************************************/

/** Header Files ***************************************************/
#include "Custom_Support.h"

/** Define Constants Here ******************************************/

// localizeGateways Size
#define BRANCH_TYPES 3
#define BRANCH_MAX 5


/** Local Function Prototypes **************************************/
// Locomotion Primitives
char moveWallOld(void);


// Pathplanning Tools
void getGateways(void);
void setGateways(void);
void metric(void);

void wavefrontMake(void);
unsigned char fourNeighborSearch(unsigned char);
void planMetric(void);

// Mapping Tools
void planMap(void);
void moveMap(void);
unsigned char shiftMap(unsigned char, unsigned char, unsigned char);
void map (void);

// Localization Tools
void planGateway(void);
char localizeGateway(void);
char matchBranch( unsigned char, unsigned char);
void localize(void);

// Firefighting tools
unsigned char checkFire(void);


/** Global Variables ***********************************************/

// moveBehavior Global Flag Variables
unsigned char matchSeeds;
unsigned char isMapping;
unsigned char isLost;
unsigned char isGoal;
unsigned char isFire;


unsigned char localizeGateways[BRANCH_TYPES][BRANCH_MAX] = {
															{0,0,0,0,0},
															{0,0,0,0,0},
															{0,0,0,0,0}};
															
unsigned char currentBranch = 0;

unsigned char ROBOT_WORLD[WORLD_ROW_SIZE][WORLD_COLUMN_SIZE] = 	{
																{0b1011, 0b1000, 0b1010, 0b1100}, 
																{0b1111, 0b0101, 0b1111, 0b0101}, 
																{0b1111, 0b0101, 0b1111, 0b0101}, 
																{0b1111, 0b0111, 0b1111, 0b0111}
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
	// initialize the robot
	initializeRobot();
	currentOrientation = NORTH;
	
	// Ask for Goal
	char isDone = 0;
	unsigned char btnHolder = UNPRESSED;
	LCD_clear();
	LCD_printf("      Goal?\n\n\n\n");
	while(!isDone){
		btnHolder = EnterTopoCommand();
		switch(btnHolder){
			case MOVE_LEFT:
				currentGoalWorld--;
				currentGoalWorld = currentGoalWorld&0b1111;
				break;
			case MOVE_FORWARD:
				isDone = 1;
				break;
			case MOVE_RIGHT:
				currentGoalWorld++;
				currentGoalWorld = currentGoalWorld&0b1111;
				break;
			default:
				break;
		}
		printMap(currentOrientation,currentGoalWorld,RESET);
		TMRSRVC_delay(100);//wait .1 seconds
	}
	
	// Ask for starting orentation
	isDone = 0;
	btnHolder = UNPRESSED;
	LCD_clear();
	LCD_printf("      Orient?\n\n\n\n");
	while(!isDone){
		btnHolder = EnterTopoCommand();
		switch(btnHolder){
			case MOVE_LEFT:
				// If we move left
				// shift our oriention CCW
				currentOrientation--;
				currentOrientation = currentOrientation&0b11;
				break;
			case MOVE_FORWARD:
				isDone = 1;
				break;
			case MOVE_RIGHT:
				// If we move right
				// shift our oriention CW
				currentOrientation++;
				currentOrientation = currentOrientation&0b11;
				break;
			default:
				break;
		}
		printMap(currentOrientation,currentGoalWorld,RESET);
		TMRSRVC_delay(100);//wait .1 seconds
	}
	
	// Ask to start
	isDone = 0;
	btnHolder = UNPRESSED;
	LCD_clear();
	LCD_printf("      Start?\n\n\n\n");
	while(!isDone){
		btnHolder = EnterTopoCommand();
		switch(btnHolder){
			case MOVE_LEFT:
				break;
			case MOVE_FORWARD:
				isDone = 1;
				break;
			case MOVE_RIGHT:
				break;
			default:
				break;
		}
		printMap(currentOrientation,currentGoalWorld,RESET);
		TMRSRVC_delay(100);//wait .1 seconds
	}
	
	// Locilize the Robot
	// localize();
	
	// Initialize State
	isLost = 1;
	oldMove = MOVE_STOP;
	
	// Localization Loop 
	while(isLost)
	{	
		// Break if not isLost
		if(!isLost){
			break;
		}
		
		//Sense Gateway
		checkIR();	
		checkWorld();
		
		//Plan using the Gateway
		planGateway();
		
		//Localize from Gateways?
		isLost = localizeGateway();
		
		//Act on the Gateway
		moveMap();
	}
		
	// // Update the currentOrientation using currentMove
	// switch(currentMove){
		// case MOVE_LEFT:
			// // If we move left
			// // shift our oriention CCW
			// currentOrientation--;
			// currentOrientation = currentOrientation&0b11;
			// break;
		// case MOVE_FORWARD:
			// break;
		// case MOVE_RIGHT:
			// // If we move right
			// // shift our oriention CW
			// currentOrientation++;
			// currentOrientation = currentOrientation&0b11;
			// break;
		// default:
			// LCD_printf("Whatz2?!");
			// break;
	// }
	
		SPKR_beep(500);	
		// LCD_clear();
		// LCD_printf("LOLZ\nI'm found!");
		// TMRSRVC_delay(3000);//wait 3 seconds
		
		LCD_clear();
		LCD_printf("      New Map\n\n\n\n");
		printMap(currentOrientation,currentCellWorld,RESET);
		TMRSRVC_delay(5000);//wait 1 seconds
		SPKR_beep(0);
		
		
	// currentCellWorld = 0;
	isFire = 0;
	
	// Go firefight
	while(!isFire){
		//Sense Gateway
		checkIR();	
		checkWorld();
		
		isFire = checkFire();
		if(isFire){
			break;
		}
		
		// Plan using Map
		planMap();
		
		// Shift the map
		currentCellWorld = shiftMap(currentCellWorld, currentMove, currentOrientation);
		
		// Act on the Map
		moveMap();
	}
	
	// // Beep for the fire SIREN
	// int ii;
	// for (ii=0; ii<=3; ii++){
		// SPKR_beep(250);	
		// TMRSRVC_delay(1000);
		// SPKR_beep(500);	
		// TMRSRVC_delay(1000);
	// }
	// SPKR_beep(0);
	
	// // Print the fire cell location
	// LCD_clear();
	// LCD_printf("Fire = %i\n\n\n\n", currentFireCell);
	// TMRSRVC_delay(5000);
	
	// Moves the Robot to the goal
	metric();
	
		
		// Print that you are at home and the fire cell location
		LCD_clear();
		LCD_printf("LOLZ\nI'm HOME\nFire at Cell: %i\n\n",currentFireCell);
		// Stop when home is reached
		STEPPER_stop(STEPPER_BOTH, STEPPER_BRK_OFF);
		// Beep when home is reached
		SPKR_beep(500);
		TMRSRVC_delay(3000);//wait 3 seconds
		SPKR_beep(0);
		TMRSRVC_delay(7000);//wait 7 seconds
		
	/**
	
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
	
	// Print the map
	LCD_clear();	
	printMap(currentOrientation,currentCellWorld,RESET);
	TMRSRVC_delay(10000);//wait 10 seconds
	LCD_clear();	
	
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
		
		
	// Goal loop
	while (1)
    {
		checkIR();	
		checkWorld();
		moveWorld();
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
	**/

}// end the CBOT_main()

/*******************************************************************
* Additional Helper Functions
********************************************************************/

/*******************************************************************
* Function:			void checkFire(void)
* Input Variables:	none
* Output Return:	unsigned char
* Overview:			Checks world for Fires
********************************************************************/
unsigned char checkFire(void)
{
	// Acquire the current gateway
	unsigned char curGate = currentGateway;
	
	// Acquire the current cell
	unsigned char curRow = (currentCellWorld>>2);
	unsigned char curCol = (currentCellWorld&0b0011);
	unsigned char curCell = ROBOT_WORLD[curRow][curCol];
	
	// Rotate the current gateway wrt to current orientation
	curGate = rotateCell (curGate, currentOrientation, TO_MAP_ROTATE);
	
	// Return the boolean value of the equality
	if (curGate != curCell){
		switch(curGate^curCell){
			case 0b0001:
				currentFireCell = (curRow<<2)+(curCol-1);
				break;
			case 0b0010:
				currentFireCell = ((curRow+1)<<2)+(curCol);
				return SUCCESS;
				break;
			case 0b0100:
				currentFireCell = (curRow<<2)+(curCol+1);
				return SUCCESS;
				break;
			case 0b1000:
				currentFireCell = ((curRow-1)<<2)+(curCol);
				return SUCCESS;
				break;
			default:
				return FAIL;
				break;
		}
		return SUCCESS;
	}
	return FAIL;
	// return 0;
}

/*******************************************************************
* Function:			void metric(void)
* Input Variables:	none
* Output Return:	none
* Overview:			Moves the robot to the goal
********************************************************************/
void metric (void)
{
	// currentCellWorld = 0b0000;
	// currentGoalWorld = 15;
	
	// Make metric map
	wavefrontMake();
	
	// Initialize State
	isGoal = 0;
	unsigned char isSiren = 0;
	
	// Metric Loop 
	while(!isGoal){
		
		if(isSiren){
			SPKR_beep(500);
			isSiren = 0;
		}
		else{
			SPKR_beep(250);
			isSiren = 1;
		}
	
		LCD_clear();
		
		switch(currentOrientation){
			case NORTH:
				LCD_printf("CurtOrent:NORTH\n");
				break;
			case EAST:
				LCD_printf("CurtOrent:EAST\n");
				break;
			case SOUTH:
				LCD_printf("CurtOrent:SOUTH\n");
				break;
			case WEST:
				LCD_printf("CurtOrent:WEST\n");
				break;
			default:
				break;
		}
	
		// Find the next orentation
		isGoal = fourNeighborSearch(currentCellWorld);
		if(isGoal){
			break;
		}
				
		switch(nextOrientation){
			case NORTH:
				LCD_printf("NextOrent:NORTH\n");
				break;
			case EAST:
				LCD_printf("NextOrent:EAST\n");
				break;
			case SOUTH:
				LCD_printf("NextOrent:SOUTH\n");
				break;
			case WEST:
				LCD_printf("NextOrent:WEST\n");
				break;
			default:
				break;
		}
		
		switch(currentMove){
			case MOVE_LEFT:
				LCD_printf("CurMOVE:LEFT\n");
				break;
			case MOVE_RIGHT:
				LCD_printf("CurMOVE:RIGHT\n");
				break;
			case MOVE_FORWARD:
				LCD_printf("CurMOVE:FORWARD\n");
				break;
			default:
				break;
		}
		
		// Plan using metric map and next orientation
		planMetric();
		
		// Act on the move
		moveMap();
		
		// Shift the map
		currentCellWorld = shiftMap(currentCellWorld, currentMove, currentOrientation);
		// TMRSRVC_delay(2000);//wait 1 seconds
	}
	SPKR_beep(0);
}

/*******************************************************************
* Function:			void localize(void)
* Input Variables:	none
* Output Return:	none
* Overview:			localize the robot
********************************************************************/
void localize (void)
{	
}

/*******************************************************************
* Function:			void map (void)
* Input Variables:	none
* Output Return:	none
* Overview:			Makes the robot map the world
********************************************************************/
void map (void)
{
	// Initialize State
	isMapping = 1;
	
	// Mapping Loop
	while(isMapping)
	{	
		//Sense
		checkIR();	
		checkWorld();
		
		//Record
		setGateways();
				
		//Plan using the Map
		planMap();
		
		//Act on the Map
		moveMap();
		
		//Shift the Map
		currentCellWorld = shiftMap(currentCellWorld, currentMove, currentOrientation);
		
		//Break?
		isMapping = !((currentCellWorldStart == currentCellWorld)&&(currentOrientationStart == currentOrientation));
		if(!isMapping){			
			break;
		}
		
		//Print Map
		LCD_clear();
		LCD_printf("      Move"BYTETOBINARYPATTERN"\n      Cell"BYTETOBINARYPATTERN"\n      Ornt"BYTETOBINARYPATTERN"\n\n",BYTETOBINARY(currentMove),BYTETOBINARY(currentCellWorld),BYTETOBINARY(currentOrientation));
		printMap(currentOrientation,currentCellWorld,RESET);
		TMRSRVC_delay(500);//wait 3 seconds
	}
	
}

/*******************************************************************
* Function:			unsigned char fourNeighborSearch(unsigned char)
* Input Variables:	the robot's current position
* Output Return:	unsigned char
* Overview:			searches the four-neighboring cells in the 
*					wavefront map
********************************************************************/
unsigned char fourNeighborSearch(unsigned char curCell)
{	
	
	// Get the cell current row and column
	unsigned char curRow = (curCell>>2);
	unsigned char curCol = (curCell&0b0011);
	
	// If our current cell is 0
	// then we have reached our goal
	if( ROBOT_METRIC_WORLD[curRow][curCol] == 0){
		return SUCCESS;
	}
	
	// Make some initial variables
	// Reset minInd value and minNeighbor
	unsigned char minVal = 100;
	unsigned char curVal = 0;
	
	// Perform a 4-neighbor search and store the lowest value
	
	// LCD_clear();
	//Check the north cell
	curRow--;
	if((curRow)<WORLD_ROW_SIZE){
		curVal = ROBOT_METRIC_WORLD[(curRow)][curCol];
		// LCD_printf("NORTH curVal: %i\n",curVal);
		// TMRSRVC_delay(1000);
		if(curVal<minVal){
			minVal = curVal;
			nextOrientation = NORTH;
		}
	}
	
	curRow++;
	curRow++;
	// Check the south cell
	if((curRow)<WORLD_ROW_SIZE){
		curVal = ROBOT_METRIC_WORLD[(curRow)][curCol];
		// LCD_printf("SOUTH curVal: %i\n",curVal);
		// TMRSRVC_delay(1000);
		if(curVal<minVal){
			minVal = curVal;
			nextOrientation = SOUTH;
		}
	}
	
	curRow--;
	curCol++;
	// Check the east cell
	if((curCol)<WORLD_COLUMN_SIZE){
		curVal = ROBOT_METRIC_WORLD[curRow][(curCol)];
		// LCD_printf("EAST curVal: %i\n",curVal);
		// TMRSRVC_delay(1000);
		if(curVal<minVal){
			minVal = curVal;
			nextOrientation = EAST;
		}
	}
	
	curCol--;
	curCol--;
	// Check the west cell
	if((curCol)<WORLD_COLUMN_SIZE){
		curVal = ROBOT_METRIC_WORLD[curRow][(curCol)];
		// LCD_printf("WEST curVal: %i\n",curVal);
		// TMRSRVC_delay(1000);
		if(curVal<minVal){
			minVal = curVal;
			nextOrientation = WEST;
		}
	}
	
	return FAIL;
}

/*******************************************************************
* Function:			void planMetric(void)
* Input Variables:	none
* Output Return:	none
* Overview:			Moves around the map using metric navigation
********************************************************************/
void planMetric (void)
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
* Input Variables:	unsigned char
* Output Return:	void
* Overview:			Makes the wavefront metric map to goal location
*					from current location 
********************************************************************/
void wavefrontMake(void)
{
	// User-defined goal location
	// unsigned char goalLocation = currentGoalWorld;
	// Extract x and y goal location
	int rowGoal = (currentGoalWorld>>2);
	int colGoal = (currentGoalWorld&0b0011);
	// Declare some variables and initialize a distance 
	unsigned int rowDelta, colDelta;
	int row, col;
	int distance = 0;
	
	// For every cell in the world
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
				rowDelta = abs(row - rowGoal);
				colDelta = abs(col - colGoal);
				// compute the distance without using sqrt
				distance = ((rowDelta*rowDelta)+(colDelta*colDelta));
				// overwrite the cells in the metric map to the actual distance values
				ROBOT_METRIC_WORLD[row][col] = distance;
			}
		}
	}
}

/*******************************************************************
* Function:			char matchBranch(unsigned char *ptROBOT_WORLD, unsigned char row, unsigned char col)
* Input Variables:	char
* Output Return:	unsigned char *, unsigned char, unsigned char
* Overview:		    Check to see if the branch is valid
*					given the map and starting seed
********************************************************************/
char matchBranch( unsigned char row, unsigned char col)
{	
	// Local variables for comparing branch and gateway 
	unsigned char branch, curMove, curOrnt, gateway, i;
	unsigned char curCell = 0;
	
	// Local variables for nested for loops 
	unsigned char curRow = row;
	unsigned char curCol = col;
	
	
	// Then check for a matching brache
	for(i = 0; i < currentBranch; i++){
	
		// Check to see if we are still inside the map
		// If we went outside, then return failure
		if((curRow>WORLD_ROW_SIZE)||(curCol>WORLD_COLUMN_SIZE)){
			return FAIL;
		}
	
		// Get current branch
		branch = localizeGateways[0][i];
		
		// Get the current move 
		curMove = localizeGateways[1][i];
		
		// Get the current orientation 
		curOrnt = localizeGateways[2][i];
		
		// Rotate the branch to reflect the map
		branch = rotateCell (branch, curOrnt, TO_MAP_ROTATE);
		
		// Get gateway from map
		// This uses pointers to get the element
		// gateway = *(ptROBOT_WORLD+curRow*WORLD_ROW_SIZE+curCol);
		gateway = ROBOT_WORLD[curRow][curCol];
		
		// If the matching pattern is broken
		// stop matching and return failure
		if(branch != gateway){
			return FAIL;
		}
		
		// Set the new cell of the next branch
		curCell = (curRow << 2) + curCol;

		// If this is the last branch
		// dont move the cell
		// so we are left with our locilized position 
		// if((i == (currentBranch-2))&&){
		// Prep for the gateway by moving with the next branch
		curCell = shiftMap(curCell, curMove, curOrnt);
		// }
				
		// Get the currrent cell of the branch
		curRow = curCell >> 2;
		curCol = curCell & 0b0011;
	}
	// If we make it through all the branches
	// then return success
	currentCellWorld = curCell;
	return SUCCESS;
}

/*******************************************************************
* Function:			char localizeGateway(void)
* Input Variables:	char
* Output Return:	void
* Overview:		    use the localizeGateways tree to localize robot
********************************************************************/
char localizeGateway( void )
{	
	unsigned char i = 0;
	
	// Get the root seed from the tree
	unsigned char localizeSeed = localizeGateways[0][0];
	
	// Local variables for nested for loops 
	unsigned char row, col;
	
	// Stores the number of matching seeds
	matchSeeds = 0;
	
	// // Stores the last matching seed index
	// unsigned char matchRow, matchCol;
	
	// Find seeds and check for matching braches
	// For ever row in the map
	for(row = 0; row < WORLD_ROW_SIZE; row++){
	
		// And For ever column in the map
		for(col = 0; col < WORLD_COLUMN_SIZE; col++){
		
			// Check to see if we have a matching seed
			// if(0b1101 == ROBOT_WORLD[row][col]){
			if(localizeSeed == ROBOT_WORLD[row][col]){
			
				//Check to see if we have a matching branch
				if(matchBranch(row,col)){
					// matchRow = row;
					// matchCol = col;
					matchSeeds++;
				}
			}			
		}
	}
	
	
	// SPKR_play_beep(500,1000,50);
	// for(i = 0; i < matchSeeds; i++){
		// SPKR_play_beep(1000,1000,50);
	// }
	
	// If we have only one remaining seed
	// Then we are localized
	if(matchSeeds == 1){
		return 0;
	}
	
	// return failure
	return 1;
}

/*******************************************************************
* Function:			void planGateway(void)
* Input Variables:	void
* Output Return:	void
* Overview:		    build the localizeGateways tree with current branch
********************************************************************/
void planGateway( void )
{	
	// If we are still lost
	// when we reach our max branch
	// Pop of the seed and use the second oldest
	// as the new seed
	unsigned char i;
	if(currentBranch >= BRANCH_MAX){
		for(i = 0; i < BRANCH_MAX; i++){
			localizeGateways[0][i] = localizeGateways[0][1+i];
			localizeGateways[1][i] = localizeGateways[1][1+i];
			localizeGateways[2][i] = localizeGateways[2][1+i];
		}
		currentBranch = BRANCH_MAX-1;
	}
	
	// Decide what the current move should be
	planMap();
	
	// Save the current gateway, move, and orientation
	localizeGateways[0][currentBranch] = currentGateway;
	localizeGateways[1][currentBranch] = currentMove;
	localizeGateways[2][currentBranch] = currentOrientation;
	
	// Update the currentOrientation using currentMove
	switch(currentMove){
		case MOVE_LEFT:
			// If we move left
			// shift our oriention CCW
			currentOrientation--;
			currentOrientation = currentOrientation&0b11;
			break;
		case MOVE_FORWARD:
			break;
		case MOVE_RIGHT:
			// If we move right
			// shift our oriention CW
			currentOrientation++;
			currentOrientation = currentOrientation&0b11;
			break;
		default:
			LCD_printf("Whatz2?!");
			break;
	}
	
	// If we have none or more than one seed
	// Increment current branch 
	currentBranch++;
}

/*******************************************************************
* Function:			void planMap(void)
* Input Variables:	void
* Output Return:	void
* Overview:		    maps the world as it moves through it
********************************************************************/
void planMap( void )
{	
	//Plan
	if(!(currentGateway&0b0001)){	
		// If we can make a left turn,
		// then spin left
		currentMove = MOVE_LEFT;
	}
	else if(!(currentGateway&0b1000)){
		// If we can't make a left turn,
		// but we can go forward,
		// go forward
		currentMove = MOVE_FORWARD;
	}
	else {
		// If we can't turn left or go forward
		// then spin right
		currentMove = MOVE_RIGHT;
	}
	
	switch(oldMove){
		case MOVE_LEFT:
			//If we turned left befor
			//then we still have yet to go forward
			currentMove = MOVE_FORWARD;
			break;
		case MOVE_FORWARD:
			break;
		case MOVE_RIGHT:		
			break;
		default:
			LCD_printf("Whatz1?!");
			break;
	}
	
	oldMove = currentMove;
}

/*******************************************************************
* Function:			void moveMap(void)
* Input Variables:	void
* Output Return:	void
* Overview:		    moves the robot through the map
********************************************************************/
void moveMap( void )
{	
	char isDone = 0;
	pidController(0,RESET);
	switch(currentMove){
		case MOVE_LEFT:
			move_arc_stwt(POINT_TURN, LEFT_TURN, 30, 30, 0);
			
			STEPPER_stop(STEPPER_BOTH, STEPPER_BRK_ON);
			TMRSRVC_delay(BRAKE_DELAY);
			STEPPER_stop(STEPPER_BOTH, STEPPER_BRK_OFF);
			break;
		case MOVE_FORWARD:
		
			setOdometry(WALL_STEP);
			while(!isDone){
				checkIR();
				isDone = moveWall();
			}
			STEPPER_stop(STEPPER_BOTH, STEPPER_BRK_ON);
			TMRSRVC_delay(BRAKE_DELAY);
			STEPPER_stop(STEPPER_BOTH, STEPPER_BRK_OFF);
			
			// move_arc_stwt(NO_TURN, 45, 10, 10, 0);
			break;
		case MOVE_RIGHT:
			move_arc_stwt(POINT_TURN, RIGHT_TURN, 30, 30, 0);
			
			STEPPER_stop(STEPPER_BOTH, STEPPER_BRK_ON);
			TMRSRVC_delay(BRAKE_DELAY);
			STEPPER_stop(STEPPER_BOTH, STEPPER_BRK_OFF);
			break;
		default:
			LCD_printf("Whatz2?!");
			break;
	}
}

/*******************************************************************
* Function:			unsigned char shiftMap(unsigned char currentCell, unsigned char curMove, unsigned char curOrient)
* Input Variables:	unsigned char, unsigned char, unsigned char
* Output Return:	unsigned char
* Overview:		    shifts the map after robot moves
********************************************************************/
unsigned char shiftMap( unsigned char currentCell, unsigned char curMove, unsigned char curOrient)
{		
	// Get the currrent location of the robot
	unsigned char curRow = currentCell >> 2;
	unsigned char curCol = currentCell & 0b0011;			
		
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
	curCell = rotateCell(curCell,curOrient,TO_MAP_ROTATE);
	
	// Set the current cell
	ROBOT_WORLD[curRow][curCol] = curCell;
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
		TMRSRVC_delay(1000);//wait 1 second
	}
}
