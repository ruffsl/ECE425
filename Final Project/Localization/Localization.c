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
#include "capi324v221.h"
#include "stdio.h"
#include "CEEN_Interfaces.h"
#include "Custom_Support.h"

/** Define Constants Here ******************************************/

// Wall Following Threshold
#define IR_WALL_F_THRESH 0
#define IR_WALL_R_THRESH 15
#define IR_WALL_L_THRESH 15
#define IR_WALL_B_THRESH 15
#define MAX_SPEED 200
#define WALL_STEP 20

// Gateway Thresholds
#define FT_GATEWAY 13
#define BK_GATEWAY 35
#define LT_GATEWAY 30
#define RT_GATEWAY 30

// localizeGateways Size
#define BRANCH_TYPES 3
#define BRANCH_MAX 5



/** Local Function Prototypes **************************************/
// Locomotion Primitives
char moveWall(void);
char moveWallOld(void);

// User Inputs
void worldInput(void);
void orientationInput(void);
void movesInput(void);

// Sensory Primitives
void checkWorld(void);

// Pathplanning Tools
char moveWorld(void);
void getGateways(void);
void setGateways(void);

// Mapping Tools
void planMap(void);
void moveMap(void);
unsigned char shiftMap(unsigned char, unsigned char, unsigned char);

// Localization Tools
void planGateway(void);
char localizeGateway(void);
char matchBranch(unsigned char *, unsigned char, unsigned char);

/** Global Variables ***********************************************/

// moveBehavior Global Flag Variables
char currentMove;
char oldMove;
char isMapping;
char isLost = 1;
unsigned char matchSeeds;

unsigned char localizeGateways[BRANCH_TYPES][BRANCH_MAX] = {{0,0,0,0,0},
															{0,0,0,0,0},
															{0,0,0,0,0}};
unsigned char currentBranch = 0;

unsigned char ROBOT_WORLD[WORLD_ROW_SIZE][WORLD_COLUMN_SIZE] = 	{
																{0b1101, 0b1111, 0b1111, 0b1101}, 
																{0b0101, 0b1111, 0b1111, 0b0101}, 
																{0b0001, 0b1010, 0b1010, 0b0100}, 
																{0b0111, 0b1111, 0b1111, 0b0111}
																};

/*******************************************************************
* Function:        void CBOT_main( void )
********************************************************************/

void CBOT_main( void )
{
	// initialize the robot
	initializeRobot();
	
	// Loop variables for print debug
	unsigned char i, branch, move, orent;
	
	// Display the map
	LCD_clear();
	LCD_printf("      New Map\n\n\n\n");
	printMap();
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();
	
	// Localization Loop 
	while(isLost)
	{	
		//Sense Gateway
		checkIR();	
		checkWorld();
		
		//Plan using the Gateway
		planGateway();
		
		//Localize from Gateways?
		isLost = localizeGateway();
		if(!isLost){
			break;
		}
				
		//Print Tree		
		LCD_clear();
		LCD_printf("Branch");
		for(i = 0; i<BRANCH_MAX; i++){
			branch = localizeGateways[0][i];
			LCD_printf("%3d", branch);
		}
		LCD_printf("Move  ");
		for(i = 0; i<BRANCH_MAX; i++){
			move = localizeGateways[1][i];
			LCD_printf("%3d", move);
		}
		LCD_printf("Ornt  ");
		for(i = 0; i<BRANCH_MAX; i++){
			orent = localizeGateways[2][i];
			LCD_printf("%3d", orent);
		}
		LCD_printf("isLost %1d ",isLost);
		LCD_printf("seeds: %1d", matchSeeds);
		TMRSRVC_delay(4000);//wait 3 seconds
		
		//Act on the Gateway
		moveMap();
	}
	
	

	/**
	while(!odometryFlag){
		moveWall();
		checkOdometry(0);
		// LCD_clear();
		// LCD_printf("%5.5f\n%5.5f\n",odometryStepL,odometryStepL);
		// TMRSRVC_delay(1000);//wait 3 seconds
	}
	**/
	
	LCD_clear();
	LCD_printf("LOLZ\nI'm found!");
	TMRSRVC_delay(3000);//wait 3 seconds
	
	LCD_clear();
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_printf("      New Map\n\n\n\n");
	printMap();
	TMRSRVC_delay(10000);//wait 10 seconds
	LCD_clear();
	
	
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
	
	isMapping = 1;
	
	
	LCD_printf("      Your Map\n\n\n\n");	
	printMap();
	TMRSRVC_delay(1000);//wait 1 seconds
	LCD_clear();	
	
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
		currentCellWorld = shiftMap(currentCellWorld, currentOrientation);
		
		//Break?
		isMapping = !((currentCellWorldStart == currentCellWorld)&&(currentOrientationStart == currentOrientation));
		if(!isMapping){			
			LCD_clear();
			LCD_printf("LOLZ\nI'm done!");
			TMRSRVC_delay(3000);//wait 3 seconds
			break;
		}
		
		//Print Map
		LCD_clear();
		LCD_printf("      Move"BYTETOBINARYPATTERN"\n      Cell"BYTETOBINARYPATTERN"\n      Ornt"BYTETOBINARYPATTERN"\n\n",BYTETOBINARY(currentMove),BYTETOBINARY(currentCellWorld),BYTETOBINARY(currentOrientation));
		printMap();
		TMRSRVC_delay(500);//wait 3 seconds
	}
	**/
	
	/**
	// Print the map
	LCD_clear();	
	printMap();
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
* Function:			char matchBranch(unsigned char *ptROBOT_WORLD, unsigned char row, unsigned char col)
* Input Variables:	char
* Output Return:	unsigned char *, unsigned char, unsigned char
* Overview:		    Check to see if the branch is valid
*					given the map and starting seed
********************************************************************/
char matchBranch( unsigned char *ptROBOT_WORLD, unsigned char row, unsigned char col)
{	
	// Local variables for comparing branch and gateway 
	unsigned char branch, curMove, curOrnt, gateway, i;
	
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
		currentCellWorld = (curRow << 2) + curCol;
		
		// Prep for the gateway by moving with the next branch		
		currentCellWorld = shiftMap(currentCellWorld, curMove, curOrnt);
		
		// Get the currrent cell of the branch
		curRow = currentCellWorld >> 2;
		curCol = currentCellWorld & 0b0011;
	}
	// If we make it through all the branches
	// then return success
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
	// Get the root seed from the tree
	unsigned char localizeSeed = localizeGateways[0][0];
	// Local variables for nested for loops 
	unsigned char row, col;
	// Stores the number of matching seeds
	matchSeeds = 0;
	// Stores the last matching seed index
	unsigned char matchRow, matchCol;
	
	// Find seeds and check for matching braches
	// For ever row in the map
	for(row = 0; row <= WORLD_ROW_SIZE; row++){
	
		// And For ever column in the map
		for(col = 0; col <= WORLD_COLUMN_SIZE; col++){
		
			// Check to see if we have a matching seed
			// if(0b1101 == ROBOT_WORLD[row][col]){
			if(localizeSeed == ROBOT_WORLD[row][col]){
			
				//Check to see if we have a matching branch
				if(matchBranch(*ROBOT_WORLD,row,col)){
					matchRow = row;
					matchCol = col;
					matchSeeds++;
				}
			}			
		}
	}
	
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
	if(currentBranch>=BRANCH_MAX){
		for(i = 0; i<=(BRANCH_MAX-1); i++){
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
			//If we move left
			// shift our oriention CCW
			currentOrientation--;
			currentOrientation = currentOrientation&0b11;
			break;
		case MOVE_RIGHT:
			//If we move right
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
		TMRSRVC_delay(500);	//wait 0.5 seconds
	}
	
	currentCellWorldStart = currentCellWorld;
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
	
	currentOrientationStart = currentOrientation;
	
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
		move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE*(2.5/5.0), 10, 10, 0);		
	}
	
	if(((oldMove == MOVE_LEFT)|(oldMove == MOVE_RIGHT))&(currentMove == MOVE_FORWARD))
	{
		move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE*(2.2/5.0), 10, 10, 0);		
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
			moveWallOld();
			// move_arc_stwt(NO_TURN, WORLD_RESOLUTION_SIZE, 10, 10, 0);
			break;
		case MOVE_RIGHT:
			LCD_printf("Right\nCurMove:%i\nGateway:%i\nNextGateway:%i\n",currentMoveWorld,currentGateway,nextGateway);
			// TMRSRVC_delay(1000);//wait 1 seconds
			move_arc_stwt(POINT_TURN, RIGHT_TURN, 10, 10, 0);
			break;
		default:
			LCD_printf("Whatz4?!");
			STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_OFF);
			while(1);
			break;
	}
	// TMRSRVC_delay(1000);//wait 1 seconds
	oldMove = currentMove;
	return 1;
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
	
		// Update odometry
		curr_step = STEPPER_get_nSteps();
		
		if(curr_step.left != 0){
			odometryStepL += WALL_STEP - curr_step.left;
		}
		if(curr_step.right != 0){
			odometryStepR += WALL_STEP - curr_step.right;
		}
		
		STEPPER_set_steps(STEPPER_BOTH,0);
	
		// Move with wall
		STEPPER_move_stnb( STEPPER_BOTH, 
		STEPPER_REV, WALL_STEP, 200, 450, STEPPER_BRK_OFF, // Left
		STEPPER_REV, WALL_STEP, 200, 450, STEPPER_BRK_OFF ); // Right
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
	
	if(curr_step.left != 0){
		odometryStepL += WALL_STEP - (curr_step.left);
	}
	if(curr_step.right != 0){
		odometryStepR += WALL_STEP - (curr_step.right);
	}
	
	STEPPER_set_steps(STEPPER_BOTH,0);
	
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
* Function:			char moveWall(void)
* Input Variables:	void
* Output Return:	char
* Overview:			This function searches for walls and adjust the 
*					robots differential steering to attempts to
*					follow them
********************************************************************/
char moveWallOld( void )
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
