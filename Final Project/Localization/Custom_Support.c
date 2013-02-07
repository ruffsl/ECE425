 /*******************************************************************
* FileName:        Custom_Support.c
* Processor:       ATmega324P
* Compiler:        GCC
*
* Code Description:
* This contains all the subfunctions needed to run Lab code
*
********************************************************************/

/** Header Files ***************************************************/     
#include "capi324v221.h"
#include "stdio.h"
#include "CEEN_Interfaces.h"
#include "Custom_Support.h"

/*******************************************************************
* Function:			void initializeRobot(void)
* Input Variables:	none
* Output Return:	none
* Overview:			This initialize the robot by using other startups
********************************************************************/
void initializeRobot(void)
{
	ATopstat = ATTINY_open();//open the tiny microcontroller
	LEopstat = LED_open(); //open the LED module
	LCopstat = LCD_open(); //open the LCD module
	STEPPER_open(); // Open STEPPER module for use
	SPKR_open(SPKR_TONE_MODE);//open the speaker in tone mode
	
	LED_open();
	I2C_open();
	ADC_open();//open the ADC module
 	ADC_set_VREF( ADC_VREF_AVCC );// Set the Voltage Reference first so VREF=5V.
	
	// Initialize IR Values and Reset Prefilter
	checkIR();
	prefilter(1);
	
	// Mistake? odometryTrigger = WORLD_RESOLUTION_SIZE*D_STEP which is about 6
	odometryTrigger = WORLD_RESOLUTION_SIZE/2.75;
	
	// pixel array for the LCD screen
	for(int i = 0; i < 4; i++) {
		for(int j = 0; j < 32; j++) {
			pix_arr[i][j] = 0x00;
		}
	}
}

/*******************************************************************
* Function:			void setOdometry( float)
* Input Variables:	void
* Output Return:	float odometry 
* Overview:		    Sets the odometry to move
********************************************************************/
void setOdometry( float odometry )
{	
	// Set the gloable odometryTrigger
	odometryTrigger = odometry;
	
	// Set the stepers
	STEPPER_move_stnb( STEPPER_BOTH, 
	STEPPER_REV, odometryTrigger, MAX_SPEED, MAX_ACL, STEPPER_BRK_OFF, // Left
	STEPPER_REV, odometryTrigger, MAX_SPEED, MAX_ACL, STEPPER_BRK_OFF ); // Right
}

/*******************************************************************
* Function:			char checkOdometry( char)
* Input Variables:	char
* Output Return:	char reset resets the odometry
* Overview:		    Checks the current odometry to the trigger and
*					sets the flag whe appropriate
********************************************************************/
char checkOdometry( char reset )
{	
	// Check for a reset
	if (reset){
		odometryTrigger = 0;
		STEPPER_set_steps(STEPPER_BOTH,odometryTrigger);
		return SUCCESS;
	}
	
	// Get the current number of steps
	curr_step = STEPPER_get_nSteps();
	
	// Check if the sum is zero
	if((curr_step.left + curr_step.right) == 0)
	{
		// if it is zero
		// then return success
		return SUCCESS;	
	}

	// But if not zero
	//then return fial
	return FAIL;
}

/*******************************************************************
* Function:			void printMap(char reset)
* Input Variables:	char
* Output Return:	void
* Overview:		    Print the map
********************************************************************/
void printMap(char reset)
{
	unsigned char r;
	unsigned char c;
	unsigned char cell;
	
	unsigned char curRow = currentCellWorld >> 2;
	unsigned char curCol = currentCellWorld & 0b0011;
	
	BOOL isrobot;
	for(r = 0; r < WORLD_ROW_SIZE; r++){
		for(c = 0; c < WORLD_COLUMN_SIZE; c++){
			cell = ROBOT_WORLD[r][c];
			isrobot = (r == curRow)&&(c == curCol);
			printCell(cell, r, c, isrobot, currentOrientation, reset);
		}	
	}
}

/*******************************************************************
* Function:			void printCell(unsigned char, unsigned char, unsigned char, BOOL isrobot, unsigned char orent)
* Input Variables:	void
* Output Return:	unsigned char, unsigned char, unsigned char, BOOL, unsigned char
* Overview:		    Prints the cell
********************************************************************/
void printCell(unsigned char cell, unsigned char r, unsigned char c, BOOL isrobot, unsigned char orent, BOOL reset){

	r = r*LCD_CELL_OFFSET;
	c = c*LCD_CELL_OFFSET;
	
	LCD_set_pixel(LCD_OFFSET - r,   c,   1);
	LCD_set_pixel(LCD_OFFSET - (r+7), c,   1);
	LCD_set_pixel(LCD_OFFSET - r,   c+7, 1);
	LCD_set_pixel(LCD_OFFSET - (r+7), c+7, 1);
	
	if(cell&0b1000){
		LCD_set_pixel(LCD_OFFSET - r, c+1, 1);
		LCD_set_pixel(LCD_OFFSET - r, c+2, 1);
		LCD_set_pixel(LCD_OFFSET - r, c+3, 1);
		LCD_set_pixel(LCD_OFFSET - r, c+4, 1);
		LCD_set_pixel(LCD_OFFSET - r, c+5, 1);		
		LCD_set_pixel(LCD_OFFSET - r, c+6, 1);		
	}
	if(cell&0b0100){
		LCD_set_pixel(LCD_OFFSET - (r+1), c+7, 1);
		LCD_set_pixel(LCD_OFFSET - (r+2), c+7, 1);
		LCD_set_pixel(LCD_OFFSET - (r+3), c+7, 1);
		LCD_set_pixel(LCD_OFFSET - (r+4), c+7, 1);
		LCD_set_pixel(LCD_OFFSET - (r+5), c+7, 1);		
		LCD_set_pixel(LCD_OFFSET - (r+6), c+7, 1);			
	}
	if(cell&0b0010){
		LCD_set_pixel(LCD_OFFSET - (r+7), c+1, 1);
		LCD_set_pixel(LCD_OFFSET - (r+7), c+2, 1);
		LCD_set_pixel(LCD_OFFSET - (r+7), c+3, 1);
		LCD_set_pixel(LCD_OFFSET - (r+7), c+4, 1);
		LCD_set_pixel(LCD_OFFSET - (r+7), c+5, 1);		
		LCD_set_pixel(LCD_OFFSET - (r+7), c+6, 1);		
	}
	if(cell&0b0001){
		LCD_set_pixel(LCD_OFFSET - (r+1), c, 1);
		LCD_set_pixel(LCD_OFFSET - (r+2), c, 1);
		LCD_set_pixel(LCD_OFFSET - (r+3), c, 1);
		LCD_set_pixel(LCD_OFFSET - (r+4), c, 1);
		LCD_set_pixel(LCD_OFFSET - (r+5), c, 1);		
		LCD_set_pixel(LCD_OFFSET - (r+6), c, 1);		
	}	
	if(isrobot){
		LCD_set_pixel(LCD_OFFSET - (r+3), c+3, isrobot);
		LCD_set_pixel(LCD_OFFSET - (r+4), c+3, isrobot);
		LCD_set_pixel(LCD_OFFSET - (r+3), c+4, isrobot);
		LCD_set_pixel(LCD_OFFSET - (r+4), c+4, isrobot);
		
		switch(orent){
			case NORTH:
				LCD_set_pixel(LCD_OFFSET - (r+2), c+3, isrobot);
				break;
			case EAST:
				LCD_set_pixel(LCD_OFFSET - (r+3), c+5, isrobot);
				break;
			case SOUTH:
				LCD_set_pixel(LCD_OFFSET - (r+5), c+4, isrobot);			
				break;
			case WEST:
				LCD_set_pixel(LCD_OFFSET - (r+4), c+2, isrobot);			
				break;
			default:
				break;
		}
	}
	else if(reset){
		LCD_set_pixel(LCD_OFFSET - (r+3), c+3, isrobot);
		LCD_set_pixel(LCD_OFFSET - (r+4), c+3, isrobot);
		LCD_set_pixel(LCD_OFFSET - (r+3), c+4, isrobot);
		LCD_set_pixel(LCD_OFFSET - (r+4), c+4, isrobot);
		
		LCD_set_pixel(LCD_OFFSET - (r+2), c+3, isrobot);
		LCD_set_pixel(LCD_OFFSET - (r+3), c+5, isrobot);
		LCD_set_pixel(LCD_OFFSET - (r+5), c+4, isrobot);
		LCD_set_pixel(LCD_OFFSET - (r+4), c+2, isrobot);
		}
}

/*******************************************************************
* Function:			void LCD_set_pixel(unsigned char , unsigned char , BOOL)
* Input Variables:	void
* Output Return:	unsigned char row, unsigned char col, BOOL val
* Overview:			a function that sets a single pixel on/off for the LCD
 * 					@param row an unsigned char that specifies the lcd row
 * 					@param col an unsigned char that specifies the lcd column
 * 					@param val a boolean that specifies the pixel value to be set
 *					LCD text print size (4 rows, 22 columns)
 *					LCD pixel print size (32 rows, 128 columns)
********************************************************************/
void LCD_set_pixel(unsigned char row, unsigned char col, BOOL val) {
	row &= 0x1F; // Limit row 0-31
	col &= 0x7F; // Limit column 0-127

	// Divide row by 8 to restrict to 0-3
	unsigned char page = row >> 3;

	// Set page and column to write next
	LCD_set_PGC_addr( page, col );
	LCD_set_next_PGC( page, col );

	// Determine new pixel value by shifting 1 into place determined
	// by remainder of divding by 8. And/or determined if pixel is on
	// or off
	if(val) {
		pix_arr[page][col] |= (1 << (row & 7));
	} else {
		pix_arr[page][col] &= ~(1 << (row & 7));
	}

	// Write the pixel data out to the lcd
	LCD_write_data( pix_arr[page][col] );
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
* Function:			void rotateCell(unsigned char,unsigned char, char)
* Input Variables:	unsigned char, unsigned char, char
* Output Return:	unsigned char
* Overview:		    Rotates the current cell to match the robot's
					orientation, so the robot can recognize gateways
********************************************************************/
unsigned char rotateCell (unsigned char worldCell, unsigned char orientation, char L)
{
	//	a temp for orientation
	unsigned char orient = orientation;
	
	//	a temp for world cell
	unsigned char cell = worldCell;
	
	// Use the given orientation as a counter 
	// to know the number of rotations to correct the map
	// for the robots current orientation
	if(L){
		while (orient!=0){
			// use the shift array to left rotate
			cell = shiftedL[cell];
			orient--;
		}
	}
	else{
		while (orient!=0){
			// use the shift array to right rotate
			cell = shiftedR[cell];
			orient--;
		}
	}
	return cell;
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