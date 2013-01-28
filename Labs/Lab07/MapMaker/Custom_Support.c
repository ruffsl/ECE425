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
	SPKR_open(SPKR_BEEP_MODE);//open the speaker in beep mode
	
	LED_open();
	I2C_open();
	ADC_open();//open the ADC module
 	ADC_set_VREF( ADC_VREF_AVCC );// Set the Voltage Reference first so VREF=5V.

	// Initialize IR Values and Reset Prefilter
	checkIR();
	prefilter(1);
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