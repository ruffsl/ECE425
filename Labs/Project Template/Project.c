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

/** Local Function Prototypes **************************************/
void sampleFunction(void);


/** Global Variables ***********************************************/
int sampleVariable = 0;
	
/*******************************************************************
* Function:        void CBOT_main( void )
********************************************************************/

void CBOT_main( void )
{
	// Initialize variables
	int btnValue=0;//value of button pushed
	int i=0;//loop counter

	BOOL	ltContact;//left contact sensor
	BOOL	rtContact;//right contact sensor

	float	ltIR = 0;//left IR sensor
	float	rtIR = 0;//right IR sensor
	float	ftIR = 0;//front IR sensor
	float 	bkIR = 0;//back IR sensor

	float	ltLght;//left light reading
	float	rtLght;//right light reading

	unsigned char data;
	unsigned char pixel1 = 0;
	unsigned char pixel2 = 0;
	unsigned char pixel3 = 0;
	unsigned char pixel4 = 0;
	unsigned char pixel5 = 0;
	unsigned char pixel6 = 0;
	unsigned char pixel7 = 0;
	unsigned char pixel8 = 0;

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
		
    }
}// end the CBOT_main()

/*******************************************************************
* Additional Helper Functions
********************************************************************/

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
