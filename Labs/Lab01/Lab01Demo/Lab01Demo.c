/*******************************************************************
* FileName:        Lab01Demo.c
* Processor:       ATmega324P
* Compiler:        
*
* Code Description:
* This code makes the robot perform 3 different movements based on the pushbutton that is pressed.
* The robot is able to move in a square, circle, and figure8. 
* When in iddle, the robot displays "Hello Dolly."
*                                                                     
*
* Creation and Revisions:
*
*      AUTHOR               DATE			COMMENTS
*      Ander Solorzano		11/30/2012
*      &
*      Ruffin White   
********************************************************************/

/** Header Files ***************************************************/     
#include "capi324v221.h"
#include "stdio.h"

/** Define Constants Here ******************************************/
#define C_BOTH 0.9369357426
#define C_LEFT 1.142900308
#define C_RIGHT 0.8570996920

// IR Constants
#define IRRIGHT_CHAN ADC_CHAN3
#define IRLEFT_CHAN ADC_CHAN4
#define IRFRONT_CHAN ADC_CHAN7
#define IRBACK_CHAN ADC_CHAN5 

// Desc: The TPA81's I2C address (See datasheet).
#define __TPA81_ADDR	0x68

// Desc: The following macro-constants define the accessible registers
//	 of the TPA81.
#define __TPA81_REVISION 0
#define __TPA81_AMBIENT	 1
#define __TPA81_PIX1	 2
#define __TPA81_PIX2	 3
#define __TPA81_PIX3	 4
#define __TPA81_PIX4	 5
#define __TPA81_PIX5	 6
#define __TPA81_PIX6	 7
#define __TPA81_PIX7	 8
#define __TPA81_PIX8	 9
	
//Light channels mapped to ADC input (see *note above)
#define	RIGHT_LIGHT_CHAN ADC_CHAN5
#define	LEFT_LIGHT_CHAN ADC_CHAN6

/** I2C Prototypes **************************************/
I2C_STATUS get_revision( unsigned char *revision );
I2C_STATUS get_ambient_temp( unsigned char *ambient_temp );
I2C_STATUS read_pixel_1( unsigned char *pixel_1);
I2C_STATUS read_pixel_2( unsigned char *pixel_2);
I2C_STATUS read_pixel_3( unsigned char *pixel_3);
I2C_STATUS read_pixel_4( unsigned char *pixel_4);
I2C_STATUS read_pixel_5( unsigned char *pixel_5);
I2C_STATUS read_pixel_6( unsigned char *pixel_6);
I2C_STATUS read_pixel_7( unsigned char *pixel_7);
I2C_STATUS read_pixel_8( unsigned char *pixel_8);

/** Local Function Prototypes **************************************/
void sampleFunction(void);

void moveSquare(void);
void moveCircle(void);
void moveFigure8(void);

//IR functions
float getLeftIR(void);
float getRightIR(void);
float getFrontIR(void);
float getBackIR(void);

//photocells functions
float getLeftLight(void);
float getRightLight(void);

//Locomotion and Buttons
int WaitButton();//button function
void MoveBot();//move the robot in a square

/** Status Variable Declarations ***********************************/
SUBSYS_OPENSTAT ATopstat;//ATTINY open status
SUBSYS_OPENSTAT LCopstat;//LCD open status
SUBSYS_OPENSTAT LEopstat;//LED open status


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
	
	LCD_clear();

	// Infinite loop
	while (1)
    {		

		btnValue = WaitButton();

		if (btnValue == 1)
		{
			LCD_printf("Make a SQUARE");
			TMRSRVC_delay(1000);//wait 1 s
			moveSquare();
			btnValue = 0;
			LCD_clear();
		}
		else if (btnValue == 2)
		{
			LCD_printf("Make a CIRCLE");
			TMRSRVC_delay(1000);//wait 1 s
			moveCircle();
			btnValue = 0;
			LCD_clear();			
		}
		else if (btnValue == 3)
		{
			LCD_printf("Make a FIGURE 8");
			TMRSRVC_delay(1000);//wait 1 s
			moveFigure8();
			btnValue = 0;
			LCD_clear();
		}
		else
		{
			LCD_printf("Hello Dolly");
		}
    }
}// end the CBOT_main()

/*******************************************************************
* Additional Helper Functions
********************************************************************/

/*******************************************************************
* Function:			void moveSquare(void)
* Input Variables:	none
* Output Return:	void
* Overview:			Make a square with sides of 3 ft
********************************************************************/
void moveSquare(void)
{
	int i;

	STEPPER_open(); // Open STEPPER module for use. 
	
	for (i=1;i<=4;i++){
		// Move BOTH wheels forward.
		STEPPER_move_stwt( STEPPER_BOTH, 
		STEPPER_FWD, 750*C_BOTH, 200, 400, STEPPER_BRK_OFF, // Left
		STEPPER_FWD, 750*C_BOTH, 200, 400, STEPPER_BRK_OFF ); // Right
		// Then TURN LEFT (~90-degrees)...
		STEPPER_move_stwt( STEPPER_BOTH, 
		STEPPER_REV, 110*C_LEFT, 200, 400, STEPPER_BRK_OFF, // Left
		STEPPER_FWD, 110*C_LEFT, 200, 400, STEPPER_BRK_OFF ); // Right
	}
}

/*******************************************************************
* Function:			void moveCircle(void)
* Input Variables:	none
* Output Return:	void
* Overview:			Make a circle with 3 ft in diameter
********************************************************************/
void moveCircle(void)
{
	STEPPER_open(); // Open STEPPER module for use. 
	// Move BOTH wheels forward and
	// make the robot move in a circle.
	STEPPER_move_stwt( STEPPER_BOTH, 
	STEPPER_FWD, 3341.106, 334.1106, 400, STEPPER_BRK_OFF, // Left
	STEPPER_FWD, 2305.88, 230.588, 400, STEPPER_BRK_OFF ); // Right
}

/*******************************************************************
* Function:			void moveFigure8(void)
* Input Variables:	none
* Output Return:	void
* Overview:			Make a figure '8'
********************************************************************/
void moveFigure8(void)
{

	STEPPER_open(); // Open STEPPER module for use. 
	// Move BOTH wheels forward and
	// make the robot move in a Figure 8.
	STEPPER_move_stwt( STEPPER_BOTH, 
	STEPPER_FWD, 3341.106, 334.1106, 400, STEPPER_BRK_OFF, // Left
	STEPPER_FWD, 2305.88, 230.588, 400, STEPPER_BRK_OFF ); // Right

	STEPPER_move_stwt( STEPPER_BOTH, 
	STEPPER_FWD, 2305.88, 230.588, 400, STEPPER_BRK_OFF, // Left
	STEPPER_FWD, 3341.106, 334.1106, 400, STEPPER_BRK_OFF ); // Right
}
/*******************************************************************
* Function:			int waitButton(void)
* Input Variables:	none
* Output Return:	int
* Overview:			Determine what button was pressed
********************************************************************/
int WaitButton( void ) 
{
    BOOL btnState1, btnState2, btnState3;//local variables - button states
	int rtnValue=0;//return the button value

	LCD_clear();
	
	if((ATopstat.state=SUBSYS_OPEN))
	{
       		// Get switch states.
		btnState1 = ATTINY_get_SW_state( ATTINY_SW3 );
		btnState2 = ATTINY_get_SW_state( ATTINY_SW4 );
		btnState3 = ATTINY_get_SW_state( ATTINY_SW5 );
		//LCD_printf("btnStates: %d %d %d \n", btnState1, btnState2, btnState3);

		if( btnState1 == TRUE ) 
		{
			LCD_printf( "SW1: Pushed\n");
			TMRSRVC_delay(1000);//wait 1 s

	                // Assume the LED subsystem opened successfully.
        	        LED_set_pattern( 0b00100000 );//turn the red LED on
                	TMRSRVC_delay(2000);//wait 2 seconds
               		LED_clr_pattern( 0b01000000 );//turn the green LED off
                	LED_clr_pattern( 0b00100000 );//turn the red LED off
			rtnValue=1;
			}//end if button 1 state open

		if( btnState2 == TRUE ) 
		{
			LCD_printf( "SW2: Pushed\n");
			TMRSRVC_delay(1000);//wait 1 s

	                // Assume the LED subsystem opened successfully.
        	        LED_set_pattern( 0b01000000 );//turn the green LED on
                	TMRSRVC_delay(2000);//wait 2 seconds
                	LED_clr_pattern( 0b01000000 );//turn the green LED off
                	LED_clr_pattern( 0b00100000 );//turn the red LED off
			rtnValue=2;
		}//end if btn 2 open

		if ( btnState3 == TRUE ) 
		{
			LCD_printf( "SW3: Pushed\n");
			TMRSRVC_delay(1000);//wait 1 s

	                // Assume the LED subsystem opened successfully.
        	        LED_set_pattern( 0b01000000 );//turn the green LED on
                	LED_set_pattern( 0b00100000 );//turn the red LED on
                	TMRSRVC_delay(2000);//wait 2 seconds
                	LED_clr_pattern( 0b01000000 );//turn the green LED off
                	LED_clr_pattern( 0b00100000 );//turn the red LED off
			rtnValue=3;
		}//end if btn 3 open
            	LCD_clear();
		return rtnValue;
	}//end AT while

}//end the WaitButton() function

/*******************************************************************
* Function:			float getLeftLight(void)
* Input Variables:	none
* Output Return:	float
* Overview:			getLeftLight() converts ADC voltage to value inverse proportional to left light
********************************************************************/
float getLeftLight( void )
{

	float voltage;
	ADC_SAMPLE adcsample;
	ADC_set_VREF( ADC_VREF_AVCC );
	ADC_set_channel( LEFT_LIGHT_CHAN );
	adcsample = ADC_sample();
	//LCD_printf( "ADC: %i\n",adcsample);
	voltage = adcsample * (5.0/ 1024.0 );
	//LCD_printf( "voltage: %f\n",voltage);
	return voltage;
}

/*******************************************************************
* Function:			float getRightLight(void)
* Input Variables:	none
* Output Return:	float
* Overview:			getRightLight() converts ADC voltage to value inverse proportional to right light
********************************************************************/
float getRightLight( void )
{
	float voltage;
	ADC_SAMPLE adcsample;
	ADC_set_VREF( ADC_VREF_AVCC );
	ADC_set_channel( RIGHT_LIGHT_CHAN );
	adcsample = ADC_sample();
	//LCD_printf( "ADC: %i\n",adcsample);
	voltage = adcsample * ( 5.0 / 1024 );
	//LCD_printf( "right: %f\n",voltage);
	return voltage;
}

/*******************************************************************
* Function:			float float getLeftIR(void)
* Input Variables:	none
* Output Return:	float
* Overview:			getLeftIR() converts ADC voltage to inches
********************************************************************/
float getLeftIR( void )
{
	float voltage;//IR range -0.4 to 5.3 V
	float distance;// (cm) 30 cm = 12 inches = 0.4 V	
	float dist;//distance in inches
	ADC_SAMPLE adcsample;
	// Set the Voltage Reference first so VREF=5V.
 	ADC_set_VREF( ADC_VREF_AVCC );
	// Set the channel we will sample from.
	ADC_set_channel( IRLEFT_CHAN );
	// Now sample it!
	adcsample = ADC_sample();
	//LCD_printf( "ADC: %i\n",adcsample);
	// Convert to meaningful voltage value.
	voltage = adcsample * ( 5.0 / 1024 );
	// Convert to distance in cm
	distance = (2914/(adcsample+5.0))-1.0;
	//Convert distance to inches
	dist = distance*0.3937;
	return dist;
}

/*******************************************************************
* Function:			float float getRightIR(void)
* Input Variables:	none
* Output Return:	float
* Overview:			getRightIR() converts ADC voltage to inches
********************************************************************/
float getRightIR( void )
{
	float voltage;//IR range -0.4 to 5.3 V
	float distance;// (cm) 30 cm = 12 inches = 0.4 V	
	float dist;//distance in inches
	ADC_SAMPLE adcsample;
	// Set the Voltage Reference first so VREF=5V.
	ADC_set_VREF( ADC_VREF_AVCC );
	// Set the channel we will sample from.
	ADC_set_channel( IRRIGHT_CHAN );
	// Now sample it!
	adcsample = ADC_sample();
	// Convert to meaningful voltage value.
	voltage = adcsample * ( 5.0 / 1024 );
	// Convert to distance in cm
	distance = (2914/(adcsample+5.0))-1.0;
	//Convert distance to inches
	dist = distance*0.3937;
	return dist;
}

/*******************************************************************
* Function:			float float getFrontIR(void)
* Input Variables:	none
* Output Return:	float
* Overview:			getFrontIR() converts ADC voltage to inches
********************************************************************/
float getFrontIR( void )
{
	//1 inch = 2.54 cm
	//1 cm = 0.3937 inches
	float voltage;//IR range -0.4 to 5.3 V
	float distance;// (cm) 30 cm = 12 inches = 0.4 V	
	float dist;//distance in inches
	ADC_SAMPLE adcsample;
	// Set the Voltage Reference first so VREF=5V.
	ADC_set_VREF( ADC_VREF_AVCC );
	// Set the channel we will sample from.
	ADC_set_channel(IRFRONT_CHAN);
	// Now sample it!
	adcsample = ADC_sample();
	// Convert to meaningful voltage value.
	voltage = adcsample * ( 5.0 / 1024 );
	// Convert to distance in cm
	distance = (2914/(adcsample+5.0))-1.0;
	//Convert distance to inches
	dist = distance*0.3937;
	return dist;
}

/*******************************************************************
* Function:			float float getBackIR(void)
* Input Variables:	none
* Output Return:	float
* Overview:			getBackIR() converts ADC voltage to inches
********************************************************************/
float getBackIR()
{
	//1 inch = 2.54 cm
	//1 cm = 0.3937 inches
	float voltage;//IR range -0.4 to 5.3 V
	float distance;// (cm) 30 cm = 12 inches = 0.4 V	
	float dist;//distance in inches
	ADC_SAMPLE adcsample;
	// Set the Voltage Reference first so VREF=5V.
	ADC_set_VREF( ADC_VREF_AVCC );
	// Set the channel we will sample from.
	ADC_set_channel(IRBACK_CHAN);
	// Now sample it!
	adcsample = ADC_sample();
	// Convert to meaningful voltage value.
	voltage = adcsample * ( 5.0 / 1024 );
	// Convert to distance in cm
	distance = (2914/(adcsample+5.0))-1.0;
	//Convert distance to inches
	dist = distance*0.3937;
	return dist;
}

/*******************************************************************
* Function:			I2C_STATUS get_revision( unsigned char *revision )
* Input Variables:	unsigned char revision
* Output Return:	I2C_STATUS
* Overview:			
********************************************************************/
I2C_STATUS get_revision( unsigned char *revision )
{

	I2C_STATUS i2c_stat;

	// Begin a transaction as master transmitter so that 
	// we can tell the sensor the register we want to read from
	// before we _actually_ read it.
	i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MT );

	// If successful...
	if( i2c_stat == I2C_STAT_OK )
	{

		// Send the register value (being 0).  This is
		// the register that contains the software revision.
		i2c_stat = I2C_MSTR_send( __TPA81_REVISION );

		// If successful...
		if( i2c_stat == I2C_STAT_OK )
		{

			// Now switch to master receiver so we can 
			// read the data in register 0 of the sensor.
			i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MR );

			// If successful...
			if( i2c_stat == I2C_STAT_OK )
			{

				// Read the data...
				i2c_stat = I2C_MSTR_get( revision, FALSE );

				// If NOT successful...
				if ( i2c_stat != I2C_STAT_OK )

					// Overwrite it with 0.
					*revision = 0;
				
			} // end if()

		} // end if()

	} // end if()

	// I'm going to assume the transaction was successful, 
	// so we need to stop it.
	I2C_MSTR_stop();

	// Also, return the status, regardless.
	return i2c_stat;

} // end get_revision()

