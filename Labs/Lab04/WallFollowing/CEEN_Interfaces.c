 /*******************************************************************
* FileName:        CEEN_Interfaces.c
* Processor:       ATmega324P
* Compiler:        
*
* Code Description:
* This contains all the subfunctions needed to run all the hardware on the CEENBoT
*
********************************************************************/

#include "capi324v221.h"
#include "stdio.h"
#include "CEEN_Interfaces.h"
 

/*******************************************************************
* Function:			int waitButton(void)
* Input Variables:	none
* Output Return:	int
* Overview:			Use a comment block like this before functions
********************************************************************/
int WaitButton( void ) 
{
    BOOL btnState1, btnState2, btnState3;//local variables - button states
	//int rtnValue=0;//return the button value
	int rtnValue=0;

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
	// LCD_printf( "voltage: %f\n",voltage);
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
	// LCD_printf( "ADC: %i\n",adcsample);
	voltage = adcsample * ( 5.0 / 1024 );
	// LCD_printf( "right: %f\n",voltage);
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
	dist = distance*1;
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
	dist = distance*1;
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
	// Note: Swaped with back IR for orientation
	ADC_set_channel(IRBACK_CHAN);
	// Now sample it!
	adcsample = ADC_sample();
	// Convert to meaningful voltage value.
	voltage = adcsample * ( 5.0 / 1024 );
	// Convert to distance in cm
	distance = (2914/(adcsample+5.0))-1.0;
	dist = distance*1;
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
	// Note: Swaped with frount IR for orientation
	ADC_set_channel(IRFRONT_CHAN);
	// Now sample it!
	adcsample = ADC_sample();
	// Convert to meaningful voltage value.
	voltage = adcsample * ( 5.0 / 1024 );
	// Convert to distance in cm
	distance = (2914/(adcsample+5.0))-1.0;
	dist = distance*1;
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


/*******************************************************************
* Function:			I2C_STATUS read_pixel_NUMBER( unsigned char *revision )
* Input Variables:	unsigned char revision
* Output Return:	I2C_STATUS
* Overview:			These are several pixel read values for each of the pixels
********************************************************************/
// get pixel one
I2C_STATUS read_pixel_1( unsigned char *pixel_1)
{
	I2C_STATUS i2c_stat;
	i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MT );
	if( i2c_stat == I2C_STAT_OK )
	{
		i2c_stat = I2C_MSTR_send(__TPA81_PIX1 );
		if( i2c_stat == I2C_STAT_OK )
		{
			i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MR );
			if( i2c_stat == I2C_STAT_OK )
			{
				i2c_stat = I2C_MSTR_get( pixel_1, FALSE );
				if ( i2c_stat != I2C_STAT_OK )
					*pixel_1 = 0;
			}
		} 
	} 
	I2C_MSTR_stop();
	return i2c_stat;
}


// get pixel two
I2C_STATUS read_pixel_2( unsigned char *pixel_2)
{
	I2C_STATUS i2c_stat;
	i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MT );
	if( i2c_stat == I2C_STAT_OK )
	{
		i2c_stat = I2C_MSTR_send(__TPA81_PIX2 );
		if( i2c_stat == I2C_STAT_OK )
		{
			i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MR );
			if( i2c_stat == I2C_STAT_OK )
			{
				i2c_stat = I2C_MSTR_get( pixel_2, FALSE );
				if ( i2c_stat != I2C_STAT_OK )
					*pixel_2 = 0;
			}
		} 
	} 
	I2C_MSTR_stop();
	return i2c_stat;
}

// get pixel three
I2C_STATUS read_pixel_3( unsigned char *pixel_3)
{
	I2C_STATUS i2c_stat;
	i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MT );
	if( i2c_stat == I2C_STAT_OK )
	{
		i2c_stat = I2C_MSTR_send(__TPA81_PIX3 );
		if( i2c_stat == I2C_STAT_OK )
		{
			i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MR );
			if( i2c_stat == I2C_STAT_OK )
			{
				i2c_stat = I2C_MSTR_get( pixel_3, FALSE );
				if ( i2c_stat != I2C_STAT_OK )
					*pixel_3 = 0;
			}
		} 
	} 
	I2C_MSTR_stop();
	return i2c_stat;
}

// get pixel four
I2C_STATUS read_pixel_4( unsigned char *pixel_4)
{
	I2C_STATUS i2c_stat;
	i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MT );
	if( i2c_stat == I2C_STAT_OK )
	{
		i2c_stat = I2C_MSTR_send(__TPA81_PIX4 );
		if( i2c_stat == I2C_STAT_OK )
		{
			i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MR );
			if( i2c_stat == I2C_STAT_OK )
			{
				i2c_stat = I2C_MSTR_get( pixel_4, FALSE );
				if ( i2c_stat != I2C_STAT_OK )
					*pixel_4 = 0;
			}
		} 
	} 
	I2C_MSTR_stop();
	return i2c_stat;
}
// get pixel five
I2C_STATUS read_pixel_5( unsigned char *pixel_5)
{
	I2C_STATUS i2c_stat;
	i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MT );
	if( i2c_stat == I2C_STAT_OK )
	{
		i2c_stat = I2C_MSTR_send(__TPA81_PIX5 );
		if( i2c_stat == I2C_STAT_OK )
		{
			i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MR );
			if( i2c_stat == I2C_STAT_OK )
			{
				i2c_stat = I2C_MSTR_get( pixel_5, FALSE );
				if ( i2c_stat != I2C_STAT_OK )
					*pixel_5 = 0;
			}
		} 
	} 
	I2C_MSTR_stop();
	return i2c_stat;
}
// get pixel six
I2C_STATUS read_pixel_6( unsigned char *pixel_6)
{
	I2C_STATUS i2c_stat;
	i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MT );
	if( i2c_stat == I2C_STAT_OK )
	{
		i2c_stat = I2C_MSTR_send(__TPA81_PIX6 );
		if( i2c_stat == I2C_STAT_OK )
		{
			i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MR );
			if( i2c_stat == I2C_STAT_OK )
			{
				i2c_stat = I2C_MSTR_get( pixel_6, FALSE );
				if ( i2c_stat != I2C_STAT_OK )
					*pixel_6 = 0;
			}
		} 
	} 
	I2C_MSTR_stop();
	return i2c_stat;
}

// get pixel seven
I2C_STATUS read_pixel_7( unsigned char *pixel_7)
{
	I2C_STATUS i2c_stat;
	i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MT );
	if( i2c_stat == I2C_STAT_OK )
	{
		i2c_stat = I2C_MSTR_send(__TPA81_PIX7 );
		if( i2c_stat == I2C_STAT_OK )
		{
			i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MR );
			if( i2c_stat == I2C_STAT_OK )
			{
				i2c_stat = I2C_MSTR_get( pixel_7, FALSE );
				if ( i2c_stat != I2C_STAT_OK )
					*pixel_7 = 0;
			}
		} 
	} 
	I2C_MSTR_stop();
	return i2c_stat;
}

// get pixel eight
I2C_STATUS read_pixel_8( unsigned char *pixel_8)
{
	I2C_STATUS i2c_stat;
	i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MT );
	if( i2c_stat == I2C_STAT_OK )
	{
		i2c_stat = I2C_MSTR_send(__TPA81_PIX8 );
		if( i2c_stat == I2C_STAT_OK )
		{
			i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MR );
			if( i2c_stat == I2C_STAT_OK )
			{
				i2c_stat = I2C_MSTR_get( pixel_8, FALSE );
				if ( i2c_stat != I2C_STAT_OK )
					*pixel_8 = 0;
			}
		} 
	} 
	I2C_MSTR_stop();
	return i2c_stat;
}

/*******************************************************************
* Function:			I2C_STATUS get_ambient_temp( unsigned char *revision )
* Input Variables:	unsigned char revision
* Output Return:	I2C_STATUS
* Overview:			Reads the ambient temperature
********************************************************************/

I2C_STATUS get_ambient_temp( unsigned char *ambient_temp )
{

	I2C_STATUS i2c_stat;

	// Begin a transaction as master transmitter so that 
	// we can tell the sensor the register we want to read from
	// before we _actually_ read it.
	i2c_stat = I2C_MSTR_start( __TPA81_ADDR, I2C_MODE_MT );

	// If successful...
	if( i2c_stat == I2C_STAT_OK )
	{

		// Send the register value (being 1).  This is
		// the register that contains the ambient temperature.
		i2c_stat = I2C_MSTR_send( __TPA81_AMBIENT );

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
				i2c_stat = I2C_MSTR_get( ambient_temp, FALSE );

				// If NOT successful...
				if ( i2c_stat != I2C_STAT_OK )

					// Overwrite it with 0.
					*ambient_temp = 0;
			
			} // end if()

		} // end if()

	} // end if()

	// I'm going to assume the transaction was successful, 
	// so we need to stop it.
	I2C_MSTR_stop();

	// Also, return the status, regardless.
	return i2c_stat;

} // end get_ambient_temp()


