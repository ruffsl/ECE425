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

/** Define Constants Here ******************************************/
//==== define constants ========
#define PI 3.14159265358979323846
#define DEG2RAD(DEG) ((DEG)*((PI)/(180.0)))
#define IRRIGHT_CHAN ADC_CHAN3
#define IRLEFT_CHAN ADC_CHAN4
#define IRFRONT_CHAN ADC_CHAN7

// ============================== global variables ========================== //
unsigned char pix_arr[4][128]; // keeps track of on/off LCD pixels

// ============================ prototypes ================================== //
void init_bot(void);
void LCD_set_pixel(unsigned char row, unsigned char col, BOOL val);
void all_pixel_test(void);
void pixel_sensor_test(void);
void draw_smiley(void);
void draw_face(void);
void draw_smile(void);
void draw_frown(void);
void printMainMenu(void);
float trim(float val, float min, float max);
int checkNoBtns(); 
float getLeftIR();
float getRightIR();
float getFrontIR();

// ============================ CBOT_main =================================== //
void CBOT_main(void) {
	unsigned char sensors;

	// Initialize.
	init_bot();

	// Clear the screen.
	LCD_clear();

	printMainMenu();
	while (1) {
        // Delay a little so that the 'Tiny is not overwhelmed with requests.
        TMRSRVC_delay( 100 ); // Wait 100ms.

        // Get state of all sensors.
        sensors = ATTINY_get_sensors();
        if ( sensors & SNSR_SW3_EDGE ) {
                SPKR_play_tone( SPKR_FREQ( 293.7 ), 250, 80 );
                all_pixel_test();
                printMainMenu();
        } else if ( sensors & SNSR_SW4_EDGE ) {
                SPKR_play_tone( SPKR_FREQ( 440 ), 250, 80 );
                pixel_sensor_test();
                printMainMenu();
        } else if ( sensors & SNSR_SW5_EDGE ) {
                SPKR_play_tone( SPKR_FREQ( 659.3 ), 250, 80 );
				draw_smiley();
                printMainMenu();
        }
	}
}


// ============================ functions =================================== //
void init_bot(void) {
	// Start by opening and initializing the needed subsystem modules:
	LCD_open();
	SPKR_open( SPKR_TONE_MODE );
	ATTINY_open();
	ADC_open();

	ADC_set_VREF( ADC_VREF_AVCC );

	// pixel array for the LCD screen
	for(int i = 0; i < 4; i++) {
		for(int j = 0; j < 128; j++) {
			pix_arr[i][j] = 0x00;
		}
	}
}

/*
 * Prints the main menu
 */
void printMainMenu(void) {
	LCD_clear();
	LCD_printf("Btn 1: All Pixels\nBtn 2: Pixel Sensor\nBtn 3: Smiley\n");
}



/*
 * a function that tests that all the pixels in the LCD
 * can be turned on an off with the LCD_set_pixel function
 */
void all_pixel_test(void) {
	while(checkNoBtns()) {
		for(unsigned char row = 0; row < LCD_PIX_HEIGHT; row++)
		{
			for( unsigned char col = 0; col < LCD_PIX_WIDTH; col++ )
			{
				LCD_set_pixel(row, col, 1);
			}
		}

		TMRSRVC_delay(2000);

		for(unsigned char row = 0; row < LCD_PIX_HEIGHT; row++)
		{
			for( unsigned char col = 0; col < LCD_PIX_WIDTH; col++ )
			{
				LCD_set_pixel(row, col, 0);
			}
		}

		TMRSRVC_delay(2000);
	}
}

/*
 * a function that can test that sensor data can be output
 * to the LCD display in a meaningful manner and the normal
 * printf still works correctly
 */
void pixel_sensor_test(void) {
	while(checkNoBtns()) {
		TMRSRVC_delay(100);
		LCD_clear();
		float leftIR = getLeftIR();
		float rightIR = getRightIR();
		float trimLeftIR = trim(2*leftIR, 0, LCD_PIX_WIDTH/2);
		float trimRightIR = trim(2*rightIR, 0, LCD_PIX_WIDTH/2);

		for(unsigned char i = 0; i < LCD_PIX_HEIGHT; i++) {
			LCD_set_pixel(i, LCD_PIX_WIDTH/2, i & 1);
			LCD_set_pixel(i, LCD_PIX_WIDTH/2 - trimLeftIR, 1);
			LCD_set_pixel(i, LCD_PIX_WIDTH/2-1 + trimRightIR, 1);
		}

		LCD_set_RC( 0, 0 );
		LCD_printf("%.1f", (double)leftIR);
		LCD_set_RC( 0, 16 );
		LCD_printf("%.1f", (double)rightIR);
	}
}


// a function to draw a smiley face
void draw_smiley (void){

	int num = 0;
	while(checkNoBtns()) 
	{
		TMRSRVC_delay(100);
		LCD_clear();

		if (num % 2==0)
		{
			draw_face();
			draw_smile();
			TMRSRVC_delay(2000);
		}
		else
		{
			draw_face();
			draw_frown();
			TMRSRVC_delay(2000);
		}

		num++;

		for(unsigned char row = 0; row < LCD_PIX_HEIGHT; row++)
		{
			for( unsigned char col = 0; col < LCD_PIX_WIDTH; col++ )
			{
				LCD_set_pixel(row, col, 0);
			}
		}
	}
}


// a function to draw face
void draw_face (void){
		int row;
		int col;

		for(unsigned char row = 29; row < 30; row++)
		{
			for( unsigned char col = 55; col < 76; col++ )
			{
				LCD_set_pixel(row, col, 1);
				LCD_set_pixel(row-26, col, 1);
			}
		}


		for(unsigned char col = 81; col < 82; col++)
		{
			for(unsigned char row = 9; row < 24; row++)
			{
				LCD_set_pixel(row, col, 1);
				LCD_set_pixel(row, col-32, 1);
			}
		}


		col=54;
		for(unsigned char row = 4; row<9; row++)
		{
				LCD_set_pixel(row, col, 1);
				LCD_set_pixel(row+20, col+26, 1);
				col--;
		}

		row=4;
		for(unsigned char col = 76; col < 81; col++)
		{
				LCD_set_pixel(row, col, 1);
				LCD_set_pixel(row+20, col-26, 1);
				row++;
		}
}

// a function to draw smile
void draw_smile (void){
		int row;

		for(unsigned char row = 16; row < 17; row++)
		{
			for( unsigned char col = 64; col < 67; col++ )
			{
				LCD_set_pixel(row, col, 1);
			}
		}


		for(unsigned char row = 9; row < 10; row++)
		{
			for( unsigned char col = 62; col < 69; col++ )
			{
				LCD_set_pixel(row, col, 1);
			}
		}

		//draw the eyes
		row = 20;
		for( unsigned char col = 58; col < 61; col++ )
		{
			LCD_set_pixel(row, col, 1);
			LCD_set_pixel(row, col+10, 1);
			LCD_set_pixel(row, 120-col, 1);
			LCD_set_pixel(row, 130-col, 1);
			row++;
		}


		//draw the mouth
		row = 9;
		for( unsigned char col = 68; col < 72; col++ )
		{
			LCD_set_pixel(row, col, 1);
			LCD_set_pixel(row, 130-col, 1);
			row++;
		}


		//draw the nose
		row = 15;
		for( unsigned char col = 63; col < 65; col++ )
		{
			LCD_set_pixel(row, col, 1);
			LCD_set_pixel(row, 130-col, 1);
			row++;
		}
}

// a function to draw frown
void draw_frown (void){
		int row;

				for(unsigned char row = 16; row < 17; row++)
		{
			for( unsigned char col = 64; col < 67; col++ )
			{
				LCD_set_pixel(row, col, 1);
			}
		}


		for(unsigned char row = 9; row < 10; row++)
		{
			for( unsigned char col = 62; col < 69; col++ )
			{
				LCD_set_pixel(row, col, 1);
			}
		}
		
		
		//draw the nose
		row = 15;
		for( unsigned char col = 63; col < 65; col++ )
		{
			LCD_set_pixel(row, col, 1);
			LCD_set_pixel(row, 130-col, 1);
			row++;
		}

		//draw the eyes
		row = 20;
		for( unsigned char col = 68; col < 73; col++ )
		{
			LCD_set_pixel(row, col, 1);
			LCD_set_pixel(row, 130-col, 1);
			row++;
		}

		//draw the mouth
		row = 6;
		for( unsigned char col = 59; col < 63; col++ )
		{
			LCD_set_pixel(row, col, 1);
			LCD_set_pixel(row, 130-col, 1);
			row++;
		}

}

/*
 * a function that sets a single pixel on/off for the LCD
 * @param row an unsigned char that specifies the lcd row
 * @param col an unsigned char that specifies the lcd column
 * @param val a boolean that specifies the pixel value to be set
 *	LCD text print size (4 rows, 22 columns)
 *	LCD pixel print size (32 rows, 128 columns)
 */
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


/*
 * a trim function that takes a val and limits it to the 
 * specified max and min values
 * @param val an integer argument.for the value to be trimmed
 * @param min an integer argument the max val can be
 * @param max an integer argument the max val can be
 * @return integer trimmed value
 */
float trim(float val, float min, float max) {
	if(val < min) {
		return min;
	} else if(val > max) {
		return max;
	} else {
		return val;
	}
}

/*
 * checks that no buttons have been pressed
 * @return BOOL true if no button has been pressed
 */
int checkNoBtns() {
	unsigned char sensors = ATTINY_get_sensors();
	return !((sensors & SNSR_SW3_EDGE) || (sensors & SNSR_SW4_EDGE) || (sensors & SNSR_SW5_EDGE));
}

// getRightIR() converts ADC voltage to inches
float getRightIR(){
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

// getFrontIR() simply sets the ADC to the Front IR sensor
// and returns the sampled voltage in inches. Short, nonblocking.
float getFrontIR(){
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

// getLeftIR() simply sets the ADC to the Left IR sensor
float getLeftIR(){
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
