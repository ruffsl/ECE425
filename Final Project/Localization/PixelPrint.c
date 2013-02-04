/*
 * PixelPrint.c
 *
 *  Created on: Dec 22, 2011
 *      Author: risdenkj
 *	Modified:	C.A. Berry, 1/3/12
 *	Desc:		This is a program to write to pixels on the LCD, modify this to
 				use this for localization and mapping to display the robot's state, 
				location in the world or mapping data
 */

// ===== Include files =======
#include "stdio.h"
// #include <stdlib.h>
// #include <math.h>
#include "capi324v221.h"

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
int checkNoBtns(); 

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
 * checks that no buttons have been pressed
 * @return BOOL true if no button has been pressed
 */
int checkNoBtns() {
	unsigned char sensors = ATTINY_get_sensors();
	return !((sensors & SNSR_SW3_EDGE) || (sensors & SNSR_SW4_EDGE) || (sensors & SNSR_SW5_EDGE));
}

