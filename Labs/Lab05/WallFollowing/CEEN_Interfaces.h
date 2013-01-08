 /*******************************************************************
* FileName:        CEEN_Interfaces.h
* Processor:       ATmega324P
* Compiler:        
*
* Code Description:
* This contains all the difines and protptypes for the subfunctions 
* needed to run all the hardware on the CEENBoT
*
********************************************************************/

#ifndef CEEN_Interfaces
#define CEEN_Interfaces



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

	/** Status Variable Declarations ***********************************/
	SUBSYS_OPENSTAT ATopstat;//ATTINY open status
	SUBSYS_OPENSTAT LCopstat;//LCD open status
	SUBSYS_OPENSTAT LEopstat;//LED open status

#endif