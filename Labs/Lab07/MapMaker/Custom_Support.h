  /*******************************************************************
* FileName:        Custom_Support.h
* Processor:       ATmega324P
* Compiler:        GCC
*
* Code Description:
* This contains all the difines and protptypes for the subfunctions 
* needed to run Lab code
********************************************************************/

#ifndef Custom_Support
#define Custom_Support

	// Flags
	#define SUCCESS 1;
	#define FAIL 0;

	// Arc Function Constants
	#define C_B 1
	#define C_L 1
	#define C_R 1
	#define D_STEP 0.1335176878
	#define WHEEL_BASE 21.3
	#define POINT_TURN 0
	#define NO_TURN 2147483647
	#define RIGHT_TURN 16.50
	#define LEFT_TURN -16.50
	
	// Movement Commands for pathplanning
	#define MOVE_LEFT 1
	#define MOVE_FORWARD 2
	#define MOVE_RIGHT 3
	#define MOVE_STOP 4

	// PID Control Gains
	// Note: these current values are adjusted for control cycles times
	// including LCD debug print statements, but not for prefilter
	#define KP 1
	#define KI 0
	#define KD 0
		
	// Maximum Magnitude Control Effort
	#define MAX_EFFORT 100
	#define PREFILTER_SIZE 30

	// Orientation constants
	#define NORTH 0
	#define EAST 1
	#define SOUTH 2
	#define WEST 3

	// Button States
	#define PRESSED 0
	#define UNPRESSED 1

	#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"
	#define BYTETOBINARY(byte)  \
	  (byte & 0x80 ? 1 : 0), \
	  (byte & 0x40 ? 1 : 0), \
	  (byte & 0x20 ? 1 : 0), \
	  (byte & 0x10 ? 1 : 0), \
	  (byte & 0x08 ? 1 : 0), \
	  (byte & 0x04 ? 1 : 0), \
	  (byte & 0x02 ? 1 : 0), \
	  (byte & 0x01 ? 1 : 0) 

	  
	// Integrator Error
	float Ierror = 0;
	// Previous Error Value
	float error_old = 0;

	 // Range Sensor Value Arrays For Prefilter
	float	ltIR_old[PREFILTER_SIZE];//left IR sensor
	float	rtIR_old[PREFILTER_SIZE];//right IR sensor
	float	ftIR_old[PREFILTER_SIZE];//front IR sensor
	float 	bkIR_old[PREFILTER_SIZE];//back IR sensor

	// Infrared Rangefinding Sensor Values
	float	ltIR = 0;//left IR sensor
	float	rtIR = 0;//right IR sensor
	float	ftIR = 0;//front IR sensor
	float 	bkIR = 0;//back IR sensor

	// Contact IR sensors
	BOOL rightContact;
	BOOL leftContact;
	
	// Light Range Sensor global variables
	float	rightLightVolt	= 0;//right light sensors
	float	leftLightVolt	= 0;//left light sensors

	// Create an shift matrix for rotating cells							
	static unsigned char shifted[16] = {0b0000, 0b0010, 0b0100, 0b0110, 
										0b1000, 0b1010, 0b1100, 0b1110, 
										0b0001, 0b0011, 0b0101, 0b0111, 
										0b1001, 0b1011, 0b1101, 0b1111};


	/** Local Function Prototypes **************************************/
	void initializeRobot(void);
	void checkIR(void);
	void checkLightSensor(void);
	void checkContactIR(void);
	void checkContactIR(void);
	void prefilter(char);
	float pidController(float ,char);
	char move_arc_stwt(float, float, float, float, BOOL);
	char move_arc_stnb(float, float, float, float, BOOL);

#endif