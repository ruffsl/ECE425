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
	#define SUCCESS 1
	#define FAIL 0
	#define TO_MAP_ROTATE 0
	#define TO_ROBOT_ROTATE 1

	// Arc Function Constants
	#define C_B 1
	#define C_L 1
	#define C_R 1
	#define D_STEP 0.1335176878
	#define WHEEL_BASE 21.3
	#define POINT_TURN 0
	#define NO_TURN 2147483647
	#define RIGHT_TURN 17.50
	#define LEFT_TURN -17.50
	
	// Obstacle Avoidance Threshold
	#define IR_OBST_F_THRESH 7
	#define IR_OBST_R_THRESH 10
	#define IR_OBST_L_THRESH 10
	#define IR_OBST_B_THRESH 7
	
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
	
	// World Size
	#define WORLD_ROW_SIZE 4
	#define WORLD_COLUMN_SIZE 4
	#define WORLD_RESOLUTION_SIZE 45.72
	
	// Maximum number of User Moves
	#define MAX_MOVE_SIZE 12
	
	// Print Size
	#define LCD_OFFSET 31
	#define LCD_CELL_OFFSET 8
	
	#define MAX_SPEED 200
	#define WALL_STEP 20

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
	float Ierror;
	// Previous Error Value
	float error_old;

	 // Range Sensor Value Arrays For Prefilter
	float	ltIR_old[PREFILTER_SIZE];//left IR sensor
	float	rtIR_old[PREFILTER_SIZE];//right IR sensor
	float	ftIR_old[PREFILTER_SIZE];//front IR sensor
	float 	bkIR_old[PREFILTER_SIZE];//back IR sensor

	// Infrared Rangefinding Sensor Values
	float	ltIR;//left IR sensor
	float	rtIR;//right IR sensor
	float	ftIR;//front IR sensor
	float 	bkIR;//back IR sensor

	// Contact IR sensors
	BOOL rightContact;
	BOOL leftContact;
	
	// Light Range Sensor global variables
	float	rightLightVolt;//right light sensors
	float	leftLightVolt;//left light sensors

	// Create an shift matrix for rotating cells							
	static unsigned char shiftedL[16] ={0b0000, 0b0010, 0b0100, 0b0110, 
										0b1000, 0b1010, 0b1100, 0b1110, 
										0b0001, 0b0011, 0b0101, 0b0111, 
										0b1001, 0b1011, 0b1101, 0b1111};
											
	static unsigned char shiftedR[16] ={0b0000, 0b1000, 0b0001, 0b1001, 
										0b0010, 0b1010, 0b0011, 0b1011, 
										0b0100, 0b1100, 0b0101, 0b1101, 
										0b0110, 0b1110, 0b0111, 0b1111};
										
	
	// Keeps track of on/off LCD pixels
	unsigned char pix_arr[4][32];
	
	
	// Map of the Robot World
	// unsigned char ROBOT_WORLD[WORLD_ROW_SIZE][WORLD_COLUMN_SIZE];
	// unsigned char ROBOT_WORLD[WORLD_ROW_SIZE][WORLD_COLUMN_SIZE] = 	{{0b1001, 0b1000, 0b1111, 0b1111}, 
																	 // {0b0101, 0b1111, 0b1111, 0b1101}, 
																	 // {0b0101, 0b1111, 0b1111, 0b0100}, 
																	 // {0b1111, 0b0111, 0b1111, 0b0111}};
																	 
extern unsigned char ROBOT_WORLD[WORLD_ROW_SIZE][WORLD_COLUMN_SIZE];

// unsigned char ROBOT_WORLD[WORLD_ROW_SIZE][WORLD_COLUMN_SIZE];

		
	// Create an array for button value commands
	unsigned char moveCommands[MAX_MOVE_SIZE];
	unsigned char moveGateways[MAX_MOVE_SIZE];
	unsigned char currentMoveWorld;
	unsigned char currentCellWorld;
	unsigned char currentCellWorldStart;
	unsigned char currentOrientation;
	unsigned char currentOrientationStart;
	unsigned char currentGateway;
	unsigned char nextGateway;
	
	
	// odometry values
	float odometryStepL;
	float odometryStepR;
	float odometryTrigger;
	unsigned char odometryFlag;
	STEPPER_STEPS curr_step;


	/** Local Function Prototypes **************************************/
	void initializeRobot(void);
	void checkIR(void);
	void checkLightSensor(void);
	void checkContactIR(void);
	void checkContactIR(void);
	void prefilter(char);
	float pidController(float ,char);
	unsigned char rotateCell(unsigned char,unsigned char, char);
	char move_arc_stwt(float, float, float, float, BOOL);
	char move_arc_stnb(float, float, float, float, BOOL);
	void LCD_set_pixel(unsigned char row, unsigned char col, BOOL val);
	void printCell(unsigned char, unsigned char, unsigned char, BOOL, unsigned char);
	void printMap(void);
	void checkOdometry(unsigned char);

#endif
