/*************************
Function declarations and variables 
used for the Display
*************************/
//Function Declarations
/****Main Functions****/
void set_pattern(int, int, int, int, int, int, int, int, int, char *);	//Assigns values to the display buffer
void decode_pattern(char *buffer);
void refresh_display();
void decode_Acc_val();						//Holds the digits to display, abc[2] == Ones digit, ..., abc[0] == Hundreds digit


/****Definitions****/
#define NEG_SIGN 10
#define BLANK_SCREEN 11
#define SW_DISP 1
#define VOLT_DISP 2
#define ACCEL_DISP 3

/*****Variables******/
int Acc_val=0, PrevAcc_val=-1;
int Acc_turn = LEVEL;			//Value used to turn on the LED 
int sw_event=0;					//integer used to store the variable for return to the switch que
int sw_num = 7;					//Tells which switch to test, 1-13
int saved_LR;					//Location of saved LR from branch and links
int prev_A=1;
int disp_index = -1;			//Used by assembly to count through array
int digit_value=0, display=4;	//number to be printed, display to turn on
int flag =0;
int abc[3]={0,0,0};				//Holds the digits to display, abc[2] == Ones digit, ..., abc[0] == Hundreds digit
int abc2[4]={0,0,0,0};			//Holds the digits to display, abc[2] == Ones digit, ..., abc[0] == Hundreds digit
int neg_flag =0;				//Used to set negative sign
static char displaybuf[17];		//declare only once
int Acc_disp_off=0;				//Set if Display should be OFF
int Volt_disp_off=0;