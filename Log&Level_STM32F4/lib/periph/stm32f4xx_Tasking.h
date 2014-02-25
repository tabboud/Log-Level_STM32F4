/*****************************************
 Functions and Variables used to determine
 the task to run and how to switch tasks
- Context Switching 
*****************************************/

//Function Declarations
/****Main Functions****/
void ADC_DMA_GO();			//Main ADC_DMA peripheral functions
void ACCEL_SPI_GO();		//Main Accel_SPI peripheral functions

/****Macros****/
#define HW32_REG(ADDRESS) (*((volatile unsigned long *)(ADDRESS))) 	//word accesses

/****Definitions****/
#define ADC_MODE 0			//Used for mode selection
#define ACCEL_MODE 1	


/*****Variables******/
int ACCEL_stop =0;			//Set when sw13 is pressed, triggers Accel app to execute sw13 code
int which_refresh=0;		//Holds the value of which mode is running, in order to determine which refresh_display() to run
int mode=-1;
int mode_select=0;
int task1_start=0;			//Runs the task0 if task1 is called first

/* Stack for each task (8Kbytes each - 1024 x 8 bytes)*/
long long ADC_DMA_GO_stack[1024], ACCEL_SPI_GO_stack[1024]; //Task0 (ADC_DMA) and Task1 (Accel_SPI)
/*Data used by OS*/
uint32_t curr_task=0; 	// Current task
uint32_t next_task=1; 	// Next task
uint32_t PSP_array[2]; 	// Process Stack Pointer for each task

