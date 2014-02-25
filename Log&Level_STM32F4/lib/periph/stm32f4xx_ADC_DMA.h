/*************************
Function declarations and variables used
by the ADC_DMA Application
*************************/
//Function Declarations
/****Main Functions****/
void ADC3_DMA_Config(uint8_t , uint32_t);
void get_Voltage();
//Other functions are defined in the stm32f4xx_ADC_DMA.asm file

/****Definitions****/
#define SWITCH_MASK 0x0000000F
#define TIME_MASK 0xFFFFFF00
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)


/*****Variables******/
__IO uint32_t ADC3ConvertedValue = 0;
__IO uint32_t ADC3ConvertedVoltage = 0;
int ADC_stop =0;
int testVoltage=0;	
int precision_cnt=0; 			//0 = No precision, (3); 1 = 1 precision, (3.1); ...
int	SW13_stop=0;
int set_negs=0;					//Flag used in Over/under situation
uint32_t ConvertedVoltage=0; 	//Used in the display function
uint32_t Prev_voltage=-1;		//Used in the disp function to test when to decode voltage
int logging=0;					//Flag for the blinking LED for logging 