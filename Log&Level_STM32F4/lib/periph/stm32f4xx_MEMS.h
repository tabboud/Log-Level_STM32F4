/*************************
Accelerometer function declarations
and variables used

*************************/
//Function Declarations
/****MEMS Main Functions****/
void SPI_GPIO_init();						//Located in stm32f4xx_MEMS.asm
uint8_t configure_MEMS();					//Located in stm32f4xx_MEMS.asm
void MEMS_write();							//Writes values to MEMS
void send_config();							//Configures the Accel Chip: Power Mode, Axes, Self Test ...
void MEMS_read(uint8_t *, uint8_t, int);	//C-Function
void my_init();								//Initializes SPI, GPIO, and configures MEMS
void TimingDelay_Decrement(void);
void moved();								//Polling of the Accelerometer!
void turn_on_LED();
void Poll_Acc();

/****Used in MEMS_Read******/
void setChip_LOW();					//Sets the chip select Low;  
void setChip_HIGH();				//Sets the chip select High; 
void send_readAddr(uint8_t);		//Sends Address of indexed register and checks TX Flag
uint16_t receive_byte();			//Returns SPI1->DR data
int get_FlagStatus(volatile unsigned int *);	//Checks the status of the RXNE Flag, C-Function

/*****Macros*****/
#define ABS(x)                           (x < 0) ? (-x) : x
#define MAX(a,b)                         (a < b) ? (b) : a


/****Definitions****/
#define RESET1 0
#define RXNE_FLAG 1
#define LEFT 1				//Used in turn_on_LED();
#define RIGHT 2
#define LEVEL 0

/*****Variables******/
uint16_t PrescalerValue = 0;
uint8_t Buffer[6];
__IO uint32_t TimingDelay = 0;
__IO int8_t XOffset;
__IO int8_t YOffset;
__IO uint8_t TempAcceleration = 0;
uint8_t Counter  = 0;
volatile unsigned int *SPI1_SR = (unsigned int *)0x40013008;	//pointer to the address of SPI1->SR
