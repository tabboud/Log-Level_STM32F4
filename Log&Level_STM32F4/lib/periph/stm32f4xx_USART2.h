/*****************************
 Functions and Variables used
 by the USART2 Periph.
*****************************/
//Function Declarations
/****Main Functions****/
void USART2_init();			//Initializes the USART2 Periph.
void get_string();			//Gets input from the user
void USART_puts(volatile char *);	//Sends a string through USART2 -> PC
int RxFlag_Status(); 		//Returns 0->flag not set, 1->Flag Set
int TcFlag_Status();		//Checks Transmission Complete Flag
//*NOTE: Other functions defined in stm32f4xx_USART2.asm

/****Definitions****/
#define MAX_STRLEN 12 		//Maximum string length of our string in characters


/*****Variables******/
volatile char received_string[MAX_STRLEN+1]; //Holds the recieved string
