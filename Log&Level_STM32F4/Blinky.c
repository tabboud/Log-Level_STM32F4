/*----------------------------------------------------------------------------
 * Name:  ENEE440: STM32F4 Project 
 * Purpose:  Logging Voltage Meter / Accelerometer Interface
 * Note(s): USART2:  PA2 (Tx Pin)
 *					 PA3 (Rx Pin)
 *	
 * 
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include "lib/stm32f4xx.h"
#include "LED.h"

//global variables used in CortexM4asmOps_01.asm but defined here
	int Cint;
	volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */
 

/**My Peripheral header files**/
#include "lib/periph/stm32f4xx_USART2.h"			
#include "lib/periph/stm32f4xx_ADC_DMA.h"
#include "lib/periph/stm32f4xx_MEMS.h"
#include "lib/periph/stm32f4xx_DISPLAY.h"			
#include "lib/periph/stm32f4xx_Tasking.h"			
#include "lib/periph/stm32f4xx_DISPLAY_Func.c"		//Holds the code for all display functions


/* Exported functions ------------------------------------------------------- */
void Delay(__IO uint32_t nTime);

/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
	msTicks++;
	switch(which_refresh){
		case SW_DISP:	 refresh_SWdisplay();    break;
		case VOLT_DISP:	 refresh_VOLTdisplay();  
						if(!(msTicks%225) && logging)
							LED_Logging_ON();		//blink LED4
						else if(!(msTicks%200))
								LED_OFF();
						break;
		case ACCEL_DISP: refresh_ACCELdisplay(); break; 
	}

 	if((msTicks%20==0)&&(getSWITCH(13) == 0)){	//Only run every 20ms!! 
		if(mode == ADC_MODE){
			if(which_refresh == SW_DISP)
				SW13_stop = 1;
			else
				ADC_stop = 1;		//Tells the ADC periph that sw13 was pressed	
		}
		if(mode == ACCEL_MODE)
			ACCEL_stop=1;			//Tells the Accel periph that sw13 was pressed
	} 
}
/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {                                              
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}


/*----------------------------------------------------------------------------
  Function that initializes Button pins
 *----------------------------------------------------------------------------*/
void BTN_Init(void) {

  RCC->AHB1ENR  |= ((1UL <<  0) );              /* Enable GPIOA clock         */

  GPIOA->MODER    &= ~((3UL << 2*0)  );         /* PA.0 is input              */
  GPIOA->OSPEEDR  &= ~((3UL << 2*0)  );         /* PA.0 is 50MHz Fast Speed   */
  GPIOA->OSPEEDR  |=  ((2UL << 2*0)  ); 
  GPIOA->PUPDR    &= ~((3UL << 2*0)  );         /* PA.0 is no Pull up         */
}

/*----------------------------------------------------------------------------
  Function that read Button pins
 *----------------------------------------------------------------------------*/
uint32_t BTN_Get(void) {

 return (GPIOA->IDR & (1UL << 0));
}





/*----------------------------------------------------------------------------
  *********************************
		MAIN FUNCTION
  **********************************
 *----------------------------------------------------------------------------*/
int main (void) {
	int i=0;
	volatile char first[13];				//Holds the users First name
  SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }

	LED_Init();		//Initialize stm32 Leds
	BTN_Init();   	//Initialize stm32 Btns
	init_var();		//Port Initializations
	switch_init();	//Initialize expansion board switches
	
/****Task Inititalization*****/
	//Starting the task scheduler
	//Create stack frame for Task0 (ADC_DMA)
	PSP_array[0] = ((unsigned int) ADC_DMA_GO_stack) + (sizeof ADC_DMA_GO_stack) - 16*4;
	HW32_REG((PSP_array[0] + (14<<2))) = (unsigned long) ADC_DMA_GO;
	HW32_REG((PSP_array[0] + (15<<2))) = 0x01000000; // initial xPSR / PC
	// Create stack frame for Task1 (ACCEL_SPI)
 	PSP_array[1] = ((unsigned int) ACCEL_SPI_GO_stack) + (sizeof ACCEL_SPI_GO_stack) - 16*4;
	HW32_REG((PSP_array[1] + (14<<2))) = (unsigned long) ACCEL_SPI_GO;
	HW32_REG((PSP_array[1] + (15<<2))) = 0x01000000; // initial xPSR / PC

	curr_task = 0; // Set current task as ADC_DMA
	__set_PSP((PSP_array[curr_task] + 16*4)); 	// Set PSP to top of task 0 stack
	NVIC_SetPriority(PendSV_IRQn, 0xFF); 		// Set PendSV to lowest possible priority
	NVIC_SetPriority(SVCall_IRQn, 0x00); 		// Set SVC to highest possible priority
	__set_CONTROL(0x3); 						// Switch to use Process Stack, unprivileged state
	__ISB(); // Execute ISB after changing CONTROL 
/**End**/
	
	USART2_init();		// initialize USART2 @ 9600 baud
	USART_puts( "Welcome to Tony Abboud's ENEE440 Project\r\n\n");	
	USART_puts( "\r\nEnter your first name (< 12 char): ");
	get_string();
	for(i=0;received_string[i]!='\0';i++) 
		first[i] = received_string[i];
	first[i]='\0';
	USART_puts( "\r\n\nHello ");
	USART_puts( first);
	USART_puts( ", please select which peripheral you want to use:\r\n");
	USART_puts( "A. ADC/DMA: Logging Voltage Meter\r\nB. SPI: Accelerometer Interface\r\n");
	get_string();		//Get the input for the mode.
	if(received_string[0] == 'a' || received_string[0] == 'A'){
		mode = ADC_MODE;
		task1_start=1;		//Dont execute task2 at startup
	}else
		mode = ACCEL_MODE;

	ADC_DMA_GO();		//Start task0 (ADC_DMA) Peripheral, regardless of mode selected
	
	while(1){		//!!should not get here
		USART_puts("\r\nError has occurred\r\n");
		Delay(1000);
	}
}

	


/*
 * @brief: - Logging Voltmeter Main Functions and Inits
 * @note: This function is an example of how to 
 *		  interface with the ADC/DMA peripheral
 *		    	 
 */
void ADC_DMA_GO(){
	if(task1_start==0){				//Accel_SPI task chosen first?
		task1_start=1;				//Yes - Run Accel_SPI task
		SVC_ACCEL_SPI();			
	}
		
	Volt_disp_off = 1;				//Turn off the Display
	which_refresh = VOLT_DISP;		//Set the refresh display function to the Voltage display
	curr_task = 0;					//Set current task to ADC_DMA
	
	uint8_t samp_time=0x00;			//Sample time for ADC; 3 cycles (0x00) default
	uint32_t samp_delay=0x00000000;	//Sample delay for ADC; 5 cycles (0x00) default
	int ov_Voltage=0, un_Voltage=0;	//Over and Under voltage
	volatile char bytes[7]; 		//Holds the char byte value of the decimal number
	ADC_DeInit();
	DMA_DeInit();		//Deinitialize structs for repeated stuff
		
		USART_puts( "\r\nADC / DMA Peripheral Example: Logging Voltmeter\r\n");
		USART_puts( "\nSetting Selection:\r\n");
		USART_puts( "Choose an ADC Sample Rate\r\n");				//ADC Sample Rate Settings
		USART_puts( "A. 3 cycles (default)\r\nB. 15 cycles\r\n");
		USART_puts( "C. 28 cycles\r\nD. 56 cycles\r\n");
		get_string();
		if(received_string[0]>='A' && received_string[0]<='Z')	//Convert to lowercase if entered is uppercase
			received_string[0] += 32;
		switch(received_string[0]){
			case 'b': samp_time = 0x01; break;		//Pass to ADC3_DMA_Config();
			case 'c': samp_time = 0x02; break;
			case 'd': samp_time = 0x03; break;
			default:  samp_time = 0x00; break;
		}
		USART_puts( "\nChoose an ADC Sampling Delay\r\n");			//ADC Sample Delay Settings
		USART_puts( "A. 5 cycles (default)\r\nB. 6 cycles\r\n");
		USART_puts( "C. 7 cycles\r\nD. 8 cycles\r\n");
		get_string();
		if(received_string[0]>='A' && received_string[0]<='Z')	
			received_string[0] += 32;
		switch(received_string[0]){
			case 'b': samp_delay = 0x00000100; break;	//Pass to ADC3_DMA_Config();
			case 'c': samp_delay = 0x00000200; break;
			case 'd': samp_delay = 0x00000300; break;
			default:  samp_delay = 0x00000000; break;
		}		
		/****Over/Under Voltage****/
		USART_puts( "\r\nPlease Enter an OVER voltage using switches 1-8.\r\n");
		USART_puts( "   *note: Use the Rotary Encoder to get more/less precision\r\n");
		USART_puts( "Press sw13 when you are done\r\n");
		get_Voltage();					//Users OVER voltage is in "testVoltage"
		ov_Voltage = testVoltage;		//Store the over voltage
		switch(precision_cnt){
			case 0: ov_Voltage*=1000; break;	//extend ov_Voltage to 4 digits
			case 1: ov_Voltage*=100;  break;	
			case 2: ov_Voltage*=10;	  break;	
		}

		USART_puts( "\r\nPlease Enter a UNDER voltage using the switches.\r\n");
		USART_puts( "   *note: Use the Rotary Encoder to get more/less precision\r\n");
		USART_puts( "Press sw13 when you are done\r\n");
		get_Voltage();					//Users voltage is in "testVoltage"
		un_Voltage = testVoltage;		//Store the under Voltage
		switch(precision_cnt){
			case 0: un_Voltage*=1000; break;	//extend un_Voltage to 4 digits
			case 1: un_Voltage*=100;  break;	
			case 2: un_Voltage*=10;	  break;	
		}
		
		USART_puts( "Configuring GPIO...\r\n");
		USART_puts( "Configuring ADC...\r\n");
		USART_puts( "Configuring DMA...\r\n");		
		ADC3_DMA_Config(samp_time, samp_delay);		//Performs configuration of GPIO/ADC/DMA
		USART_puts( "\r\nPress sw 13 to run/stop logging\r\n");
		while(1){
			if(getSWITCH(13)==0){	//Wait until user presses the sw13 to begin logging
				Delay(5);			//5ms Debounce
				if(getSWITCH(13)!=0){
					ADC_stop=0;
					break;
				}
			}
			Delay(1);		//Test switch again after 1ms
		}		

	Volt_disp_off = 0;		//Turn on the Voltage Display
	/* Start ADC3 Software Conversion */ 
	ADC3_START();
	while (1){
		curr_task = 0;
		logging = 1;		//Start logging voltage
		which_refresh = VOLT_DISP;		
		 ADC3ConvertedVoltage = ADC3ConvertedValue *3300/0xFFF;
		ConvertedVoltage = ADC3ConvertedVoltage;	//Store voltage for 7-seg display
		//Get index into ascii, then add '0' to get ascii value of digit.
		 bytes[0] = ((ADC3ConvertedVoltage/1000)%10)+'0';
		 bytes[1] = '.';
		 bytes[2] = ((ADC3ConvertedVoltage/100)%10)+'0';
		 bytes[3] = ((ADC3ConvertedVoltage/10)%10)+'0'; 
		 bytes[4] = (ADC3ConvertedVoltage%10)+'0';
		 bytes[5] = 'V';
		 bytes[6] = '\0';
		USART_puts( "\r\nVoltage: "); //Display the Voltage on the Screen (Voltage: XXXXV)
		USART_puts( bytes); 
		USART_puts( "\r\n");		
		
		/*Test against the over/under voltage*/
		 if((ADC3ConvertedVoltage > ov_Voltage || ADC3ConvertedVoltage < un_Voltage) && ADC3ConvertedVoltage!=0){
		//	ADC_Cmd(ADC3, DISABLE);		//Turn chip off - Caused the chip to stop running!!
			set_negs = 1;
			logging = 0;
			USART_puts( "Voltage has exceeded the over/under limit!!\r\n");
			USART_puts( "The chip has turned off.\r\n");
			USART_puts( "\r\nPress SW 12 to turn back on and continue.\r\n");
			while(1){
				LED_Cutoff_ON();		//Turn on Red LED 2 and 5
				Delay(5);
				while(getSWITCH(12)==0){	//Wait until user presses the sw13 to begin logging
					Delay(3);				//3ms Debounce
					if(getSWITCH(12)!=0){
						set_negs=0;
						break;
					}
				}
				if(set_negs==0) break;
			}	
			LED_OFF();		//Turn off LEDS
		}


		Delay(1000);
		if(ADC_stop){	//SW13 was pressed
			logging = 0;
			while(getSWITCH(13)==0) //Wait until the edge trigger	
				Delay(5);			
			ADC_stop=0;
			//ADC_DMACmd(ADC3, DISABLE);	//Turn chip off - found in MEMS.asm
			USART_puts( "\r\nTo continue, select an option\r\n");
			USART_puts( "A. Change to ACCEL/SPI Peripheral\r\n");
			USART_puts( "B. Continue using the ADC/DMA\r\n");
			USART_puts( "C. Restart the ADC/DMA App\r\n");
			get_string();
			if(received_string[0]>='A' && received_string[0]<='Z')	//Convert to lowercase if entered is uppercase
				received_string[0] += 32;	
			switch(received_string[0]){
				case 'a':	//ConvertedVoltage = 0;	//Set to zero so display will show 0
							mode = ACCEL_MODE;
							USART_puts("\r\nSwitching Apps...\r\n");
							Delay(1000);			//Delay 1 second for message above
							SVC_ACCEL_SPI();		//Exit from ADC/DMA demo / change to ACC demo by (SVC #1)
							mode = ADC_MODE;		//Change mode back to ADC for the display
							USART_puts("\r\nContinuing the Voltage Meter app...\r\n");
							USART_puts( "\r\nTo pause or switch peripherals, press sw 13...\r\n");
							break;
				case 'c':	ConvertedVoltage = 0;	//Set to zero so display will show 0
							mode = ADC_MODE;
							ADC_DMA_GO();			//Restart the ADC/DMA demo --> returns back to this function	
				case 'b':	break;					//Do Nothing
			}
			logging = 1;
		}
	}
	
}


/*
 * @brief: - Accelerometer/SPI main Functions and Inits 
 * @note: This function is an example of how to 
 *		  interface with the stm32f4		 
 *		  accelerometer / SPI peripheral
 */
void ACCEL_SPI_GO(){
	Acc_disp_off=1;					//Turn off the display
	which_refresh = ACCEL_DISP;		//Set the refresh display function to the Accelerometer
	curr_task = 1;					//Set current task to Accel/SPI
	
	USART_puts( "\nAccelerometer / SPI Peripheral Example\r\n");
	USART_puts( "\r\nTo begin press sw13\r\n");
	while(1){
		if(getSWITCH(13)==0){	//Wait until user presses the sw13 to begin logging
			Delay(5);			//5ms Debounce
			if(getSWITCH(13)!=0){
				ACCEL_stop=0;
				break;
			}
		}
		Delay(1);				//Poll sw13 every 1 ms
	}

	Acc_disp_off=0;				//turn on display
	USART_puts( "Configuring GPIO...\r\n");
	USART_puts( "Configuring MEMS...\r\n");
	USART_puts( "Configuring SPI...\r\n");
	USART_puts( "\r\nTo pause or switch peripherals, press sw 13...\r\n");
	my_init();					//Initialize the SPI
	Delay(30);					//Required delay for Accel chip
	MEMS_read(Buffer, 0xE9, 6);	//Get offset buffers    
	XOffset = Buffer[0];		//offsets
	YOffset = Buffer[2];

	
	while(1){
		curr_task = 1;
		which_refresh = ACCEL_DISP;		//Set the refresh display function to the Accelerometer
		Poll_Acc();
		Delay(5);
		turn_on_LED();
		if(ACCEL_stop){					//Wait until user presses the sw13 to begin logging
			while(getSWITCH(13)==0); 	//Wait until the edge trigger			
			ACCEL_stop =0;
			setChip_LOW();				//Turn chip off - found in stm32f4x_MEMS.asm
			USART_puts( "\r\nTo continue, select an option\r\n");
			USART_puts( "A. Change to ADC/DMA Peripheral\r\n");
			USART_puts( "B. Continue using the Accelerometer\r\n");
			USART_puts( "C. Restart the Accel_SPI App\r\n");
			get_string();
			if(received_string[0]>='A' && received_string[0]<='Z')	//Convert to lowercase if entered is uppercase
				received_string[0] += 32;
			switch(received_string[0]){
				case 'a':	mode = ADC_MODE;
							USART_puts("\r\nSwitching Apps...\r\n");
							Delay(1000);			//Delay 1 second for message above
							SVC_ADC_DMA();			//Assembly function to switch to task0 (ADC_DMA)
							mode = ACCEL_MODE;
							ACCEL_stop=0;
							USART_puts("\r\nContinuing the Accelerometer Interface app...\r\n");
							USART_puts( "\r\nTo pause or switch peripherals, press sw 13...\r\n");
							break;
				case 'c':	Acc_val=0;				//Get Accel ready for reset
							Acc_turn=LEVEL;
							mode = ACCEL_MODE;
							ACCEL_stop = 0;
							ACCEL_SPI_GO();			//Restart the Accel/SPI demo 
							break;
				case 'b':	setChip_HIGH();
							ACCEL_stop = 0;			//Double check that this is 0
							USART_puts( "\r\nTo pause or switch peripherals, press sw 13...\r\n");
							continue;
			}
		}
	}
}



/************************	
	ACCEL/SPI FUNCTIONS
*************************/
void Poll_Acc(){
 uint8_t temp1, temp2 = 0;
  
  if (TimingDelay != 0x00)
    TimingDelay_Decrement();
  else{
    Counter++;
    if (Counter == 10){
      Buffer[0] = 0;
      Buffer[2] = 0;

     
	  MEMS_read(Buffer, 0xE9, 6);
	
      /* Remove the offsets values from data */
      Buffer[0] -= XOffset;		//buffer[0] = X-axis
      Buffer[2] -= YOffset;		//buffer[2] = Y-axis
  
      /* Update autoreload and capture compare registers value*/
      temp1 = ABS((int8_t)(Buffer[0]));
      temp2 = ABS((int8_t)(Buffer[2]));
      TempAcceleration = MAX(temp1, temp2);

      if(TempAcceleration != 0)
      {									//"-10" cooresponds to the amount of tilt needed to change
        if ((int8_t)Buffer[0] < -10)	//Tilted backwards -- Count downwards
        	Acc_val--;
      
        if ((int8_t)Buffer[0] > 10)		//Tilted forwards -- count upwards
        {	if(Acc_val!=999) 
			Acc_val++;
		}
        if ((int8_t)Buffer[2] > 10)		//Y-Axis Tilt Left
        	Acc_turn = LEFT;
        
        if ((int8_t)Buffer[2] < -10)	//Y-Axis Tilt Right
        	Acc_turn = RIGHT;
        
        if (((int8_t)Buffer[2] <= 10) && ((int8_t)Buffer[2] >= -10))	//Y-Axis Tilt = LEVEL
        	Acc_turn = LEVEL;
       }
      Counter = 0x00;
    }
  }
}

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
    TimingDelay--;
}

void my_init(){
	SPI_GPIO_init();		//Located in stm32f4xx_MEMS.asm
	MEMS_write();			//Configure SPI communication with ACC chip
}

void MEMS_write(){	  
  /* Write value to MEMS CTRL_REG1 regsister */
	setChip_LOW();
  	send_writeAddr();		//Send the write address (0x20)
	send_config();			//Send 0xC7 which is all of the settings for the chip!
    setChip_HIGH();
}

 //Configure MEMS: Assembly Functions
void send_config(){		
  /* Loop while DR register in not emplty */
  test_TX_flag();
  /* Send Accelerometer settings through SPI peripheral
		Settings: Output Data Rate 	= 400 Hz
				  Low Power Mode	= Enabled
				  X-Y-Z Axis		= Enabled
				  Self Test			= Normal
  */ 
  send_settings();
  /* Wait to receive a Byte */
  test_RX_flag();
  /* Return the Byte read from the SPI bus */
  return_byte();
return;
}


void MEMS_read(uint8_t* pBuffer, uint8_t ReadAddr, int NumByteToRead){
  /* Set chip select Low at the start of the transmission */
  setChip_LOW();	
  
  /* Send the Address of the indexed register */
 	send_readAddr(0xE9);	
	while (get_FlagStatus(SPI1_SR) == RESET1);
	receive_byte();		
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00){
    /* Send dummy byte (0x00) to generate the SPI clock to LIS302DL (Slave device) */
	/* Send a Byte through the SPI peripheral */
	send_readAddr(0x00);
	while (get_FlagStatus(SPI1_SR) == RESET1);
	*pBuffer = (uint8_t)receive_byte();
    NumByteToRead--;
    pBuffer++;
  }  
  /* Set chip select High at the end of the transmission */ 
  setChip_HIGH();
}

int get_FlagStatus(volatile unsigned int *SPIx){
	int status=0;
	if((*SPIx & RXNE_FLAG) != 0)		//RXNE_FLAG == bit 1 in SPI->SR
		status=1;						//Rx Flag is Set
	else
		status=0;						//Rx Flag is Reset
	
return status;
}

void turn_on_LED(){
	setONES();
	switch(Acc_turn){
		case LEVEL: level(); 	  break;
		case LEFT:	turn_left();  break;
		case RIGHT:	turn_right(); break;
	}

}

void decode_Acc_val(){
	int temp = Acc_val;
	if(Acc_val>998) Acc_val = 999;	//Saturation points
	if(Acc_val<-998) Acc_val = -999;
	PrevAcc_val=Acc_val;
	if(neg_flag){					
		temp=Acc_val*-1;			//Make number positive in order to stay between 0-9
	}
	abc[2]=temp%10;					//Get ones digit
	abc[1]=(temp/10)%10;			//Get tens digit
	abc[0]=(temp/100)%10;			//Get hundreds digit
}
/******END*******/



/**********************
	USART FUNCTIONS
***********************/
/*
 * @brief: - This function is used to receive input from the serial terminal
 *			 and store the result into the global array, received_string[]
 *		   - The function will return if a max of 12 characters are entered,	
 *			 or the user hits the enter key.
 */
void get_string(){
	char ch;
	int cnt=0;
	while (1){
		while(!RxFlag_Status());	//Continue looping until USART2->SR.RXNE is set
		ch = (uint16_t)getChar();
		if( (ch != 0xA && ch!=0xD) && (cnt < 12) ){		//0xA == '\n'; 0xD == Carriage return; 12 == MAX_STR_LEN
			received_string[cnt] = ch;
			cnt++;
			continue;	//go back to while(1)
		}
		else{ 
			received_string[cnt] = '\0';
			return;		//!!Only exit from the function
		}
	}
}

void USART_puts(volatile char *s){
	while(*s){
		// wait until data register is empty
		while( !TcFlag_Status());	//Check Transmission Complete Flag
		USART2_SendData(*s);		//Send the data through USART2
		*s++;						//Increment the index of the array
	}	
}
/******END*******/




/**********************
	ADC/DMA FUNCTIONS
***********************/
void ADC3_DMA_Config(uint8_t sample_time, uint32_t sample_delay){
	DMA_config();
	ADC_config(sample_delay);			//Pass SampDelay - 5cycles
	ADC_Ch10_Enab(sample_time);			//Pass SampTime
}

void get_Voltage(){			//Function is used to get over/under voltage from user via switches and RE
	int x, i, flag1=0;
	which_refresh = SW_DISP;	//Change refresh_display function to display the switches; SW_DISP also reads the switches in systick
	testVoltage=0;				//Set voltage to 0 at start of function
	RE_reset();					//precision_cnt = 0;
while(1){
		x=RE_position();
		if(x==1 && precision_cnt>0){		//Clockwise rotation
			testVoltage/=10;				//Less precision for voltage
			precision_cnt--;
		}else if(x==-1 && precision_cnt<3){ //CounterClockwise rotation
				testVoltage*=10;			//More precision!	
				precision_cnt++;
			}
		if(SW13_stop){			//Wait until user presses the sw13 to begin logging
			Delay(3);			//3ms Debounce
			while(1){
				if(getSWITCH(13)!=0){
					SW13_stop=0;
					which_refresh = VOLT_DISP;
					return;		//!!Only exit out of function!
				}
				Delay(3);
			}
		}

	Delay(7);

	for(i=0;i<8;i++){
		if(sw_que[i] & 0x00000080){			//Mask sw_que to test sw_que[] bit 7: Action: PRESSED
			if(i==7){						//Test msTicks for > 3ms == debounced. If i == have to test with sw_qu[0]
				if(((sw_que[0]&SWITCH_MASK)==(sw_que[i]&SWITCH_MASK)) && (((sw_que[0]&TIME_MASK)-(sw_que[i]&TIME_MASK)) >= 3))	//Same switch number AND atleast a 3ms delay for debounce
					flag1=1;
			}
			if((flag1==1) || (((sw_que[i+1]&SWITCH_MASK)==(sw_que[i]&SWITCH_MASK)) && (((sw_que[i+1]&TIME_MASK)-(sw_que[i]&TIME_MASK)) >= 3))){	//Same switch number AND atleast a 3ms delay for debounce
				flag1=0;
				switch(sw_que[i]&SWITCH_MASK){
					case 0: break;
					case 1: if((testVoltage/1000)%10<9) 
								testVoltage+=1000; 
							break;
					case 2: if((testVoltage/1000)%10>0) 
								testVoltage-=1000; 
								break; 			
					case 3: if((testVoltage/100)%10<9) 
								testVoltage+=100; 
							break;							
					case 4: if((testVoltage/100)%10>0) 
								testVoltage-=100; 
							break;							
					case 5: if((testVoltage/10)%10<9) 
								testVoltage+=10; 
							break;
					case 6: if((testVoltage/10)%10>0) 
								testVoltage-=10; 
							break;							
					case 7: if((testVoltage%10)<9)
								testVoltage++; 
							break;
					case 8: if((testVoltage%10)>0)
								testVoltage--; 
							break;						
				}
			}				
		}	//else its NOT PRESSED	 	
	}
}


}





