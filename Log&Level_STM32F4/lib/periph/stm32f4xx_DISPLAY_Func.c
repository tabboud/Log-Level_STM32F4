/*----------------------------------------------------------------------------
 * Name:  Display Functions for ENEE440 project
 * 
 *----------------------------------------------------------------------------*/


/***************************
Variables used by get_sw()
***************************/	
int prev_sw_num=0;
int saved_LR;			//Location of saved LR from branch and links; Also used in display_this();
int sw_que[8] = {0};	//Switch que: Holds msTicks, switch number, and un/pressed.
int que_index=0;		//Index into the que array, 0-7

/**************************
	Display Functions
***************************/
void set_pattern(int a, int b, int c, int d, int e, int f, int g, int h, int task, char *arr){	//Assigns values to the display buffer
	int i=0;			//int task: decides CA or AN: 2=CA, 3=AN
	if(task == 3)	
		i=8;				//AN: store into a[8] - a[15]
			if(a)			//if(a==1) set to 0x0A
				arr[i] = 0x0A;				//0x0A == Turn OFF
			else
				arr[i] = 0;					//0 == Turn ON
			if(b)
				arr[i+1] = 0x0A;
			else
				arr[i+1] = 0;
			if(c)
				arr[i+2] = 0x0A;
			else
				arr[i+2] = 0;
			if(d)												//DISPLAY4
				arr[i+3] = 0x0A;	
			else
				arr[i+3] = 0;
			if(e)												//DISPLAY3
				arr[i+4] = 0x0A;
			else
				arr[i+4] = 0;
			if(f)												//COLON
				arr[i+5] = 0x0A;
			else
				arr[i+5] = 0;
			if(g)												//DISPLAY2
				arr[i+6] = 0x0A;
			else
				arr[i+6] = 0;
			if(h)												//DISPLAY1
				arr[i+7] = 0x0A;		
			else
				arr[i+7] = 0;
		
return;
}

void decode_pattern(char *buffer){
	switch(digit_value){	//Decodes pattern for CATHODE
		case 0: set_pattern(0,0,0,0,0,0,1,1,2,buffer); break;								//digit_value: number you want to display		
		case 1: set_pattern(1,0,0,1,1,1,1,1,2,buffer); break;  
		case 2: set_pattern(0,0,1,0,0,1,0,1,2,buffer); break;
		case 3: set_pattern(0,0,0,0,1,1,0,1,2,buffer); break;
		case 4: set_pattern(1,0,0,1,1,0,0,1,2,buffer); break;
		case 5: set_pattern(0,1,0,0,1,0,0,1,2,buffer); break;
		case 6: set_pattern(0,1,0,0,0,0,0,1,2,buffer); break;
		case 7: set_pattern(0,0,0,1,1,0,1,1,2,buffer); break;
		case 8: set_pattern(0,0,0,0,0,0,0,1,2,buffer); break;
		case 9: set_pattern(0,0,0,0,1,0,0,1,2,buffer); break;
		case NEG_SIGN: set_pattern(1,1,1,1,1,1,0,1,2,buffer); break;
		case BLANK_SCREEN: set_pattern(1,1,1,1,1,1,1,1,2,buffer); break;
	}
	
	switch(display){	//Decodes pattern for ANODE
		case 1: set_pattern(1,1,1,1,1,1,1,0,3,buffer); break;
		case 2: set_pattern(1,1,1,1,1,1,0,1,3,buffer); break;
		case 3: set_pattern(1,1,1,1,0,1,1,1,3,buffer); break;
		case 4: set_pattern(1,1,1,0,1,1,1,1,3,buffer); break;		
	}
}
/****END*****/

/******************************
Accelerometer Refresh display 
******************************/
void refresh_ACCELdisplay(){  
	disp_index=-1;				//Used in the assembly to count through the array.
	DISPLAY_off();

	if(Acc_val<0) 
		neg_flag=1;					//Set flag for decode_Acc_val(). Enables modulation correctly and sets the negative sign

	if(Acc_val!=PrevAcc_val)
		decode_Acc_val();			//Decode the Acc value into 3 digits by modulation. Also handles saturation
	if(display==1){
		if(neg_flag){				//RE_val negative?
			neg_flag=0;				//Set flag back to 0
			digit_value = NEG_SIGN;		//set to display the '-'
		}else{digit_value = BLANK_SCREEN;}
		decode_pattern(displaybuf);
		setONES();
		display_this(displaybuf);	 //Implement a function to write the CA/AN patterns to the display buf
		//display=4;		
		DISPLAY_on();			//!!Does not return here, it returns to DISPLAY_on() below, then decrements, which is reason for if statement below
	}else{
		digit_value = abc[display-2];
		if(Acc_disp_off)					
			digit_value = BLANK_SCREEN;		//Turn off display
		decode_pattern(displaybuf);
		setONES();
		display_this(displaybuf);			//Implement a function to write the CA/AN patterns to the display buf
		DISPLAY_on();
		display--;
	}
	if(display<1) display = 4;
}
/****END*****/


/******************************
Functions to implement the 
user input on Over/Under Voltage
and Switch Display
******************************/
void read_switches(){
	if(precision_cnt==0)			//Just test switch 7 and 8
		if(sw_num==9) sw_num=7;
	if(precision_cnt==1)			//Test switch 5,6,7,8
		if(sw_num==9) sw_num=5;
	if(precision_cnt==2)			//Test switch 3,4,5,6,7,8
		if(sw_num==9) sw_num=3;
									//Otherwise test switch 1-8
	if(que_index == 8) que_index = 0;	//Set que back to 0th position to overwrite
	if(sw_num == 9) sw_num = 1;			//Set sw_num back to test switch 1

	if(getSWITCH(sw_num) == 0){			//Switch is pressed
		if(prev_sw_num != sw_num){		//Suppresses repeated storage of the same switch still pressed
			sw_que[que_index++] = load_SWque(msTicks, sw_num, 1);	//Load pressed into switch que, DONT increment sw_num since we need to wait until debounced
			prev_sw_num = sw_num;
		}
	}
	else{
		sw_que[que_index++] = load_SWque(msTicks, sw_num, 0);		//Load unpressed into switch que		
		sw_num++;				//Increment to test next switch
		prev_sw_num = 0;
	}
}

void refresh_SWdisplay(){  
	static int j=4;
	disp_index=-1;
	read_switches();
	DISPLAY_off();
	display = j;
	switch(j){
		case 1: digit_value = (testVoltage/1000)%10	; break; 
		case 2: digit_value = (testVoltage/100)%10	; break;
		case 3: digit_value = (testVoltage/10)%10	; break;
		case 4: digit_value = (testVoltage)%10		; break;
		}

	decode_pattern(displaybuf);
	setONES();
	if(precision_cnt==1 && j==3)	write_PD();
	if(precision_cnt==2 && j==2)	write_PD();
	if(precision_cnt==3 && j==1)	write_PD();
	if(j==4)
		j=1;
	else
		j++;
	display_this(displaybuf);			//Implement a function to write the CA/AN patterns to the display buf
	DISPLAY_on();

}
/*****END*****/


/******************************
Functions used for the voltage display
*******************************/
void decode_voltage(){
	Prev_voltage=ConvertedVoltage;
	
	abc2[0]=(ConvertedVoltage/1000)%10;			//Get thousands digit
	abc2[1]=(ConvertedVoltage/100)%10;			//Get hundreds digit
	abc2[2]=(ConvertedVoltage/10)%10;			//Get tens digit
	abc2[3]=ConvertedVoltage%10;					//Get ones digit
}

void refresh_VOLTdisplay(){
	disp_index=-1;				//Used in the assembly to count through the array.
	DISPLAY_off();
	
	if(ConvertedVoltage!=Prev_voltage && msTicks%100==0)
		decode_voltage();			//Decode the RE value into 3 digits by modulation. Also handles saturation
	if(set_negs==0)
		digit_value = abc2[display-1];
	else
		digit_value = NEG_SIGN;
	if(Volt_disp_off)
			digit_value = BLANK_SCREEN;	//Turn off display
	decode_pattern(displaybuf);
	setONES();
	if(display == 1 && set_negs==0 && Volt_disp_off==0) 
		write_PD();
	display_this(displaybuf);			//Implement a function to write the CA/AN patterns to the display buf
	DISPLAY_on();
	display--;
	if(display<1) display = 4;
}
/******END******/