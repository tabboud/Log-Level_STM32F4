**********************************************
Log & Level
**********************************************
Author: Tony Abboud
Description:
	Logging Voltage Meter - ADC3/DMA2/GPIO/USART2
	Accelerometer Display - SPI1/GPIO/USART2
	API					  - USART2
**********************************************


CONTENTS of Package	
-------------------------	

stm32f4_PROJECT directory:	
		+lib directory:
			-All source and header files added/linked to main.
			-core directory
				*Contains header files used by stm32f4xx.h
			-periph directory
				*Contains all of the .asm, include, and header files needed to implement the peripherals
		+Blinky.c
			-Location of main() and API for each peripheral application
		+SimpleStartSTM32F4_01.asm
			-Startup assembly code for stm32f4 board
		+stm32f4_P24v04_definitions.asm
			-Assembly macros tailored to the P24v04 Expansion Board
		+CortexM4asmOps_01.asm
			-Assembly code which contains initilizations, look-up tables for, and essential code for displaying and decoding functions
		+LED.c/LED.h
			-Initialization and use of the onboard (stm32f4) LEDS
		+system_stm32f4xx.c
			-System initialization code including SystemcoreClock and crystal frequencies
		+make.bat/clean.bat
			-make and clean batch files to compile/link the project and to clean the project after running

Documented_code directory:
		-Contains two well commented peripheral codes
		-STM32 ST-Link Utility 
			-Directions for use are in the UserManual.pdf
**********************************************


HOW TO USE/RUN PROJECT
-----------------------
	
Software Needed:
	1. STM32 ST-LINK Utility or Keil uVision 4.xx
	2. Serial Terminal (PuTTY, RealTerm, ...)
	
Compile/Link Project: 
	1. Navigate to the main project directory (i.e. Abboud_ENEE440_Project/)
	2. Double click on clean.bat file to remove any previous execution files. Press "Enter" when prompted
	2. Double click on make.bat file to compile and link all of the header and source files

Program the stm32f4 board: (Using STM32 ST-LINK Utility)
	1. Ensure that the Expansion Board is connected correctly
	2. Connect the usb mini to the STLINKV2 (Top of the stm32f4) usb connection and the other side to the PC
	3. Open up STM32 ST-LINK Utility
	4. Press File/Open_File, then navigate to the blinky.hex file created from the make.bat
	5. Double click on the .hex file to open
	6. Once the file is loaded into the ST-Link Utility, press Target/Program_&_Verify/Start
	7. The program will then be loaded onto the stm32f4 board.
	8. The board is now programmed
Run the Project Live: (Using PuTTY)
	1. Disconnect the usb mini and connect it to the JNEX connector on the left side of the Expansion board
	2. Open up the PuTTY terminal and navigate to the serial connection
	3. On the PuTTY configuration choose "Serial" from the left side category
	4. Change the "serial line to connect to" to the COM port that the JNEX has connected to
		**Look in the Device Manager to see which COM port is connected
	5. Change the serial configurations to 96008N1
		Speed(baud)	: 9600
		Data bits	: 8
		Parity bits	: None
		Stop Bits	: 1
	6. Press "Open"
	7. Once the serial terminal comes up, press the reset button the the board
	8. You will then be prompted with the display message
	9. You are now running the program live
	10.To quit the program simply disconnect the usb

	
Logging Voltmeter: ADC/DMA
	1. Choose 'A' from the main menu and press enter.
	2. Select settings for Time and Delay and press enter
	3. Use sw 1-8 to enter an over/under voltage. Press sw13 to store voltage
	4. Press sw 13 to start logging
	5. Press sw 13 to pause at any time	

Accelerometer: SPI
	1. Choose 'B' from the main menu and press enter
	2. Press sw 13 to start
	3. Tilt the board forward to count up
	4. Tilt the board backwards to count down
	5. Tilt the board to the right to display a green left arrow
	6. Tilt the board to the left to display a red right arrow
	7. Press sw 13 to pause at any time	

****************************************************	
	
	
