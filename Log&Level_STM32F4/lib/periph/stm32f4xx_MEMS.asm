@;Assembly for the Accelerometer Interface
@; Author: Tony Abboud	
@;

@; --- characterize target syntax, processor
	.syntax unified				@; ARM Unified Assembler Language (UAL). 
	.thumb						@; Use thumb instructions only

	.data						@; start the _initialized_ RAM data section
								@; global? initialized? variables
	
	.bss						@;start an uninitialized RAM data section
	.align						@;pad memory if necessary to align on a word boundary for word storage 

	
	
	
@;*** definitions ***  
 @; from DM00031020.pdf Table 2. STM32F4xx register boundary addresses 
  .equ RCC_BASE, 0x40023800
  
  
@; from DM00031020.pdf Section 6.3 RCC registers (pp 168-)	!!??put these in a .include? 
  .equ RCC_APB1ENR, RCC_BASE+0x40		@;APB1 peripheral CLOCK (Enable) register; SPI, TIM, USART
  .equ RCC_APB1RSTR,RCC_BASE+0x20		@;APB1 peripheral reset register; UART, SPI, TIM
  .equ RCC_AHB1ENR,RCC_BASE+0x30		@;AHB1 peripheral CLOCK (Enable) register; DMA, GPIO
  .equ RCC_AHB1RSTR,RCC_BASE+0x10		@;AHB1 peripheral reset register; GPIOx's and DMA's
  .equ RCC_APB2ENR,RCC_BASE+0x44		@;APB2 			  CLOCK register; SPI, TIM, USART
  .equ RCC_APB2RSTR,RCC_BASE+0x24		@;APB2 			  reset register; SPI, TIM, USART
  
	.equ GPIOA_BASE, 0x40020000	@; from DM00031020.pdf Table 2. pg. 65 STM32F4xx register boundary addresses
	.equ GPIOE_BASE, 0x40021000
@; from DM00031020.pdf section 7.4 GPIO registers (pp 198-) and Table 31. GPIO register map and reset values (pp 203--)
  .equ MODER, 0x00	
  .equ OTYPER, 0x04
  .equ OSPEEDR, 0x08
  .equ PUPDR, 0x0C
  .equ IDR, 0x10
  .equ ODR, 0x14
  .equ BSRR, 0x18
  .equ LCKR, 0x1C
  .equ AFRL, 0x20
  .equ AFRH, 0x24
  
  
@;*** MACROS ***
.macro MOV_imm32 reg val		
	movw \reg,#(0xFFFF & (\val))
	movt \reg,#((0xFFFF0000 & (\val))>>16)
.endm


.macro PORTBIT_write base, pin, val
	ldr r4, =\base				@;Load in the GPIOx_BASE address
	.if (\val == 1)			
		mov r3, #1<<\pin		@;Write to lower 16 bits of BSSR (BSRRL) to set the bit
	.else	
		mov r3, #1<<(\pin +16)	@;Write to upper 16 bits of BSSR (BSRRH) to clear the bit
	.endif	
	str r3, [r4, #24]			@;Store into the BSRR
.endm

.macro SET_bit addr bit @;logical OR position 'bit' at 'addr' with 1 
	ldr r2, =\addr
	ldr r1,[r2]
	ORR r1,#(1<<\bit)
	str r1,[r2]	
.endm

.macro CLR_bit addr bit @;logical AND position 'bit' at 'addr' with 0 
	ldr r2, =\addr
	ldr r1,[r2]
	BIC r1,#(1<<\bit)
	str r1,[r2]	
.endm

.macro TST_bit addr bit	@;read 'bit' at addr, return bit value in bit0 of r0 and 'Z' flag set/clear if bit=0/1
	MOV_imm32 r3,(\addr)
	ldr r3,[r3]
	ands r3,#(1<<\bit)
	lsr r3,#\bit
.endm


.macro PORTBIT_config bit, GPIOx_BASE, MODE, OTYPE, OSPEED, PUPD, AF

	ldr r2, =\GPIOx_BASE		@;Load base address

	ldr r1,[r2,#MODER]
	bic r1,(3 << (2*\bit))		@;Clear bits 
	orr r1,(\MODE << (2*\bit)) 	@;Set the specified mode at location of bit
	str r1,[r2,#MODER]

	ldr r1,[r2,#OTYPER]
	bic r1,(1 << \bit)
	orr r1,(\OTYPE << (1*\bit)) 
	str r1,[r2,#OTYPER]
	
	ldr r1,[r2,#OSPEEDR]
	bic r1,(3 << (2*\bit))
	orr r1,(\OSPEED << (2*\bit))
	str r1,[r2,#OSPEEDR]
                
	ldr r1,[r2,#PUPDR]
	bic r1,(3 << (2*\bit))
	orr r1,(\PUPD << (2*\bit))	
	str r1,[r2,#PUPDR]

	.iflt (\bit - 8)		 @;use AFRLR (AFR[0]) for configuration; Pin 0-7
		ldr r1,[r2,#AFRL]
		bic r1,(0xF << (4*\bit))
		orr r1,(\AF << (4*\bit))	
		str r1,[r2,#AFRL]

	.else 					@;use AFRH (AFR[1]) for configuration; Pin 8-15
		ldr r1,[r2,#AFRH]
		bic r1,(0xF << (4*(\bit-8)))
		orr r1,(\AF << (4*(\bit-8)))	
		str r1,[r2,#AFRH]
	.endif

.endm 

.macro PORTBIT_init mode, base, pin		
	.ifeq \mode-0						@;mode == STD_OUTPIN
		ldr r2, =\base					@;Load in the GPIOx_BASE
		ldr r1, [r2, #MODER]			@;Load contents of base into R1
		bic r1, r1, #3<<(\pin<<1)		@;Clear specified bits 
		str r1, [r2, #MODER]			@;GPIOx->MODER (32)
		ldr r1, [r2, #MODER]			@;
		orr r1, r1, #1<<(\pin<<1)		@;Set the mode == OUTPUT
		str r1, [r2, #MODER] 
	
		ldr r1, [r2, #OTYPER]			@;GPIOx->OTYPER: (16)	
		bic r1, r1, #1<<\pin			@;set pin to 0 == output Push-Pull(reset state)
		str r1, [r2, #OTYPER]			
		
		ldr r1, [r2, #OSPEEDR]			@;GPIOx->OSPEEDR: (32) 
		bic r1, r1, #3<<(\pin<<1)		@;clear bits
		str r1, [r2, #OSPEEDR]
		ldr r1, [r2, #OSPEEDR]			
		orr r1,(2 << (2*\pin))			@;Set speed to 50Mhz (fast speed)
		str r1, [r2, #OSPEEDR]
			
		ldr r1, [r2, #PUPDR]			@;GPIOx->PUPDR: (32)
		bic r1, r1, #3<<(\pin<<1)		@;Clear bits
		str r1, [r2, #PUPDR]
		ldr r1, [r2, #PUPDR]
		orr r1, r1, #1<<(\pin<<1)		@;set to 01 == Pull up
		str r1, [r2, #PUPDR]			@;end PUPDR 
		.else
			.ifeq \mode-1						@;mode == STD_INPIN
				ldr r2, =\base					@;Load in the GPIOx_BASE
				ldr r1, [r2, #MODER]			
				bic r1, r1, #3<<(\pin<<1)		@;Clear bits: Sets to Input
				str r1, [r2, #MODER]
				
				ldr r1, [r2, #OSPEEDR]			@;OSPEEDR   
				bic r1, r1, #3<<(\pin<<1)		
				str r1, [r2, #OSPEEDR]
				ldr r1, [r2, #OSPEEDR]			
				orr r1,(2 << (2*\pin))			@;Set speed to 50Mhz (fast speed)
				str r1, [r2, #OSPEEDR]				
				
				ldr r1, [r2, #PUPDR]			@;PUPDR
				bic r1, r1, #3<<(\pin<<1)		@;Clear: No Pull Up
				str r1, [r2, #PUPDR]			
				.else
					.ifeq \mode-2					@;mode == PULLUP_INPIN
						ldr r2, =\base				@;Load in the GPIOx_BASE
						ldr r1, [r2, #MODER]			
						bic r1, r1, #3<<(\pin<<1)	@;Clear bits: Sets to Input
						str r1, [r2, #MODER]
						
						ldr r1, [r2, #OSPEEDR]			@;OSPEEDR   
						bic r1, r1, #3<<(\pin<<1)		
						str r1, [r2, #OSPEEDR]
						ldr r1, [r2, #OSPEEDR]			
						orr r1,(2 << (2*\pin))			@;Set speed to 50Mhz (fast speed)
						str r1, [r2, #OSPEEDR]
						
						ldr r1, [r2, #PUPDR]
						bic r1, r1, #3<<(\pin<<1)	@;Clear bits: no pull-up
						str r1, [r2, #PUPDR]
						ldr r1, [r2, #PUPDR]
						orr r1, r1, #1<<(\pin<<1)	@;Set bits: Enable Pull-Up
						str r1, [r2, #PUPDR]		
					.endif
			.endif
	.endif
.endm



@; --- begin code memory
	.text						@;start the code section



@;SPI1
@;void SPI_GPIO_init(); Written based on, void LIS302DL_LowLevel_Init(void), in stm32f4_discovery_lis302dl.c
	.global SPI_GPIO_init		
	.thumb_func
SPI_GPIO_init:
	SET_bit RCC_APB2ENR, 12		@;Enable clock for the SPI1 periph (bit 12 of APB2ENR)  
	SET_bit RCC_AHB1ENR, 1		@;Enable clock for SCK, MOSI, and MISO (GPIOA == bit 1 of AHB1ENR) 
	SET_bit RCC_AHB1ENR, 4		@;Enable clock for CS, INT1, INT2, GPIO Clock (GPIOE == bit 1 of AHB1ENR)
	
	@;Configure PA5 (SCK), PA6 (MISO), PA7 (MOSI) as SPI1 AF
	@;			   bit, GPIOx_BASE,	MODE, OTYPE,  OSPEED,  PUPD,   AF	
	PORTBIT_config 	5,	GPIOA_BASE,	2,		0,		2,		2,		5		@; mode=2 (AF), PUPD=2 (Pull down), AF=5 (SPI1) 
	PORTBIT_config 	6,	GPIOA_BASE,	2,		0,		2,		2,		5		@; mode=2 (AF), PUPD=2 (Pull down), AF=5 (SPI1) 
	PORTBIT_config 	7,	GPIOA_BASE,	2,		0,		2,		2,		5		@; mode=2 (AF), PUPD=2 (Pull down), AF=5 (SPI1) 

	SET_bit RCC_APB2RSTR, 12	@;Force APB2 Reset on SPI1	(pg. 236 DM00031020.pdf)
	CLR_bit RCC_APB2RSTR, 12	@;Release APB2 reset on SPI1
	

	.include "lib/periph/stm32f4xx_SPI_registers.inc"		@;!!Register addresses for SPI
	
@;/******* SPI configuration ********/	
@;direction=0, NSS=0x200, Firstbit=0, BaudRate_prescaler=0x8, SPI_mode=0x104, CPOL=0, CPHA=0, CRC_Poly=7 
	MOV_imm32 r2,absSPI1_CR1			@;Move address of SPI1->CR1 into r2
	ldr r1, [r2]						@;Load contents of SPI1->CR1
	orrs r1, r1, 0x030C					@;Set bits to configuration above
	str r1, [r2]						@;Save SPI->CR1
  	CLR_bit absSPI1_I2SCFGR, 11			@;Activiate the SPI mode (Reset I2SMOD bit 11)
	MOV_imm32 r2,absSPI1_CRCPR			@;Move address of SPI1->CRCPR into r2
	mov r1, #7							
	str r1, [r2]						@;Write to SPI->CRCPR = 7; (CRC_Polynomial)

@;Enable SPI1 peripheral
	SET_bit absSPI1_CR1, 6				@;Set SPI->CR1 SPE (SPI Enable, bit 6)

@;Configure PE3 for Accel Chip Select
	@;			mode  Addr.    pin
	PORTBIT_init 0, GPIOE_BASE,	3		@; mode=1 (Output), PUPD=0 (NONE), AF=0 (NONE) 

@;Set Chip Select High
	PORTBIT_write GPIOE_BASE, 3, 1		@;Set GPIOE pin 3; GPIOE->BSRRL
  
@;Configure PE0 (INT1) and PE1 (INT2) to detect Interrupts 
 	PORTBIT_init 1,	GPIOE_BASE,	0
 	PORTBIT_init 1,	GPIOE_BASE,	1
bx lr


@;/***Configure MEMS: data rate, power mode, full scale, self test and axes *****/
	.global configure_MEMS		@;uint8_t configure_MEMS();
	.thumb_func
configure_MEMS:
	@;define Control Register (CTRL_REG1) settings, from stm32f4_discovery_lis302dl.h
	.equ ODRATE, 	(1<<7)  @;Output Data Rate, 400Hz
	.equ PMODE, 	(1<<6)	@;Low Power Mode Active
	.equ FULL_SC,	(0<<5)	@;Full Scale
	.equ STP,		(0<<4)	@;Self Test Normal
	.equ STM,		(0<<3)	@;Self Test Normal
	.equ ZAXISEN, 	(1<<2)	@;Z-Axis Enable
	.equ YAXISEN, 	(1<<1)	@;Y-Axis Enable
	.equ XAXISEN, 	(1<<0)	@;X-Axis Enable
	.equ ACC_settings, ODRATE|PMODE|FULL_SC|STM|STP|ZAXISEN|YAXISEN|XAXISEN
	MOV_imm32 r0, (ACC_settings)
bx lr


@;Equivallent to LIS302DL_Write(); Assumes number of bytes to send == 1
	.global send_writeAddr		@;Sends a Byte through the SPI interface and return the Byte received from the SPI bus.
	.thumb_func
send_writeAddr:
	TST_bit absSPI1_SR, 1		@;Test bit 1, SPI->SR[1] (TXE Flag)				
	cmp r0, #0					
	IT eq						@;TX Flag == 0?
	beq send_writeAddr			@;Yes - Branch back to 1, test again; Can get stuck here!
								@;No -  Transmission is complete
@;Send Write Address
	MOV_imm32 r2, absSPI1_DR	@;Load SPI1 Data Register Addres
	mov r1, #0x20				@;0x20 -> WriteAddress
	strh r1, [r2]				@;SPI1->DR = WriteAddress
	b 2f
	
@;Wait to Recieve Data
2:	TST_bit absSPI1_SR, 0		@;Test bit 0, SPI->SR[0] (RXNE Flag)	
	cmp r0, #0						
	IT eq						@;TX Flag == 0?
	beq 2b						@;Yes - Branch back to 2, test again; Can get stuck here!
								@;No  - Recieved all data (Dont do anything with it here)
	MOV_imm32 r2, absSPI1_DR	@;Get data into r0
	ldrh r0, [r2]				@;Data not used here
bx lr


	.global test_TX_flag	
	.thumb_func
test_TX_flag:
	TST_bit absSPI1_SR, 1		@;Test bit 1, SPI->SR[1] (TXE Flag)
	cmp r3, #0
	IT eq						@;TX Flag == 0?
	beq test_TX_flag			@;Yes - Branch back to 1, test again; Can get stuck here!
bx lr							@;No -  Transmission is complete

	.global test_RX_flag	
	.thumb_func
test_RX_flag:
	TST_bit absSPI1_SR, 0		@;Test bit 0, SPI->SR[0] (RXNE Flag)
	cmp r3, #0
	IT eq						@;RX Flag == 0?
	beq test_RX_flag			@;Yes - Branch back to 1, test again; Can get stuck here!
bx lr							@;No -  Receiving is complete	

	
/*	;Accelerometer Settings
;	ODRATE, 	(1<<7)  @;Output Data Rate, 400Hz
;	PMODE, 		(1<<6)	@;Low Power Mode Active
;	FULL_SC,	(0<<5)	@;Full Scale
;	STP,		(0<<4)	@;Self Test Normal
;	STM,		(0<<3)	@;Self Test Normal
;	ZAXISEN, 	(1<<2)	@;Z-Axis Enable
;	YAXISEN, 	(1<<1)	@;Y-Axis Enable
;	XAXISEN, 	(1<<0)	@;X-Axis Enable
;	Acc_Settings = 0xC7
*/
	.global send_settings
	.thumb_func
send_settings:
	MOV_imm32 r2, absSPI1_DR	@;Load SPI1 Data Register Address
	mov r1, #0xC7				@;0xC7 -> Accelerometer Settings
	strb r1, [r2]				@;SPI1->DR = 0xC7
bx lr

	.global return_byte
	.thumb_func
return_byte:
	MOV_imm32 r2, absSPI1_DR	@;Load SPI1 DR register Addr
	ldrb r0, [r2]				@;Read data from DR register into R0
bx lr



	.global setChip_LOW			@;Turns the Acc chip OFF, blocking transmission of data
	.thumb_func
setChip_LOW:
		PORTBIT_write GPIOE_BASE, 3, 0		@;Reset GPIOE pin 3; GPIOE->BSRRH
bx lr	
	
	.global setChip_HIGH		@;Turns the accelerometer chip ON allowing transmission of data 
	.thumb_func
setChip_HIGH:
		PORTBIT_write GPIOE_BASE, 3, 1		@;Reset GPIOE pin 3; GPIOE->BSRRH
bx lr

	.global send_readAddr		@;Checks the TXE flag, then sends the readAddress, 0xE9
	.thumb_func
send_readAddr:
	MOV_imm32 r3, absSPI1_SR		@;Move address of SPI1->SR into r3
	ldr r3, [r3]					@;Load SPI->SR 
	ands r3, r3, #0x2				@;AND SR with SPI_I2S_FLAG_TXE (0x2) 
	cmp r3, #0					
	IT eq							@;TX Flag == 0?
	beq send_readAddr				@;Yes - Branch back to 1, test again; Can get stuck here!
									@;No -  Transmission is complete
	@;SEND DATA						
	MOV_imm32 r3, absSPI1_DR		@;Load SPI1 Data Register Address
	strh r0, [r3]					@;SPI1->DR = ReadAddress, 
bx lr

		.global receive_byte
		.thumb_func
receive_byte:
	MOV_imm32 r3, absSPI1_DR		@;Get data from SPI1 Data Register
	ldrh r0, [r3]
bx lr



