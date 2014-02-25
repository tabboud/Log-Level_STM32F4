@;Assembly for the USART2: PA2 (Tx Pin)
@;	Functions              PA3 (Rx Pin)
@;	
@;	
@;

@; --- characterize target syntax, processor
	.syntax unified				@; ARM Unified Assembler Language (UAL). 
	.thumb						@; Use thumb instructions only

	.data						@; start the _initialized_ RAM data section
								@; global? initialized? variables
	
	.bss						@;start an uninitialized RAM data section
	.align						@;pad memory if necessary to align on a word boundary for word storage 

	
	
	
@;*** definitions ***  //!!Put into a .include file?

.include "lib/periph/stm32f4xx_USART2_registers.inc"		@;!!Register addresses for SPI

 @; from DM00031020.pdf Table 2. STM32F4xx register boundary addresses 
  .equ RCC_BASE, 0x40023800
   
@; from DM00031020.pdf Section 6.3 RCC registers (pp 168-)	!!??put these in a .include? 
  .equ RCC_APB1ENR, RCC_BASE+0x40		@;APB1 peripheral CLOCK (Enable) register; SPI, TIM, USART
  .equ RCC_APB1RSTR,RCC_BASE+0x20		@;APB1 peripheral reset register; UART, SPI, TIM
  .equ RCC_AHB1ENR,RCC_BASE+0x30		@;AHB1 peripheral CLOCK (Enable) register; DMA, GPIO
  .equ RCC_AHB1RSTR,RCC_BASE+0x10		@;AHB1 peripheral reset register; GPIOx's and DMA's
  .equ RCC_APB2ENR,RCC_BASE+0x44		@;APB2 			  CLOCK (Enable) register; SPI, TIM, USART
  .equ RCC_APB2RSTR,RCC_BASE+0x24		@;APB2 			  reset register; SPI, TIM, USART
  
	.equ GPIOA_BASE, 0x40020000	@; from DM00031020.pdf Table 2. pg. 65 STM32F4xx register boundary addresses
	.equ GPIOE_BASE, 0x40021000
@; from DM00031020.pdf section 7.4 GPIO registers (pp 198-) and Table 31. GPIO register map and reset values (pp 203--)
  .equ MODER, 0x00	@;!!should redo with 'rel-' prefix
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
	@;Can replace with a .include from my file
.macro MOV_imm32 reg val		@;example of use: MOV_imm32 r0,0x12345678 !!note: no '#' on immediate value
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

@;WORKS!						@;Equivallent to USART_SendData in USART_puts();
	.global USART2_SendData		@;void USART2_SendData(voltile char)		
	.thumb_func					@;R0 holds the char
USART2_SendData:
@;Step 1: check USART2->SR & (1<<6): Check TC (Transmission Complete) flag
@;Loop here until it is set!
	TST_bit absUSART2_SR, 6		@;Test TC (Transmission Complete) flag in USART2->SR; Returns ANDS value into R3			
	cmp r3, #0					@;TC == 0?
	IT eq
	beq USART2_SendData			@;Yes, continue looping
								@;No break loop
@;Step 2: Send the data now
@;  USARTx->DR = (Data & (uint16_t)0x01FF);
	ldr r1, =0x000001FF
	ands r0, r0, r1
	MOV_imm32 r1, absUSART2_DR		
	str r0, [r1]				@;Store data (char) into USART2->DR
bx LR



@;WORKS!	
@;Initialize the ports PA2, PA3, and configure the USART
	.global USART2_init
	.thumb_func
USART2_init:
@;Step 1: Enable clocks
	SET_bit RCC_APB1ENR, 17			@;enable clock for USART2 (pin 17)
	SET_bit RCC_AHB1ENR, 0			@;enable clock for GPIOA 

@;Step 2: GPIO Init: Configure PA2 (TX), PA3 (RX) as USART2 AF
	@;			   bit, GPIOx_BASE,	MODE, OTYPE,  OSPEED,  PUPD,   AF	
	PORTBIT_config 	2,	GPIOA_BASE,	2,		0,		2,		1,		7		@; mode=2 (AF), OSPEED=2 (50Mhz), PUPD=1 (Pull Up), AF=7 (USART2) 
	PORTBIT_config 	3,	GPIOA_BASE,	2,		0,		2,		1,		7		@; mode=2 (AF), OSPEED=2 (50Mhz), PUPD=1 (Pull Up), AF=7 (USART2) 

@;Step 3: USART2 Configurations
	@;USART2->CR2 config - Set Stop bit to 1 bit (0x00) default
	CLR_bit absUSART2_CR2, 12	@;Clear STOP[13:12] bits
	CLR_bit absUSART2_CR2, 13	
	
	@;USART2->CR1 config
	CLR_bit absUSART2_CR1, 12	@;Clear M bit		; Defines Wordlength = 8 bits
	CLR_bit absUSART2_CR1, 10	@;Clear PCE bit		; Defines Parity Val = None
	CLR_bit absUSART2_CR1, 9	@;Clear PS bit		; Defines Parity Val = None
	CLR_bit absUSART2_CR1, 3	@;Clear TE bit		;NOT NEEDED, just set values
	CLR_bit absUSART2_CR1, 2	@;Clear RE bit
	
	SET_bit absUSART2_CR1, 3	@;Transmitter Enable,  TE bit
	SET_bit absUSART2_CR1, 2	@;Receiver Enable ,    RE bit
	
	@;USART2->CR3 config
	CLR_bit absUSART2_CR3, 9	@;Clear CTSE bit	; Defines No Hardware flow control
	CLR_bit absUSART2_CR3, 8	@;Clear RTSE bit	; Defines No Hardware flow control
	
	@;USART2->BRR config (Sets Baud rate to 9600)
	MOV_imm32 r0, absUSART2_BRR	@;Load address of BRR register
	ldr r1, =0x00000683
	strh r1, [r0]
	    @;apbclock = RCC_ClocksStatus.PCLK1_Frequency;   --->  apbclock = 0x00f42400 == 16 000 000
		@;integerdivider = ((25 * apbclock) / (4 * (USART_InitStruct->USART_BaudRate)));   ---> integerdivider = 0x17D78400 / 0x9600 = 0x28B0
		@;USARTx->BRR = (uint16_t)tmpreg; ==> tmpreg = 0x683
	
@;Step 4: Enable the USART2 Peripheral
	SET_bit absUSART2_CR1, 13	@;Set the UE (USART Enable) bit in the USART2->CR1 Register to Enable	
bx LR
	
		
	.global RxFlag_Status		@;Check USART2->SR.RXNE bit 5 and return value
	.thumb_func
RxFlag_Status:
	TST_bit absUSART2_SR, 5
	mov r0, r3					@;Move value of bit into r0 to return
bx LR


	.global TcFlag_Status		@;Check USART2->SR.TC bit 6 and return value
	.thumb_func
TcFlag_Status:
	TST_bit absUSART2_SR, 6
	mov r0, r3					@;Move value of bit into r0 to return
bx LR

	
	.global getChar		@;char getChar();	//Returns the character read in from the user.
	.thumb_func
getChar:
	MOV_imm32 r0, absUSART2_DR	@;Load addr of USART2->DR
	ldr r0, [r0]
	ldrh r1, =0x01FF
	ands r0, r0, r1
bx LR
	
	
	
