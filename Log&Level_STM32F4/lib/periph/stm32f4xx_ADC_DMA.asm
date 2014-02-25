@;Assembly for the ADC/DMA Peripherals
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

.include "lib/periph/stm32f4xx_ADC_DMA_registers.inc"		@;!!Register addresses for SPI

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
	.equ GPIOC_BASE, 0x40020800	
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
				bic r1, r1, #3<<(\pin<<1)		@;Clear bits: 
				str r1, [r2, #MODER]
				@;!!ADDED for Analog input
				ldr r1, [r2, #MODER]
				orr r1, r1, #3<<(\pin<<1)		@;Set input to ANALOG mode
				str r1, [r2, #MODER]
				@;End
				
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

@;Replaces config();
	.global DMA_config		@;void DMA_config();
	.thumb_func
DMA_config:
@;Step 1: Enable ADC3, DMA2 and GPIO Clocks**********************
	SET_bit RCC_AHB1ENR, 2			@;enable clock for GPIOC 
	SET_bit RCC_APB2ENR, 10			@;enable clock for ADC3
	SET_bit RCC_AHB1ENR, 22			@;enable clock for DMA2
	@;Restart DMA2
	SET_bit RCC_AHB1RSTR,22			@;reset DMA2			
	CLR_bit RCC_AHB1RSTR,22			@;end DMA2 reset		

@;Step 2: DMA2 Channel 2 Stream 0 Configuration******************
	.equ CHSEL,		(2<<25)	@;channel = 2 
	.equ MBURST,	(0<<23)	@;single transfer from memory
	.equ PBURST,	(0<<21)	@;single-transfer to peripheral 
	.equ CT,		(0<<19)	@;current target of transfer is first buffer		
	.equ DBM,		(0<<18)	@;Disable double buffer mode 
	.equ PL,		(2<<16)	@;DMA priority level -- High
	.equ PINCOS,	(0<<15)	@;peripheral increment -- none
	.equ MSIZE,		(1<<13)	@;memory data size = half-word
	.equ PSIZE,		(1<<11)	@;peripheral data size = half-word
	.equ MINC,		(0<<10)	@;Disable, memory address pointer does not increment 1 MSIZE per transfer
	.equ PINC,		(0<<9)	@;peripheral address pointer does not increment
	.equ CIRC,		(1<<8)	@;circular mode is enabled -- in reality this is 'don't care' when DBM=1
	.equ DIR,		(0<<6)	@;transfer direction is Peripheral to memory
	.equ PFCTRL,	(0<<5)	@;DMA is the flow controller (necessary to get circular and/or double buffer?)
	.equ TCIE,		(0<<4)	@;transfer-complete interrupt is disabled
	.equ HTIE,		(0<<3)	@;half-transfer interrupt is disabled
	.equ TEIE,		(0<<2)	@;transfer-error interrupt is disabled
	.equ DMEIE,		(0<<1)	@;direct-mode-error interrupt is disabled
	.equ EN,		(0<<0)	@;stream enable = 0 while changing CR bits as protection 
	.equ DMA2_S0CR_settings,CHSEL|MBURST|PBURST|CT|DBM|PL|PINCOS|MSIZE|PSIZE|MINC|PINC|CIRC|DIR|PFCTRL|TCIE|HTIE|TEIE|DMEIE|EN
	MOV_imm32 r1,(DMA2_S0CR_settings)
	MOV_imm32 r2,absDMA2_S0CR
	str r1,[r2]

	CLR_bit absDMA2_S0FCR, 2	@;Clear DMDIS bit		;Disables FIFO mode
	CLR_bit absDMA2_S0FCR, 1	@;Clear FTH[1:0] bits
	CLR_bit absDMA2_S0FCR, 0
	SET_bit absDMA2_S0FCR, 1	@;Set FTH = 0x01 --> FIFOThreshold halfFull
	SET_bit absDMA2_S0NDTR, 1	@;Set Buffer Size=1; DMA2->DMA_S0NDTR = 1; 
	@;Set the Peripheral base address
	MOV_imm32 r1,0x4001224C			@;ADC3_DR_ADDRESS
	MOV_imm32 r2,absDMA2_S0PAR		@;where DMA2 will send its data
	str r1,[r2]
	@;Set Memory Base Address
	.extern ADC3ConvertedValue
	ldr r1, =ADC3ConvertedValue		@;Get address of ADC3ConvertedValue from other .c file
	MOV_imm32 r2, absDMA2_S0M0AR
	str r1, [r2]
@;Step 3: Enable DMA2 Channel 2 Stream 0****************************
	SET_bit absDMA2_S0CR, 0	
	
@;Step 4: GPIO (PC0) Config: Analog input, No Pull-up****************
	PORTBIT_init 1, GPIOC_BASE, 0
bx LR




	.global ADC_config		@;void ADC_config(int SampleDelay)
	.thumb_func
ADC_config:
@;Step 1: ADC Common Init********************************************
	@;ADC_Mode 		= Independent 	(MULTI	= 0)
	@;ADC_Prescaler = Div2 			(ADCPRE	= 0)
	@;DMA_Access	= Disabled 		(DMA	= 0)
	@;Sample_Delay	= USER ENTERED	(DELAY	= 0, 1, 2, 3) (5 to 8 Cycles)
	ldr r2, =0xFFFC30E0		@;CR_CLEAR_MASK
	MOV_imm32 r3, absADC_CCR
	ands r2, r2, r3
	str r2, [r3]			@;Clear MULTI, DELAY, DMA, and ADCPRE bits, Sets values for above values
	ldr r2, [r3]			@;Load ADC_CCR into r1
	orrs r2, r2, r0			@;Set the DELAY[]
	str r2, [r3]			

@;Step 2: ADC3 Init********************************************
	@;Resolution			= 12b		(RES	= 0)
	@;ScanConvMode			= Disable	(SCAN 	= 0)
	@;ContinuousConvMode	= Enable	(CONT 	= 1)
	@;ExtTrigConvEdge		= None		(EXTEN 	= 0)
	@;ExtTrigConv			= 0			(EXTSEL = 0)
	@;DataAlign				= Right		(ALIGN	= 0)
	@;NbrOfConversion		= 1			(L		= 0)
	
	@;ADC3->CR1 Configuration
	ldr r2, =0xFCFFFEFF			@;CR1_CLEAR_MASK
	MOV_imm32 r3, absADC3_CR1
	ands r2, r2, r3
	str r2, [r3]				@;Clear RES and SCAN bits; No need to set any bits since RES and SCAN are set to 0
	
	@;ADC3->CR2 Configuration
	ldr r2, =0xC0FFF7FD			@;CR2_CLEAR_MASK
	MOV_imm32 r3, absADC3_CR2
	ands r2, r2, r3
	str r2, [r3]				@;Clear CONT, EXTEN, EXTSEL,and ALIGN bits
	SET_bit absADC3_CR2, 1		@;Set CONT = 1
	
	@;ADC3->SQR1 Configuration
	ldr r2, =0xFF0FFFFF			@;SQR1_L_RESET
	MOV_imm32 r3, absADC3_SQR1
	ands r2, r2, r3
	str r2, [r3]				@;Clear L bits; sets NbrOfConversions to 1
bx LR


	.global ADC_Ch10_Enab		@;ADC_Ch10_config(int SampleTime)
	.thumb_func
ADC_Ch10_Enab:
@;Step 1: ADC Channel 10 settings
	CLR_bit absADC3_SMPR1, 0	@;Clear SMP registers cooresponding to Channel 10
	CLR_bit absADC3_SMPR1, 1
	CLR_bit absADC3_SMPR1, 2
	MOV_imm32 r2, absADC3_SMPR1
	ldr r1, [r2]
	orrs r1, r1, r0				@;Set the SampleTime given by the user
	str r1, [r2]
	
	ldr r1, =0xFFFFFFE0			@;Clear Mask for SQR3 Register
	MOV_imm32 r2, absADC3_SQR3
	ldr r3, [r2]
	ands r1, r1, r3				@;Apply clear mask
	str r1, [r2]
	SET_bit absADC3_SQR3 1		@;Set RANK = bit 1 = 0x2
	SET_bit absADC3_SQR3 3		@;Set RANK = bit 1 = 0x2

	
@;Step 2: Enable DMA Request After Last Transfer
	SET_bit absADC3_CR2, 9		@;Set ADC3->CR2 DDS = 1 ; DMA Disable Selection
@;Step 3: ADC3_DMA Enable
	SET_bit absADC3_CR2, 8		@;Set ADC3->CR2 DMA = 1 
@;Step 4: ADC3 Enable
	SET_bit absADC3_CR2, 0		@;Set ADC3->CR2 ADON = 1 	
bx LR
	
	
	.global ADC3_START		@;void ADC3_START();
	.thumb_func
ADC3_START:
	SET_bit absADC3_CR2, 30		@;Set ADC3->CR2.SWSTART = 1;
bx LR


	.global ADC_DeInit		@;void ADC_DeInit();
	.thumb_func
ADC_DeInit:
	SET_bit RCC_APB2RSTR, 8			@;Reset ADC peripheral	
	CLR_bit RCC_APB2RSTR, 8			@;Release ADC reset
bx LR



	.global DMA_DeInit		@;void DMA_DeInit()
	.thumb_func
DMA_DeInit:
@;Step 1: Disable/Reset DMA2 Stream0
	MOV_imm32 r1, absDMA2_S0CR
	ldr r2, [r1]
	ldr r3, =0xFFFFFFFE
	ands r3, r3, r2
	str r3, [r1]
	mov r3, #0
	str r3, [r1]
@;Step 2: Reset NDTR, PAR, M0AR, M1AR, FCR, FIFO, and LIFCR registers
	MOV_imm32 r1, absDMA2_S0NDTR
	mov r2, #0
	str r2, [r1]					@;RESET NDTR REG.
	MOV_imm32 r1, absDMA2_S0PAR
	mov r2, #0
	str r2, [r1]					@;RESET PAR REG.
	MOV_imm32 r1, absDMA2_S0M0AR
	mov r2, #0
	str r2, [r1]					@;RESET M0AR REG.
	MOV_imm32 r1, absDMA2_S0M1AR
	mov r2, #0
	str r2, [r1]					@;RESET M1AR REG.
	MOV_imm32 r1, absDMA2_S0FCR
	MOV_imm32 r2, 0x00000021
	str r2, [r1]					@;RESET FCR REG.
	MOV_imm32 r1, absDMA2_LIFCR
	mov r2, #0x3D
	str r2, [r1]					@;RESET LIFCR REG.	
bx LR
	
	
	

	
	
	
