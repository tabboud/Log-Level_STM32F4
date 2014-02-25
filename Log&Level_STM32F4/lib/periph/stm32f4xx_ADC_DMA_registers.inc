@;stm32f4xx_ADC_DMA_registers.inc: absolute and relative ADC DMA register addresses 
@; derived from ST RM0090 reference manual DM00031020.pdf pages 53,243-246

@;ADC3 register group base address
.equ ADC3_base		,0x40012200
.equ ADC_com_base	,0x40012300

@;ADC3 absolute register addresses
.equ absADC3_SR				,ADC3_base+0x00
.equ absADC3_CR1			,ADC3_base+0x04
.equ absADC3_CR2			,ADC3_base+0x08
.equ absADC3_SMPR1			,ADC3_base+0x0C
.equ absADC3_SMPR2			,ADC3_base+0x10
.equ absADC3_JOFR1			,ADC3_base+0x14
.equ absADC3_JOFR2			,ADC3_base+0x18
.equ absADC3_JOFR3			,ADC3_base+0x1C
.equ absADC3_JOFR4			,ADC3_base+0x20
.equ absADC3_HTR			,ADC3_base+0x24
.equ absADC3_LTR			,ADC3_base+0x28
.equ absADC3_SQR1			,ADC3_base+0x2C
.equ absADC3_SQR2			,ADC3_base+0x30
.equ absADC3_SQR3			,ADC3_base+0x34
.equ absADC3_JSQR			,ADC3_base+0x38
.equ absADC3_JDR1			,ADC3_base+0x3C
.equ absADC3_JDR2			,ADC3_base+0x40
.equ absADC3_JDR3			,ADC3_base+0x44
.equ absADC3_JDR4			,ADC3_base+0x48
.equ absADC3_DR				,ADC3_base+0x4C

@;ADC Common Register addresses
.equ absADC_CSR				,ADC_com_base+0x00
.equ absADC_CCR				,ADC_com_base+0x04
.equ absADC_CDR				,ADC_com_base+0x08


@;DMA register group base addresses
.equ DMA2_base     	,0x40026400

@; DMA2 absolute register addresses
.equ absDMA2_LISR 	,DMA2_base+0x0000
.equ absDMA2_HISR	,DMA2_base+0x0004 
.equ absDMA2_LIFCR	,DMA2_base+0x0008 
.equ absDMA2_HIFCR	,DMA2_base+0x000C 

@;DMA channel 2 stream 0; USED
.equ absDMA2_S0CR	,DMA2_base+0x0010 
.equ absDMA2_S0NDTR	,DMA2_base+0x0014 
.equ absDMA2_S0PAR	,DMA2_base+0x0018 
.equ absDMA2_S0M0AR	,DMA2_base+0x001C 
.equ absDMA2_S0M1AR	,DMA2_base+0x0020 
.equ absDMA2_S0FCR	,DMA2_base+0x0024 

