@;stm32f4xx_USART2_registers.inc: absolute and relative USART register addresses 
@; derived from ST RM0090 reference manual DM00031020.pdf pages 67,906

@;USART2 register base address
.equ USART2_base     	,0x40004400

@;USART2 relative register offsets
.equ relUSART2_SR	,0x0000
.equ relUSART2_DR	,0x0004 
.equ relUSART2_BRR	,0x0008 
.equ relUSART2_CR1	,0x000C 
.equ relUSART2_CR2	,0x0010 
.equ relUSART2_CR3	,0x0014 
.equ relUSART2_GTPR	,0x0018 

@; USART2 absolute register addresses
.equ absUSART2_SR 	,USART2_base+0x00
.equ absUSART2_DR 	,USART2_base+0x04 
.equ absUSART2_BRR	,USART2_base+0x08 
.equ absUSART2_CR1	,USART2_base+0x0C 
.equ absUSART2_CR2	,USART2_base+0x10 
.equ absUSART2_CR3	,USART2_base+0x14 
.equ absUSART2_GTPR	,USART2_base+0x18 

