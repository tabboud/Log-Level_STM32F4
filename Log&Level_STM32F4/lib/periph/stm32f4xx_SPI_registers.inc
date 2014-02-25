@;stm32f4xx_SPI_registers.inc: absolute and relative SPI register addresses 
@; derived from ST RM0090 reference manual DM00031020.pdf pages 63,906

@;DMA register group base addresses
.equ SPI1_base     	,0x40013000

@;SPI register offsets
.equ relSPI1_CR1 	 ,0x0000
.equ relSPI1_CR2 	 ,0x0004 
.equ relSPI1_SR  	 ,0x0008 
.equ relSPI1_DR  	 ,0x000C 
.equ relSPI1_CRCPR   ,0x0010 
.equ relSPI1_RXCRCR  ,0x0014 
.equ relSPI1_TXCRCR  ,0x0018 
.equ relSPI1_I2SCFGR ,0x001C 
.equ relSPI1_I2SPR   ,0x0020 

@; SPI1 absolute register addresses
.equ absSPI1_CR1 	 ,SPI1_base+0x00
.equ absSPI1_CR2 	 ,SPI1_base+0x04 
.equ absSPI1_SR  	 ,SPI1_base+0x08 
.equ absSPI1_DR  	 ,SPI1_base+0x0C 
.equ absSPI1_CRCPR   ,SPI1_base+0x10 
.equ absSPI1_RXCRCR  ,SPI1_base+0x14 
.equ absSPI1_TXCRCR  ,SPI1_base+0x18 
.equ absSPI1_I2SCFGR ,SPI1_base+0x1C 
.equ absSPI1_I2SPR   ,SPI1_base+0x20 

