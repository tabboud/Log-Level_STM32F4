REM  make.bat: compile stm32f4 project (ADC/SPI demos) 
set path=.\;C:\yagarto_gcc472\bin;

REM assemble with '-g' omitted where we want to hide things in the AXF
arm-none-eabi-as -g -mcpu=cortex-m4 -o aDemo.o CortexM4asmOps_01.asm
arm-none-eabi-as -g -mcpu=cortex-m4 -o aMems.o lib/periph/stm32f4xx_MEMS.asm
arm-none-eabi-as -g -mcpu=cortex-m4 -o aUSART2.o lib/periph/stm32f4xx_USART2.asm
arm-none-eabi-as -g -mcpu=cortex-m4 -o aADC_DMA.o lib/periph/stm32f4xx_ADC_DMA.asm
arm-none-eabi-as -g -mcpu=cortex-m4 -o aTasking.o lib/periph/stm32f4xx_Tasking.asm
arm-none-eabi-as -g -mcpu=cortex-m4 -o aStartup.o SimpleStartSTM32F4_01.asm

REM compiling C
arm-none-eabi-gcc -I./  -c -mthumb -O0 -g -mcpu=cortex-m4 -save-temps system_stm32f4xx.c -o cSys32.o
arm-none-eabi-gcc -I./  -c -mthumb -O0 -g -mcpu=cortex-m4 -save-temps Blinky.c -o cMain.o
arm-none-eabi-gcc -I./  -c -mthumb -O0 -g -mcpu=cortex-m4 -save-temps LED.c -o cLED.o

REM linking
arm-none-eabi-gcc -nostartfiles -g -Wl,--no-gc-sections -Wl,-Map,Blinky.map -Wl,-T linkBlinkySTM32F4_01.ld -oBlinky.elf aStartup.o aDemo.o aMems.o aUSART2.o aADC_DMA.o aTasking.o cLED.o cMain.o cSys32.o -lgcc

REM hex file
arm-none-eabi-objcopy -O ihex Blinky.elf Blinky.hex

REM AXF file
copy Blinky.elf Blinky.AXF
pause

REM list file
arm-none-eabi-objdump -S  Blinky.axf >Blinky.lst
