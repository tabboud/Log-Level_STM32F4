@;Assembly used for Task Switching
@;	Includes:
@;		PendSV_Handler
@;		SVC_Handler
@;		PendSV EN 
@;		SVC calls

@; --- characterize target syntax, processor
	.syntax unified				@; ARM Unified Assembler Language (UAL). 
	.thumb						@; Use thumb instructions only

	.data						@; start the _initialized_ RAM data section
								@; global? initialized? variables
	
	.bss						@;start an uninitialized RAM data section
	.align						@;pad memory if necessary to align on a word boundary for word storage 


@; --- begin code memory
	.text						@;start the code section

	.equ ICSR,0xE000ED04		@;Interrupt Control and State Register
	.equ PENDSVSET,28			@; bit location in ICSR to set PendSV interrupt pending
	
	.global PendSV_Handler 		@;void PendSV_Handler(void); triggered by SVC call to switch tasks 
	.thumb_func
PendSV_Handler:					@;****Context Switching Code*****
@;---Save Current Context---	
	MRS r0, PSP					@;Get current Process stack pointer 
	stmdb r0!, {r4-r11}			@;Save R4 to R11 in the Tasks stack
	.extern curr_task			
	ldr r1, =curr_task
	ldr r2, [r1]				@;Get the current task value
	.extern PSP_array
	ldr r3, =PSP_array			@;Get the PSP_array address
	str r0, [r3, r2, LSL #2]	@;Save the PSP value into the PSP array
@;---Load Next Context---
	.extern next_task			
	ldr r4, =next_task
	ldr r4, [r4]				@;Get next task value
	str r4, [r1]				@;Set curr_task = next_task
	ldr r0, [r3, r4, LSL #2]	@;Load PSP value from PSP_array
	ldmia r0!, {r4-r11}			@;Load R4 to R11 into next task stack
	
	MSR PSP, r0					@;Set PSP to next task
	bx LR						
	
	.ltorg					@;start a local literal pool here	
	

	.global SVC_Handler		@;void SVC_Handler; Initiates a task switch
	.thumb_func
SVC_Handler:
	ldr r1,=ICSR			@;Interrupt Control and State Register
	mov r0,(1<<PENDSVSET)	@;PendSV set bit 	
	str r0,[r1]				@;Enable PendSV interrupt
	
	MRS r0, PSP				@;Load PSP value into R0
	LDR r1, [r0, #24]		@;Get stacked PC value from stack frame
	ldrb r0, [r1, #-2]		@;Get first byte of the SVC instruction (SVC number is in R0)
	.extern next_task
	ldr r1, =next_task		@;Get next task addr.
	cmp r0, #0
	IT eq			@;SVC #0?
	beq 1f			@;Yes - Branch to assign next_task = 0;
	mov r2, #1		@;No  - next_task = 1;
	str r2, [r1]	@;(next_task -> Accel_SPI)
	bx lr
1: 	mov r2, #0
	str r2, [r1]	@;next_task = 0; (next_task -> ADC_DMA)
	bx lr

	
	.global SVC_ACCEL_SPI		@;Switch to task 1 (Accel_SPI)
	.thumb_func
SVC_ACCEL_SPI:
	SVC #1
bx lr	

	.global SVC_ADC_DMA			@;Switch to task 0 (ADC_DMA)
	.thumb_func
SVC_ADC_DMA:
	SVC #0
bx lr
