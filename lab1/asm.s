/*
 * asm.s
 *
 * author: Furkan Cayci 
 *
 * description: Added the necessary stuff for turning on the green LED on the 
 *   G031K8 Nucleo board. Mostly for teaching.
 */


.syntax unified
.cpu cortex-m0plus
.fpu softvfp
.thumb


/* make linker see this */
.global Reset_Handler

/* get these from linker script */
.word _sdata
.word _edata
.word _sbss
.word _ebss


/* define peripheral addresses from RM0444 page 57, Tables 3-4 */
.equ RCC_BASE,         (0x40021000)          // RCC base address
.equ RCC_IOPENR,       (RCC_BASE   + (0x34)) // RCC IOPENR register offset

.equ GPIOA_BASE, (0x50000000) //GPIOA Base Adress
.equ GPIOB_BASE, (0x50000400) //GPIOB Base Adress

.equ GPIOA_MODER,      (GPIOA_BASE + (0x00)) // GPIOA MODER register offset
.equ GPIOB_MODER,      (GPIOB_BASE + (0x00)) // GPIOB MODER register offset

.equ GPIOA_ODR,        (GPIOA_BASE + (0x14)) // GPIOA ODR register offset
.equ GPIOB_ODR,        (GPIOB_BASE + (0x14)) // GPIOB ODR register offset

/* vector table, +1 thumb mode */
.section .vectors
vector_table:
	.word _estack             /*     Stack pointer */
	.word Reset_Handler +1    /*     Reset handler */
	.word Default_Handler +1  /*       NMI handler */
	.word Default_Handler +1  /* HardFault handler */
	/* add rest of them here if needed */


/* reset handler */
.section .text
Reset_Handler:
	/* set stack pointer */
	ldr r0, =_estack
	mov sp, r0

	/* initialize data and bss
	 * not necessary for rom only code
	 * */
	bl init_data
	/* call main */
	bl main
	/* trap if returned */
	b .


/* initialize data and bss sections */
.section .text
init_data:

	/* copy rom to ram */
	ldr r0, =_sdata
	ldr r1, =_edata
	ldr r2, =_sidata
	movs r3, #0
	b LoopCopyDataInit

	CopyDataInit:
		ldr r4, [r2, r3]
		str r4, [r0, r3]
		adds r3, r3, #4

	LoopCopyDataInit:
		adds r4, r0, r3
		cmp r4, r1
		bcc CopyDataInit

	/* zero bss */
	ldr r2, =_sbss
	ldr r4, =_ebss
	movs r3, #0
	b LoopFillZerobss

	FillZerobss:
		str  r3, [r2]
		adds r2, r2, #4

	LoopFillZerobss:
		cmp r2, r4
		bcc FillZerobss

	bx lr


/* default handler */
.section .text
Default_Handler:
	b Default_Handler


/* main function */
.section .text
main:
	//////Enable RCC_IOPENR For A//////
	ldr r0,=RCC_IOPENR
	ldr r1, [r0]

	movs r2, #0x01 //0000 0001, For enabling RCC mask
	orrs r1, r1, r2
	str r1, [r0] //Enable RCC mask
	///////////////////////////////////

	///////////GPIOx_MODER/////////
	ldr r0,=GPIOA_MODER
	ldr r1,[r0]

	ldr r2,=0x30000
	mvns r2,r2
	ands r1, r1, r2
	ldr r2, =0x10000
	orrs r1,r1,r2

	str r1, [r0]
	///////////////////////////////

	///////////GPIOA_ODR//////////
	ldr r0,=GPIOA_ODR
	ldr r1, [r0]

	ldr r2,=0x100
	orrs r1, r1, r2

	str r1, [r0]


	/* for(;;); */
	b .

	/* this should never get executed */
	nop











