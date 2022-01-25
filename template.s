// STM32F4 Discovery - Assembly template for CO LAB course
//
// Based on: https://github.com/fcayci/stm32f4-assembly
//
// Turns on an LED attached to GPIOD Pin 12
// We need to enable the clock for GPIOD and set up pin 12 as output.
//
// We also measure delay by using SysTick Timer, coresponding code is included
//
// Start with enabling thumb 32 mode since Cortex-M4 do not work with arm mode
// Unified syntax is used to enable good of the both words...

// Make sure to run arm-none-eabi-objdump.exe -d prj1.elf to check if
// the assembler used proper instructions. (Like ADDS)

.thumb
.syntax unified
.cpu cortex-m4

//.arch armv7e-m

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////
// Definitions section. Define all the registers and
// constants here for code readability.

// Constants
.equ     LEDDELAY,      4000
// By default 16MHz internal HSI clock is enabled
// Internal loop takes 4 cycles

// Register Addresses
// You can find the base addresses for all peripherals from Memory Map section
// RM0090 on page 64. Then the offsets can be found on their relevant sections.

// RCC   base address is 0x40023800
//   AHB1ENR register offset is 0x30
.equ     RCC_AHB1ENR,   0x40023830 // RCC AHB1 peripheral clock reg (page 180)

// GPIOD base address is 0x40020C00
.equ     GPIOD_BASE,   0x40020C00 // GPIOD base address)
//   MODER register offset is 0x00
.equ     GPIOD_MODER,   0x00 // GPIOD port mode register (page 281)
//   ODR   register offset is 0x14
.equ     GPIOD_ODR,     0x14 // GPIOD output data register (page 283)
//   BSSR   register offset is 0x18
.equ     GPIOD_BSSR,     0x18 // GPIOD port set/reset register (page 284)

//Absolute addresses
// .equ     GPIOD_MODER,   0x40020C00 // GPIOD port mode register (page 281)
// .equ     GPIOD_ODR,     0x40020C14 // GPIOD output data register (page 283)
// .equ     GPIOD_BSSR,    0x40020C18 // GPIOD port set/reset register (page 284)

// SysTick Timer definitions
.equ     SCS, 			0xe000e000
.equ     SCS_SYST_CSR,	0x10	// Control/Status register
.equ     SCS_SYST_RVR,	0x14	// Value to countdown from
.equ     SCS_SYST_CVR,	0x18	// Current value

.equ	 SYSTICK_RELOAD_1MS,	15999  //1 msec at 16MHz ...  16 000 000 / 500 - 1


// Values for BSSR register - pin 12
.equ     LEDs_ON,       0x0000F000
.equ     LEDs_OFF,   	0xF0000000


@ -- Zero initialized data ---
.section .data
LEDSTAT: .word 0

// Start of text section
.section .text
///////////////////////////////////////////////////////////////////////////////
// Vectors
///////////////////////////////////////////////////////////////////////////////
// Vector table start
// Add all other processor specific exceptions/interrupts in order here
	.long    __StackTop                 // Top of the stack. from linker script
	.long    _start +1                  // reset location, +1 for thumb mode
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler

  /* External Interrupts */
  .word     WWDG_IRQHandler                   /* Window WatchDog              */
  .word     PVD_IRQHandler                    /* PVD through EXTI Line detection */
  .word     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */
  .word     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */
  .word     FLASH_IRQHandler                  /* FLASH                        */
  .word     RCC_IRQHandler                    /* RCC                          */
  .word     EXTI0_IRQHandler                  /* EXTI Line0                   */
  .word     EXTI1_IRQHandler                  /* EXTI Line1                   */
  .word     EXTI2_IRQHandler                  /* EXTI Line2                   */
  .word     EXTI3_IRQHandler                  /* EXTI Line3                   */
  .word     EXTI4_IRQHandler                  /* EXTI Line4                   */
  .word     DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */
  .word     DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */
  .word     DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */
  .word     DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */
  .word     DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */
  .word     DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */
  .word     DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */
  .word     ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */
  .word     CAN1_TX_IRQHandler                /* CAN1 TX                      */
  .word     CAN1_RX0_IRQHandler               /* CAN1 RX0                     */
  .word     CAN1_RX1_IRQHandler               /* CAN1 RX1                     */
  .word     CAN1_SCE_IRQHandler               /* CAN1 SCE                     */
  .word     EXTI9_5_IRQHandler                /* External Line[9:5]s          */
  .word     TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */
  .word     TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */
  .word     TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
  .word     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */
  .word     TIM2_IRQHandler                   /* TIM2                         */
  .word     TIM3_IRQHandler                   /* TIM3                         */
  .word     TIM4_IRQHandler                   /* TIM4                         */
  .word     I2C1_EV_IRQHandler                /* I2C1 Event                   */
  .word     I2C1_ER_IRQHandler                /* I2C1 Error                   */
  .word     I2C2_EV_IRQHandler                /* I2C2 Event                   */
  .word     I2C2_ER_IRQHandler                /* I2C2 Error                   */
  .word     SPI1_IRQHandler                   /* SPI1                         */
  .word     SPI2_IRQHandler                   /* SPI2                         */
  .word     USART1_IRQHandler                 /* USART1                       */
  .word     USART2_IRQHandler                 /* USART2                       */
  .word     USART3_IRQHandler                 /* USART3                       */
  .word     EXTI15_10_IRQHandler              /* External Line[15:10]s        */
  .word     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */
  .word     OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */
  .word     TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */
  .word     TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */
  .word     TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
  .word     TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */
  .word     DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */
  .word     FSMC_IRQHandler                   /* FSMC                         */
  .word     SDIO_IRQHandler                   /* SDIO                         */
  .word     TIM5_IRQHandler                   /* TIM5                         */
  .word     SPI3_IRQHandler                   /* SPI3                         */
  .word     UART4_IRQHandler                  /* UART4                        */
  .word     UART5_IRQHandler                  /* UART5                        */
  .word     TIM6_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */
  .word     TIM7_IRQHandler                   /* TIM7                         */
  .word     DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */
  .word     DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */
  .word     DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */
  .word     DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */
  .word     DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */
  .word     ETH_IRQHandler                    /* Ethernet                     */
  .word     ETH_WKUP_IRQHandler               /* Ethernet Wakeup through EXTI line */
  .word     CAN2_TX_IRQHandler                /* CAN2 TX                      */
  .word     CAN2_RX0_IRQHandler               /* CAN2 RX0                     */
  .word     CAN2_RX1_IRQHandler               /* CAN2 RX1                     */
  .word     CAN2_SCE_IRQHandler               /* CAN2 SCE                     */
  .word     OTG_FS_IRQHandler                 /* USB OTG FS                   */
  .word     DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */
  .word     DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */
  .word     DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */
  .word     USART6_IRQHandler                 /* USART6                       */
  .word     I2C3_EV_IRQHandler                /* I2C3 event                   */
  .word     I2C3_ER_IRQHandler                /* I2C3 error                   */
  .word     OTG_HS_EP1_OUT_IRQHandler         /* USB OTG HS End Point 1 Out   */
  .word     OTG_HS_EP1_IN_IRQHandler          /* USB OTG HS End Point 1 In    */
  .word     OTG_HS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */
  .word     OTG_HS_IRQHandler                 /* USB OTG HS                   */
  .word     DCMI_IRQHandler                   /* DCMI                         */
  .word     0                                 /* CRYP crypto                  */
  .word     HASH_RNG_IRQHandler               /* Hash and Rng                 */
  .word     FPU_IRQHandler                    /* FPU                          */


/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/
   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler

   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler

   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,Default_Handler

   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,Default_Handler

   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,Default_Handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler

   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,Default_Handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler

   .weak      SysTick_Handler
//   .thumb_set SysTick_Handler,Default_Handler

   .weak      WWDG_IRQHandler
   .thumb_set WWDG_IRQHandler,Default_Handler

   .weak      PVD_IRQHandler
   .thumb_set PVD_IRQHandler,Default_Handler

   .weak      TAMP_STAMP_IRQHandler
   .thumb_set TAMP_STAMP_IRQHandler,Default_Handler

   .weak      RTC_WKUP_IRQHandler
   .thumb_set RTC_WKUP_IRQHandler,Default_Handler

   .weak      FLASH_IRQHandler
   .thumb_set FLASH_IRQHandler,Default_Handler

   .weak      RCC_IRQHandler
   .thumb_set RCC_IRQHandler,Default_Handler

   .weak      EXTI0_IRQHandler
   .thumb_set EXTI0_IRQHandler,Default_Handler

   .weak      EXTI1_IRQHandler
   .thumb_set EXTI1_IRQHandler,Default_Handler

   .weak      EXTI2_IRQHandler
   .thumb_set EXTI2_IRQHandler,Default_Handler

   .weak      EXTI3_IRQHandler
   .thumb_set EXTI3_IRQHandler,Default_Handler

   .weak      EXTI4_IRQHandler
   .thumb_set EXTI4_IRQHandler,Default_Handler

   .weak      DMA1_Stream0_IRQHandler
   .thumb_set DMA1_Stream0_IRQHandler,Default_Handler

   .weak      DMA1_Stream1_IRQHandler
   .thumb_set DMA1_Stream1_IRQHandler,Default_Handler

   .weak      DMA1_Stream2_IRQHandler
   .thumb_set DMA1_Stream2_IRQHandler,Default_Handler

   .weak      DMA1_Stream3_IRQHandler
   .thumb_set DMA1_Stream3_IRQHandler,Default_Handler

   .weak      DMA1_Stream4_IRQHandler
   .thumb_set DMA1_Stream4_IRQHandler,Default_Handler

   .weak      DMA1_Stream5_IRQHandler
   .thumb_set DMA1_Stream5_IRQHandler,Default_Handler

   .weak      DMA1_Stream6_IRQHandler
   .thumb_set DMA1_Stream6_IRQHandler,Default_Handler

   .weak      ADC_IRQHandler
   .thumb_set ADC_IRQHandler,Default_Handler

   .weak      CAN1_TX_IRQHandler
   .thumb_set CAN1_TX_IRQHandler,Default_Handler

   .weak      CAN1_RX0_IRQHandler
   .thumb_set CAN1_RX0_IRQHandler,Default_Handler

   .weak      CAN1_RX1_IRQHandler
   .thumb_set CAN1_RX1_IRQHandler,Default_Handler

   .weak      CAN1_SCE_IRQHandler
   .thumb_set CAN1_SCE_IRQHandler,Default_Handler

   .weak      EXTI9_5_IRQHandler
   .thumb_set EXTI9_5_IRQHandler,Default_Handler

   .weak      TIM1_BRK_TIM9_IRQHandler
   .thumb_set TIM1_BRK_TIM9_IRQHandler,Default_Handler

   .weak      TIM1_UP_TIM10_IRQHandler
   .thumb_set TIM1_UP_TIM10_IRQHandler,Default_Handler

   .weak      TIM1_TRG_COM_TIM11_IRQHandler
   .thumb_set TIM1_TRG_COM_TIM11_IRQHandler,Default_Handler

   .weak      TIM1_CC_IRQHandler
   .thumb_set TIM1_CC_IRQHandler,Default_Handler

   .weak      TIM2_IRQHandler
   .thumb_set TIM2_IRQHandler,Default_Handler

   .weak      TIM3_IRQHandler
   .thumb_set TIM3_IRQHandler,Default_Handler

   .weak      TIM4_IRQHandler
   .thumb_set TIM4_IRQHandler,Default_Handler

   .weak      I2C1_EV_IRQHandler
   .thumb_set I2C1_EV_IRQHandler,Default_Handler

   .weak      I2C1_ER_IRQHandler
   .thumb_set I2C1_ER_IRQHandler,Default_Handler

   .weak      I2C2_EV_IRQHandler
   .thumb_set I2C2_EV_IRQHandler,Default_Handler

   .weak      I2C2_ER_IRQHandler
   .thumb_set I2C2_ER_IRQHandler,Default_Handler

   .weak      SPI1_IRQHandler
   .thumb_set SPI1_IRQHandler,Default_Handler

   .weak      SPI2_IRQHandler
   .thumb_set SPI2_IRQHandler,Default_Handler

   .weak      USART1_IRQHandler
   .thumb_set USART1_IRQHandler,Default_Handler

   .weak      USART2_IRQHandler
   .thumb_set USART2_IRQHandler,Default_Handler

   .weak      USART3_IRQHandler
   .thumb_set USART3_IRQHandler,Default_Handler

   .weak      EXTI15_10_IRQHandler
   .thumb_set EXTI15_10_IRQHandler,Default_Handler

   .weak      RTC_Alarm_IRQHandler
   .thumb_set RTC_Alarm_IRQHandler,Default_Handler

   .weak      OTG_FS_WKUP_IRQHandler
   .thumb_set OTG_FS_WKUP_IRQHandler,Default_Handler

   .weak      TIM8_BRK_TIM12_IRQHandler
   .thumb_set TIM8_BRK_TIM12_IRQHandler,Default_Handler

   .weak      TIM8_UP_TIM13_IRQHandler
   .thumb_set TIM8_UP_TIM13_IRQHandler,Default_Handler

   .weak      TIM8_TRG_COM_TIM14_IRQHandler
   .thumb_set TIM8_TRG_COM_TIM14_IRQHandler,Default_Handler

   .weak      TIM8_CC_IRQHandler
   .thumb_set TIM8_CC_IRQHandler,Default_Handler

   .weak      DMA1_Stream7_IRQHandler
   .thumb_set DMA1_Stream7_IRQHandler,Default_Handler

   .weak      FSMC_IRQHandler
   .thumb_set FSMC_IRQHandler,Default_Handler

   .weak      SDIO_IRQHandler
   .thumb_set SDIO_IRQHandler,Default_Handler

   .weak      TIM5_IRQHandler
   .thumb_set TIM5_IRQHandler,Default_Handler

   .weak      SPI3_IRQHandler
   .thumb_set SPI3_IRQHandler,Default_Handler

   .weak      UART4_IRQHandler
   .thumb_set UART4_IRQHandler,Default_Handler

   .weak      UART5_IRQHandler
   .thumb_set UART5_IRQHandler,Default_Handler

   .weak      TIM6_DAC_IRQHandler
   .thumb_set TIM6_DAC_IRQHandler,Default_Handler

   .weak      TIM7_IRQHandler
   .thumb_set TIM7_IRQHandler,Default_Handler

   .weak      DMA2_Stream0_IRQHandler
   .thumb_set DMA2_Stream0_IRQHandler,Default_Handler

   .weak      DMA2_Stream1_IRQHandler
   .thumb_set DMA2_Stream1_IRQHandler,Default_Handler

   .weak      DMA2_Stream2_IRQHandler
   .thumb_set DMA2_Stream2_IRQHandler,Default_Handler

   .weak      DMA2_Stream3_IRQHandler
   .thumb_set DMA2_Stream3_IRQHandler,Default_Handler

   .weak      DMA2_Stream4_IRQHandler
   .thumb_set DMA2_Stream4_IRQHandler,Default_Handler

   .weak      ETH_IRQHandler
   .thumb_set ETH_IRQHandler,Default_Handler

   .weak      ETH_WKUP_IRQHandler
   .thumb_set ETH_WKUP_IRQHandler,Default_Handler

   .weak      CAN2_TX_IRQHandler
   .thumb_set CAN2_TX_IRQHandler,Default_Handler

   .weak      CAN2_RX0_IRQHandler
   .thumb_set CAN2_RX0_IRQHandler,Default_Handler

   .weak      CAN2_RX1_IRQHandler
   .thumb_set CAN2_RX1_IRQHandler,Default_Handler

   .weak      CAN2_SCE_IRQHandler
   .thumb_set CAN2_SCE_IRQHandler,Default_Handler

   .weak      OTG_FS_IRQHandler
   .thumb_set OTG_FS_IRQHandler,Default_Handler

   .weak      DMA2_Stream5_IRQHandler
   .thumb_set DMA2_Stream5_IRQHandler,Default_Handler

   .weak      DMA2_Stream6_IRQHandler
   .thumb_set DMA2_Stream6_IRQHandler,Default_Handler

   .weak      DMA2_Stream7_IRQHandler
   .thumb_set DMA2_Stream7_IRQHandler,Default_Handler

   .weak      USART6_IRQHandler
   .thumb_set USART6_IRQHandler,Default_Handler

   .weak      I2C3_EV_IRQHandler
   .thumb_set I2C3_EV_IRQHandler,Default_Handler

   .weak      I2C3_ER_IRQHandler
   .thumb_set I2C3_ER_IRQHandler,Default_Handler

   .weak      OTG_HS_EP1_OUT_IRQHandler
   .thumb_set OTG_HS_EP1_OUT_IRQHandler,Default_Handler

   .weak      OTG_HS_EP1_IN_IRQHandler
   .thumb_set OTG_HS_EP1_IN_IRQHandler,Default_Handler

   .weak      OTG_HS_WKUP_IRQHandler
   .thumb_set OTG_HS_WKUP_IRQHandler,Default_Handler

   .weak      OTG_HS_IRQHandler
   .thumb_set OTG_HS_IRQHandler,Default_Handler

   .weak      DCMI_IRQHandler
   .thumb_set DCMI_IRQHandler,Default_Handler

   .weak      HASH_RNG_IRQHandler
   .thumb_set HASH_RNG_IRQHandler,Default_Handler

   .weak      FPU_IRQHandler
   .thumb_set FPU_IRQHandler,Default_Handler

///////////////////////////////////////////////////////////////////////////////
// Main code starts from here
///////////////////////////////////////////////////////////////////////////////

_start:
    bl 	INIT_IO

    mov	r8,#0     // Clear ms counter

    bl	INIT_TC_PSP

loop:
//	bl LED_ON
//	mov r0, 500
//	bl DELAYTC
//	bl LED_OFF
//	mov r0, 500
//	bl DELAYTC
	b loop                      // Jump to loop

INIT_TC_PSP:
  push {r0, r1, lr}
	ldr r1, =SCS

	ldr r0, =SYSTICK_RELOAD_1MS
	str r0, [r1, #SCS_SYST_RVR]

	ldr r0, =0
	str r0, [r1, #SCS_SYST_CVR]

	ldr r0, =7      // TickINT Bit also set to 1
	str r0, [r1, #SCS_SYST_CSR]

  pop {r0, r1, pc}


.global SysTick_Handler
.type  SysTick_Handler, %function
SysTick_Handler:
    push {r3, r4, r5, r6, lr}

//    adr	r3,LEDSTAT
//    ldr r4,[r3]
//    cmp	r4,#0
    add r8,r8,#1
    cmp r8,#500
    blo RET

    mov	r8,#0    // reset ms counter

    cmp r7,#0    // check flag
	beq LOFF

	mov r5, #LEDs_OFF
//	mov r4,#0
//    str r4,[r3]
    mov r7,#0

	b CONT

LOFF: mov r5, #LEDs_ON
//      mov r4,#0xff
      mov r7,#0xff
      str r4,[r3]


CONT:
	// Set GPIOD Pins through BSSR register
	ldr    r6, =GPIOD_BASE       // Load GPIOD BASE address to r6
	str    r5, [r6,#GPIOD_BSSR] // Write to BSRR register
RET: pop {r3, r4, r5, r6, pc}
//    bx	lr


Default_Handler:
Infinite_Loop:
  b  Infinite_Loop



INIT_TC:
  push {r0, r1, lr}
	ldr r1, =SCS

	ldr r0, =SYSTICK_RELOAD_1MS
	str r0, [r1, #SCS_SYST_RVR]

	ldr r0, =0
	str r0, [r1, #SCS_SYST_CVR]

	ldr r0, =5
	str r0, [r1, #SCS_SYST_CSR]

  pop {r0, r1, pc}


INIT_IO:
  push {r5, r6, lr}
	// Enable GPIOD Peripheral Clock (bit 3 in AHB1ENR register)
	ldr r6, = RCC_AHB1ENR       // Load peripheral clock reg address to r6
	ldr r5, [r6]                // Read its content to r5
	orr r5, 0x00000008          // Set bit 3 to enable GPIOD clock
	str r5, [r6]                // Store result in peripheral clock register

	// Make GPIOD Pin12 as output pin (bits 25:24 in MODER register)
	ldr r6, =GPIOD_BASE       // Load GPIOD BASE address to r6
	ldr r5, [r6,#GPIOD_MODER]  // Read GPIOD_MODER content to r5
	and r5, 0x00FFFFFF          // Clear bits 31-24 for P12-15
	orr r5, 0x55000000          // Write 01 to bits 31-24 for P12-15
	str r5, [r6]                // Store result in GPIOD MODER register
  pop {r5, r6, pc}



LED_ON:
    push {r5, r6, lr}
	// Set GPIOD Pins to 1 (through BSSR register)
	ldr    r6, =GPIOD_BASE       // Load GPIOD BASE address to r6
	mov    r5, #LEDs_ON
	str    r5, [r6,#GPIOD_BSSR] // Write to BSRR register
  	pop {r5, r6, pc}

LED_OFF:
    push {r5, r6, lr}
	// Set GPIOD Pins to 0 (through BSSR register)
	ldr    r6, =GPIOD_BASE       // Load GPIOD BASE address to r6
	mov    r5, #LEDs_OFF
	str    r5, [r6,#GPIOD_BSSR] // Write to BSRR register
  	pop {r5, r6, pc}


DELAYTC:
    push {r1, r2, lr}
    ldr r1, =SCS


LOOPTC:	ldr r2, [r1, #SCS_SYST_CSR]
		tst r2, #0x10000
		beq LOOPTC

      subs r0, r0, 1
      bne LOOPTC
    pop {r1, r2, pc}

DELAY:
    push {r1, lr}

MSEC: ldr r1,=LEDDELAY
LOOP:    subs r1, r1, 1
         bne LOOP
      subs r0, r0, 1
      bne MSEC
    pop {r1, pc}
