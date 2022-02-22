.thumb
.syntax unified
.cpu cortex-m4

//.arch armv7e-m  
//RCC
.equ  RCC_BASE,           0x40023800
.equ  RCC_CR,             0x00
.equ  RCC_AHB1ENR,        0x30
.equ  RCC_AHB2ENR,        0x34
.equ  RCC_APB2ENR,        0x44
.equ  RCC_PLLCFGR,        0x4

// SPI1
.equ  SPI1_BASE,          0x40013000

// SPI register offsets
.equ  SPI_CR2,            0x4
.equ  SPI_SR,             0x8
.equ  SPI_DR,             0xC
.equ  SPI_CRCPR,          0x10
.equ  SPI_RXCRCR,         0x14
.equ  SPI_TXCRCR,         0x18
.equ  SPI_I2SCFGR,        0x1C
.equ  SPI_I2SPR,          0x20

// SPI_CR1 config
.equ  SPI_CR1_CFG,        0x4844

// GPIOA
.equ  GPIOA_BASE,         0x40020000
.equ  GPIO_ODR,           0x14
.equ  GPIO_BSRR,          0x18
.equ  GPIO_AFRL,          0x20
.equ  GPIO_AFRH,          0x24

// ADC
.equ  ADC1_BASE,          0x40012000
.equ  ADC2_BASE,          0x40012100
.equ  ADC3_BASE,          0x40012200
.equ  ADC_SR,             0x0
.equ  ADC_CR1,            0x4
.equ  ADC_CR2,            0x8
.equ  ADC_DR,             0x4c
.equ  ADC_HTR,            0x24
.equ  ADC_LTR,            0x28
.equ  ADC_SQR1,           0x2c
.equ  ADC_SQR2,           0x30
.equ  ADC_SQR3,           0x34
.equ  ADC_SMPR1,          0x0C
.equ  ADC_SMPR2,          0x10
.equ  ADC_CCR,            0x04

// DMA
.equ  DMA1_BASE,          0x40026000
.equ  DMA2_BASE,          0x40026400
.equ  DMA_LISR,           0x0
.equ  DMA_HISR,           0x4
.equ  DMA_S0CR,           0x10 + 0x18 * 0
.equ  DMA_S3CR,           0x10 + 0x18 * 3
.equ  DMA_S0_NDTR,        0x14 + 0x18 * 0
.equ  DMA_S0PAR,          0x18 + 0x18 * 0
.equ  DMA_S0M0AR,         0x1c + 0x18 * 0
.equ  DMA_S0M1AR,         0x20 + 0x18 * 0


.equ  ADDR_DECODE,        0x900
.equ  ADDR_INTENSITY,     0xA00
.equ  ADDR_SHUTDOWN,      0xC00
.equ  ADDR_SCAN_LIMIT,    0xB00
.equ  ADDR_DISPLAY_TEST,  0xF00
.equ  COL0,               0x100
.equ  COL1,               0x200
.equ  COL2,               0x300
.equ  COL3,               0x400
.equ  COL4,               0x500
.equ  COL5,               0x600
.equ  COL6,               0x700
.equ  COL7,               0x800

// SysTick Timer definitions
.equ  SCS, 			          0xe000e000
.equ  SCS_SYST_CSR,	      0x10	// Control/Status register
.equ  SCS_SYST_RVR,	      0x14	// Value to countdown from
.equ  SCS_SYST_CVR,	      0x18	// Current value

.equ  SYSTICK_RELOAD_200MS,	3199999  //200 ms at 16MHz; (16Mhz/1000 * 200)-1

.equ  RNG_BASE,           0x50060800
.equ  RNG_CR,             0
.equ  RNG_SR,             0x4
.equ  RNG_DR,             0x8


.section .rodata
ROWS: .byte 1, 2, 4, 8, 16, 32, 64, 128

.section .data
.section .bss
JOYSTICK_POS: .space 4

PIXEL_DATA: .space 8 // Image that will be drawn on the screen

SNAKE_POS: .space 128 //(8x8 x HWORD)
SNAKE_LENGTH: .space 1
DIRECTION: .space 1
/* SNAKE DIRECTION
0 -> left
1 -> right
2 -> up
3 -> down
*/
FOOD_POS: .space 2

UPDATED: .space 1
.section .text

vector_table:
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
  // .thumb_set SysTick_Handler,Default_Handler

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


_start:
  bl init_io
  bl init_spi1
  bl init_tc

  bl init_adc1

  bl init_dma2
  bl start_adc1

  bl init_rng
  bl configure_matrix
  bl init_snake
  bl place_food

game_loop:
  // Check if already updated in ongoing game loop and wait until next loop begins
  ldr r0, =UPDATED
  ldrb r0, [r0]
  cmp r0, #1
  beq game_loop

  bl update_direction
  bl update_snake
  bl render
  ldr r1, =UPDATED
  mov r0, #1
  strb r0, [r1]
  b game_loop

eat_food:
  push { r5, r6, lr }
  ldr r5, =SNAKE_LENGTH
  ldrb r6, [r5]
  add r6, #1
  strb r6, [r5]
  bl place_food
  pop { r5, r6, pc }
place_food:
  push { r1, r2, r3, r4, r5, r6, lr }
  generate_food_position:
    // Generate column
    bl fetch_random_number
    // Get 3 MS-bits
    mov r2, r1, lsr 29
    // Add one because columns start from 0x100
    add r2, #1
    // Shift to column position
    mov r2, r2, lsl 8
    // Generate row
    bl fetch_random_number
    // Get 3 MS-bits
    mov r3, r1, lsr 29
    mov r4, #1
    lsl r4, r3
    // Create position in r2
    orr r2, r2, r4
    b check_food_in_snake

  spawn_food:
    ldr r5, =FOOD_POS
    strh r2, [r5]
  pop { r1, r2, r3, r4, r5, r6, pc }

// Pass in food position through r2
check_food_in_snake:
  ldr r5, =SNAKE_POS
  ldr r4, =SNAKE_LENGTH
  ldrb r4, [r4]

  check_food_in_snake_loop:
    ldrh r3, [r5], #2
    cmp r3, r2
    beq generate_food_position
    sub r4, #1
    cmp r4, #0
    bne check_food_in_snake_loop
    b spawn_food
  

// returns random number in r1
fetch_random_number:
  push {r5, r6, lr}
  ldr r5, =RNG_BASE
  check_rng_ready:
    ldr r6, [r5, #RNG_SR]
    tst r6, #1
    beq check_rng_ready
  ldr r1, [r5, #RNG_DR]
  pop {r5, r6, pc}

update_snake:
  push { r4, r5, r6, lr }
  // Load head position into r0, because move_{left,right...} functions expect it in r0
  ldr r4, =SNAKE_POS
  ldrh r0, [r4]
  // Load direction and jump into the appropriate function
  ldr r5, =DIRECTION
  ldrb r6, [r5]
  cmp r6, #0
  beq move_left
  cmp r6, #1
  beq move_right
  cmp r6, #2
  beq move_up
  cmp r6, #3
  beq move_down

  // Shift the snake in memory
  move_snake:
    // Already have SNAKE_POS address in r4
    ldr r1, =FOOD_POS
    ldrh r1, [r1]
    cmp r0, r1
    it eq
    bleq eat_food
    // Already have SNAKE_POS address in r4
    ldr r3, =SNAKE_LENGTH
    ldrb r3, [r3]
    // Multiply length by 2 because of HWROD
    add r3, r3, r3
    sub r3, #2
    move_snake_loop:
      ldrh r2, [r4, r3]
      // Reset the game if the new head pos is in it's body
      cmp r2, r0
      beq reset_game
      add r3, #2
      strh r2, [r4, r3]
      subs r3, #4
      cmp r3, #-2
      bne move_snake_loop
      strh r0, [r4]
  pop { r4, r5, r6, pc }

reset_game:
  // Reinitialize snake to 3 pixels
  bl init_snake
  // Replace food, in case it was placed where the snake is after init_snake
  bl place_food
  // Continue game
  b game_loop  


// Supply with current head position in r0
move_left:
  sub r0, 0x100
  cmp r0, #0x100
  it lt
  addlt r0, #0x800
  b move_snake
move_up:
  // F00 -> and row
  // FF -> and col
  and r1, r0, #0xFF
  mov r1, r1, lsr 1
  cmp r1, #0
  it eq
  moveq r1, #128
  and r0, #0xF00
  orr r0, r1
  b move_snake
move_right:
  add r0, 0x100
  cmp r0, #0x900
  it gt
  subgt r0, #0x800
  b move_snake
move_down:
  // F00 -> and row
  // FF -> and col
  and r1, r0, #0xFF
  mov r1, r1, lsl 1
  cmp r1, #256
  it eq
  moveq r1, #1
  and r0, #0xF00
  eor r0, r1
  b move_snake

update_direction:
  push { r2, r3, r4, r5, r6, lr }
  ldr r6, =JOYSTICK_POS

  ldr r3, =DIRECTION
  ldrb r2, [r3]
  cmp r2, #2
  blt check_y
  bge check_x

  check_x:
    ldrh r5, [r6]
    cmp r5, #800 // Joystick is pointing left
    itt gt
    movgt r2, #0
    strbgt r2, [r3]
    
    cmp r5, #200 // Joystick is pointing right
    itt lt
    movlt r2, #1
    strblt r2, [r3]
    b exit_update_direction
    
  check_y:
    ldrh r5, [r6, #2]
    cmp r5, #800 // Joystick is pointing up
    itt gt
    movgt r2, #2
    strbgt r2, [r3]
    
    cmp r5, #200 // Joystick is pointing down
    itt lt
    movlt r2, #3
    strblt r2, [r3]

  exit_update_direction:
  pop { r2, r3, r4, r5, r6, pc }

update_pixel_data:
  push { r0, r1, r2, r3, r4, r5, r6, lr }
  ldr r5, =PIXEL_DATA
  // Clear pixel data
  mov r6, #0
  str r6, [r5]
  str r6, [r5, #4]

  // Draw snake
  ldr r4, =SNAKE_LENGTH
  ldrb r4, [r4]
  add r4, r4, r4
  ldr r3, =SNAKE_POS
  add r4, r4, r3
  update_snake_pixels_loop:
    // Load one snake pixel
    ldrh r2, [r3], #2
    // Find out column of pixel
    mov r1, r2, lsr #8
    sub r1, #1
    // Extract row data from snake pixel
    and r2, 0xff
    // Orr row data with pixels that are already in same column
    ldrb r0, [r5, r1]
    orr r0, r2
    // Store, loop again till you reach the end of the snake
    strb r0, [r5, r1]
    cmp r3, r4
    bne update_snake_pixels_loop

  cmp r6, #1
  beq end_update_pixel_data
  mov r6, #1
  // TODO: Draw food
  ldr r3, =FOOD_POS
  mov r4, r3
  add r4, #2
  // Use existing function to orr rows and stuff like that
  b update_snake_pixels_loop
  end_update_pixel_data:
  pop { r0, r1, r2, r3, r4, r5, r6, pc }

// Render column by column
render:
  push { r0, r1, r2, r5, lr }
  bl update_pixel_data
  bl clear_display
  mov r3, #0x100
  ldr r5, =PIXEL_DATA
  render_loop:
    ldrb r0, [r5], #1
    orr r0, r3
    bl transmit_spi1
    add r3, #0x100
    cmp r3, #0x900
    bne render_loop
  pop { r0, r1, r2, r5, pc }

init_snake:
  push { r5, r6, lr }
  // Init length
  ldr r5, =SNAKE_LENGTH
  mov r6, #3
  strb r6, [r5]

  // Init direction
  ldr r5, =DIRECTION
  mov r6, #0
  strb r6, [r5]

  ldr r6, =SNAKE_POS
  ldr r5, =COL5+8
  strh r5, [r6], #2
  add r5, #0x100
  strh r5, [r6], #2
  add r5, #0x100
  strh r5, [r6]

  pop { r5, r6, pc }

clear_display:
  push { r5, r6, lr }

  ldr r0, =COL0
  clear_display_loop:
    bl transmit_spi1
    add r0, #0x100
    cmp r0, #0x900
    bne clear_display_loop

  pop { r5, r6, pc }

configure_matrix:
  push { r0, lr }
  ldr r0, =ADDR_SHUTDOWN
  add r0, #1
  bl transmit_spi1

  ldr r0, =ADDR_DECODE
  bl transmit_spi1

  ldr r0, =ADDR_INTENSITY
  add r0, #1
  bl transmit_spi1
  
  ldr r0, =ADDR_SCAN_LIMIT
  add r0, #7
  bl transmit_spi1

  ldr r0, =ADDR_DISPLAY_TEST
  bl transmit_spi1

  bl clear_display
  pop { r0, pc }

// pass data through r0
transmit_spi1:
  push { r5, r6, lr }
  ldr r5, =GPIOA_BASE
  ldr r6, [r5, #GPIO_ODR]
  eor r6, #(1<<4)
  str r6, [r5, #GPIO_ODR]

  ldr r5, =SPI1_BASE
  // Transmit contents of r0
  str r0, [r5, #SPI_DR]

  wait_txe_spi1:
    ldr r6, [r5, #SPI_SR]
    tst r6, #2
    beq wait_txe_spi1
    wait_bsy_spi1:
      ldr r6, [r5, #SPI_SR]
      tst r6, #128
      bne wait_bsy_spi1

  ldr r5, =GPIOA_BASE
  ldr r6, [r5, #GPIO_ODR]
  orr r6, #(1<<4)
  str r6, [r5, #GPIO_ODR]
  pop { r5, r6, pc }


// pass in baud rate through r5
init_spi1:
  push { r5, r6, lr }

  // ENABLE Clock
  ldr r5, =RCC_BASE
  ldr r6, [r5, #RCC_APB2ENR]
  // Bit 12: SPI1 clock enable
  orr r6, #(1 << 12)
  str r6, [r5, #RCC_APB2ENR]

  ldr r5, =SPI1_BASE
  mov r6, 0
  str r6, [r5]
  
  mov r6, #0b00 // CPOL,CPHA
  orr r6, #(1<<14) // BIDIOE
  orr r6, #(1<<2) // MSTR
  orr r6, #(0b11<<8) // SSI, SSM
  orr r6, #(1<<11) // DFF
  str r6, [r5]

  // Enable SPI
  ldr r6, [r5]
  orr r6, #(1<<6) // SPE
  str r6, [r5]

  pop { r5, r6, pc }

init_dma2:
  push { r5, r6, lr }
  // Enable DMA2 clock
  ldr r5, =RCC_BASE
  ldr r6, [r5, #RCC_AHB1ENR]
  orr r6, #(1<<22)
  str r6, [r5, #RCC_AHB1ENR]
  
  ldr r5, =DMA2_BASE
  mov r6, #0
  str r6, [r5, #DMA_S0CR]

  wait_dma2_s0_reset:
    ldr r6, [r5, #DMA_S0CR]
    tst r6, #1
    bne wait_dma2_s0_reset
  
  mov r6, #0
  str r6, [r5, #DMA_LISR]
  str r6, [r5, #DMA_HISR]

  // CHSEL: 0, DIR: Peripheral to memmory, 
  mov r6, #0
 
  // PSIZE, MSIZE hword
  orr r6, #(0b01<<11)
  orr r6, #(0b01<<13)

  // Priority very high
  orr r6, #(0b11 << 16)

  // CIRC MODE ENABLE
  orr r6, #(1<<8)

  // TCIE = 1
  orr r6, #(1<<4)

  // MEMORY INC
  orr r6, #(1<<10)
  str r6, [r5, #DMA_S0CR]

  // N of data items to transfer: 2
  mov r6, #2
  str r6, [r5, #DMA_S0_NDTR]

  ldr r5, =DMA2_BASE
  ldr r6, =ADC1_BASE + ADC_DR
  str r6, [r5, #DMA_S0PAR]
  
  ldr r6, =JOYSTICK_POS
  str r6, [r5, #DMA_S0M0AR]

  // Enable stream
  ldr r6, [r5, #DMA_S0CR]
  orr r6, #1
  str r6, [r5, #DMA_S0CR]
  pop { r5, r6, pc }

start_adc1:
  push { r5, r6, lr }
  ldr r5, =ADC1_BASE
  mov r6, #0
  str r6, [r5, #ADC_SR]
  // START
  ldr r6, [r5, #ADC_CR2]
  orr r6, #(1<<30)
  str r6, [r5, #ADC_CR2]
  pop { r5, r6, pc }

init_adc1:
  push { r5, r6, lr }
  // Enable ADC1 clock
  ldr r5, =RCC_BASE
  ldr r6, [r5, #RCC_APB2ENR]
  orr r6, #(1<<8)
  str r6, [r5, #RCC_APB2ENR]

  ldr r5, =ADC1_BASE
  mov r6, #0
  str r6, [r5, #ADC_CR2]
  str r6, [r5, #ADC_CR1]
  str r6, [r5, #ADC_SMPR1]
  str r6, [r5, #ADC_SMPR2]
  str r6, [r5, #ADC_SQR1]
  str r6, [r5, #ADC_SQR2]
  str r6, [r5, #ADC_SQR3]
  str r6, [r5, #ADC_CCR]
  
  // CR1 settings
  // Set RES to 10bits
  mov r6, #(0b01<<24)

  // SCAN MODE
  orr r6, #(1<<8)

  // EOCIE
  // orr r6, #(1<<5)
  str r6, [r5, #ADC_CR1]

  //CR2 settings
  // set CONT mode
  mov r6, #0b10
  // EOC after each conversion
  orr r6, #(1<<10)
  // Enable DMA, DDS
  orr r6, #(0b11<<8)
  str r6, [r5, #ADC_CR2]

  // Two conversions
  mov r6, #(1<<20)
  str r6, [r5, #ADC_SQR1]

  // Select order, PA1, PA2
  mov r6, #1
  orr r6, #(2 << 5)
  str r6, [r5, #ADC_SQR3]

  // SAMPLE TIME
  mov r6, #9
  str r6, [r5, #ADC_SMPR2]

  // Enable ADC1
  ldr r6, [r5, #ADC_CR2]
  orr r6, #1
  str r6, [r5, #ADC_CR2]

  pop { r5, r6, pc }


.global SysTick_Handler
.type SysTick_Handler, %function

SysTick_Handler:
  push { r5, r6, lr }
  // Reset UPDATED
  ldr r5, =UPDATED
  mov r6, #0
  strb r6, [r5]
  pop { r5, r6, pc }

Default_Handler:
Infinite_Loop:
  b  Infinite_Loop

init_io:
  push { r5, r6, lr }

  // Enable GPIOA clock
  ldr r5, =RCC_BASE
  ldr r6, [r5, #RCC_AHB1ENR]
  orr r6, #1
  str r6, [r5, #RCC_AHB1ENR]

  // GPIOA_MODER  
  ldr r5, =GPIOA_BASE
  // set pins 1,2 as analog, 4 as regular GPIO; used as -> (SPI1 NSS)
  ldr r6, [r5]
  orr r6, #(0b11 << 2)
  orr r6, #(0b11 << 4)
  orr r6, #(0b01 << 8)
  // Select alternate function mode (MODR register) on pins 5 (SPI1 SCK) and 7 (SPI1 MOSI)
  orr r6, #(0b10 << 10)
  orr r6, #(0b10 << 14)
  str r6, [r5]

  // Set AF5 (SPI1) for pins 5 and 7
  ldr r6, [r5, #GPIO_AFRL]
  orr r6, r6, #(0b101 << 28)
  orr r6, r6, #(0b101 << 20)
  str r6, [r5, #GPIO_AFRL]

  ldr r6, [r5, #GPIO_ODR]
  orr r6, #(1<<4)
  str r6, [r5, #GPIO_ODR]

  pop { r5, r6, pc }

init_tc:
  push {r5, r6, lr}
	ldr r6, =SCS

	ldr r5, =SYSTICK_RELOAD_200MS
	str r5, [r6, #SCS_SYST_RVR]

	mov r5, #0
	str r5, [r6, #SCS_SYST_CVR]
  
  // Enable counter
	orr r5, #1
  // Enable interrupt
  orr r5, #2
  // Processor clock as clock source
  orr r5, #4
	str r5, [r6, #SCS_SYST_CSR]

  pop {r5, r6, pc}

// delay:
//     push {r5, r6, lr}
//     ldr r5, =SCS
//   LOOPTC:	ldr r6, [r5, #SCS_SYST_CSR]
//       tst r6, #0x10000
//       beq LOOPTC

//         subs r0, r0, 1
//         bne LOOPTC
//       pop {r5, r6, pc}

init_rng:
  push { r5, r6, lr }
  ldr r5, =RCC_BASE
  // PLL config RNG_OUT = ((PLL_IN -> 16mhz) * (PLLN/PLLM)) / PLLQ = 48mhz
  // set pllm as 2 
  mov r6, #2
  // Set PLLN as 12
  orr r6, #(12<<6)
  // Set PLLQ as 2
  orr r6, #(2<<24)
  str r6, [r5, #RCC_PLLCFGR]

  // Enable RNG clock
  ldr r6, [r5, #RCC_AHB2ENR]
  orr r6, #(1<<6)
  str r6, [r5, #RCC_AHB2ENR]

  // Enable PLL
  ldr r6, [r5, #RCC_CR]
  orr r6, #(1<<24)
  str r6, [r5, #RCC_CR]

  ldr r5, =RNG_BASE
  // Turn off rgn, reset SR
  mov r6, #0
  str r6, [r5, #RNG_CR]
  str r6, [r5, #RNG_SR]
  // Start RNG
  mov r6, #4
  str r6, [r5, #RNG_CR]

  // Wait for first number and read it
  wait_first_rng:
    ldr r6, [r5, #RNG_SR]
    tst r6, #1
    beq wait_first_rng
  ldr r6, [r5, #RNG_DR]
  pop { r5, r6, pc }
