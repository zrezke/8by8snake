.thumb
.syntax unified
.cpu cortex-m4

//.arch armv7e-m  
@RCC
.equ  RCC_BASE,  0x40023800
.equ  RCC_AHB1ENR,  0x30
.equ  RCC_APB2ENR,  0x44

@ SPI1
.equ  SPI1_BASE,  0x40013000

@ SPI register offsets
.equ  SPI_CR2,  0x4
.equ  SPI_SR, 0x8
.equ  SPI_DR, 0xC
.equ  SPI_CRCPR,  0x10
.equ  SPI_RXCRCR, 0x14
.equ  SPI_TXCRCR, 0x18
.equ  SPI_I2SCFGR,  0x1C
.equ  SPI_I2SPR,  0x20

@ SPI_CR1 config
.equ  SPI_CR1_CFG,  0x4844
@ .equ  SPI_LSBFIRST_TX_ONLY, 

@ GPIOA
.equ  GPIOA_BASE, 0x40020000
.equ  GPIO_ODR, 0x14
.equ  GPIO_BSRR,  0x18
.equ  GPIO_AFRL,  0x20
.equ  GPIO_AFRH,  0x24

.equ  ADDR_DECODE, 0x900
.equ  ADDR_INTENSITY, 0xA00
.equ  ADDR_SHUTDOWN,  0xC00
.equ  ADDR_SCAN_LIMIT,  0xB00
.equ  ADDR_DISPLAY_TEST,  0xF00
.equ  COL0, 0x100
.equ  COL1, 0x200
.equ  COL2, 0x300
.equ  COL3, 0x400
.equ  COL4, 0x500
.equ  COL5, 0x600
.equ  COL6, 0x700
.equ  COL7, 0x800

// SysTick Timer definitions
.equ     SCS, 			0xe000e000
.equ     SCS_SYST_CSR,	0x10	// Control/Status register
.equ     SCS_SYST_RVR,	0x14	// Value to countdown from
.equ     SCS_SYST_CVR,	0x18	// Current value

.equ	 SYSTICK_RELOAD_1MS,	15999  //1 msec at 16MHz ...  16 000 000 / 500 - 1


.section .text

.long    __StackTop                 // Top of the stack. from linker script
.long    _start +1


_start:
  bl init_io
  bl init_spi1
  bl init_tc
  
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

  ldr r1, =COL0

main:

  ldr r0, =ADDR_DISPLAY_TEST
  bl transmit_spi1
  add r1, #1
  mov r0, r1
  @ add r1, #20
  bl transmit_spi1
  mov r0, #5
  bl delay
  @ ldr r0, =COL1
  @ add r0, r0, #0x2
  @ bl transmit_spi1
  @ mov r0, #100
  @ bl delay
  b main


clear_display:
  push { r5, r6, lr }

  ldr r0, =COL0
  clear_display_loop:
    bl transmit_spi1
    add r0, #256
    cmp r0, #0x900
    bne clear_display_loop

  pop { r5, r6, pc }


@ pass data through r0
transmit_spi1:
  push { r5, r6, lr }
  ldr r5, =GPIOA_BASE
  ldr r6, [r5, #GPIO_ODR]
  eor r6, #(1<<4)
  str r6, [r5, #GPIO_ODR]

  ldr r5, =SPI1_BASE
  ldr r6, [r5]
  orr r6, #(1<<6)
  str r6, [r5]

  transmit_spi1_loop:
    ldr r6, [r5, #SPI_SR]
    tst r6, #2
    beq transmit_spi1_loop
    @ Transmit contents of r0
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


@ pass in baud rate through r5
init_spi1:
  push { r5, r6, lr }

  @ ENABLE Clock
  ldr r5, =RCC_BASE
  ldr r6, [r5, #RCC_APB2ENR]
  @ Bit 12: SPI1 clock enable
  orr r6, #(1 << 12)
  str r6, [r5, #RCC_APB2ENR]

  ldr r5, =SPI1_BASE
  mov r6, 0
  str r6, [r5]
  
  mov r6, #0b00 @ CPOL,CPHA
  orr r6, #(3<<3) @ BR /16
  orr r6, #(0b11<<8) @ SSI, SSM
  orr r6, #(1<<11) @ DFF
  orr r6, #(1<<14) @ BIDIOE
  str r6, [r5]

  ldr r6, [r5]
  orr r6, #(0b10001<<2) @mstr and spe
  str r6, [r5]

  pop { r5, r6, pc }


init_io:
  push { r5, r6, lr }

  @ Enable clock
  ldr r5, =RCC_BASE
  ldr r6, [r5, #RCC_AHB1ENR]
  orr r6, #1
  str r6, [r5, #RCC_AHB1ENR]
  ldr r5, =GPIOA_BASE
  @ Select alternate function mode (MODR register) on pins 4 (SPI1 NSS), 5 (SPI1 SCK) and 7 (SPI1 MOSI)
  ldr r6, [r5]
  orr r6, #(0b01 << 8)
  orr r6, #(0b10 << 10)
  orr r6, #(0b10 << 14)
  str r6, [r5]

  @ Set AF5 (SPI1) for pins 4, 5 and 7
  ldr r6, [r5, #GPIO_AFRL]
  orr r6, r6, #(0b101 << 28)
  orr r6, r6, #(0b101 << 20)
  @ orr r6, r6, #(0b101 << 16)
  str r6, [r5, #GPIO_AFRL]

  ldr r6, [r5, #GPIO_ODR]
  orr r6, #(1<<4)
  str r6, [r5, #GPIO_ODR]

  pop { r5, r6, pc }

init_tc:
  push {r5, r6, lr}
	ldr r6, =SCS

	ldr r5, =SYSTICK_RELOAD_1MS
	str r5, [r6, #SCS_SYST_RVR]

	ldr r5, =0
	str r5, [r6, #SCS_SYST_CVR]

	ldr r5, =5
	str r5, [r6, #SCS_SYST_CSR]

  pop {r5, r6, pc}

delay:
    push {r5, r6, lr}
    ldr r5, =SCS
  LOOPTC:	ldr r6, [r5, #SCS_SYST_CSR]
      tst r6, #0x10000
      beq LOOPTC

        subs r0, r0, 1
        bne LOOPTC
      pop {r5, r6, pc}
