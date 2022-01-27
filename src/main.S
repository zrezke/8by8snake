
.thumb
.syntax unified
.cpu cortex-m4

@ SPI1
.equ  SPI1_BASE,  0x40013000

@ SPI register offsets
.equ  SPI_CR2,  0x04
.equ  SPI_SR, 0x08
.equ  SPI_DR, 0x0C
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
.equ  GPIO_AFRL,  0x20
.equ  GPIO_AFRH,  0x24




main:
  b main

@ pass in baud rate through r5
init_spi1:
  push { lr }
  ldr r5, =SPI1_BASE
  mov r6, 0
  str r6, [r5]
  mov r6, #SPI_CR1_CFG
  str r6, [r5]
  pop { pc }


init_io:
  push { lr }
  ldr r5, =GPIOA_BASE
  @ Select alternate function mode on pins 8 and 9
  mov r6, #(0b11 << 8)
  str r6, [r5]

  @ Set AF5 (SPI1) for pins 8 and 9
  ldr r6, [r5, #GPIO_AFRH]
  orr r6, r6, #0x55
  str r6, [r5, #GPIO_AFRH]

  pop { pc }
