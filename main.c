/**
  ******************************************************************************
  * @file    main.c
  * @author  Group 16
  * @date    Oct 16, 2024
  * @brief   ECE 362 Project
  ******************************************************************************
*/

// Include librarires
#include "stm32f0xx.h"

// Define constants

// Declare global variables


/*
Function lists
1. System Initialization
  - initialize peripherals
  - initialize gpio
  - initialize internal clock
2. Access SD card
  - initialize spi1 (communicate with SD card)
  - enable card
  - disable card
3. Handle WAV file
  - open
  - close
  - read
4. DAC
  - initialize DAC
  - configure DAC pins
  - start song
  - stop song
5. Timer
  - initialize TIM6 (for DAC)
  - TIM6 interrupt handler
  - initialize TIM3 (for button)
  - TIM3 interrupt handler
6. TFT Display
  - configure pins for display
  - initialize spi2 (communicate with TFT)
  - initialize TFT display
  - display song title
  - display playing mode
  - display stop mode
7. Button
  - configure buttons
  - read state of click button
  - read state of direction button
  - read state of home button
  - button handler
8.
*/

// How to read file?



int main(void) {

}

void init_button(void) {
  // Enable the RCC clock for Port C
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Configure pins PC0 – PC3 to be inputs
  GPIOC->MODER &= ~((0x3) | (0x3 << 2) | (0x3 << 4) | (0x3 << 6)); // input mode

  // Configure pins PC0 – PC3 to be internally pulled low
  GPIOC->PUPDR &= ~((0x3 << 0) | (0x3 << 2) | (0x3 << 4) | (0x3 << 6)); // clear
  GPIOC->PUPDR |= (0x2) | (0x2 << 2) | (0x2 << 4) | (0x2 << 6); // set to be pulled down
}

void init_DMA(void) {
  // Enable the RCC clock for Port A
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // Configure PA4 as DAC output
  GPIOA->MODER |= (0x3 << 8); // analog mode

  // Enable the RCC clock for the DAC
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;

  // Select a TIM6 TRGO trigger
  DAC->CR &= ~((0x3 << 3) | (0x3 << 4) | (0x3 << 5)); // clear (TIM 6 TRGO event)

  // Enable the trigger for the DAC
  DAC->CR |= (0x1 << 2);

  // Enable the DAC
  DAC->CR |= 0x1;
}

void init_spi1_slow(void) {
  // Enable the RCC clock for SPI1 Port B
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  // Configure PB3 - PB5
  GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5); // clear
  GPIOB->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1); // set to alternate mode

  // Set alternate function
  GPIOB->AFR[0] &= ~(0xFFF << 12);

  // SPI1 configuration
  SPI1->CR1 = SPI_CR1_MSTR; // master selection
  SPI1->CR1 |= SPI_CR1_BR | SPI_CR1_SSM | SPI_CR1_SSI;

  // Set data size to 8 bit
  SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
  SPI1->CR2 &= ~SPI_CR2_DS_3;

  // Set FIFO reception threshold bit to 1
  SPI1->CR2 = SPI_CR2_FRXTH;

  // Enable the SPI channel
  SPI1->CR1 |= SPI_CR1_SPE;
}

void enable_sdcard(void) {
  // Set PB2 low
  GPIOB->BSRR = (0x1 << 18);
}

void disable_sdcard(void) {
  // Set PB2 high
  GPIOB->BSRR = (0x1 << 2);
}

void init_sdcard_io(void) {
  init_spi1_slow();

  // Set PB2 as an output
  GPIOB->MODER &= ~(GPIO_MODER_MODER2); // clear
  GPIOB->MODER |= ~(GPIO_MODER_MODER2_0); // set to output mode

  disable_sdcard();
}

void sdcard_io_high_speed(void) {
  // Disable the SPI1 channel
  SPI1->CR1 &= ~SPI_CR1_SPE;

  // Set the SPI1 BR so that the clock rate is 12MHz
  SPI1->CR1 &= ~(0x7 << 3); // clear
  SPI1->CR1 |= (0x1 << 3); // (f/4)

  // Enable the SPI channel
  SPI1->CR1 |= SPI_CR1_SPE;
}