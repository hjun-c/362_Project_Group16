/**
  ******************************************************************************
  * @file    main.c
  * @author  Group 10
  * @date    Oct 16, 2024
  * @brief   ECE 362 Project
  ******************************************************************************
*/

// Include librarires
#include "stm32f0xx.h"
#include "fatfs.h"

// Define constants

// Declare global variables
FIL wavFile;
UINT bytesRead;

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


int main(void) {

}
// Open the WAV file
int openFile(const char* filename) {
  if (f_open(&wavFile, filename, FA_READ) == FR_OK) {
    return 1;
  } else {
    return 0;
  }
}

// Close the WAV file
void closeFile() {
  f_close(&wavFile);
}

// read WAV file
int readFile(uint8_t* buffer, uint32_t size) {
  f_read(&wavFile, buffer, size, &bytesRead);
  return bytesRead;
}

// Initialize TIM6 that triggers audio sample
void init_tim6(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

  TIM6->PSC = 0;
  TIM6->ARR = (48000000 / sampleRate) - 1;

  TIM6->DIER |= TIM_DIER_UIE;
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

// Interrupt handler for TIM6
void TIM6_DAC_IRQHandler(void) {
  if (TIM6->SR & TIM_SR_UIF) { // check if interrupt flag was updated
    TIM6->SR &= ~TIM_SR_UIF; // clear the interrupt flag
    playNextSong(); // play next song
  }
}