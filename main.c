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
#include "ff.h"

// Define constants
#define RATE 20000
#define DEBOUNCE_TIME 15
#define BUFFER_SIZE 4096

// Declare global variables
FIL file;
DIR Dir;
FATFS fatfs;
FRESULT fr;
UINT bytesRead;
int button_state = 0;
int debounce_counter = 0;
int samp_freq = 0;
uint8_t buffer[BUFFER_SIZE];

// Function Declaration
void internal_clock();
//==============================
void init_buttons(void);
void scan_buttons(void);
void init_tim7(void);
void TIM7_IRQHandler(void);
//==============================
void init_DAC(void);
void init_tim6(void);
void TIM6_DAC_IRQHandler(void);
void play_song(char* song_title);
void stop_song(void);
//==============================
void init_spi1_slow(void);
void enable_sdcard(void);
void disable_sdcard(void);
void init_sdcard_io(void);
void sdcard_io_high_speed(void);
void init_sd_card(void);
//==============================
void init_lcd_spi(void);
void handle_button1(void);
void handle_button2(void);
void handle_button3(void);
void handle_button4(void);
void display_song_list(void);
void display_playing_song(void);
void display_song_stopped(void);
//==============================
void read_file(char filename[]);

// structure of WAV file header (for storing information)
struct WavFileHeader {
  char ck_id[4]; // chunk id
  int ck_size; // chunk size
  char sub_ck_id[4]; // sub chunk 1 id
  int sub_ck_size; // sub chuck 1 size
  int audio_format;
  int num_channels;
  int sample_rate;
  int byte_rate;
  int block_align;
  int bits_per_sample;
  char sub_ck2_id[4]; // sub chunk 2 id
  int sub_ck2_size; // sub chunk 2 size
  int data_size;
} header;

int main(void) {
  internal_clock();
}

void init_buttons(void) {
  // Enable the RCC clock for Port C
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Configure pins PC0 – PC3 as inputs
  // Button 1: up
  // Button 2: down
  // Button 3: play or stop
  // Button 4: back to song list
  GPIOC->MODER &= ~((0x3) | (0x3 << 2) | (0x3 << 4) | (0x3 << 6)); // input mode

  // Configure pins PC0 – PC3 to be internally pulled down
  // (Default state will be floating if no button pressed)
  GPIOC->PUPDR &= ~((0x3 << 0) | (0x3 << 2) | (0x3 << 4) | (0x3 << 6)); // clear
  GPIOC->PUPDR |= (0x2) | (0x2 << 2) | (0x2 << 4) | (0x2 << 6); // set to be pulled down
}

void scan_buttons(void) {
  if (debounce_counter > 0) {
    debounce_counter--;
    return;
  }

  // Check which button was pressed
  int new_button_state = (GPIOC->IDR & 0xF);

  if (button_state != new_button_state) {
    button_state = new_button_state;
    debounce_counter = DEBOUNCE_TIME;

    if (button_state & 0x1) {
      // move cursor up
    }
    if (button_state & 0x2) {
      // move cursor down
    }
    if (button_state & 0x4) {
      // select
    }
    if (button_state & 0x8) {
      // back to song list
    }
  }
}

void init_tim7(void) {
  // Enable the RCC clock for TIM7
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

  // Set the Prescaler (PSC) and Auto-Reload Register (ARR)
  TIM7->PSC = 480-1;
  TIM7->ARR = 100-1;

  // Enable the UIE bit in the DIER to enable the UIE flag
  TIM7->DIER |= TIM_DIER_UIE;

  // Enable the interrupt for Timer 7 in the NVIC ISER
  NVIC_EnableIRQ(TIM7_IRQn);

  // Enable Timer 7 by setting the CEN bit in the Timer 7 Control Register 1
  TIM7->CR1 |= TIM_CR1_CEN;
}

void TIM7_IRQHandler(void) {
  scan_buttons();
}

void init_DAC(void) {
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

void init_tim6(void) {
  // Enable the RCC clock for TIm6
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

  // Set the prescaler and reload value
  TIM6->PSC = 100 - 1;
  TIM6->ARR = (48000000 / (header.sample_rate * (TIM6->ARR + 1))) - 1;

  // Set the value that enables a TRGO on an Update event
  TIM6->CR2 &= ~((0x3 << 4) | (0x3 << 5) | (0x3 << 6)); // clear
  TIM6->CR2 |= (0x1 << 5);

  // Enable the UIE bit in the DIER to enable the UIE flag
  TIM6->DIER |= TIM_DIER_UIE;

  // Enable the interrupt for the Timer 6
  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  // Enable the Timer 6
  TIM6->CR1 |= TIM_CR1_CEN;
}

void TIM6_DAC_IRQHandler(void) {
  // Acknowledge the interrupt
  if (TIM6->SR & TIM_SR_UIF) {
    TIM6->SR &= ~TIM_SR_UIF; // clear the update interrupt flag
  }

  // Output the next sample to the DAC
  int samp;

  // Send data to DAC->DHR12R1
  DAC->DHR12R1 = samp;
}

void play_song(char* song_title) {
  read_file(song_title);
  f_lseek(&file, 0); // move pointer
  f_read(&file, &buffer[0], BUFFER_SIZE, &bytesRead); // read audio data
  // send buffer to dma
}

void stop_song(void) {

}

void init_spi1_slow(void) {
  // Enable the RCC clock for SPI1 and Port B
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

void init_sd_card(void) {
  f_mount(&fatfs, "", 0);
}

void init_lcd_spi(void) {
  // Enable the RCC clock for Port B
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  // Configure PB8, PB11, and PB14 as output
  GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14); // clear
  GPIOB->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0); // set to output mode

  init_spi1_slow();
  sdcard_io_high_speed();
}

void handle_button1(void) {
  spi_cmd(); // move the cursor up
}

void handle_button2(void) {
  spi_cmd(); // move the cursor down
}

void handle_button3(void) {
  spi_cmd(); // move the cursor to the home position
}

void handle_button4(void) {
  // cursor doesn't needed
}

void display_song_list(void) {

}

void display_playing_song(void) {

}

void display_song_stopped(void) {

}

void read_file(char filename[]) {
  fr = f_mount(&fatfs, "", 1);

  if (fr == FR_OK) {
    if (f_open(&file, filename, FA_READ) == FR_OK) {
        fr = f_read(&file, &header, sizeof(header), bytesRead);
    }
  }
}
