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
#define AUDIO_BUFFER_SIZE 8000
#define HALF_BUFFER_SIZE 4000
#define DEBOUNCE_TIME 15
#define MAX_SONGS 10

// Declare global variables
FIL file;
FATFS fatfs;
FRESULT fr;
uint32_t bytes_read = 0;
uint32_t bytes_unread = 0;
uint8_t audio_buffer[AUDIO_BUFFER_SIZE]; // buffer for DMA half-transfer
int debounce_counter = 0;
int button_state = 0;
int is_playing = 0;
int is_stopped = 0;
int data_offset = 0; // current offset of data to be transferred by DMA
int buffer_offset = 0; // current offset of audio buffer
char* filename; // song title
char song_list[MAX_SONGS]; // list of song titles in SD card

// Function Declaration

// Structure of WAV File Header
struct WavFileHeader {
  char chunk_id[4];
  int chunk_size;
  int format;
  char subchunk1_id[4];
  int subchunk1_size;
  short int audio_format;
  short int num_channels;
  int sample_rate;
  int byte_rate;
  short int block_align;
  short int bits_per_sample;
  char subchunk2_id[4];
  int subchunk2_size;
} header;

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
  } // need to check how to set debounce delay

  // Check which button was pressed
  int new_button_state = (GPIOC->IDR & 0xF);

  if (button_state != new_button_state) {
    button_state = new_button_state;
    debounce_counter = DEBOUNCE_TIME;

    if (button_state & 0x1) {
      handle_button1();
    }
    if (button_state & 0x2) {
      handle_button2();
    }
    if (button_state & 0x4) {
      handle_button3();
    }
    if (button_state & 0x8) {
      handle_button4();
    }
  }
}

void init_tim7(void) {
  // Enable the RCC clock for TIM7
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

  // Set the Prescaler (PSC) and Auto-Reload Register (ARR)
  // Frequency: 1ms
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

void init_dac(void) {
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
  TIM6->ARR = 100 - 1;
  TIM6->PSC = (48000000 / (header.sample_rate * (TIM6->ARR + 1))) - 1;

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

void enable_tim6(void) {
  TIM6->CR1 |= TIM_CR1_CEN;
}

void disable_tim6(void) {
  TIM6->CR1 &= ~TIM_CR1_CEN;
}

void TIM6_DAC_IRQHandler(void) {
  // Acknowledge the interrupt
  if (TIM6->SR & TIM_SR_UIF) {
    TIM6->SR &= ~TIM_SR_UIF; // clear the update interrupt flag
  }
}

void load_header(void) {
  fr = f_mount(&fatfs, "", 1);

  if (fr == FR_OK) {
    if (f_open(&file, filename, FA_READ) == FR_OK) {
        fr = f_read(&file, &header, sizeof(header), &bytes_read);
        data_offset += bytes_read;
    }
  }
}

void load_audio_data(uint32_t bytes_to_read) {
  fr = f_lseek(&file, data_offset);
  if (fr == FR_OK) {
    fr = f_read(&file, audio_buffer[buffer_offset], bytes_to_read, &bytes_read);
    if (fr == FR_OK) {
      data_offset += bytes_read;
      bytes_unread = bytes_to_read - bytes_read;
      buffer_offset += bytes_read;
      if (buffer_offset >= AUDIO_BUFFER_SIZE) {
        buffer_offset = 0;
      }
    }
  }
}

void init_dma(void) {
  // Enable the RCC clock to the DMA controller
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  // Turn off the enable bit for the channel
  DMA1_Channel3->CCR &= ~DMA_CCR_EN;

  // Set CMAR to the address of the msg array
  DMA1_Channel3->CMAR |= (uint32_t)audio_buffer;

  // Set CPAR to the address of the GPIOB_ODR register
  DMA1_Channel3->CPAR |= (uint32_t)(&(DAC->DHR12L1));

  // Set CNDTR
  DMA1_Channel3->CNDTR = HALF_BUFFER_SIZE; // 4000 bytes

  // Set the DIR for copying from-memory-to-peripheral
  DMA1_Channel3->CCR |= DMA_CCR_DIR;

  // Set the MINC to increment the CMAR for every transfer
  DMA1_Channel3->CCR |= DMA_CCR_MINC;

  // Set the size of M and P to 16-bit
  DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1;

  // Set the channel for CIRC operation
  DMA1_Channel3->CCR |= DMA_CCR_CIRC;
  
  // Enable half-transfer
  DMA1_Channel3->CCR |= DMA_CCR_HTIE;

  // Enable transfer-complete interrupt
  DMA1_Channel3->CCR |= DMA_CCR_TCIE;

  // Enable DMA channel
  DMA1_Channel3->CCR |= DMA_CCR_EN;

  // Enable DMA1 channel3 interrupt
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

void enable_dma(void) {
  DMA1_Channel1->CCR |= DMA_CCR_CIRC;
}

void disable_dma(void) {
  DMA1_Channel1->CCR &= ~DMA_CCR_CIRC;
}

void DMA1_Channel3_IRQHandler(void) {
  if (DMA1->ISR & DMA_ISR_HTIF3) {
    DMA1->IFCR |= DMA_IFCR_CHTIF3; // clear half-transfer interrupt flag
    load_audio_data(HALF_BUFFER_SIZE); 
  }

  if (DMA1->ISR & DMA_ISR_TCIF3) {
    DMA1->IFCR |= DMA_IFCR_CTCIF3; // clear transfer-complete interrupt flag
    load_audio_data(HALF_BUFFER_SIZE);
  }
}

void stop_song(void) {
  disable_dma();
  disable_tim6();
  data_offset = f_tell(&file); // save current file (data) position
}

void resume_song(void) {
  if (bytes_unread > 0) {
    load_audio_data(bytes_unread);
  }
  enable_dma();
  enable_tim6();
}

void handle_button1(void) {
  // move the cursor up
}

void handle_button2(void) {
  // move the cursor down
}

void handle_button3(void) {
  // click button
  if (!is_playing) {
    is_playing = 1;
    is_stopped = 0;
    // play song
  }
}

void handle_button4(void) {
  // resume / stop button
  if ((is_playing == 1) && (is_stopped == 0)) {
    stop_song();
    is_playing = 0;
    is_stopped = 1;
  } else if ((is_playing == 0) && (is_stopped == 1)) {
    resume_song();
    is_playing = 1;
    is_stopped = 0;
  }
}

// Read the file name in SD card and store them in a song list
void read_song_list(void) {

}

// Display a song list on TFT display
void display_song_list(void) {

}

// Read input from the button and determine which song was selected
void read_selected_song(void) {
  
}

//===============================================================================================================
// Additional instruction on Lab7 manual
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

void init_lcd_spi(void) {
  // Enable the RCC clock for Port B
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  // Configure PB8, PB11, and PB14 as output
  GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14); // clear
  GPIOB->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0); // set to output mode

  init_spi1_slow();
  sdcard_io_high_speed();
}