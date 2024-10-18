void intit_spi1_slow(void) {
  // This function should configure SPI1 and the GPIOB MODER and AFR registers for pins PB3 (SCK), PB4 (MISO), and PB5 (MOSI). Configure it as follows:

  // Set the baud rate divisor to the maximum value to make the SPI baud rate as low as possible.
  // Set it to "Master Mode".
  // Set the word (data) size to 8-bit.
  // Configure "Software Slave Management" and "Internal Slave Select".
  // Set the "FIFO reception threshold" bit in CR2 so that the SPI channel immediately releases a received 8-bit value.
  // Enable the SPI channel.
}

void enable_sdcard(void) {
  // Set PB2 low to enable the SD card
}

void disable_sdcard(void) {
  // set PB2 high to disable the SD card
}

void init_sdcard_io(void) {
  // Create a subroutine named init_sdcard_io that does the following:

  // Calls init_spi1_slow().
  // Configures PB2 as an output.
  // Calls disable_sdcard().
}

void sdcard_io_high_speed(void) {
  // This function is called after SPI1 is initialized. It should do the following:

  // Disable the SPI1 channel.
  // Set the SPI1 Baud Rate register so that the clock rate is 12 MHz. (You may need to set this lower if your SD card does not reliably work at this rate.)
  // Re-enable the SPI1 channel.
}

void init_lcd_spi(void) {
  // Configure PB8, PB11, and PB14 as GPIO outputs. Don't forget the clock to GPIOB.
  // Call init_spi1_slow() to configure SPI1.
  // Call sdcard_io_high_speed() to make SPI1 fast.
  // The LCD can work at a rate that is usually - higher than the SD card. If you use the MSP2202 for your project, consider putting it on a separate SPI channel than the SD card interface and running it at a 24 MHz baud rate.
}