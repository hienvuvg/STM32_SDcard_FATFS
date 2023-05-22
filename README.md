# STM32_SDcard_FATFS
Writing into an SD card using SPI

Configuration: change SPI fast speed to 20 MHz in /FATFS/Target/user_diskio_spi.c based on SPI source clock

The writing speed is dependent on the card type and brand 

Maximum writable file size: 483.5 MB
