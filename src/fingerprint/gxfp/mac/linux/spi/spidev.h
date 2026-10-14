/* Emulated spidev: transfers go through MateBookFP.kext (mac_hw.c). */
#pragma once
#include <stdint.h>
struct spi_ioc_transfer {
    uint64_t tx_buf, rx_buf;
    uint32_t len, speed_hz;
    uint16_t delay_usecs;
    uint8_t bits_per_word, cs_change, tx_nbits, rx_nbits, word_delay_usecs, pad;
};
#define SPI_MODE_0 0
#define SPI_CS_HIGH 0x04
#define SPI_IOC_MESSAGE(n) (0x53000000u | (unsigned)(n))
#define SPI_IOC_WR_MODE 0x53100001u
#define SPI_IOC_WR_BITS_PER_WORD 0x53100003u
#define SPI_IOC_WR_MAX_SPEED_HZ 0x53100004u
int gx_mac_ioctl(int fd, unsigned long req, ...);
#define ioctl gx_mac_ioctl
