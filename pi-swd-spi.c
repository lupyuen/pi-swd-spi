/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */
//  Testing feasibility of SWD protocol implemented with Raspberry Pi Bidirectional SPI
//  Why implement SWD over Bidirectional SPI on Raspberry Pi?  Because SWD over Bit-Banging GPIO has timing issues that affect OpenOCD flashing...
//  https://gist.github.com/lupyuen/18e66c3e81e11050a10d1192c5b84bb0

//  Based on https://raw.githubusercontent.com/raspberrypi/linux/rpi-3.10.y/Documentation/spi/spidev_test.c
//  SWD Protocol: https://annals-csis.org/proceedings/2012/pliks/279.pdf
//  See also: https://github.com/MarkDing/swd_programing_sram
//  Pi SPI Hardware: https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md
//  Pi SPI Kernel Driver: https://github.com/raspberrypi/linux/blob/rpi-3.12.y/drivers/spi/spi-bcm2708.c
//  BCM2835 Peripherals Datasheet: https://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2835/BCM2835-ARM-Peripherals.pdf
//  SWD mapped to SPI bytes: https://docs.google.com/spreadsheets/d/12oXe1MTTEZVIbdmFXsOgOXVFHCQnYVvIw6fRpIQZybg/edit#gid=0

//  To test:
//  Connect SWDIO to MOSI (Pin P1-19, Yellow)
//  Connect SWDCLK to SCLK (Pin P1-23, Blue)
//  Connect 3.3V and GND
//  sudo raspi-config
//  Interfacing Options --> SPI --> Yes
//  Finish --> Yes
//  Compile and run: clear ; cd ~/pi-swd-spi ; git pull ; gcc -o pi-swd-spi pi-swd-spi.c ; ./pi-swd-spi

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

//  From https://github.com/ntfreak/openocd/blob/master/src/jtag/swd.h
//  Note: We must flip all bytes from LSB to MSB because LSB is not supported on Broadcom SPI.

/**
 * JTAG-to-SWD sequence.
 *
 * The JTAG-to-SWD sequence is at least 50 TCK/SWCLK cycles with TMS/SWDIO
 * high, putting either interface logic into reset state, followed by a
 * specific 16-bit sequence and finally a line reset in case the SWJ-DP was
 * already in SWD mode.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_jtag_to_swd[] = {
	/* At least 50 TCK/SWCLK cycles with TMS/SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* Switching sequence from JTAG to SWD */
	0x9e, 0xe7,
	/* At least 50 TCK/SWCLK cycles with TMS/SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* At least 2 idle (low) cycles */
	0x00,
};
static const unsigned swd_seq_jtag_to_swd_len = 136;  //  Number of bits

//  End of https://github.com/ntfreak/openocd/blob/master/src/jtag/swd.h

/// SWD Sequence to Read Register 0 (IDCODE), prepadded with 2 null bits bits to fill up 6 bytes. Byte-aligned, will not cause overrun error.
/// A transaction must be followed by another transaction or at least 8 idle cycles to ensure that data is clocked through the AP.
/// After clocking out the data parity bit, continue to clock the SW-DP serial interface until it has clocked out at least 8 more clock rising edges, before stopping the clock.
static const uint8_t  swd_read_idcode[]   = { 0xa5 };
static const unsigned swd_read_idcode_len = 8;  //  Number of bits

/// SWD Sequence to Read Register 0 (IDCODE), prepadded with 2 null bits bits to fill up 6 bytes. Byte-aligned, next request will not cause overrun error.
static const uint8_t  swd_read_idcode_prepadded[]   = { 0x00, 0x94, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };  //  With null byte (8 cycles idle) before and after
static const unsigned swd_read_idcode_prepadded_len = 64;  //  Number of bits

/// SWD Sequence to Read Register 4 (CTRL/STAT), with 2 trailing undefined bits short of 6 bytes. NOT byte-aligned, will cause overrun error.
static const uint8_t  swd_read_ctrlstat[]   = { 0x8d };
static const unsigned swd_read_ctrlstat_len = 8;  //  Number of bits

/// SWD Sequence to Write Register 0 (ABORT). Clears all sticky flags: 
/// STICKYORUN: overrun error flag,
/// WDATAERR: write data error flag,
/// STICKYERR: sticky error flag,
/// STICKYCMP: sticky compare flag.
/// Byte-aligned, will not cause overrun error.
static const uint8_t  swd_write_abort[]   = { 0x00, 0x81, 0xd3, 0x03, 0x00, 0x00, 0x00, 0x00 };  //  With null byte (8 cycles idle) before and after
static const unsigned swd_write_abort_len = 64;  //  Number of bits

//  SPI Configuration
static const char *device = "/dev/spidev0.0";  //  SPI device name. If missing, enable SPI in raspi-config.
static uint8_t mode = 0  //  Note: LSB mode is not supported on Broadcom. We must flip LSB to MSB ourselves.
    | SPI_NO_CS  //  1 device per bus, no Chip Select
    | SPI_3WIRE  //  Bidirectional mode, data in and out pin shared
    ;            //  Data is valid on first rising edge of the clock, so CPOL=0 and CPHA=0
static uint8_t bits = 8;         //  8 bits per word
static uint32_t speed = 1953000;  //  1,953 kHz. Previously 500000
static uint16_t delay = 0;       //  SPI driver latency: https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=19489

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s);

/// The byte at index i is the value of i with all bits flipped. https://stackoverflow.com/questions/746171/efficient-algorithm-for-bit-reversal-from-msb-lsb-to-lsb-msb-in-c
static const uint8_t reverse_byte[] = {  
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

#define MAX_SPI_SIZE 256
static uint8_t reverse_buf[MAX_SPI_SIZE];

/* From https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md:
    Bidirectional or "3-wire" mode is supported by the spi-bcm2835 kernel module. 
    Please note that in this mode, either the tx or rx field of the spi_transfer 
    struct must be a NULL pointer, since only half-duplex communication is possible. 
    Otherwise, the transfer will fail. The spidev_test.c source code does not consider 
    this correctly, and therefore does not work at all in 3-wire mode. */

/// Transmit len bytes of buf (assumed to be in LSB format) to the SPI device in MSB format
static void spi_transmit(int fd, const uint8_t *buf, unsigned int len) {
    //  Reverse LSB to MSB for entire buf into reversed buffer.
    if (len >= MAX_SPI_SIZE) { printf("len=%d ", len); pabort("spi_transmit overflow"); return; }
    for (unsigned int i = 0; i < len; i++) {
        uint8_t b = buf[i];
        reverse_buf[i] = reverse_byte[(uint8_t) b];
    }
    {
        printf("spi_transmit: len=%d\n", len);
        for (unsigned int i = 0; i < len; i++) {
            if (i > 0 && i % 8 == 0) { puts(""); }
            printf("%.2X ", reverse_buf[i]);
        }
        puts("");
    }
    //  Transmit the reversed buffer to SPI device.
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long) reverse_buf,
		.rx_buf = (unsigned long) NULL,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    //  Check SPI result.
	if (ret < 1) { pabort("spi_transmit failed"); }
}

/// Receive len bytes from SPI device (assumed to be in MSB format) and write into buf in LSB format
static void spi_receive(int fd, uint8_t *buf, unsigned int len) {
    //  Receive the reversed buffer from SPI device.
    printf("spi_receive: len=%d\n", len);
    if (len >= MAX_SPI_SIZE) { printf("len=%d ", len); pabort("spi_receive overflow"); return; }
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long) NULL,
		.rx_buf = (unsigned long) reverse_buf,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    //  Check SPI result.
	if (ret < 1) { pabort("spi_receive failed"); }
    //  Reverse MSB to LSB from reversed buffer into buf.
    for (unsigned int i = 0; i < len; i++) {
        uint8_t b = reverse_buf[i];
        buf[i] = reverse_byte[(uint8_t) b];
    }
    {
        for (unsigned int i = 0; i < len; i++) {
            if (i > 0 && i % 8 == 0) { puts(""); }
            printf("%.2X ", buf[i]);
        }
        puts("");
    }
}

/// Transmit preamble to resync target with host
static void transmit_resync(int fd) {
    //  Transmit JTAG-to-SWD sequence. Need to transmit every time because the SWD read/write command has extra 2 undefined bits that will confuse the target.
    puts("\nTransmit JTAG-to-SWD sequence...");
    spi_transmit(fd, swd_seq_jtag_to_swd, swd_seq_jtag_to_swd_len / 8);

    //  Transmit command to read Register 0 (IDCODE).  This is mandatory after JTAG-to-SWD sequence, according to SWD protocol.  We prepad with 2 null bits so that the next command will be byte-aligned.
    puts("\nTransmit prepadded command to read IDCODE...");
    spi_transmit(fd, swd_read_idcode_prepadded, swd_read_idcode_prepadded_len / 8);

    //  Transmit command to write Register 0 (ABORT) and clear all sticky flags.  Error flags must be cleared before sending next transaction to target.
    puts("\nTransmit command to write ABORT...");
    spi_transmit(fd, swd_write_abort, swd_write_abort_len / 8);
}

/// Read IDCODE register
static void read_idcode(int fd) {
    //  Transmit command to read Register 0 (IDCODE).
    puts("\nTransmit unpadded command to read IDCODE...");
    spi_transmit(fd, swd_read_idcode, swd_read_idcode_len / 8);

    //  Read response (38 bits)
    const int buf_size = 5;
    uint8_t buf[buf_size];
    puts("\nReceive value of IDCODE...");
    spi_receive(fd, buf, buf_size);
}

/// Read CTRL/STAT register
static void read_ctrlstat(int fd) {
    //  Transmit command to read Register 4 (CTRL/STAT).
    puts("\nTransmit unpadded command to read CTRL/STAT...");
    spi_transmit(fd, swd_read_ctrlstat, swd_read_ctrlstat_len / 8);

    //  Read response (38 bits)
    const int buf_size = 5;
    uint8_t buf[buf_size];
    puts("\nReceive value of CTRL/STAT...");
    spi_receive(fd, buf, buf_size);
}

/// Transmit and receive data to/from SPI device
static void spi_transfer(int fd) {
    for (int i = 0; i <= 1; i++) {  //  Test twice
        printf("\n---- Test #%d\n\n", i + 1);
        //  Must resync because previous request is not byte-aligned.
        transmit_resync(fd);
        read_idcode(fd);

        //  Must resync because previous request is not byte-aligned.
        transmit_resync(fd);
        read_ctrlstat(fd);
    }
}

int main(int argc, char *argv[]) {
	//  Previously: parse_opts(argc, argv);
	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

    //  Open SPI device.
	int fd = open(device, O_RDWR);
	if (fd < 0) { pabort("can't open device"); }

    //  Set SPI mode to read and write.
	int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) { pabort("can't set spi mode"); }
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1) { pabort("can't get spi mode"); }

    //  Set SPI read and write bits per word.
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) { pabort("can't set bits per word"); }
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1) { pabort("can't get bits per word"); }

    //  Set SPI read and write max speed.
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) { pabort("can't set max speed hz"); }
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) { pabort("can't get max speed hz"); }

    //  Transmit and receive to/from SPI device.
	spi_transfer(fd);

    //  Close SPI device.
	close(fd);
	return ret;
}

static void pabort(const char *s) {
	perror(s);
	abort();
}

#ifdef NOTUSED
Expected output:

spi mode: 80
bits per word: 8
max speed: 1953000 Hz (1953 KHz)

---- Test #1


Transmit JTAG-to-SWD sequence...
spi_transmit: len=17
FF FF FF FF FF FF FF 79 
E7 FF FF FF FF FF FF FF 
00 

Transmit prepadded command to read IDCODE...
spi_transmit: len=8
00 29 40 00 00 00 00 00 

Transmit command to write ABORT...
spi_transmit: len=8
00 81 CB C0 00 00 00 00 

Transmit unpadded command to read IDCODE...
spi_transmit: len=1
A5 

Receive value of IDCODE...
spi_receive: len=5
73 47 01 BA E2 

Transmit JTAG-to-SWD sequence...
spi_transmit: len=17
FF FF FF FF FF FF FF 79 
E7 FF FF FF FF FF FF FF 
00 

Transmit prepadded command to read IDCODE...
spi_transmit: len=8
00 29 40 00 00 00 00 00 

Transmit command to write ABORT...
spi_transmit: len=8
00 81 CB C0 00 00 00 00 

Transmit unpadded command to read CTRL/STAT...
spi_transmit: len=1
B1 

Receive value of CTRL/STAT...
spi_receive: len=5
03 00 00 00 EF 

---- Test #2


Transmit JTAG-to-SWD sequence...
spi_transmit: len=17
FF FF FF FF FF FF FF 79 
E7 FF FF FF FF FF FF FF 
00 

Transmit prepadded command to read IDCODE...
spi_transmit: len=8
00 29 40 00 00 00 00 00 

Transmit command to write ABORT...
spi_transmit: len=8
00 81 CB C0 00 00 00 00 

Transmit unpadded command to read IDCODE...
spi_transmit: len=1
A5 

Receive value of IDCODE...
spi_receive: len=5
73 47 01 BA E2 

Transmit JTAG-to-SWD sequence...
spi_transmit: len=17
FF FF FF FF FF FF FF 79 
E7 FF FF FF FF FF FF FF 
00 

Transmit prepadded command to read IDCODE...
spi_transmit: len=8
00 29 40 00 00 00 00 00 

Transmit command to write ABORT...
spi_transmit: len=8
00 81 CB C0 00 00 00 00 

Transmit unpadded command to read CTRL/STAT...
spi_transmit: len=1
B1 

Receive value of CTRL/STAT...
spi_receive: len=5
03 00 00 00 EF 

    static void transfer(int fd) {
        int ret;
        uint8_t tx[] = {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0x40, 0x00, 0x00, 0x00, 0x00, 0x95,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xAD,
            0xF0, 0x0D,
        };
        uint8_t rx[ARRAY_SIZE(tx)] = {0, };
        /* Bidirectional or "3-wire" mode is supported by the spi-bcm2835 kernel module. 
        Please note that in this mode, either the tx or rx field of the spi_transfer 
        struct must be a NULL pointer, since only half-duplex communication is possible. 
        Otherwise, the transfer will fail. The spidev_test.c source code does not consider 
        this correctly, and therefore does not work at all in 3-wire mode. */
        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)tx,
            .rx_buf = (unsigned long)rx,
            .len = ARRAY_SIZE(tx),
            .delay_usecs = delay,
            .speed_hz = speed,
            .bits_per_word = bits,
        };

        ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1)
            pabort("can't send spi message");

        for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
            if (!(ret % 6))
                puts("");
            printf("%.2X ", rx[ret]);
        }
        puts("");
    }

    static void print_usage(const char *prog) {
        printf("Usage: %s [-DsbdlHOLC3]\n", prog);
        puts("  -D --device   device to use (default /dev/spidev1.1)\n"
            "  -s --speed    max speed (Hz)\n"
            "  -d --delay    delay (usec)\n"
            "  -b --bpw      bits per word \n"
            "  -l --loop     loopback\n"
            "  -H --cpha     clock phase\n"
            "  -O --cpol     clock polarity\n"
            "  -L --lsb      least significant bit first\n"
            "  -C --cs-high  chip select active high\n"
            "  -3 --3wire    SI/SO signals shared\n");
        exit(1);
    }

    static void parse_opts(int argc, char *argv[]) {
        while (1) {
            static const struct option lopts[] = {
                { "device",  1, 0, 'D' },
                { "speed",   1, 0, 's' },
                { "delay",   1, 0, 'd' },
                { "bpw",     1, 0, 'b' },
                { "loop",    0, 0, 'l' },
                { "cpha",    0, 0, 'H' },
                { "cpol",    0, 0, 'O' },
                { "lsb",     0, 0, 'L' },
                { "cs-high", 0, 0, 'C' },
                { "3wire",   0, 0, '3' },
                { "no-cs",   0, 0, 'N' },
                { "ready",   0, 0, 'R' },
                { NULL, 0, 0, 0 },
            };
            int c;

            c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);

            if (c == -1)
                break;

            switch (c) {
            case 'D':
                device = optarg;
                break;
            case 's':
                speed = atoi(optarg);
                break;
            case 'd':
                delay = atoi(optarg);
                break;
            case 'b':
                bits = atoi(optarg);
                break;
            case 'l':
                mode |= SPI_LOOP;
                break;
            case 'H':
                mode |= SPI_CPHA;
                break;
            case 'O':
                mode |= SPI_CPOL;
                break;
            case 'L':
                mode |= SPI_LSB_FIRST;
                break;
            case 'C':
                mode |= SPI_CS_HIGH;
                break;
            case '3':
                mode |= SPI_3WIRE;
                break;
            case 'N':
                mode |= SPI_NO_CS;
                break;
            case 'R':
                mode |= SPI_READY;
                break;
            default:
                print_usage(argv[0]);
                break;
            }
        }
    }

    //  Enable debug log in openocd/src/jtag/drivers/bitbang.c:

    static void bitbang_exchange(bool rnw, uint8_t buf[], unsigned int offset, unsigned int bit_cnt)
    {
        { ////
            printf("****%s offset %d bits %2d:", rnw ? "target" : "host  ", offset, bit_cnt);
            if (!rnw && buf) {
                for (unsigned int i = 0; i < (bit_cnt + 7) / 8; i++) {
                    printf(" %02x", buf[i]);
                }
            }
            printf("\n");
        } ////
        //// LOG_DEBUG("bitbang_swd_read_reg");
#endif  // NOTUSED