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
//  Based on https://raw.githubusercontent.com/raspberrypi/linux/rpi-3.10.y/Documentation/spi/spidev_test.c
//  SWD Protocol: https://annals-csis.org/proceedings/2012/pliks/279.pdf
//  See also: https://github.com/MarkDing/swd_programing_sram
//  Pi SPI Hardware: https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md
//  Pi SPI Kernel Driver: https://github.com/raspberrypi/linux/blob/rpi-3.12.y/drivers/spi/spi-bcm2708.c
//  BCM2835 Peripherals Datasheet: https://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2835/BCM2835-ARM-Peripherals.pdf
//  SWD mapped to SPI bytes: https://docs.google.com/spreadsheets/d/12oXe1MTTEZVIbdmFXsOgOXVFHCQnYVvIw6fRpIQZybg/edit#gid=0

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
 * SWD Line reset.
 *
 * SWD Line reset is at least 50 SWCLK cycles with SWDIO driven high,
 * followed by at least two idle (low) cycle.
 * Bits are stored (and transmitted) LSB-first.
 */
static const uint8_t swd_seq_line_reset[] = {
	/* At least 50 SWCLK cycles with SWDIO high */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* At least 2 idle (low) cycles */
	0x00,
};
static const unsigned swd_seq_line_reset_len = 64;

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
static const unsigned swd_seq_jtag_to_swd_len = 136;

//  End of https://github.com/ntfreak/openocd/blob/master/src/jtag/swd.h

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev1.1";
static uint8_t mode = 0  //  Note: LSB mode is not supported on Broadcom. We must flip LSB to MSB ourselves.
    //| SPI_CPOL   //  Clock polarity
    //| SPI_CPHA   //  Clock phase
    | SPI_NO_CS  //  1 device per bus, no Chip Select
    | SPI_3WIRE  //  Bidirectional mode, data in and out pin shared
    ;
static uint8_t bits = 8;
static uint32_t speed = 122000;  //  Previously 500000
static uint16_t delay = 0;

static void transfer(int fd)
{
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

static void print_usage(const char *prog)
{
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

static void parse_opts(int argc, char *argv[])
{
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

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;

	parse_opts(argc, argv);

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	transfer(fd);

	close(fd);

	return ret;
}