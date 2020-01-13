# pi-swd-spi
Implementation of SWD protocol with Raspberry Pi Bidirectional SPI

Why implement SWD over Bidirectional SPI on Raspberry Pi?  Because SWD over Bit-Banging GPIO has timing issues that affect OpenOCD flashing...

https://gist.github.com/lupyuen/18e66c3e81e11050a10d1192c5b84bb0

Based on SWD protocol from...

https://annals-csis.org/proceedings/2012/pliks/279.pdf
