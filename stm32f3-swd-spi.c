//  For reference only: https://community.st.com/s/question/0D50X00009Xkeut/implementing-swd-with-spi-on-stm32f3
#include <stdint.h>

static void _SetSWDIOasOutput()
{
    LL_GPIO_SetPinMode(tgt_SWDIO_GPIO_Port, tgt_SWDIO_Pin, LL_GPIO_MODE_ALTERNATE);
}
static void _SetSWDIOasInput()
{
    LL_GPIO_SetPinMode(tgt_SWDIO_GPIO_Port, tgt_SWDIO_Pin, LL_GPIO_MODE_INPUT);
}
static inline uint32_t SWDIO_IN()
{
    uint8_t b;
    LL_GPIO_ResetOutputPin(tgt_SWDCLK_GPIO_Port, tgt_SWDCLK_Pin);
    b = LL_GPIO_IsInputPinSet(tgt_SWDIO_GPIO_Port, tgt_SWDIO_Pin);
    delay();
    LL_GPIO_SetOutputPin(tgt_SWDCLK_GPIO_Port, tgt_SWDCLK_Pin);
    return b;
}
static inline uint32_t SW_ShiftIn(uint8_t bits)
{
    int i;
    uint32_t in = 0;
    LL_GPIO_SetPinMode(tgt_SWDCLK_GPIO_Port, tgt_SWDCLK_Pin, LL_GPIO_MODE_OUTPUT);
    for (i = 0; i < bits; i++)
    {
        in = (in >> 1) | ((SWDIO_IN() & 1) << (bits - 1));
    }
    LL_GPIO_SetPinMode(tgt_SWDCLK_GPIO_Port, tgt_SWDCLK_Pin, LL_GPIO_MODE_ALTERNATE);
    return in;
}

static inline uint32_t SW_ShiftInBytes(uint8_t bytes)
{
    int i;
    uint32_t tmp;
    for (i = 0; i < bytes; i++)
    {
        LL_SPI_TransmitData8(SWD_SPI_DEVICE, 0xff);
    }
    for (i = 0; i < bytes; i++)
    {
        while (!LL_SPI_IsActiveFlag_RXNE(SWD_SPI_DEVICE))
            ;
        ((uint8_t *)&tmp)[i] = LL_SPI_ReceiveData8(SWD_SPI_DEVICE);
    }
    return tmp;
}
static inline void SW_ShiftOutBytes(uint32_t data, uint8_t bytes)
{
    int i;
    for (i = 0; i < bytes; i++)
    {
        LL_SPI_TransmitData8(SWD_SPI_DEVICE, data);
        data = data >> 8;
    }
    for (i = 0; i < bytes; i++)
    {
        while (!LL_SPI_IsActiveFlag_RXNE(SWD_SPI_DEVICE))
            ;
        LL_SPI_ReceiveData8(SWD_SPI_DEVICE);
    }
}
static inline uint32_t Parity(uint32_t x)
{
    uint32_t y;
    y = x ^ (x >> 1);
    y = y ^ (y >> 2);
    y = y ^ (y >> 4);
    y = y ^ (y >> 8);
    y = y ^ (y >> 16);
    return y & 1;
}
static uint32_t SWD_TransactionBB(uint32_t req, uint32_t *data)
{
    uint32_t ack;
    uint32_t pbit;
    SW_ShiftOutBytes(req, 1);       // Send header
    _SetSWDIOasInput();             // Set pin direction
    ack = (SW_ShiftIn(4) >> 1) & 7; // ACK, toss the turnaround bit
    switch (ack)
    {
    case SW_ACK_OK: // good to go
        if (req & SW_REQ_RnW)
        {                                // read
            *data = SW_ShiftInBytes(4);  // get data
            pbit = (SW_ShiftIn(2) >> 1); // get parity bit, toss turnaround
            if (pbit ^ Parity(*data))
            { // parity check
                ack = SW_ACK_PARITY_ERR;
            }
            _SetSWDIOasOutput(); // restore direction
        }
        else
        {                                       // write
            SW_ShiftIn(1);                      // turnaround
            _SetSWDIOasOutput();                // restore direction
            SW_ShiftOutBytes(*data, 4);         // data
            SW_ShiftOutBytes(Parity(*data), 1); // parity
        }
        break;
    case SW_ACK_WAIT:
    case SW_ACK_FAULT:
        SW_ShiftIn(1);       // turnaround
        _SetSWDIOasOutput(); // restore direction
        break;
    default:                 // no ack, back off in case of data phase
        SW_ShiftInBytes(4);  // data
        SW_ShiftIn(2);       // parity + turn
        _SetSWDIOasOutput(); // restore direction
    }
    return ack;
}