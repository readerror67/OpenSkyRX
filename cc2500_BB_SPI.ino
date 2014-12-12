//-------------------------------
//-------------------------------
//CC2500 SPI routines
//-------------------------------
//-------------------------------
#include <util/delay.h>

void cc2500_readFifo(uint8_t *dpbuffer, int len)
{
    ReadRegisterMulti(CC2500_3F_RXFIFO | CC2500_READ_BURST, dpbuffer, len);
}

//----------------------
static void ReadRegisterMulti(uint8_t address, uint8_t data[], uint8_t length)
{
    unsigned char i;

    CS_off;
    _spi_write(address);
    for (i = 0; i < length; i++) {
        data[i] = _spi_read();
    }
    CS_on;
}

//*********************************************

void CC2500_WriteRegisterMulti(uint8_t address, const uint8_t data[], uint8_t length)
{
    CS_off;
    _spi_write(CC2500_WRITE_BURST | address);
    for (int i = 0; i < length; i++) {
        _spi_write(data[i]);
    }
    CS_on;
}

void cc2500_writeFifo(uint8_t *dpbuffer, uint8_t len)
{
    cc2500_strobe(CC2500_SFTX);//0x3B
    CC2500_WriteRegisterMulti(CC2500_3F_TXFIFO, dpbuffer, len);
    cc2500_strobe(CC2500_STX);//0x35
}

//--------------------------------------
void _spi_write(uint8_t command)
{
    uint8_t n = 8;
    SCK_off;//SCK start low
    MO_off;
    while (n--) {
        if (command & 0x80)
            MO_on;
        else
            MO_off;
        SCK_on;
        NOP();
        SCK_off;
        command = command << 1;
    }
    MO_on;
}

//----------------------------
void cc2500_writeReg(uint8_t address, uint8_t data)  //same as 7105
{
    CS_off;
    _spi_write(address);
    NOP();
    _spi_write(data);
    CS_on;
}

uint8_t _spi_read(void)
{
    uint8_t result;
    uint8_t i;
    result = 0;
    for (i = 0; i < 8; i++) {
        if (MI_1) ///
            result = (result << 1) | 0x01;
        else
            result = result << 1;
        SCK_on;
        NOP();
        SCK_off;
        NOP();
    }
    return result;
}

//--------------------------------------------
unsigned char cc2500_readReg(unsigned char address)
{
    uint8_t result;
    CS_off;
    address |= 0x80; //bit 7 =1 for reading
    _spi_write(address);
    result = _spi_read();
    CS_on;
    return (result);
}
//------------------------
void cc2500_strobe(uint8_t address)
{
    CS_off;
    _spi_write(address);
    CS_on;
}
//------------------------
void cc2500_resetChip(void)
{
    // Toggle chip select signal
    CS_on;
    _delay_us(30);
    CS_off;
    _delay_us(30);
    CS_on;
    _delay_us(45);
    cc2500_strobe(CC2500_SRES);
    _delay_ms(100);
}