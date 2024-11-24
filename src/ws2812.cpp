#include "WS2812.h"
#include <stdlib.h>

#define RGB_H() port->BSHR = pin
#define RGB_L() port->BCR = pin

void WS2812_class::init(unsigned char _num, uint32_t _GPIO_pin)
{
    num = _num;
    RGB_buf = new uint32_t[num * 3];
    GPIO_pin = _GPIO_pin;
    port = get_GPIO_Port(CH_PORT(digitalPinToPinName(GPIO_pin)));
    pin = CH_GPIO_PIN(digitalPinToPinName(GPIO_pin));
    RGB_L();

    pinMode(_GPIO_pin, OUTPUT);
    clear();
}
WS2812_class::~WS2812_class()
{
    delete RGB_buf;
}

void WS2812_class::clear(void)
{
    unsigned char i;
    for (i = 0; i < num * 3; i++)
    {
        RGB_buf[i] = 0;
    }
}
void WS2812_class::RST(void)
{
    RGB_L();
    delayMicroseconds(50);
}

void WS2812_class::updata()
{
    int i, j;
    int max = num * 3;
    uint32_t DATA;
    for (i = 0; i < max; i++)
    {
        DATA = RGB_buf[i];
        for (j = 0; j < 24; j++)
        {
            if ((DATA >> j) & 0x01)
                RGB_H();
            else
                RGB_L();
            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();//@144Mhz
        }
    }
    RST();
}
void WS2812_class::set_RGB(unsigned char R, unsigned char G, unsigned char B, unsigned char index)
{
    uint32_t DATA = 0;
    int i;
    for (i = 0; i < 8; i++)
    {
        DATA <<= 3;
        DATA |= 1;
        DATA |= ((G >> i) & 1) << 1;
    }
    RGB_buf[index + 0] = DATA;
    DATA = 0;
    for (i = 0; i < 8; i++)
    {
        DATA <<= 3;
        DATA |= 1;
        DATA |= ((R >> i) & 1) << 1;
    }
    RGB_buf[index + 1] = DATA;
    DATA = 0;
    for (i = 0; i < 8; i++)
    {
        DATA <<= 3;
        DATA |= 1;
        DATA |= ((B >> i) & 1) << 1;
    }
    RGB_buf[index + 2] = DATA;
}
