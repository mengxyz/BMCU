#pragma once
#include "ch32v20x.h"
#include <Arduino.h>
#include "stdlib.h"
#include "Debug_log.h"
#include "Flash_saves.h"
#include "Motion_control.h"
#include "_bus_hardware.h"
#include "ams.h"
#include "ahub_bus.h"
#include "bambu_bus_ams.h"
#include "time64.h"
#include "many_soft_AS5600.h"
#include "ADC_DMA.h"
#include "rsa.h"

#define delay_any_us(time)\
{\
    const uint64_t _delay_any_div_time =(uint64_t)(8000000.0/time);\
    SysTick->SR &= ~(1 << 0);\
    SysTick->CMP = SystemCoreClock/_delay_any_div_time;\
    SysTick->CTLR |= (1 << 5) |(1 << 4)| (1 << 0);\
\
    while(!(SysTick->SR & 1));\
    SysTick->CTLR &= ~(1 << 0);\
}

#define delay_any_ms(time)\
{\
    const uint64_t _delay_any_div_time =(uint64_t)(80000.0/time);\
    SysTick->SR &= ~(1 << 0);\
    SysTick->CMP = SystemCoreClock/_delay_any_div_time;\
    SysTick->CTLR |= (1 << 5) |(1 << 4)| (1 << 0);\
\
    while(!(SysTick->SR & 1));\
    SysTick->CTLR &= ~(1 << 0);\
}
extern void MC_STU_RGB_set(unsigned char CHx,unsigned char R, unsigned char G, unsigned char B);
extern void MC_PULL_ONLINE_RGB_set(unsigned char CHx, unsigned char R, unsigned char G, unsigned char B);
extern void ams_datas_set_need_to_save();
//#include "AMCU.h"
