#include <Arduino.h>
#include "main.h"

extern void debug_send_run();
#include "WS2812.h"
WS2812_class SYS_RGB;
WS2812_class RGBOUT[4];

void RGB_init()
{
    SYS_RGB.init(1, PD1);
    RGBOUT[0].init(2, PA11);
    RGBOUT[1].init(2, PA8);
    RGBOUT[2].init(2, PB1);
    RGBOUT[3].init(2, PB0);
}
void RGB_update()
{
    uint64_t time = get_time64();
    static uint64_t time_set = 0;
    if (time > time_set)
    {
        time_set = time + 50;
        SYS_RGB.updata();
        RGBOUT[0].updata();
        RGBOUT[1].updata();
        RGBOUT[2].updata();
        RGBOUT[3].updata();
    }
}
void MC_STU_RGB_set(unsigned char CHx, unsigned char R, unsigned char G, unsigned char B)
{
    RGBOUT[CHx].set_RGB(R, G, B, 0);
}
void MC_PULL_ONLINE_RGB_set(unsigned char CHx, unsigned char R, unsigned char G, unsigned char B)
{
    RGBOUT[CHx].set_RGB(R, G, B, 1);
}

#define use_flash_addr ((uint32_t)0x0800F000)
bool ams_datas_read()
{
    return Flash_reads(&ams, sizeof(ams), use_flash_addr);
}
bool ams_datas_need_to_save = false;
void ams_datas_set_need_to_save()
{
    ams_datas_need_to_save = true;
}
void ams_datas_save_run()
{
    if (ams_datas_need_to_save)
    {
        Flash_saves(&ams, sizeof(ams), use_flash_addr);
        ams_datas_need_to_save = false;
    }
}

void setup()
{
    WWDG_DeInit();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE); // 关闭看门狗
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // 配置中断优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); // 1位抢占优先级:因为硬件压栈深度是2 , 0,1级中断都可以使用硬件压栈

    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    RGB_init();
    SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
    RGBOUT[0].set_RGB(0x00, 0x00, 0x00, 0);
    RGBOUT[1].set_RGB(0x00, 0x00, 0x00, 0);
    RGBOUT[2].set_RGB(0x00, 0x00, 0x00, 0);
    RGBOUT[3].set_RGB(0x00, 0x00, 0x00, 0);
    RGB_update();
    delay(50);
    DEBUG_init();
    // rsa_datas_init();
    ams_init();
    Flash_saves_init();
    if (ams_datas_read() == false)
    {
        ams_datas_set_need_to_save();
    }
    Motion_control_init();

    bus_init();
    // ahubus_init();
    DEBUG("RST");
}

// extern double distance_count;
uint8_t R = 0, G = 0, B = 0;
uint8_t T_to_tangle(uint32_t time)
{
    time = time % 256;
    if (time > 128)
        return 512 - time * 2;
    else
        return time * 2;
}

void loop()
{

    while (1)
    {
        ahubus_package_type ahub_stu = ahubus_run();
        bambubus_package_type bambubus_stu = bambubus_run();
        bus_port_to_host.recv_data_len = 0;
        bus_port_to_host.send_package();

        // int stu =-1;
        static int error = 0;
        bool motion_can_run = false;
        uint16_t device_type = bambubus_ams_address;
        if ((ahub_stu != ahubus_package_type::none) || (bambubus_stu != bambubus_package_type::none)) // have data/offline
        {
            motion_can_run = true;
            if ((ahub_stu != ahubus_package_type::error) || (bambubus_stu != bambubus_package_type::error)) // have data
            {
                error = 0;
                if (bambubus_stu == bambubus_package_type::heartbeat)
                {
                    if (device_type == host_device_type_ams_lite)
                    {
                        SYS_RGB.set_RGB(0x00, 0x00, 0x10, 0);
                        bus_host_device_type = host_device_type_ams_lite;
                    }
                    else if (device_type == host_device_type_ams)
                    {
                        SYS_RGB.set_RGB(0x10, 0x10, 0x00, 0);
                        bus_host_device_type = host_device_type_ams;
                    }
                    RGB_update();
                }
                if (ahub_stu == ahubus_package_type::heartbeat)
                {
                    SYS_RGB.set_RGB(0x00, 0x10, 0x00, 0);
                    bus_host_device_type = host_device_type_ahub;
                    RGB_update();
                }
                ams_datas_save_run();
            }
            else// offline
            {
                error = -1;
                SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
                RGB_update();
            }
        }
        else // wait for data
        {
        }
        // if (motion_can_run)
        Motion_control_run(error);
    }
}
