#include "Motion_control.h"

void MC_PWM_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 开启复用时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 开启TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 开启TIM3时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 开启TIM4时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 定时器基础配置
    TIM_TimeBaseStructure.TIM_Period = 999;  // 周期
    TIM_TimeBaseStructure.TIM_Prescaler = 9; // 预分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // PWM模式配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // 占空比
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure); // PA15
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // PB3
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // PB4
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); // PB5
    TIM_OC1Init(TIM4, &TIM_OCInitStructure); // PB6
    TIM_OC2Init(TIM4, &TIM_OCInitStructure); // PB7
    TIM_OC3Init(TIM4, &TIM_OCInitStructure); // PB8
    TIM_OC4Init(TIM4, &TIM_OCInitStructure); // PB9

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);    // TIM2完全映射-CH1-PA15/CH2-PB3
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // TIM3部分映射-CH1-PB4/CH2-PB5
    // GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);//TIM4不映射-CH1-PB6/CH2-PB7/CH3-PB8/CH4-PB9

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

uint8_t PULL_key_stu[4] = {0, 0, 0, 0};
uint8_t PULL_key_change[4] = {0, 0, 0, 0};
#define PWM_lim 1000

class MOTOR_PID
{
public:
    float P = 40;
    float I = 1;
    // float I = 1;
    float D = 0;
    // float D = 0.005;
    float I_save = 0;
    float E_last = 0;
    float pid_MAX = PWM_lim;
    float pid_MIN = -PWM_lim;
    float pid_range = (pid_MAX - pid_MIN) / 2;
    void init(float P_set, float I_set)
    {
        P = P_set;
        I = I_set;
        I_save = 0;
    }
    float caculate(float E, float time_E)
    {

        float I_save_set = (I_save + E * time_E);
        if ((abs(I * I_save_set) < pid_range)) // 对I限幅
            I_save = I_save_set;               // 线性I系数

        float ouput_buf = P * (E + I * (I_save) + D * (E - E_last) / time_E);
        if (ouput_buf > pid_MAX)
            ouput_buf = pid_MAX;
        if (ouput_buf < pid_MIN)
            ouput_buf = pid_MIN;

        E_last = E;
        return ouput_buf;
    }
    void clear()
    {
        I_save = 0;
        E_last = 0;
    }
};
class _MOTOR_CONTROL
{
public:
    int motion = 0;
    int CHx = 0;
    uint64_t motor_stop_time = 0;
    MOTOR_PID PID;

    _MOTOR_CONTROL(int _CHx)
    {
        CHx = _CHx;
        motor_stop_time = 0;
        motion = 0;
    }
    void set_motion(int _motion, uint64_t over_time)
    {
        uint64_t time_now = get_time64();
        motor_stop_time = time_now + over_time;
        motion = _motion;
    }
    int get_motion()
    {
        return motion;
    }
    void run(float now_speed)
    {
        uint64_t time_now = get_time64();
        static uint64_t time_last = 0;
        float speed_set = 0;
        if (time_now >= motor_stop_time)
        {
            motion = 0;
        }
        if ((motion == 0) || (get_filament_online(CHx) == false))
        {
            PID.clear();
            Motion_control_set_PWM(CHx, 0);
            return;
        }
        if (motion == 1) // send
        {
            speed_set = 10;
        }
        else if (motion == 2) // slowly send
        {
            speed_set = 2;
        }
        else if (motion == -1) // pull
        {
            speed_set = -10;
        }
        else if (motion == 100) // over pressure
        {
            speed_set = 10;
        }

        float x = PID.caculate(now_speed - speed_set, (float)(time_now - time_last) / 1000);
        if (x > 10)
            x += 200;
        else if (x < 10)
            x -= 200;
        else
            x = 0;
        if (x > PWM_lim)
            x = PWM_lim;
        if (x < -PWM_lim)
            x = -PWM_lim;
        Motion_control_set_PWM(CHx, -x);
        time_last = time_now;
    }
};
_MOTOR_CONTROL MOTOR_CONTROL[4] = {_MOTOR_CONTROL(0), _MOTOR_CONTROL(1), _MOTOR_CONTROL(2), _MOTOR_CONTROL(3)};
void MC_PULL_key_read(void)
{
    PULL_key_stu[3] = digitalRead(PB12);
    PULL_key_stu[2] = digitalRead(PB13);
    PULL_key_stu[1] = digitalRead(PB14);
    PULL_key_stu[0] = digitalRead(PB15);
}
void MC_PULL_key_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    MC_PULL_key_read();
}

uint8_t ONLINE_key_stu[4] = {0, 0, 0, 0};
uint8_t ONLINE_key_change[4] = {0, 0, 0, 0};
void MC_ONLINE_key_read(void)
{
    ONLINE_key_stu[3] = digitalRead(PC13);
    ONLINE_key_stu[2] = digitalRead(PC14);
    ONLINE_key_stu[1] = digitalRead(PC15);
    ONLINE_key_stu[0] = digitalRead(PD0);
    /*ONLINE_key_stu[3] = 0;
    ONLINE_key_stu[2] = 0;
    ONLINE_key_stu[1] = 0;
    ONLINE_key_stu[0] = 0;*/
}
void MC_ONLINE_key_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    MC_ONLINE_key_read();
}

void Motion_control_set_PWM(uint8_t CHx, int PWM)
{
    uint16_t set1 = 0, set2 = 0;
    if (PWM > 0)
    {
        set1 = PWM;
    }
    else if (PWM < 0)
    {
        set2 = -PWM;
    }
    switch (CHx)
    {
    case 3:
        TIM_SetCompare1(TIM2, set1);
        TIM_SetCompare2(TIM2, set2);
        break;
    case 2:
        TIM_SetCompare1(TIM3, set1);
        TIM_SetCompare2(TIM3, set2);
        break;
    case 1:
        TIM_SetCompare1(TIM4, set1);
        TIM_SetCompare2(TIM4, set2);
        break;
    case 0:
        TIM_SetCompare3(TIM4, set1);
        TIM_SetCompare4(TIM4, set2);
        break;
    }
}
AS5600_soft_IIC_many MC_AS5600;
// uint32_t AS5600_SCL[] = {PA0, PA2, PA4, PA6};
// uint32_t AS5600_SDA[] = {PA1, PA3, PA5, PA7};
uint32_t AS5600_SCL[] = {PA6, PA4, PA2, PA0};
uint32_t AS5600_SDA[] = {PA7, PA5, PA3, PA1};
void Motion_control_init()
{
    MC_PWM_init();
    MC_PULL_key_init();
    MC_ONLINE_key_init();
    MC_AS5600.init(AS5600_SCL, AS5600_SDA, 4);
    /*for (auto i : MOTOR_CONTROL)
    {
        i.stu = 0;
        i.set_stop_time = 0;
    }*/
    /*for (auto i : motor_caculate_PID)
    {
        i.P = 30;
        i.I = 0.1;
        i.D = 0;
    }*/
}
#define AS5600_PI 3.1415926535897932384626433832795
#define speed_filter_k 300
float speed_as5600[4] = {0, 0, 0, 0};
void AS5600_distance_updata()
{
    static int32_t distance_save[4] = {0, 0, 0, 0};
    static uint64_t time_last = 0;
    uint64_t time_now = get_time64();
    MC_AS5600.updata_angle();
    for (int i = 0; i < 4; i++)
    {
        if ((MC_AS5600.online[i] == false) || (MC_AS5600.magnet_stu[i] == -1))
        {
            distance_save[i] = 0;
            speed_as5600[i] = 0;
            continue;
        }

        int32_t cir_E = 0;
        int32_t last_distance = distance_save[i];
        int32_t now_distance = MC_AS5600.raw_angle[i];
        float distance_E;
        if ((now_distance > 3072) && (last_distance <= 1024))
        {
            cir_E = -4096;
        }
        else if ((now_distance <= 1024) && (last_distance > 3072))
        {
            cir_E = 4096;
        }

        distance_E = (float)(now_distance - last_distance + cir_E) * AS5600_PI * 7.5 / 4096; // D=7.5mm
        distance_save[i] = now_distance;
        float T = (float)(time_now - time_last);
        float speedx = distance_E / T * 1000;
        T = speed_filter_k / (T + speed_filter_k);
        speed_as5600[i] = speedx * (1 - T) + speed_as5600[i] * T; // mm/s
        if (get_filament_motion(i) != need_send_out)
            add_filament_meters(i, distance_E / 1000);
    }
    time_last = time_now;
}

void motor_motion_run()
{
    int num = get_now_filament_num();

    switch (get_filament_motion(num))
    {
    case need_send_out:
        RGB_set(num, 0x00, 0xFF, 0x00);
        MOTOR_CONTROL[num].set_motion(1, 100);
        break;
    case need_pull_back:
        RGB_set(num, 0xFF, 0x00, 0xFF);
        MOTOR_CONTROL[num].set_motion(-1, 100);
        break;
    case on_use:
        if (MOTOR_CONTROL[num].get_motion() == 1)
        {
            MOTOR_CONTROL[num].set_motion(2, 2000);
        }
        else if (MOTOR_CONTROL[num].get_motion() != 2)
        {
            if (PULL_key_stu[num] == 0)
                MOTOR_CONTROL[num].set_motion(100, 100);
        }
        RGB_set(num, 0xFF, 0xFF, 0xFF);
        break;
    case idle:
        MOTOR_CONTROL[num].set_motion(0, 100);
        RGB_set(num, 0x00, 0x00, 0x37);
        break;
    }
    for (int i = 0; i < 4; i++)
        MOTOR_CONTROL[i].run(speed_as5600[i]);
}

void Motion_control_run(int error)
{
    MC_PULL_key_read();
    MC_ONLINE_key_read();

    AS5600_distance_updata();
    for (int i = 0; i < 4; i++)
    {
        if ((ONLINE_key_stu[i] == 0) && (MC_AS5600.online[i] == true))
        {
            set_filament_online(i, true);
        }
        else
        {
            set_filament_online(i, false);
        }
    }
    if (error)
    {
        for (int i = 0; i < 4; i++)
        {
            set_filament_online(i, false);
            if (PULL_key_stu[i] == 0)
            {
                RGB_set(i, 0xFF, 0x00, 0x00);
                if (ONLINE_key_stu[i] == 0)
                {
                    RGB_set(i, 0xFF, 0x00, 0xFF);
                }
            }
            else if (ONLINE_key_stu[i] == 0)
            {
                RGB_set(i, 0x00, 0x00, 0xFF);
            }
            else
            {
                RGB_set(i, 0x00, 0x00, 0x00);
            }
        }
    }
    else
        for (int i = 0; i < 4; i++)
            RGB_set(i, 0x00, 0x00, 0x37);
    motor_motion_run();
}
