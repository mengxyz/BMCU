#include "Motion_control.h"

AS5600_soft_IIC_many MC_AS5600;
uint32_t AS5600_SCL[] = {PB15, PB14, PB13, PB12};
uint32_t AS5600_SDA[] = {PD0, PC15, PC14, PC13};
#define AS5600_PI 3.1415926535897932384626433832795
#define speed_filter_k 100
float speed_as5600[4] = {0, 0, 0, 0};

float MC_PULL_stu_raw[4] = {1.65, 1.65, 1.65, 1.65};
int MC_PULL_stu[4] = {0, 0, 0, 0};
float MC_ONLINE_key_stu_raw[4] = {0, 0, 0, 0};
int MC_ONLINE_key_stu[4] = {0, 0, 0, 0};
bool filament_channel_inserted[4] = {false, false, false, false}; // 通道是否插入

void MC_PULL_ONLINE_init()
{
    uint8_t online_stu[4];
    uint8_t pull_stu[4];

    // 分别上下拉，检测是否有悬空通道
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIOA->BCR = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    delay(1);
    online_stu[0] = GPIOA->INDR & GPIO_Pin_7;
    pull_stu[0] = GPIOA->INDR & GPIO_Pin_6;
    online_stu[1] = GPIOA->INDR & GPIO_Pin_5;
    pull_stu[1] = GPIOA->INDR & GPIO_Pin_4;
    online_stu[2] = GPIOA->INDR & GPIO_Pin_3;
    pull_stu[2] = GPIOA->INDR & GPIO_Pin_2;
    online_stu[3] = GPIOA->INDR & GPIO_Pin_1;
    pull_stu[3] = GPIOA->INDR & GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIOA->BSHR = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    delay(1);
    if ((online_stu[0] == (GPIOA->INDR & GPIO_Pin_7)) && (pull_stu[0] == (GPIOA->INDR & GPIO_Pin_6)))
        filament_channel_inserted[0] = true;
    else
        filament_channel_inserted[0] = false;
    if ((online_stu[1] == (GPIOA->INDR & GPIO_Pin_5)) && (pull_stu[1] == (GPIOA->INDR & GPIO_Pin_4)))
        filament_channel_inserted[1] = true;
    else
        filament_channel_inserted[1] = false;
    if ((online_stu[2] == (GPIOA->INDR & GPIO_Pin_3)) && (pull_stu[2] == (GPIOA->INDR & GPIO_Pin_2)))
        filament_channel_inserted[2] = true;
    else
        filament_channel_inserted[2] = false;
    if ((online_stu[3] == (GPIOA->INDR & GPIO_Pin_1)) && (pull_stu[3] == (GPIOA->INDR & GPIO_Pin_0)))
        filament_channel_inserted[3] = true;
    else
        filament_channel_inserted[3] = false;

    // 将IO切换为ADC模式
    ADC_DMA_init();
}

void MC_PULL_ONLINE_read()
{
    float *data = ADC_DMA_get_value();
    MC_PULL_stu_raw[3] = data[0];
    MC_ONLINE_key_stu_raw[3] = data[1];
    MC_PULL_stu_raw[2] = data[2];
    MC_ONLINE_key_stu_raw[2] = data[3];
    MC_PULL_stu_raw[1] = data[4];
    MC_ONLINE_key_stu_raw[1] = data[5];
    MC_PULL_stu_raw[0] = data[6];
    MC_ONLINE_key_stu_raw[0] = data[7];

    uint8_t num = ams[motion_control_ams_num].now_filament_num; // 当前通道号
    if ((num != 0xFF) && (num < 4))
    {
        float pressure_raw = MC_PULL_stu_raw[num];
        pressure_raw = ((float)((pressure_raw - 1.5) / (2.1 - 1.5))) * 0xFFFF; // 百分比,乘以16位
        int pressure;
        if (pressure_raw > 0xFFFF)
            pressure = 0xFFFF;
        else if (pressure_raw < 0)
            pressure = 0;
        else
            pressure = (int)pressure_raw;
        ams[motion_control_ams_num].pressure = pressure;
    }
    else
        ams[motion_control_ams_num].pressure = 0xFFFF;
    for (int i = 0; i < 4; i++)
    {

        if (MC_PULL_stu_raw[i] > 1.85) // 大于1.85V,表示压力过高
        {
            MC_PULL_stu[i] = 1;
        }
        else if (MC_PULL_stu_raw[i] < 1.45) // 小于1.45V，表示压力过低
        {
            MC_PULL_stu[i] = -1;
        }
        else // 1.45~1.85之间，在正常误差范围内，无需动作
        {
            MC_PULL_stu[i] = 0;
        }

        if ((MC_ONLINE_key_stu_raw[i] > 1.65) && (filament_channel_inserted[i])) // 大于1.65V，为高电平
        {
            MC_ONLINE_key_stu[i] = 1;
        }
        else // 小于1.65V，为低电平
        {
            MC_ONLINE_key_stu[i] = 0;
        }
    }
}

#define PWM_lim 1000

struct alignas(4) Motion_control_save_struct
{
    int Motion_control_dir[4];
    int check = 0x40614061;
} Motion_control_data_save;

#define Motion_control_save_flash_addr ((uint32_t)0x0800E000)
bool Motion_control_read()
{
    return Flash_reads(&Motion_control_data_save, sizeof(Motion_control_save_struct), Motion_control_save_flash_addr);
}
void Motion_control_save()
{
    Flash_saves(&Motion_control_data_save, sizeof(Motion_control_save_struct), Motion_control_save_flash_addr);
}

class MOTOR_PID
{

    float P = 0;
    float I = 0;
    float D = 0;
    float I_save = 0;
    float E_last = 0;
    float pid_MAX = PWM_lim;
    float pid_MIN = -PWM_lim;
    float pid_range = (pid_MAX - pid_MIN) / 2;

public:
    MOTOR_PID()
    {
        pid_MAX = PWM_lim;
        pid_MIN = -PWM_lim;
        pid_range = (pid_MAX - pid_MIN) / 2;
    }
    MOTOR_PID(float P_set, float I_set, float D_set)
    {
        init_PID(P_set, I_set, D_set);
        pid_MAX = PWM_lim;
        pid_MIN = -PWM_lim;
        pid_range = (pid_MAX - pid_MIN) / 2;
    }
    void init_PID(float P_set, float I_set, float D_set) // 注意，采用了PID独立的计算方法，I和D默认已乘P
    {
        P = P_set;
        I = I_set;
        D = D_set;
        I_save = 0;
        E_last = 0;
    }
    float caculate(float E, float time_E)
    {
        I_save += I * E * time_E;
        if (I_save > pid_range) // 对I限幅
            I_save = pid_range;
        if (I_save < -pid_range)
            I_save = -pid_range;

        float ouput_buf;
        if (time_E != 0) // 防止快速调用
            ouput_buf = P * E + I_save + D * (E - E_last) / time_E;
        else
            ouput_buf = P * E + I_save;

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

enum class filament_motion_enum
{
    filament_motion_send,
    filament_motion_redetect,
    filament_motion_slow_send,
    filament_motion_pull,
    filament_motion_stop,
    filament_motion_pressure_ctrl_on_use,
    filament_motion_pressure_ctrl_idle,
};
enum class pressure_control_enum
{
    less_pressure,
    all,
    over_pressure
};

class _MOTOR_CONTROL
{
public:
    filament_motion_enum motion = filament_motion_enum::filament_motion_stop;
    int CHx = 0;
    uint64_t motor_stop_time = 0;
    MOTOR_PID PID_speed = MOTOR_PID(2, 20, 0);
    MOTOR_PID PID_pressure = MOTOR_PID(2000, 0, 0);
    float pwm_zero = 500;
    float dir = 0;
    int x1 = 0;
    _MOTOR_CONTROL(int _CHx)
    {
        CHx = _CHx;
        motor_stop_time = 0;
        motion = filament_motion_enum::filament_motion_stop;
    }

    void set_pwm_zero(float _pwm_zero)
    {
        pwm_zero = _pwm_zero;
    }
    void set_motion(filament_motion_enum _motion, uint64_t over_time)
    {
        uint64_t time_now = get_time64();
        motor_stop_time = time_now + over_time;
        if (motion != _motion)
        {
            motion = _motion;
            PID_speed.clear();
        }
    }
    filament_motion_enum get_motion()
    {
        return motion;
    }
    float _get_x_by_pressure(float pressure_voltage, float control_voltage, float time_E, pressure_control_enum control_type)
    {
        float x = 0;
        switch (control_type)
        {
        case pressure_control_enum::all: // 全范围控制
        {
            x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            break;
        }
        case pressure_control_enum::less_pressure: // 仅低压控制
        {
            if (pressure_voltage < control_voltage)
            {
                x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            }
            break;
        }
        case pressure_control_enum::over_pressure: // 仅高压控制
        {
            if (pressure_voltage > control_voltage)
            {
                x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - control_voltage, time_E);
            }
            break;
        }
        }
        if (x > 0) // 将控制力转为平方增强，平方会消掉正负，需要判断
            x = x * x / 250;
        else
            x = -x * x / 250;
        return x;
    }
    void run(float time_E)
    {

        float speed_set = 0;
        float now_speed = speed_as5600[CHx];
        float x = 0;

        if (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle) // 在空闲状态
        {
            if (MC_PULL_stu[CHx] != 0)
            {
                x = dir * PID_pressure.caculate(MC_PULL_stu_raw[CHx] - 1.65, time_E);
            }
            else
            {
                x = 0;
                PID_pressure.clear();
            }
        }
        else if (motion == filament_motion_enum::filament_motion_redetect) // 退出到断料，需要重新进料
        {
            x = -dir * 900; // 立刻以高速送回去
        }
        else if (MC_ONLINE_key_stu[CHx] != 0) // 通道在运行状态，并且有耗材
        {
            if (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) // 在使用状态
            {
                if (MC_PULL_stu_raw[CHx] < 1.7)
                {
                    x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.7, time_E, pressure_control_enum::all);
                }
                else if (MC_PULL_stu_raw[CHx] > 1.75)
                {
                    x = _get_x_by_pressure(MC_PULL_stu_raw[CHx], 1.75, time_E, pressure_control_enum::over_pressure);
                }
            }
            else
            {
                if (motion == filament_motion_enum::filament_motion_stop) // 要求停止
                {
                    PID_speed.clear();
                    Motion_control_set_PWM(CHx, 0);
                    return;
                }
                if (motion == filament_motion_enum::filament_motion_send) // 送料中
                {
                    if (MC_PULL_stu_raw[CHx] < 1.8) // 压力主动到1.8V位置
                        speed_set = 50;
                    else
                    {
                        x = 0;
                        Motion_control_set_PWM(CHx, x);
                        return;
                    }
                }
                if (motion == filament_motion_enum::filament_motion_slow_send) // 要求缓慢送料
                {
                    speed_set = 3;
                }
                if (motion == filament_motion_enum::filament_motion_pull) // 回抽
                {
                    speed_set = -50;
                }
                x = dir * PID_speed.caculate(now_speed - speed_set, time_E);
            }
        }
        else // 运行过程中耗材用完，需要停止电机控制
        {
            x = 0;
        }

        if (x > 10)
            x += pwm_zero;
        else if (x < -10)
            x -= pwm_zero;
        else
            x = 0;

        if (x > PWM_lim)
        {
            x = PWM_lim;
        }
        if (x < -PWM_lim)
        {
            x = -PWM_lim;
        }

        Motion_control_set_PWM(CHx, x);
    }
};
_MOTOR_CONTROL MOTOR_CONTROL[4] = {_MOTOR_CONTROL(0), _MOTOR_CONTROL(1), _MOTOR_CONTROL(2), _MOTOR_CONTROL(3)};

void Motion_control_set_PWM(uint8_t CHx, int PWM) // 传递到硬件层控制电机的PWM
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
    else // PWM==0
    {
        set1 = 1000;
        set2 = 1000;
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

int32_t as5600_distance_save[4] = {0, 0, 0, 0};
void AS5600_distance_updata() // 读取as5600，更新相关的数据
{
    static uint64_t time_last = 0;
    uint64_t time_now;
    float T;
    do
    {
        time_now = get_time64();
    } while (time_now <= time_last); // T!=0
    T = (float)(time_now - time_last);
    MC_AS5600.updata_angle();
    for (int i = 0; i < 4; i++)
    {
        if ((MC_AS5600.online[i] == false))
        {
            as5600_distance_save[i] = 0;
            speed_as5600[i] = 0;
            continue;
        }

        int32_t cir_E = 0;
        int32_t last_distance = as5600_distance_save[i];
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

        distance_E = -(float)(now_distance - last_distance + cir_E) * AS5600_PI * 7.5 / 4096; // D=7.5mm，加负号是因为AS5600正对磁铁
        as5600_distance_save[i] = now_distance;

        float speedx = distance_E / T * 1000;
        // T = speed_filter_k / (T + speed_filter_k);
        speed_as5600[i] = speedx; // * (1 - T) + speed_as5600[i] * T; // mm/s
        ams[motion_control_ams_num].filament[i].meters += distance_E / 1000;
    }
    time_last = time_now;
}

enum filament_now_position_enum
{
    filament_idle,
    filament_sending_out,
    filament_using,
    filament_pulling_back,
    filament_redetect,
};
int filament_now_position[4];
float filament_pull_back_meters[4];

bool motor_motion_filamnet_pull_back_to_online_key() // 用于在模拟P系列AMS工作模式时将耗材退出到断料，完毕后将wait设置为false即可
{
    bool wait = false;
    
    for (int i = 0; i < 4; i++)
    {
        switch (filament_now_position[i])
        {
        case filament_pulling_back://回抽阶段判断
            MC_STU_RGB_set(i, 0xFF, 0x00, 0xFF);
            if (abs(ams[motion_control_ams_num].filament[i].meters - (filament_pull_back_meters[i])) >= motion_control_pull_back_distance) // 回抽距离超过设定值
            {
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100);
                filament_now_position[i] = filament_redetect;
            }
            else if (MC_ONLINE_key_stu[i] == 0)// 检测到断料
            {
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100);
                filament_now_position[i] = filament_redetect;
            }
            else//否则继续回抽
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pull, 100);
            
            
            wait = true;
            break;
        case filament_redetect: // 保证耗材重新送入
            MC_STU_RGB_set(i, 0xFF, 0xFF, 0x00);
            if (MC_ONLINE_key_stu[i] == 0)
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_redetect, 100);
            else
            {
                MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100);
                filament_now_position[i] = filament_idle;
                ams[motion_control_ams_num].filament_use_flag = 0x00;
                ams[motion_control_ams_num].filament[i].motion = _filament_motion::idle;
            }
            wait = true;
            break;
        default:
            break;
        }
    }
    return wait;
}
void motor_motion_switch() // 通道状态切换函数，只控制当前在使用的通道，其他通道设置为停止
{
    uint8_t num = ams[motion_control_ams_num].now_filament_num; // 当前通道号
    _filament_motion motion;
    if (num < 4)
        motion = ams[motion_control_ams_num].filament[num].motion;
    else
        motion = _filament_motion::idle;

    for (int i = 0; i < 4; i++)
    {
        if (i != num)
        {
            filament_now_position[i] = filament_idle;
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 1000);
        }
        else if (MC_ONLINE_key_stu[num] != 0) // 通道耗材未用完
        {
            switch (motion) // 判断模拟器状态
            {
            case _filament_motion::send_out: // 需要进料
                MC_STU_RGB_set(num, 0x00, 0xFF, 0x00);
                filament_now_position[num] = filament_sending_out;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_send, 100);
                break;
            case _filament_motion::pull_back:
                MC_STU_RGB_set(num, 0xFF, 0x00, 0xFF);
                filament_now_position[num] = filament_pulling_back;
                filament_pull_back_meters[num] = ams[motion_control_ams_num].filament[num].meters;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pull, 100);
                break;
            case _filament_motion::before_pull_back:
            case _filament_motion::on_use:
            {
                static uint64_t time_end = 0;
                uint64_t time_now = get_time64();
                if (filament_now_position[num] == filament_sending_out)
                {
                    filament_now_position[num] = filament_using;
                    time_end = time_now + 3000;
                }
                else if (filament_now_position[num] == filament_using)
                {
                    if (time_now > time_end)
                        MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_on_use, 20);
                    else
                        MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_slow_send, 100);
                }
                MC_STU_RGB_set(num, 0xFF, 0xFF, 0xFF);
                break;
            }
            case _filament_motion::idle:
                filament_now_position[num] = filament_idle;
                MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100);
                MC_STU_RGB_set(num, 0x00, 0x00, 0x37);
                break;
            }
        }
        else // 通道在使用过程中耗材用完
        {
            filament_now_position[num] = filament_idle;
            MOTOR_CONTROL[num].set_motion(filament_motion_enum::filament_motion_pressure_ctrl_idle, 100);
            MC_STU_RGB_set(num, 0x00, 0x00, 0x37);
        }
    }
}
// 根据AMS模拟器的信息，来调度电机
void motor_motion_run(int error)
{
    uint64_t time_now = get_time64();
    static uint64_t time_last = 0;
    float time_E = time_now - time_last; // 获取时间差（ms）
    time_E = time_E / 1000;              // 切换到单位s
    uint16_t device_type = bus_host_device_type;
    
    if (!error) // 正常模式
    {
        if (device_type == host_device_type_ams_lite) // AMS lite模式
        {
            #ifdef AMS_LITE_LONG_RETRACT
            if (!motor_motion_filamnet_pull_back_to_online_key()) // 优先等待是否有通道需要回退到断料
            {
                motor_motion_switch(); // 调度电机
            }
            #else
            motor_motion_switch(); // 调度电机
            #endif
        }
        else if (device_type == host_device_type_ams) // AMS模式
        {
            if (!motor_motion_filamnet_pull_back_to_online_key()) // 优先等待是否有通道需要回退到断料
            {
                motor_motion_switch(); // 调度电机
            }
        }
        else if (device_type == host_device_type_ahub) // ahub模式，由ahub控制退料距离
        {
            motor_motion_switch(); // 调度电机
        }
    }
    else // error模式
    {
        for (int i = 0; i < 4; i++)
            MOTOR_CONTROL[i].set_motion(filament_motion_enum::filament_motion_stop, 100); // 关闭电机
    }

    for (int i = 0; i < 4; i++)
    {
        /*if (!get_filament_online(i)) // 通道不在线则电机不允许工作
            MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);*/
        MOTOR_CONTROL[i].run(time_E); // 根据状态信息来驱动电机

        if (MC_PULL_stu[i] == 1)
        {
            MC_PULL_ONLINE_RGB_set(i, 0x10, 0x00, 0x00); // 压力过大，红灯
        }
        else if (MC_PULL_stu[i] == 0)
        {
            if (MC_ONLINE_key_stu[i] != 0)
            {
                MC_PULL_ONLINE_RGB_set(i, 0x10, 0x10, 0x10); // 有耗材，无压力，白色
            }
            else
            {
                MC_PULL_ONLINE_RGB_set(i, 0x00, 0x00, 0x00); // 无耗材，不亮灯
            }
        }
        else if (MC_PULL_stu[i] == -1)
        {
            MC_PULL_ONLINE_RGB_set(i, 0x00, 0x00, 0x10); // 压力过小，蓝灯
        }
    }
    time_last = time_now;
}
// 运动控制函数
void Motion_control_run(int error)
{
    MC_PULL_ONLINE_read();

    AS5600_distance_updata();
    for (int i = 0; i < 4; i++)
    {
        if ((MC_ONLINE_key_stu[i] != 0))
        {
            ams[motion_control_ams_num].filament[i].online = true;
        }
        else if ((filament_now_position[i] == filament_redetect) || (filament_now_position[i] == filament_pulling_back))
        {
            ams[motion_control_ams_num].filament[i].online = true;
        }
        else
        {
            ams[motion_control_ams_num].filament[i].online = false;
        }
    }
    if (error)
    {
        for (int i = 0; i < 4; i++)
        {
            ams[motion_control_ams_num].filament[i].online = false;
            if (MC_ONLINE_key_stu[i] != 0)
            {
                MC_STU_RGB_set(i, 0x00, 0x00, 0xFF);
            }
            else
            {
                MC_STU_RGB_set(i, 0x00, 0x00, 0x00);
            }
        }
    }
    else
        for (int i = 0; i < 4; i++)
            MC_STU_RGB_set(i, 0x00, 0x00, 0x37);

    motor_motion_run(error);

    for (int i = 0; i < 4; i++)
    {
        if ((MC_AS5600.online[i] == false) || (MC_AS5600.magnet_stu[i] == -1)) // AS5600 error
        {
            MC_STU_RGB_set(i, 0xFF, 0x00, 0x00);
        }
    }
}
// 设置PWM驱动电机
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
    TIM_TimeBaseStructure.TIM_Period = 999;  // 周期（x+1）
    TIM_TimeBaseStructure.TIM_Prescaler = 1; // 预分频（x+1）
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
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);       // TIM4不映射-CH1-PB6/CH2-PB7/CH3-PB8/CH4-PB9

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
// 获取PWM摩擦力零点（弃用，假设为50%占空比）
void MOTOR_get_pwm_zero()
{
    float pwm_zero[4] = {0, 0, 0, 0};
    MC_AS5600.updata_angle();

    int16_t last_angle[4];
    for (int index = 0; index < 4; index++)
    {
        last_angle[index] = MC_AS5600.raw_angle[index];
    }
    for (int pwm = 300; pwm < 1000; pwm += 10)
    {
        MC_AS5600.updata_angle();
        for (int index = 0; index < 4; index++)
        {

            if (pwm_zero[index] == 0)
            {
                if (abs(MC_AS5600.raw_angle[index] - last_angle[index]) > 50)
                {
                    pwm_zero[index] = pwm;
                    pwm_zero[index] *= 0.90;
                    Motion_control_set_PWM(index, 0);
                }
                else if ((MC_AS5600.online[index] == true))
                {
                    Motion_control_set_PWM(index, -pwm);
                }
            }
            else
            {
                Motion_control_set_PWM(index, 0);
            }
        }
        delay(100);
    }
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(pwm_zero[index]);
    }
}
// 将角度数值转化为角度差数值
int M5600_angle_dis(int16_t angle1, int16_t angle2)
{

    int cir_E = angle1 - angle2;
    if ((angle1 > 3072) && (angle2 <= 1024))
    {
        cir_E = -4096;
    }
    else if ((angle1 <= 1024) && (angle2 > 3072))
    {
        cir_E = 4096;
    }
    return cir_E;
}

// 测试电机运动方向
void MOTOR_get_dir()
{
    int dir[4] = {0, 0, 0, 0};
    bool done = false;
    bool have_data = Motion_control_read();
    if (!have_data)
    {
        for (int index = 0; index < 4; index++)
        {
            Motion_control_data_save.Motion_control_dir[index] = 0;
        }
    }
    MC_AS5600.updata_angle(); // 读取5600的初始角度值

    int16_t last_angle[4];
    for (int index = 0; index < 4; index++)
    {
        last_angle[index] = MC_AS5600.raw_angle[index];                  // 将初始角度值记录下来
        dir[index] = Motion_control_data_save.Motion_control_dir[index]; // 记录flash中的dir数据
    }
    // bool need_test = false; // 是否需要检测
    bool need_save = false; // 是否需要更新状态
    for (int index = 0; index < 4; index++)
    {
        if ((MC_AS5600.online[index] == true) && (filament_channel_inserted[index] = true)) // 有5600，并且通道在线
        {
            if (Motion_control_data_save.Motion_control_dir[index] == 0) // 之前测试结果为0，需要测试
            {
                Motion_control_set_PWM(index, 1000); // 打开电机
                // need_test = true;                    // 设置需要测试
                need_save = true; // 有状态更新
            }
        }
        else if (dir[index] != 0)
        {
            dir[index] = 0;   // 通道不在线，清空它的方向数据
            need_save = true; // 有状态更新
        }
    }
    int i = 0;
    while (done == false)
    {
        done = true;

        delay(10);                // 间隔10ms检测一次
        MC_AS5600.updata_angle(); // 更新角度数据

        if (i++ > 200) // 超过2s无响应
        {
            for (int index = 0; index < 4; index++)
            {
                Motion_control_set_PWM(index, 0);                       // 停止
                Motion_control_data_save.Motion_control_dir[index] = 0; // 方向设为0
            }
            break; // 跳出循环
        }
        for (int index = 0; index < 4; index++) // 遍历
        {
            if ((MC_AS5600.online[index] == true) && (Motion_control_data_save.Motion_control_dir[index] == 0)) // 对于新的通道
            {
                int angle_dis = M5600_angle_dis(MC_AS5600.raw_angle[index], last_angle[index]);
                if (abs(angle_dis) > 163) // 移动超过1mm
                {
                    Motion_control_set_PWM(index, 0); // 停止
                    if (angle_dis > 0)                // 这里AS600正对着磁铁，和背贴方向是反的
                    {
                        dir[index] = 1;
                    }
                    else
                    {
                        dir[index] = -1;
                    }
                }
                else
                {
                    done = false; // 没有移动。继续等待
                }
            }
        }
    }
    for (int index = 0; index < 4; index++) // 遍历四个电机
    {
        Motion_control_data_save.Motion_control_dir[index] = dir[index]; // 数据复制
    }
    if (need_save) // 如果需要保存数据
    {
        Motion_control_save(); // 数据保存
    }
}
// 初始化电机
void MOTOR_init()
{

    MC_PWM_init();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    // MOTOR_get_pwm_zero();
    MOTOR_get_dir();
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(500);
        MOTOR_CONTROL[index].dir = Motion_control_data_save.Motion_control_dir[index];
    }
}
extern void RGB_update();
void Motion_control_init() // 初始化所有运动和传感器
{
    ams[motion_control_ams_num].online = true;
    ams[motion_control_ams_num].ams_type = 0x03; // 0x03:xMCU，不具备烘干器
    MC_PULL_ONLINE_init();
    MC_PULL_ONLINE_read();

    MC_AS5600.init(AS5600_SCL, AS5600_SDA, 4);
    MC_AS5600.updata_angle();

    for (int i = 0; i < 4; i++)
    {
        if (!MC_AS5600.online[i]) // 用AS5600是否有信号来判断通道是否插入
        {
            filament_channel_inserted[i] = false;
        }
        if ((MC_PULL_stu_raw[i] > 3.0) || (MC_PULL_stu_raw[i] < 0.3)) // 用缓冲器位置是否正常来判断是否插入
        {
            filament_channel_inserted[i] = false;
        }
        as5600_distance_save[i] = MC_AS5600.raw_angle[i]; // 记录5600的初始角度
        filament_now_position[i] = filament_idle;         // 将通道初始状态设置为空闲
    }

    MOTOR_init();
    // DEBUG
    /*for (int i = 0; i < 4; i++)
    {
        filament_channel_inserted[i] = true;
    }*/
    //
    /*
    //这是一段阻塞的DEBUG代码
    while (1)
    {
        delay(10);
        MC_PULL_ONLINE_read();

        for (int i = 0; i < 4; i++)
        {
            MOTOR_CONTROL[i].set_motion(filament_motion_pressure_ctrl_on_use, 100);
            if (!get_filament_online(i)) // 通道不在线则电机不允许工作
                MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);
            MOTOR_CONTROL[i].run(0); // 根据状态信息来驱动电机
        }
        char s[100];
        int n = sprintf(s, "%d\n", (int)(MC_PULL_stu_raw[3] * 1000));
        DEBUG_num(s, n);
    }*/
}
