#include "bambu_bus_ams.h"
#include "crc_compute.h"
#include "stdio.h"
#include "string.h"

uint8_t bambubus_ams_map[4] = {0, 1, 2, 3};

crc8 bambubus_crc_8(0x39, 0x66, 0);
crc16 bambubus_crc_16(0x1021, 0x913D, 0);

uint16_t bambubus_ams_address = 0x0000;

uint8_t *bambubus_package_maker_buf = bus_port_to_host.send_data_buf; // 默认共享回应数据存储
int *bambubus_package_res_length = &bus_port_to_host.send_data_len;   // 回应数据包长度

bool package_check_crc16(uint8_t *data, int data_length)
{
    data_length -= 2;
    uint16_t num = bambubus_crc_16.crc_compute(data, data_length);
    if ((data[(data_length)] == (num & 0xFF)) && (data[(data_length + 1)] == ((num >> 8) & 0xFF)))
        return true;
    return false;
}

#include <stdio.h>
#include "ams.h"

void bambubus_init()
{
    // data_save.BambuBus_now_filament_num = 0xFF;
}

void package_add_crc(uint8_t *data, int send_data_length) // 为数据包添加crc校验
{
    if (data[1] & 0x80) // 获取数据包头位置
    {

        data[3] = bambubus_crc_8.crc_compute(data, 3); // 短帧头校验
    }
    else
    {
        data[6] = bambubus_crc_8.crc_compute(data, 6); // 长帧头校验
    }
    send_data_length -= 2;                                              // 校验位之前的数据长度
    uint16_t num = bambubus_crc_16.crc_compute(data, send_data_length); // 计算crc16校验
    data[(send_data_length)] = num & 0xFF;                              // crc16校验低字节
    data[(send_data_length + 1)] = num >> 8;                            // crc16校验高字节
}

struct bambubus_long_packge_data
{
    uint16_t package_number;
    uint16_t package_length;
    uint8_t crc8;
    uint16_t target_address;
    uint16_t source_address;
    uint16_t type;
    uint8_t *datas;
    uint16_t data_length;
} __attribute__((packed));

void bambubus_long_package_get(bambubus_long_packge_data *data)
{
    bambubus_package_maker_buf[0] = 0x3D;
    bambubus_package_maker_buf[1] = 0x00;
    data->package_length = data->data_length + 15;
    memcpy(bambubus_package_maker_buf + 2, data, 11);
    memcpy(bambubus_package_maker_buf + 13, data->datas, data->data_length);
    package_add_crc(bambubus_package_maker_buf, data->data_length + 15);
    *bambubus_package_res_length = data->data_length + 15;
}

void bambubus_long_package_analysis(uint8_t *buf, int data_length, bambubus_long_packge_data *data)
{
    memcpy(data, buf + 2, 11);
    data->datas = buf + 13;
    data->data_length = data_length - 15; // +2byte CRC16
}

bambubus_long_packge_data printer_data_long;
// 用于分辨bambubus数据包是否应该处理，类型是什么
bambubus_package_type get_packge_type(unsigned char *buf, int length)
{
    if (buf[0] != 0x3D)
    {
        return bambubus_package_type::none;
    }
    if (package_check_crc16(buf, length) == false)
    {
        return bambubus_package_type::none;
    }
    if (buf[1] == 0xC5)
    {

        switch (buf[4])
        {
        case 0x03:
            return bambubus_package_type::filament_motion_short;
        case 0x04:
            return bambubus_package_type::filament_motion_long;
        case 0x05:
            return bambubus_package_type::online_detect;
        case 0x06:
            return bambubus_package_type::REQx6;
        case 0x07:
            return bambubus_package_type::NFC_detect;
        case 0x08:
            return bambubus_package_type::set_filament_info;
        case 0x20:
            return bambubus_package_type::heartbeat;
        default:
            return bambubus_package_type::ETC;
        }
    }
    else if (buf[1] == 0x05)
    {
        bambubus_long_package_analysis(buf, length, &printer_data_long);
        if (printer_data_long.target_address == host_device_type_ams)
        {
#ifdef AMS_type_ams
            bambubus_ams_address = host_device_type_ams;
#else
            return bambubus_package_type::none;
#endif
        }
        else if (printer_data_long.target_address == host_device_type_ams_lite)
        {
#ifdef AMS_type_ams_lite
            bambubus_ams_address = host_device_type_ams_lite;
#else
            return bambubus_package_type::none;
#endif
        }
        else
        {
            return bambubus_package_type::none;
        }

        switch (printer_data_long.type)
        {
        case 0x21A:
            return bambubus_package_type::MC_online;
        case 0x211:
            return bambubus_package_type::read_filament_info;
        case 0x218:
            return bambubus_package_type::set_filament_info_type2;
        case 0x103:
            return bambubus_package_type::version;
        case 0x402:
            return bambubus_package_type::serial_number;
        case 0x40D:
            return bambubus_package_type::read_cert;
        case 0x40E:
            return bambubus_package_type::send_cert_verify;
        case 0x411:
            return bambubus_package_type::cert_datas_sync;
        default:
            return bambubus_package_type::ETC;
        }
    }
    return bambubus_package_type::none;
}
uint8_t package_num = 0;

uint8_t get_filament_left_char(_ams *ams)
{
    uint8_t data = 0;
    for (int i = 0; i < 4; i++)
    {
        if (ams->filament[i].online == true)
        {
            data |= (0x1 << i) << i; // 1<<(2*i)
            if (bambubus_ams_address == host_device_type_ams)
                if (ams->filament[i].motion != _filament_motion::idle)
                {
                    data |= (0x2 << i) << i; // 2<<(2*i)
                }
        }
    }
    return data;
}

bool set_motion(unsigned char read_num, unsigned char statu_flags, unsigned char fliment_motion_flag, uint8_t ams_num)
{
    static uint64_t time_last = 0;
    uint64_t time_now = get_time64();
    uint64_t time_used = time_now - time_last;
    time_last = time_now;
    _ams *ams_ptr = &ams[bambubus_ams_map[ams_num]];
    if (bambubus_ams_address == host_device_type_ams) // AMS08
    {
        if (read_num < 4)
        {
            if ((statu_flags == 0x03) && (fliment_motion_flag == 0x00)) // 03 00
            {
                if (ams_ptr->now_filament_num != read_num) // on change
                {
                    if (ams_ptr->now_filament_num < 4)
                    {
                        ams_ptr->filament[ams_ptr->now_filament_num].motion = _filament_motion::idle;
                        ams_ptr->filament_use_flag = 0x00;
                        ams_ptr->pressure = 0xFFFF;
                    }
                    bus_now_ams_num = bambubus_ams_map[ams_num];
                    ams_ptr->now_filament_num = read_num;
                }
                ams_ptr->filament[read_num].motion = _filament_motion::send_out;
                ams_ptr->filament_use_flag = 0x02;
                ams_ptr->pressure = 0x4700;
            }
            else if ((statu_flags == 0x09)) // 09 A5 / 09 3F -进料检测阶段
            {
                ams_ptr->filament_use_flag = 0x04;
                ams_ptr->pressure = 0x2B00;
            }
            else if ((statu_flags == 0x07) && (fliment_motion_flag == 0x7F)) // 07 7F
            {
                ams_ptr->filament[read_num].motion = _filament_motion::on_use;
                ams_ptr->filament_use_flag = 0x04;
                ams_ptr->pressure = 0x2B00;
            }
        }
        else if ((read_num == 0xFF))
        {
            if ((statu_flags == 0x03) && (fliment_motion_flag == 0x00)) // 03 00(FF)
            {

                if (ams_ptr->now_filament_num < 4)
                {
                    _filament *filament = &(ams_ptr->filament[ams_ptr->now_filament_num]);
                    if (filament->motion == _filament_motion::on_use)
                    {
                        filament->motion = _filament_motion::pull_back;
                        ams_ptr->filament_use_flag = 0x02;
                    }
                    ams_ptr->pressure = 0x4700;
                }
            }
            else
            {
                for (auto i = 0; i < 4; i++)
                {
                    ams_ptr->filament[i].motion = _filament_motion::idle;
                    ams_ptr->pressure = 0xFFFF;
                }
                ams_ptr->now_filament_num = read_num;
            }
        }
    }
    else if (bambubus_ams_address == host_device_type_ams_lite) // AMS lite
    {
        if (read_num < 4)
        {
            if ((statu_flags == 0x03) && (fliment_motion_flag == 0x3F)) // 03 3F
            {
                ams_ptr->filament[read_num].motion = _filament_motion::pull_back;
                ams_ptr->filament_use_flag = 0x00;
            }
            else if ((statu_flags == 0x03) && (fliment_motion_flag == 0xBF)) // 03 BF
            {
                bus_now_ams_num = bambubus_ams_map[ams_num];

                if (ams_ptr->filament[read_num].motion != _filament_motion::send_out)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        ams_ptr->filament[i].motion = _filament_motion::idle;
                    }
                    ams_ptr->now_filament_num = read_num;
                }
                ams_ptr->filament[read_num].motion = _filament_motion::send_out;
                ams_ptr->filament_use_flag = 0x02;
            }
            else if ((statu_flags == 0x07) && (fliment_motion_flag == 0x00)) // 07 00
            {
                bus_now_ams_num = bambubus_ams_map[ams_num];

                if ((ams_ptr->filament[read_num].motion == _filament_motion::send_out) || (ams_ptr->filament[read_num].motion == _filament_motion::idle))
                {
                    ams_ptr->filament[read_num].motion = _filament_motion::on_use;
                    ams_ptr->filament[read_num].meters_virtual_count = 0;
                    ams_ptr->now_filament_num = read_num;
                }
                else if (ams_ptr->filament[read_num].motion == _filament_motion::before_pull_back)
                {
                }
                else if (ams_ptr->filament[read_num].meters_virtual_count < 10000) // 10s virtual data
                {
                    ams_ptr->filament[read_num].meters += (float)time_used / 300000; // 3.333mm/s
                    ams_ptr->filament[read_num].meters_virtual_count += time_used;
                }
                if (ams_ptr->filament[read_num].motion == _filament_motion::on_use)
                {
                    ams_ptr->filament_use_flag = 0x04;
                }
                /*if (data_save.filament[read_num].motion_set == need_pull_back)
                {
                    data_save.filament[read_num].motion_set = idle;
                    data_save.filament_use_flag = 0x00;
                }*/
            }
            else if ((statu_flags == 0x07) && (fliment_motion_flag == 0x66)) // 07 66 printer ready return back filament
            {
                // ams->filament[read_num].motion = _filament_motion::before_pull_back;
            }
            else if ((statu_flags == 0x07) && (fliment_motion_flag == 0x26)) // 07 26 printer ready pull in filament
            {
                ams_ptr->filament_use_flag = 0x04;
            }
        }
        else if ((read_num == 0xFF) && (statu_flags == 0x01))
        {
            if (ams_ptr->now_filament_num < 4)
            {
                if (ams_ptr->filament[ams_ptr->now_filament_num].motion != _filament_motion::on_use)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        ams_ptr->filament[i].motion = _filament_motion::idle;
                    }
                    ams_ptr->filament_use_flag = 0x00;
                }
            }
            else
            {
                for (int i = 0; i < 4; i++)
                {
                    ams_ptr->filament[i].motion = _filament_motion::idle;
                }
                ams_ptr->filament_use_flag = 0x00;
            }
            ams_ptr->now_filament_num = read_num;
        }
    }
    else if (bambubus_ams_address == 0x0000) // none
    {
    }
    else
        return false;
    return true;
}
// 3D C5 0C C8 03 00 07 00 7F 02 36 54
struct bambubus_printer_motion_package_struct
{
    uint8_t magic_byte;
    uint8_t flag;
    uint8_t length;
    uint8_t crc8;
    uint8_t command;
    uint8_t ams_num;
    uint8_t statu_flag;
    uint8_t filamnet_channel;
    uint8_t motion_flag;
    uint8_t unknow;
    uint16_t crc16;
} __attribute__((packed));
struct bambubus_ams_motion_package_struct
{
    uint8_t magic_byte = 0x3D;
    uint8_t flag = 0xC0;
    uint8_t length = 0x2C;
    uint8_t crc8;
    uint8_t command = 0x03;
    uint8_t ams_num = 0;
    uint8_t unknow1 = 0x00;
    uint8_t filament_use_flag = 0x00;
    uint8_t filament_channel = 0x00;
    float meters = 0;
    uint16_t pressure = 0;
    uint16_t unknow2 = 0xFFFF;
    uint8_t unknow3[12];
    uint8_t filament_stu_flag = 0x00;
    uint32_t last1 = 0xFFFFFFFF;
    uint32_t last2 = 0x01010101;
    uint8_t filament_channel_2 = 0x00;
    uint8_t last4 = 0x00;
    uint16_t last5 = 0x0000;
    uint16_t crc16;
} __attribute__((packed));
// 3D F0 2C C1 03 00 00 00 FF 00 00 00 00 6F F0 FB FF 36 00 00 00 F8 FF F7 FF 00 00 27 00 55 F8 EE F9 F0 B7 BA B9 B2 00 00 00 00 88 E6
// 3D D0 2C D1 03 03 00 02 00 00 00 80 3F FF FF FF FF 36 00 00 00 00 00 00 00 00 00 27 00 55 FF FF FF FF 01 01 01 01 00 00 00 00 15 95
const bambubus_ams_motion_package_struct _bambubus_ams_motion_package_struct_init_data =
    {
        magic_byte : 0x3D,
        flag : 0xC0,
        length : 0x2C,
        command : 0x03,
        unknow1 : 0x00,
        unknow2 : 0xFFFF,
        unknow3 : {
            0x36,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x27,
            0x00,
        },
        last1 : 0xFFFFFFFF,
        last2 : 0x01010101,
        last4 : 0x00,
        last5 : 0x0000,
    };
void get_package_motion(bambubus_printer_motion_package_struct *package_recv)
{

    unsigned char ams_num = package_recv->ams_num;
    unsigned char filament_channel = package_recv->filamnet_channel;
    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }

    _ams *ams_ptr = &ams[bambubus_ams_map[ams_num]];
    if (!set_motion(filament_channel, package_recv->statu_flag, package_recv->motion_flag, ams_num))
        return;

    bambubus_ams_motion_package_struct *package_send = (bambubus_ams_motion_package_struct *)bambubus_package_maker_buf;
    memcpy(package_send, &_bambubus_ams_motion_package_struct_init_data, sizeof(bambubus_ams_motion_package_struct));
    package_send->flag = 0xC0 | (package_num << 3);
    package_send->ams_num = ams_num;
    package_send->filament_use_flag = ams_ptr->filament_use_flag;
    package_send->filament_channel = ams_ptr->now_filament_num;
    package_send->filament_channel_2 = ams_ptr->now_filament_num;
    {
        static float meters = 1;
        uint16_t pressure = 0xFFFF;
        if ((ams_ptr->now_filament_num != 0xFF) && (ams_ptr->now_filament_num < 4))
        {
            meters = ams_ptr->filament[ams_ptr->now_filament_num].meters;
            if (bambubus_ams_address == host_device_type_ams_lite)
            {
                meters = -meters;
            }
        }
        pressure = ams_ptr->pressure;
        package_send->meters = meters;
        package_send->pressure = pressure;
        package_send->filament_stu_flag = get_filament_left_char(ams_ptr);
    }
    if (package_num < 7)
        package_num++;
    else
        package_num = 0;

    package_add_crc(bambubus_package_maker_buf, sizeof(bambubus_ams_motion_package_struct));
    *bambubus_package_res_length = sizeof(bambubus_ams_motion_package_struct);
}
// 3D C5 0D F1 04 00 01 00 03 FF 00 B2 C4
struct bambubus_printer_stu_motion_package_struct
{
    uint8_t magic_byte;
    uint8_t flag;
    uint8_t length;
    uint8_t crc8;
    uint8_t command;
    uint8_t ams_num;
    uint8_t statu_flag;
    uint8_t motion_flag;
    uint8_t unknow1;
    uint8_t filamnet_channel;
    uint8_t unknow2;
    uint16_t crc16;
} __attribute__((packed));
int x = sizeof(bambubus_printer_stu_motion_package_struct);

struct bambubus_ams_stu_motion_package_struct
{
    uint8_t magic_byte = 0x3D;
    uint8_t flag = 0xC0;
    uint8_t length = 0x3C;
    uint8_t crc8;
    uint8_t command = 0x04;
    uint8_t ams_num_stu = 0;
    uint16_t temperature = 0; // 温度，单位0.1℃
    uint8_t humidity = 0;     // 湿度，单位%
    uint8_t filament_online_flag[3];
    uint8_t filament_channel_stu = 0x00;
    uint8_t filament_flag_wait_NFC = 0x00;
    uint8_t unknow_stu[3];
    uint8_t ams_num = 0;
    uint8_t unknow1 = 0x00;
    uint8_t filament_use_flag = 0x00;
    uint8_t filament_channel = 0x00;
    float meters = 0;
    uint16_t pressure = 0;
    uint16_t unknow2 = 0xFFFF;
    uint8_t unknow3[12];
    uint8_t filament_stu_flag = 0x00;
    uint32_t last1 = 0xFFFFFFFF;
    uint32_t last2 = 0x01010101;
    uint32_t last3 = 0x00000000;
    uint32_t last4 = 0xFFFFFFFF;
    uint16_t crc16;
} __attribute__((packed));

const bambubus_ams_stu_motion_package_struct _bambubus_ams_stu_motion_package_struct_init_data =
    {
        magic_byte : 0x3D,
        flag : 0xC0,
        length : 0x3C,
        command : 0x04,
        unknow_stu : {
            0x01,
            0x00,
            0x00,
        },
        unknow1 : 0x00,
        unknow2 : 0xFFFF,
        unknow3 : {
            0x36,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x27,
            0x00,
        },
        last1 : 0xFFFFFFFF,
        last2 : 0x01010101,
        last3 : 0x00000000,
        last4 : 0xFFFFFFFF
    };
void get_package_stu_motion(bambubus_printer_stu_motion_package_struct *package_recv)
{
    unsigned char filament_flag_on = 0x00;
    unsigned char filament_flag_NFC = 0x00;
    unsigned char ams_num = package_recv->ams_num;
    unsigned char filament_channel = package_recv->filamnet_channel;

    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }
    _ams *ams_ptr = &ams[bambubus_ams_map[ams_num]];
    for (auto i = 0; i < 4; i++)
    {
        if (ams_ptr->filament[i].online == true)
        {
            filament_flag_on |= 1 << i;
        }
    }
    if (!set_motion(filament_channel, package_recv->statu_flag, package_recv->motion_flag, ams_num))
        return;
    bambubus_ams_stu_motion_package_struct *package_send = (bambubus_ams_stu_motion_package_struct *)bambubus_package_maker_buf;
    memcpy(package_send, &_bambubus_ams_stu_motion_package_struct_init_data, sizeof(bambubus_ams_stu_motion_package_struct));

    int16_t temperature = (ams_ptr->filament[0].compartment_temperature);
    temperature += (ams_ptr->filament[1].compartment_temperature);
    temperature += (ams_ptr->filament[2].compartment_temperature);
    temperature += (ams_ptr->filament[3].compartment_temperature);
    temperature = (temperature * 10); // 转化为单位0.1℃
    if (temperature < 0)              // Bambubus 温度不能为负数
    {
        temperature = 0;
    }
    temperature = temperature >> 2; // 平均温度
    uint16_t humidity = (ams_ptr->filament[0].compartment_humidity);
    humidity += (ams_ptr->filament[1].compartment_humidity);
    humidity += (ams_ptr->filament[2].compartment_humidity);
    humidity += (ams_ptr->filament[3].compartment_humidity);
    humidity = humidity >> 2; // 平均湿度

    package_send->flag = 0xC0 | (package_num << 3);
    package_send->ams_num_stu = ams_num;
    package_send->temperature = temperature;
    package_send->humidity = humidity;
    package_send->filament_online_flag[0] = filament_flag_on - filament_flag_NFC;
    package_send->filament_online_flag[1] = filament_flag_on - filament_flag_NFC;
    package_send->filament_online_flag[2] = filament_flag_on - filament_flag_NFC;
    package_send->filament_channel_stu = filament_channel;
    package_send->filament_flag_wait_NFC = filament_flag_NFC;
    package_send->ams_num = ams_num;
    package_send->filament_use_flag = ams_ptr->filament_use_flag;
    package_send->filament_channel = ams_ptr->now_filament_num;
    {
        static float meters = 1;
        uint16_t pressure = 0xFFFF;
        if ((ams_ptr->now_filament_num != 0xFF) && (ams_ptr->now_filament_num < 4))
        {
            meters = ams_ptr->filament[ams_ptr->now_filament_num].meters;
            if (bambubus_ams_address == host_device_type_ams_lite)
            {
                meters = -meters;
            }
        }
        pressure = ams_ptr->pressure;
        package_send->meters = meters;
        package_send->pressure = pressure;
        package_send->filament_stu_flag = get_filament_left_char(ams_ptr);
    }
    package_add_crc(bambubus_package_maker_buf, sizeof(bambubus_ams_stu_motion_package_struct));
    if (package_num < 7)
        package_num++;
    else
        package_num = 0;
    *bambubus_package_res_length = sizeof(bambubus_ams_stu_motion_package_struct);
}

uint8_t online_detect_res[4][29] = {
    {0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x00,
     0x0D, 0x0E, 0xA0, 0x35, 0x35, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
     0x33, 0xF0},
    {0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x00,
     0x0D, 0x0E, 0xA1, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
     0x33, 0xF0},
    {0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x00,
     0x0D, 0x0E, 0xA2, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
     0x33, 0xF0},
    {0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x00,
     0x0D, 0x0E, 0xA3, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
     0x33, 0xF0}};

bool have_registered[4] = {false, false, false, false};

void get_package_online_detect(unsigned char *buf, int length)
{
    if ((buf[5] == 0x00)) // 注册AMS序号用
    {
        uint8_t count = 0;
        for (int i = 0; i < 4; i++)
        {
            if (have_registered[i] == true)
                continue;
            if (ams[bambubus_ams_map[i]].online != true)
                continue;

            online_detect_res[i][0] = 0x3D; // 帧头
            online_detect_res[i][1] = 0xC0; // flag
            online_detect_res[i][2] = 29;   // 数据长度-29字节
            online_detect_res[i][3] = 0xB4; // CRC8
            online_detect_res[i][4] = 0x05; // 命令号
            online_detect_res[i][5] = 0x00; // 命令号
            online_detect_res[i][6] = i;    // AMS号码
            package_add_crc(online_detect_res[i], 29);
            memcpy(bambubus_package_maker_buf, online_detect_res[i], 29);
            count = 29;
            *bambubus_package_res_length = count;
            bus_port_to_host.send_package();
            delay(1);
        }
    }

    if (buf[5] == 0x01)
    {
        uint8_t ams_num = buf[6];
        if (ams_num >= 4)
        {
            return;
        }
        if (ams[bambubus_ams_map[ams_num]].online != true)
        {
            have_registered[ams_num] = false;
            return;
        }
        online_detect_res[ams_num][0] = 0x3D;    // 帧头
        online_detect_res[ams_num][1] = 0xC0;    // flag
        online_detect_res[ams_num][2] = 29;      // 数据长度-29字节
        online_detect_res[ams_num][3] = 0xB4;    // CRC8
        online_detect_res[ams_num][4] = 0x05;    // 命令号
        online_detect_res[ams_num][5] = 0x01;    // 命令号
        online_detect_res[ams_num][6] = ams_num; // AMS号码
        
        package_add_crc(online_detect_res[ams_num], 29); // 添加校验

        if (memcmp(online_detect_res[ams_num] + 7, buf + 7, 20) == 0)
        {
            have_registered[ams_num] = true;
        }
        else
        {
            have_registered[ams_num] = false;
        }
        memcpy(bambubus_package_maker_buf, online_detect_res[ams_num], 29);
        *bambubus_package_res_length = 29;
    }
    // bambubus_package_res_length = 0;
}

unsigned char long_packge_MC_online[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void get_package_long_packge_MC_online(unsigned char *buf, int length)
{
    bambubus_long_packge_data data;
    uint8_t ams_num = printer_data_long.datas[0];

    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }

    bambubus_long_package_analysis(buf, length, &printer_data_long);

    data.datas = long_packge_MC_online;
    data.datas[0] = ams_num;
    data.data_length = sizeof(long_packge_MC_online);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    bambubus_long_package_get(&data);
}
unsigned char long_packge_filament[] =
    {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x47, 0x46, 0x42, 0x30, 0x30, 0x00, 0x00, 0x00,
        0x41, 0x42, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xDD, 0xB1, 0xD4, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x18, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void get_package_long_packge_filament(unsigned char *buf, int length)
{
    bambubus_long_packge_data data;

    uint8_t ams_num = printer_data_long.datas[0];
    uint8_t filament_num = printer_data_long.datas[1];

    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }
    _ams *ams_ptr = ams + bambubus_ams_map[ams_num];
    long_packge_filament[0] = ams_num;
    long_packge_filament[1] = filament_num;
    memcpy(long_packge_filament + 19, ams_ptr->filament[filament_num].bambubus_filament_id, sizeof(ams_ptr->filament[filament_num].bambubus_filament_id));
    memcpy(long_packge_filament + 27, ams_ptr->filament[filament_num].name, sizeof(ams_ptr->filament[filament_num].name));
    long_packge_filament[59] = ams_ptr->filament[filament_num].color_R;
    long_packge_filament[60] = ams_ptr->filament[filament_num].color_G;
    long_packge_filament[61] = ams_ptr->filament[filament_num].color_B;
    long_packge_filament[62] = ams_ptr->filament[filament_num].color_A;
    memcpy(long_packge_filament + 79, &ams_ptr->filament[filament_num].temperature_max, 2);
    memcpy(long_packge_filament + 81, &ams_ptr->filament[filament_num].temperature_min, 2);

    data.datas = long_packge_filament;
    data.data_length = sizeof(long_packge_filament);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    bambubus_long_package_get(&data);
}

unsigned char long_packge_version_serial_number[] = {16, // 序列号长度=15
                                                     '0', 'E', 'A', '0', '3', '0', '3', '0', '3', '0', '3', '0', '0', '0', '0',
                                                     '0', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                     0x0E, 0xA0, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, // serial_number#2
                                                     0x30, 0x30, 0x30, 0x30,
                                                     0xFF, 0xFF, 0xFF, 0xFF,
                                                     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBB, 0x44, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

void get_package_long_packge_serial_number(unsigned char *buf, int length)
{
    bambubus_long_packge_data data;
    uint8_t ams_num = printer_data_long.datas[33];
    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }
    long_packge_version_serial_number[4] = 0x30 + ams_num; // 防止SN重复
    long_packge_version_serial_number[34] = 0xA0 + ams_num;
    if ((bambubus_ams_address != host_device_type_ams) && (bambubus_ams_address != host_device_type_ams_lite))
    {
        return;
    }

    // long_packge_version_serial_number[0] = ams_ptr->serial_number_length;

    // memcpy(long_packge_version_serial_number + 1, ams_ptr->serial_number, ams_ptr->serial_number_length);
    data.datas = long_packge_version_serial_number;
    data.data_length = sizeof(long_packge_version_serial_number);
    data.datas[65] = ams_num;

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    bambubus_long_package_get(&data);
}

unsigned char long_packge_version_version_and_name_AMS_lite[] = {0x00, 0x00, 0x00, 0x01, // verison number
                                                                 0x41, 0x4D, 0x53, 0x5F, 0x46, 0x31, 0x30, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char long_packge_version_version_and_name_AMS08[] = {0x00, 0x00, 0x00, 0x00, // verison number
                                                              0x41, 0x4D, 0x53, 0x30, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void get_package_long_packge_version(unsigned char *buf, int length)
{
    bambubus_long_packge_data data;
    uint8_t ams_num = printer_data_long.datas[0];

    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }
    unsigned char *long_packge_version_version_and_name;

    if (bambubus_ams_address == host_device_type_ams)
    {
        long_packge_version_version_and_name = long_packge_version_version_and_name_AMS08;
    }
    else if (bambubus_ams_address == host_device_type_ams_lite)
    {
        long_packge_version_version_and_name = long_packge_version_version_and_name_AMS_lite;
    }
    else
    {
        return;
    }

    data.datas = long_packge_version_version_and_name;
    data.data_length = sizeof(long_packge_version_version_and_name_AMS08);
    data.datas[20] = ams_num;

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    bambubus_long_package_get(&data);
}
unsigned char s = 0x01;

unsigned char set_filament_res[] = {0x3D, 0xC0, 0x08, 0xB2, 0x08, 0x60, 0xB4, 0x04};
void get_package_set_filament(unsigned char *buf, int length)
{
    uint8_t read_num = buf[5];
    uint8_t ams_num = read_num & 0xF0;
    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }
    _ams *ams_ptr = ams + bambubus_ams_map[ams_num];
    read_num = read_num & 0x0F;
    memcpy(ams_ptr->filament[read_num].bambubus_filament_id, buf + 7, sizeof(ams_ptr->filament[read_num].bambubus_filament_id));
    ams_ptr->filament[read_num].color_R = buf[15];
    ams_ptr->filament[read_num].color_G = buf[16];
    ams_ptr->filament[read_num].color_B = buf[17];
    ams_ptr->filament[read_num].color_A = buf[18];
    memcpy(&ams_ptr->filament[read_num].temperature_min, buf + 19, 2);
    memcpy(&ams_ptr->filament[read_num].temperature_max, buf + 21, 2);
    memcpy(&ams_ptr->filament[read_num].name, buf + 23, sizeof(ams_ptr->filament[read_num].name));
    memcpy(bambubus_package_maker_buf, set_filament_res, sizeof(set_filament_res));
    *bambubus_package_res_length = sizeof(set_filament_res);
}
unsigned char set_filament_res_type2[] = {0x00, 0x00, 0x00};
void get_package_set_filament_type2(unsigned char *buf, int length)
{
    bambubus_long_packge_data data;
    uint8_t ams_num = printer_data_long.datas[0];
    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }
    _ams *ams_ptr = ams + bambubus_ams_map[ams_num];
    uint8_t read_num = printer_data_long.datas[1];
    memcpy(ams_ptr->filament[read_num].bambubus_filament_id, printer_data_long.datas + 2, sizeof(ams_ptr->filament[read_num].bambubus_filament_id));
    ams_ptr->filament[read_num].color_R = printer_data_long.datas[10];
    ams_ptr->filament[read_num].color_G = printer_data_long.datas[11];
    ams_ptr->filament[read_num].color_B = printer_data_long.datas[12];
    ams_ptr->filament[read_num].color_A = printer_data_long.datas[13];

    memcpy(&ams_ptr->filament[read_num].temperature_min, printer_data_long.datas + 14, 2);
    memcpy(&ams_ptr->filament[read_num].temperature_max, printer_data_long.datas + 16, 2);
    memcpy(ams_ptr->filament[read_num].name, printer_data_long.datas + 18, 16);

    set_filament_res_type2[0] = ams_num;
    set_filament_res_type2[1] = read_num;
    set_filament_res_type2[2] = 0x00;
    data.datas = set_filament_res_type2;
    data.data_length = sizeof(set_filament_res_type2);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    bambubus_long_package_get(&data);
}

int cert_length = 800;
extern const unsigned char ams_lite_cert[800];
uint8_t send_cert_times = 0;          // 分多次发送证书，进行计数
bool have_cert_req = false;           // 是否已经收到了证书请求
uint16_t cert_req_package_number = 0; // 证书请求包的包序号
uint8_t cert_req_ams_num = 0;         // 证书请求包的ams号
void get_package_cert_req(unsigned char *buf, int length)
{
    uint8_t ams_num = printer_data_long.datas[4];
    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }
    cert_req_package_number = printer_data_long.package_number;
    have_cert_req = true;
    send_cert_times = 0;
    cert_req_ams_num = ams_num;
}

void get_package_send_cert()
{
    uint16_t send_cert_base_adr = 0; // 此次发送的起始地址
    if (have_cert_req == false)
    {
        return;
    }
    if (send_cert_times < cert_length / 0xA0)
    {
        send_cert_base_adr = send_cert_times * 0xA0;
        send_cert_times++;
    }
    else
    {
        send_cert_times = 0;
        have_cert_req = false;
        return;
    }
    uint8_t ams_num = cert_req_ams_num;
    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }

    bambubus_long_packge_data data;
    data.package_length = 0xBB;                  // 187字节
    data.data_length = data.package_length - 15; // 172字节
    data.package_number = cert_req_package_number;
    data.type = 0x40D; // 证书内容命令号
    data.source_address = printer_data_long.target_address;
    data.target_address = 0x0A00;

    bambubus_package_maker_buf[0] = 0x3D;
    bambubus_package_maker_buf[1] = 0x00;
    memcpy(bambubus_package_maker_buf + 2, &data, 11);
    bambubus_package_maker_buf[13] = 0x01;
    bambubus_package_maker_buf[14] = 0x00;
    bambubus_package_maker_buf[15] = 0xA8; // 168字节
    bambubus_package_maker_buf[16] = 0x00;
    bambubus_package_maker_buf[17] = ams_num;
    bambubus_package_maker_buf[18] = 0x00;
    bambubus_package_maker_buf[19] = 0x20;
    bambubus_package_maker_buf[20] = 0x03;
    bambubus_package_maker_buf[21] = send_cert_base_adr & 0xFF;
    bambubus_package_maker_buf[22] = (send_cert_base_adr >> 8) & 0xFF;
    bambubus_package_maker_buf[23] = 0xA0; // 160字节
    bambubus_package_maker_buf[24] = 0x00;
    memcpy(bambubus_package_maker_buf + 25, ams_lite_cert + send_cert_base_adr, 0xA0);
    package_add_crc(bambubus_package_maker_buf, data.package_length);
    *bambubus_package_res_length = data.package_length;
}
uint8_t cert_verfy_datas[32 + 19] = {0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01, 0x05, 0x00, 0x04, 0x20};
uint8_t cert_verfy_datas_res[256] = {};
uint8_t cert_verfy_times = 0;           // 分四次发送验证结果，进行计数
bool have_cert_verfy_req = false;       // 是否已经收到了验证请求
uint16_t cert_verfy_package_number = 0; // 验证请求包的包序号
uint8_t cert_verfy_ams_num = 0;         // 验证请求包的ams号
void get_package_cert_verfy(unsigned char *buf, int length)
{
    uint8_t ams_num = printer_data_long.datas[4];
    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }
    cert_verfy_package_number = printer_data_long.package_number;
    have_cert_verfy_req = true;
    cert_verfy_times = 0;
    cert_verfy_ams_num = ams_num;
    memcpy(cert_verfy_datas + 19, printer_data_long.datas + 5, 32);
    // 计算验证结果
    uint32_t cert_verfy_datas_res_len = 256;
    rsa_private_encrypt(cert_verfy_datas_res, &cert_verfy_datas_res_len, cert_verfy_datas, 32 + 19, &sk);
}

void get_package_send_cert_verfy()
{
    uint16_t send_cert_verfy_base_adr = 0; // 此次发送的起始地址
    if (have_cert_verfy_req == false)
    {
        return;
    }
    if (cert_verfy_times < 4) // 0-3一共四次
    {
        send_cert_verfy_base_adr = cert_verfy_times * 64;
        cert_verfy_times++;
    }
    else
    {
        cert_verfy_times = 0;
        have_cert_verfy_req = false;
        return;
    }
    uint8_t ams_num = cert_verfy_ams_num;
    if ((ams_num >= 4) || (ams[bambubus_ams_map[ams_num]].online != true))
    {
        return;
    }

    bambubus_long_packge_data data;
    data.package_length = 0x5B;                  // 91字节
    data.data_length = data.package_length - 15; // 76字节
    data.package_number = cert_req_package_number;
    data.type = 0x40E; // 验证命令号
    data.source_address = printer_data_long.target_address;
    data.target_address = 0x0A00;

    bambubus_package_maker_buf[0] = 0x3D;
    bambubus_package_maker_buf[1] = 0x00;
    memcpy(bambubus_package_maker_buf + 2, &data, 11);
    bambubus_package_maker_buf[13] = 0x01;
    bambubus_package_maker_buf[14] = 0x00;
    bambubus_package_maker_buf[15] = 0x48; // 72字节
    bambubus_package_maker_buf[16] = 0x00;
    bambubus_package_maker_buf[17] = ams_num;
    bambubus_package_maker_buf[18] = 0x00;
    bambubus_package_maker_buf[19] = 0x00;
    bambubus_package_maker_buf[20] = 0x01;
    bambubus_package_maker_buf[21] = send_cert_verfy_base_adr & 0xFF;
    bambubus_package_maker_buf[22] = (send_cert_verfy_base_adr >> 8) & 0xFF;
    bambubus_package_maker_buf[23] = 0x40; // 64字节
    bambubus_package_maker_buf[24] = 0x00;
    memcpy(bambubus_package_maker_buf + 25, cert_verfy_datas_res + send_cert_verfy_base_adr, 0x40);
    package_add_crc(bambubus_package_maker_buf, data.package_length);
    *bambubus_package_res_length = data.package_length;
}
void delay_before_send()
{
    int i = 0;
    while (i < 1800)
    {
        i++;
        __NOP();
    }
}

bambubus_package_type bambubus_run()
{
    bambubus_package_type stu = bambubus_package_type::none;
    static uint64_t time_set = 0;
    static uint64_t time_motion = 0;

    uint64_t timex = get_time64();

    if ((bus_port_to_host.recv_data_len > 0) && (bus_port_to_host.bus_recv_data_ptr[0] == 0x3D))
    {
        uint8_t *bambubus_data_buf = bus_port_to_host.bus_recv_data_ptr;
        int Bambubus_recv_data_length = bus_port_to_host.recv_data_len;

        stu = get_packge_type(bambubus_data_buf, Bambubus_recv_data_length); // have_data

        switch (stu)
        {
        case bambubus_package_type::heartbeat:
            time_set = timex + 1000;
            break;
        case bambubus_package_type::filament_motion_short:
            get_package_motion((bambubus_printer_motion_package_struct *)bambubus_data_buf);
            break;
        case bambubus_package_type::filament_motion_long:
            get_package_stu_motion((bambubus_printer_stu_motion_package_struct *)bambubus_data_buf);
            time_motion = timex + 1000;
            break;
        case bambubus_package_type::online_detect:
            get_package_online_detect(bambubus_data_buf, Bambubus_recv_data_length);
            break;
        case bambubus_package_type::REQx6:
            // send_for_REQx6(buf_X, data_length);
            break;
        case bambubus_package_type::MC_online:
            get_package_long_packge_MC_online(bambubus_data_buf, Bambubus_recv_data_length);
            break;
        case bambubus_package_type::read_filament_info:
            get_package_long_packge_filament(bambubus_data_buf, Bambubus_recv_data_length);
            break;
        case bambubus_package_type::version:
            get_package_long_packge_version(bambubus_data_buf, Bambubus_recv_data_length);
            break;
        case bambubus_package_type::serial_number:
            get_package_long_packge_serial_number(bambubus_data_buf, Bambubus_recv_data_length);
            break;
        case bambubus_package_type::NFC_detect:
            // send_for_NFC_detect(buf_X, data_length);
            break;
        case bambubus_package_type::set_filament_info:
            get_package_set_filament(bambubus_data_buf, Bambubus_recv_data_length);
            ams_datas_set_need_to_save();
            break;
        case bambubus_package_type::set_filament_info_type2:
            get_package_set_filament_type2(bambubus_data_buf, Bambubus_recv_data_length);
            ams_datas_set_need_to_save();
            break;
        case bambubus_package_type::read_cert:
            // get_package_cert_req(bambubus_data_buf, Bambubus_recv_data_length);
            break;
        case bambubus_package_type::send_cert_verify:
            // get_package_cert_verfy(bambubus_data_buf, Bambubus_recv_data_length);
            break;
        case bambubus_package_type::cert_datas_sync:
            // get_package_send_cert();
            // get_package_send_cert_verfy();
            break;
        default:
            break;
        }
        bus_port_to_host.recv_data_len = 0;
        delay_before_send();
    }
    if (timex > time_set)
    {
        stu = bambubus_package_type::error; // offline
    }
    if (timex > time_motion)
    {
    }
    return stu;
}

// 证书-800字节
const unsigned char ams_lite_cert[800] = {
    0x30, 0x82, 0x02, 0xE3, 0x30, 0x82, 0x01, 0xCB, 0x02, 0x14, 0x1A, 0x91, 0xAA, 0xED, 0x91, 0x2E, 0x9E, 0x4F, 0xDF, 0x1B,
    0xE2, 0x12, 0x38, 0x04, 0x0A, 0x82, 0x29, 0x50, 0x93, 0x64, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D,
    0x01, 0x01, 0x0B, 0x05, 0x00, 0x30, 0x42, 0x31, 0x0B, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x43, 0x4E,
    0x31, 0x22, 0x30, 0x20, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x0C, 0x19, 0x42, 0x42, 0x4C, 0x20, 0x54, 0x65, 0x63, 0x68, 0x6E,
    0x6F, 0x6C, 0x6F, 0x67, 0x69, 0x65, 0x73, 0x20, 0x43, 0x6F, 0x2E, 0x2C, 0x20, 0x4C, 0x74, 0x64, 0x31, 0x0F, 0x30, 0x0D,
    0x06, 0x03, 0x55, 0x04, 0x03, 0x0C, 0x06, 0x42, 0x42, 0x4C, 0x20, 0x43, 0x41, 0x30, 0x1E, 0x17, 0x0D, 0x32, 0x33, 0x31,
    0x32, 0x32, 0x31, 0x30, 0x31, 0x34, 0x36, 0x34, 0x35, 0x5A, 0x17, 0x0D, 0x33, 0x33, 0x31, 0x32, 0x31, 0x38, 0x30, 0x31,
    0x34, 0x36, 0x34, 0x35, 0x5A, 0x30, 0x1A, 0x31, 0x18, 0x30, 0x16, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0C, 0x0F, 0x30, 0x33,
    0x43, 0x31, 0x32, 0x41, 0x33, 0x43, 0x30, 0x34, 0x30, 0x30, 0x35, 0x32, 0x39, 0x30, 0x82, 0x01, 0x22, 0x30, 0x0D, 0x06,
    0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0F, 0x00, 0x30, 0x82, 0x01,
    // 0x0A, 0x02, 0x82, 0x01, 0x01, 0x00, 0xCE, 0x19, 0xB1, 0xAD, 0xD7, 0x1D, 0x7A, 0x5A, 0xB0, 0xCD, 0x9E, 0x4D, 0x06, 0x92,
    0x0A, 0x02, 0x82, 0x01, 0x01, 0x00, 0xBE, 0x19, 0xB1, 0xAD, 0xD7, 0x1D, 0x7A, 0x5A, 0xB0, 0xCD, 0x9E, 0x4D, 0x06, 0x92,
    0x2B, 0x36, 0x97, 0x56, 0x59, 0xCD, 0x07, 0xD9, 0x4E, 0xD2, 0xE1, 0x41, 0xAC, 0xBC, 0xC0, 0xDE, 0x94, 0x66, 0xAD, 0xB5,
    0x39, 0xF0, 0x73, 0xB0, 0xA0, 0xA6, 0x71, 0xC7, 0x3A, 0xE9, 0x22, 0x5A, 0x94, 0x08, 0x0C, 0xB2, 0xAF, 0x70, 0x83, 0x5E,
    0x77, 0xA2, 0xA4, 0xA5, 0x11, 0xED, 0x0D, 0xDD, 0x5D, 0xD3, 0x2F, 0xDC, 0x54, 0x1D, 0x2B, 0x52, 0x43, 0x4B, 0x0D, 0x79,
    0x15, 0xA3, 0x70, 0xE0, 0x42, 0xC0, 0xDF, 0x7A, 0xA3, 0x89, 0x84, 0xA6, 0x0F, 0xAF, 0x49, 0xAD, 0xBF, 0xB9, 0xE4, 0x58,
    0xD2, 0x61, 0xE8, 0x9B, 0x25, 0x2E, 0x8D, 0xD9, 0x18, 0x1E, 0xE2, 0x81, 0x31, 0xF1, 0x5E, 0x83, 0x21, 0x05, 0xE3, 0x4B,
    0xDB, 0x2E, 0xE9, 0x47, 0x4E, 0x36, 0xD0, 0xE0, 0x65, 0xF3, 0xFA, 0x36, 0x7B, 0x14, 0xD1, 0xDD, 0x85, 0x39, 0x8F, 0x12,
    0x17, 0xD1, 0x95, 0xF9, 0x40, 0xEA, 0x52, 0xB9, 0x1A, 0xE0, 0xCF, 0x97, 0xA6, 0x8B, 0x9F, 0x80, 0x3F, 0xB3, 0xA2, 0x2C,
    0xAF, 0x51, 0x5E, 0x0E, 0xB9, 0x73, 0x97, 0xA1, 0xC1, 0x9E, 0xD9, 0x24, 0xCA, 0xCA, 0x8E, 0xA1, 0x27, 0x5B, 0x48, 0x3D,
    0xB1, 0xCF, 0x86, 0x23, 0x36, 0xFE, 0xB5, 0x8A, 0x57, 0x42, 0x00, 0xFB, 0x21, 0x65, 0x46, 0xBC, 0xA9, 0xA0, 0x7C, 0x55,
    0xBF, 0x43, 0x3C, 0x2F, 0x12, 0x2C, 0xCD, 0x36, 0xEC, 0xA9, 0xF9, 0xA0, 0x19, 0x1E, 0x72, 0x89, 0xC0, 0x59, 0xB6, 0x49,
    0xCF, 0x63, 0x78, 0x36, 0xAD, 0xED, 0x06, 0x5C, 0x5A, 0x47, 0x90, 0xF2, 0xA9, 0x9C, 0xEF, 0xD3, 0xB3, 0x19, 0x19, 0xF5,
    0xC6, 0xC1, 0x9B, 0x1B, 0x84, 0x28, 0x6A, 0x3B, 0x34, 0x28, 0xEE, 0x9B, 0x4B, 0x0B, 0x77, 0xCD, 0x97, 0x17, 0x63, 0x76,
    0x2D, 0x93, 0x02, 0x03, 0x01, 0x00, 0x01, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x0B,
    0x05, 0x00, 0x03, 0x82, 0x01, 0x01, 0x00, 0x58, 0x6F, 0xE5, 0xBF, 0xDC, 0x87, 0x7D, 0x70, 0xC9, 0xF8, 0xC5, 0x11, 0xC5,
    0x64, 0xE5, 0x6D, 0x37, 0x00, 0x1F, 0x71, 0x5A, 0xAB, 0x03, 0xB7, 0x16, 0xC4, 0x1C, 0x28, 0xBE, 0x03, 0xD1, 0xFC, 0x84,
    0x84, 0xC4, 0x95, 0x5F, 0x78, 0xA7, 0x1A, 0xC0, 0xD3, 0x75, 0xBE, 0x1A, 0x0B, 0x3F, 0xAB, 0x4C, 0xE3, 0x40, 0x75, 0x3D,
    0x8F, 0x52, 0x59, 0x8F, 0x0D, 0xB0, 0xCA, 0x32, 0x6A, 0x43, 0x93, 0x60, 0xE9, 0x72, 0x5A, 0x00, 0x18, 0x07, 0x55, 0xC4,
    0xD5, 0xC9, 0x30, 0x0A, 0xCE, 0xCA, 0x52, 0x4F, 0x0C, 0xEC, 0xD6, 0xB9, 0xC2, 0xD8, 0x1F, 0x31, 0xA4, 0x7E, 0xB4, 0xC8,
    0xFA, 0x8B, 0xE8, 0x4F, 0x34, 0x58, 0xA7, 0x9A, 0xAB, 0xF3, 0xF6, 0xED, 0x4E, 0xA2, 0x1D, 0xF6, 0xA5, 0xE8, 0x00, 0xD2,
    0x5C, 0x9F, 0xE0, 0x58, 0x63, 0x60, 0xFF, 0xDE, 0xDE, 0xDF, 0xEE, 0xD7, 0x35, 0xD3, 0x46, 0x72, 0x5E, 0xF1, 0x78, 0x15,
    0xF9, 0x5B, 0x21, 0x59, 0xB8, 0x22, 0x35, 0x19, 0x44, 0xEB, 0x16, 0xE0, 0x0E, 0x47, 0xDC, 0xC9, 0xAA, 0xAA, 0xA3, 0x52,
    0x17, 0x42, 0x85, 0xAB, 0x20, 0x4B, 0xB7, 0xE3, 0xAD, 0xD4, 0xF6, 0xC3, 0xC4, 0xC4, 0x77, 0x1C, 0x75, 0x1D, 0xC1, 0x42,
    0x9C, 0xA2, 0x9D, 0xF6, 0x4C, 0xD8, 0xFF, 0x64, 0x29, 0x06, 0xAD, 0xBB, 0x5B, 0x0D, 0x03, 0x80, 0x8B, 0x02, 0x02, 0x11,
    0xFD, 0x36, 0x53, 0xA2, 0x0D, 0x34, 0x99, 0xC0, 0x95, 0x34, 0x34, 0x76, 0x39, 0x0B, 0x96, 0xD1, 0x08, 0xF8, 0xBC, 0x77,
    0x28, 0x37, 0x75, 0xB7, 0x66, 0x4B, 0xF9, 0xC9, 0xA4, 0x2C, 0x4B, 0xAA, 0x7D, 0x46, 0x74, 0x77, 0x53, 0x91, 0xEF, 0x32,
    0xB3, 0x78, 0x55, 0x96, 0x0F, 0x66, 0x20, 0x14, 0x8E, 0x1A, 0xAD, 0xC0, 0xFE, 0xBD, 0x58, 0xAA, 0x16, 0xBD, 0x75, 0xAE,
    0x6C, 0xDA, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
