#pragma once
#include "main.h"
#include "crc_compute.h"
#include "Debug_log.h"

extern crc8 _bus_crc_8;
enum class _bus_data_type : uint8_t
{
    bambubus = 0x3D,
    ahub_bus = 0x33,
    none = 0x00
};

class _bus_port_deal // 中断数据处理
{
public:
    uint8_t send_data_buf[1280] __attribute__((aligned(4)));
private:
    uint8_t recv_data_buf[2][1280] __attribute__((aligned(4)));
    int _index = 0;
    int length = 999;
    uint8_t data_length_index = 0;
    uint8_t data_CRC8_index = 0;
    _bus_data_type irq_package_type = _bus_data_type::none;
    uint8_t *bus_irq_data_ptr = recv_data_buf[0];
    void (*port_send_datas)(uint8_t *data, uint16_t len);
public:
    uint8_t *bus_recv_data_ptr = recv_data_buf[0];
    int recv_data_len = 0;
    int send_data_len = 0;
    _bus_data_type bus_package_type = _bus_data_type::none;
    bool idle = true;
    void init(void (*_port_send_datas)(uint8_t *data, uint16_t len))
    {
        _index = 0;
        bus_irq_data_ptr = recv_data_buf[0];
        bus_recv_data_ptr = recv_data_buf[1];
        recv_data_len = 0;
        port_send_datas = _port_send_datas;
    }
    void irq(uint8_t data)
    {
        if (_index == 0) // 状态为等待帧头
        {
            if (data == 0x3D || data == 0x33) // 0x3D-Bambubus帧头
            {
                bus_irq_data_ptr[0] = data;
                data_length_index = 4;        // 未知数据包类型，假定数据包长度的位置
                length = data_CRC8_index = 6; // 未知数据包类型，假定CRC8校验字位置
                _index = 1;
                irq_package_type = (_bus_data_type)data;
            }
            return;
        }
        else // 具备帧头的情况下，后续数据
        {
            bus_irq_data_ptr[_index] = data;
            if (_index == 1) // 包类型字节
            {
                if (data & 0x80) // 短帧头-Bambubus和ahub_bus兼容
                {
                    data_length_index = 2;
                    data_CRC8_index = 3;
                }
                else // 长帧头-仅Bambubus
                {
                    data_length_index = 4;
                    data_CRC8_index = 6;
                }
            }
            if (_index == data_length_index) // 数据长度字节
            {
                if (irq_package_type == _bus_data_type::bambubus)
                {
                    length = data; // Bambubus数据总长度=长度字节
                }
                else if (irq_package_type == _bus_data_type::ahub_bus)
                {
                    length = (((int)data) << 2) + 12; // ahub_bus数据总长度=长度字节*4+12
                }
            }
            if (_index == data_CRC8_index) // CRC8校验字节
            {
                if (data != _bus_crc_8.crc_compute(bus_irq_data_ptr, data_CRC8_index)) // CRC8校验错误，重置
                {
                    _index = 0;
                    return;
                }
            }
            ++_index;
            if (_index >= length) // 接收完毕，交换缓冲器指针
            {
                _index = 0;
                if (recv_data_len == 0) // 上一个数据包已处理，可以交换缓冲区
                {
                    uint8_t *_data_ptr = bus_recv_data_ptr; // 临时存储输出指针
                    bus_recv_data_ptr = bus_irq_data_ptr;   // 输出指向此次接收的数据内容
                    bus_irq_data_ptr = _data_ptr;           // 输入定向到原来的输出缓冲区
                    recv_data_len = length;             // 记录接收数据长度
                    bus_package_type = irq_package_type;    // 顺便记录数据包类型
                }
            }
            if (_index >= 999) // 接收错误，重置
            {
                _index = 0;
            }
        }
    }
    void send_package()
    {
        if ((send_data_len > 0)&&(send_data_len<=1280))
        {
            port_send_datas(send_data_buf, send_data_len);
            send_data_len = 0;
        }
    }
    void send_package(uint8_t *data, uint16_t len)
    {
        if (len > 0)
        {
            port_send_datas(data, len);
        }
    }
}__attribute__((aligned(4)));

extern _bus_port_deal bus_port_to_host;
extern void bus_init();

#define host_device_type_none 0x0000
#define host_device_type_ahub 0x0001
#define host_device_type_ams 0x0700
#define host_device_type_ams_lite 0x1200
extern uint16_t bus_host_device_type;