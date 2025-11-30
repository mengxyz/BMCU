#include "ahub_bus.h"
#include "crc_compute.h"
/*
AHUB BUS 协议
注：AHUB本身从设备上限为255，但因RAM限制，故实际采用64个AMS的数据，
AHUB一共4个接口，每个接口给予最高16个AMS的限额，作为一主一从使用，
因此最多有三级挂载，64个AMS，每个AMS最多挂载4个耗材，总共256个耗材位。
*/

uint8_t *ahubus_package_maker_buf_host = bus_port_to_host.send_data_buf;
int *ahubus_package_data_length_host = &bus_port_to_host.send_data_len;
crc8 ahubus_crc_8(0x39, 0x66, 0);

#define ahubus_map_port_adr_to_index(port, adr) (((uint8_t)port << 4) + ((uint8_t)adr >> 2))

// 对数据包进行CRC校验，并且返回包类型
ahubus_package_type ahubus_get_package_type(uint8_t *package_recv_buf)
{
    uint16_t package_type = 0;
    if (package_recv_buf[0] != 0x33)
    {
        return ahubus_package_type::none;
    }
    uint32_t *crc32_ptr = (uint32_t *)package_recv_buf;
    CRC->CTLR = 1;                        // 写1清零CRC
    int length = package_recv_buf[2] + 2; // buf[2]:长度字节，为内容长度（四字节单位）-1，加上帧头四字节，总共需要+2，因为尾部是CRC，不计入
    for (int i = 0; i < length; i++)
    {
        CRC->DATAR = crc32_ptr[i]; // 计算CRC32
    }
    if (CRC->DATAR != crc32_ptr[length]) // 判断CRC32是否正确
    {
        return ahubus_package_type::none; // 校验失败
    }

    package_type = package_recv_buf[4];
    return (ahubus_package_type)package_type;
}
void ahubus_init()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}

int ahubus_package_add_crc(uint8_t *buf)
{
    uint32_t *crc32_ptr = (uint32_t *)buf;
    int length = buf[2] + 2;                   // buf[2]:长度字节，为内容长度（四字节单位）-1，加上帧头四字节，总共需要+2，因为尾部是CRC，不计入
    buf[3] = ahubus_crc_8.crc_compute(buf, 3); // 计算头部CRC8
    CRC->CTLR = 1;                             // 写1清零CRC
    for (int i = 0; i < length; i++)
    {
        CRC->DATAR = crc32_ptr[i]; // 如果上一步CRC计算未完成，写操作会自动阻塞
    }
    crc32_ptr[length] = CRC->DATAR; // 将CRC32结果写入包尾
    return (length + 1) << 2;       // 返回总数据长度
}
// 作为从机向主机回应心跳包
void ahubus_slave_get_package_heartbeat(uint8_t *buf)
{
    uint8_t ahubus_ams_numbers = 0;
    ahubus_package_maker_buf_host[0] = 0x33;
    ahubus_package_maker_buf_host[1] = buf[1];
    // buf[2]~[3]分别是包长度和crc8，需要等数据准备好再填写
    ahubus_package_maker_buf_host[4] = 0x01;

    uint8_t *EQPT_data_ptr = ahubus_package_maker_buf_host + 6;

    for (uint8_t i = 0; i < ams_max_number; i++)
    {
        if (ams[i].online == true)
        {
            EQPT_data_ptr[0] = i << 2;
            EQPT_data_ptr[1] = ams[i].ams_type;
            EQPT_data_ptr += 2;
            ahubus_ams_numbers++;
        }
    }

    ahubus_package_maker_buf_host[5] = ahubus_ams_numbers;
    if ((ahubus_ams_numbers & 0x01) == 0) // 如果设备数量为偶数，需要额外填充2字节
    {
        EQPT_data_ptr[0] = 0x00;
        EQPT_data_ptr[1] = 0x00;
        EQPT_data_ptr += 2;
    }
    ahubus_package_maker_buf_host[2] = ((EQPT_data_ptr - ahubus_package_maker_buf_host) >> 2) - 2; // 内容长度
    *ahubus_package_data_length_host = ahubus_package_add_crc(ahubus_package_maker_buf_host);      // 封包，添加CRC，并记录包长度
    return;
}

enum class ahubus_query_type : uint8_t
{
    ams_name = 0x01,
    filament_info = 0x02,
    filament_stu = 0x04,
    dryer_stu = 0x05,
    all_filament_stu = 0x06,
};

struct ahubus_package_query_head
{
    uint8_t magic_byte;
    uint8_t flag;
    uint8_t length;
    uint8_t crc8;
    uint8_t command;
    ahubus_query_type query_type;
    uint8_t query_adr;
    uint8_t data_struct_count;
    uint8_t data[0];
} __attribute__((packed));

const ahubus_package_query_head ahubus_host_package_query_init = {
    .magic_byte = 0x33,
    .flag = 0x80,
    .command = 0x02,
};

// 作为从机回应主机查询
void ahubus_slave_get_package_query(uint8_t *buf)
{
    ahubus_query_type query_type = ((ahubus_package_query_head *)buf)->query_type;
    uint8_t query_adr = ((ahubus_package_query_head *)buf)->query_adr;
    memcpy(ahubus_package_maker_buf_host, &ahubus_host_package_query_init, sizeof(ahubus_package_query_head));
    ((ahubus_package_query_head *)ahubus_package_maker_buf_host)->query_adr = query_adr;
    ((ahubus_package_query_head *)ahubus_package_maker_buf_host)->query_type = query_type;
    uint8_t *data_ptr = ahubus_package_maker_buf_host + 4;
#ifdef xMCU
    query_adr = query_adr >> 4;
#endif
    switch (query_type)
    {
    case ahubus_query_type::ams_name:
        memcpy(data_ptr + 4, ams[query_adr].name, 8);                                           // 复制AMS名称
        ((ahubus_package_query_head *)ahubus_package_maker_buf_host)->length = 2;               // 内容长度/4-1
        ((ahubus_package_query_head *)ahubus_package_maker_buf_host)->data_struct_count = 0x01; // 数据结构体数量
        break;
    case ahubus_query_type::filament_info:
        memcpy(data_ptr + 4, ams[query_adr].filament[0].bambubus_filament_id, 44); // 耗材信息字段从此地址开始，一共44字节
        memcpy(data_ptr + 48, ams[query_adr].filament[1].bambubus_filament_id, 44);
        memcpy(data_ptr + 92, ams[query_adr].filament[2].bambubus_filament_id, 44);
        memcpy(data_ptr + 136, ams[query_adr].filament[3].bambubus_filament_id, 44);
        ((ahubus_package_query_head *)ahubus_package_maker_buf_host)->length = 44; // 内容长度/4-1
        data_ptr[3] = 0x01;                                                        // 数据结构体数量
        break;
    case ahubus_query_type::filament_stu:
        bus_now_ams_num = query_adr;
        memcpy(data_ptr + 4, &(ams[query_adr].filament[0].motion), 8); // 耗材状态信息字段从此地址开始，一共8字节
        memcpy(data_ptr + 12, &(ams[query_adr].filament[1].motion), 8);
        memcpy(data_ptr + 20, &(ams[query_adr].filament[2].motion), 8);
        memcpy(data_ptr + 28, &(ams[query_adr].filament[3].motion), 8);
        if (ams[query_adr].filament[0].online) // 最高位为耗材在线状态
            data_ptr[4] |= 0x80;
        if (ams[query_adr].filament[1].online)
            data_ptr[12] |= 0x80;
        if (ams[query_adr].filament[2].online)
            data_ptr[20] |= 0x80;
        if (ams[query_adr].filament[3].online)
            data_ptr[28] |= 0x80;
        ((ahubus_package_query_head *)ahubus_package_maker_buf_host)->length = 8;               // 内容长度/4-1
        ((ahubus_package_query_head *)ahubus_package_maker_buf_host)->data_struct_count = 0x01; // 数据结构体数量
        break;
    case ahubus_query_type::dryer_stu:
        memcpy(data_ptr + 4, &(ams[query_adr].filament[0].dryer_power), 4); // 烘干器信息字段从此地址开始，一共4字节
        memcpy(data_ptr + 8, &(ams[query_adr].filament[1].dryer_power), 4);
        memcpy(data_ptr + 12, &(ams[query_adr].filament[2].dryer_power), 4);
        memcpy(data_ptr + 16, &(ams[query_adr].filament[3].dryer_power), 4);
        ((ahubus_package_query_head *)ahubus_package_maker_buf_host)->length = 4;               // 内容长度/4-1
        ((ahubus_package_query_head *)ahubus_package_maker_buf_host)->data_struct_count = 0x01; // 数据结构体数量
        break;
    case ahubus_query_type::all_filament_stu:
    {
        uint8_t *ams_filament_data_ptr = data_ptr + 4;
        uint8_t ams_data_count = 0;
        for (uint8_t i = 0; i < ams_max_number; i++)
        {
            if (ams[i].online)
            {
#ifdef xMCU
                ams_filament_data_ptr[0] = i << 4;
#else
                ams_filament_data_ptr[0] = i;
#endif
                ams_filament_data_ptr[1] = ams[i].online;
                ams_filament_data_ptr[2] = (uint8_t)ams[i].filament[0].motion;
                ams_filament_data_ptr[3] = (uint8_t)ams[i].filament[0].seal_status;
                ams_filament_data_ptr[4] = (uint8_t)ams[i].filament[1].motion;
                ams_filament_data_ptr[5] = (uint8_t)ams[i].filament[1].seal_status;
                ams_filament_data_ptr[6] = (uint8_t)ams[i].filament[2].motion;
                ams_filament_data_ptr[7] = (uint8_t)ams[i].filament[2].seal_status;
                ams_filament_data_ptr[8] = (uint8_t)ams[i].filament[3].motion;
                ams_filament_data_ptr[9] = (uint8_t)ams[i].filament[3].seal_status;
                if (ams[i].filament[0].online)
                    ams_filament_data_ptr[2] |= 0x80;
                if (ams[i].filament[1].online)
                    ams_filament_data_ptr[4] |= 0x80;
                if (ams[i].filament[2].online)
                    ams_filament_data_ptr[6] |= 0x80;
                if (ams[i].filament[3].online)
                    ams_filament_data_ptr[8] |= 0x80;
                ams_filament_data_ptr += 10;
                ams_data_count++;
            }
        }
        ((ahubus_package_query_head *)ahubus_package_maker_buf_host)->length = ((ams_filament_data_ptr - data_ptr) >> 2) - 1; // 内容长度/4-1
        ((ahubus_package_query_head *)ahubus_package_maker_buf_host)->data_struct_count = ams_data_count;                     // 数据结构体数量
        break;
    }
    default:
        return;
        break;
    }
    *ahubus_package_data_length_host = ahubus_package_add_crc(ahubus_package_maker_buf_host); // 封包，添加CRC，并记录包长度
}

// enum class ahubus_set_type : uint8_t;
// 已经在头文件定义

struct ahubus_sync_req_list_struct
{
    uint8_t ams_num;
    uint8_t filament_channel;
    ahubus_set_type type;
};

struct ahubus_package_set_head
{
    uint8_t magic_byte;
    uint8_t flag;
    uint8_t length;
    uint8_t crc8;
    uint8_t command;
    uint8_t set_type;
    uint8_t set_adr;
    uint8_t data_struct_count;
    uint8_t data[0];
} __attribute__((packed));

const ahubus_package_set_head ahubus_host_package_set_init = {
    .magic_byte = 0x33,
    .flag = 0x80,
    .command = 0x03,
};

// 作为从机回应主机设置
void ahubus_slave_get_package_set(uint8_t *buf)
{
    ahubus_set_type set_type = (ahubus_set_type)((ahubus_package_set_head *)buf)->set_type;
    uint8_t set_adr = ((ahubus_package_set_head *)buf)->set_adr;
    uint8_t *data_ptr = buf + 4;

#ifdef xMCU
    set_adr = set_adr >> 4;
#endif

    if (set_adr >= ams_max_number)
        return;
    switch (set_type)
    {
    case ahubus_set_type::filament_info:
    {
        uint8_t filament_channel = data_ptr[48];
        memcpy(&(ams[set_adr].filament[filament_channel].bambubus_filament_id), data_ptr + 4, 44);
        ams_datas_set_need_to_save();
        break;
    }
    case ahubus_set_type::dryer_stu:
    {
        uint8_t dryer_channel = data_ptr[8];
        memcpy(&(ams[set_adr].filament[dryer_channel].dryer_power), data_ptr + 4, 4);
        break;
    }
    case ahubus_set_type::all_filament_stu:
    {
        uint8_t data_struct_count = ((ahubus_package_set_head *)buf)->data_struct_count;
        uint8_t *data_struct_ptr = data_ptr + 4;
        // DEBUG_num(&data_struct_count, 1);
        for (uint8_t i = 0; i < data_struct_count; i++)
        {
            uint8_t ams_adr = data_struct_ptr[0];
#ifdef xMCU
            ams_adr = ams_adr >> 4;
#endif
            if (ams_adr > ams_max_number)
                continue;
            ams[ams_adr].now_filament_num = data_struct_ptr[1];
            ams[ams_adr].filament[0].motion = (_filament_motion)(data_struct_ptr[2] & 0x7F);
            ams[ams_adr].filament[1].motion = (_filament_motion)(data_struct_ptr[3] & 0x7F);
            ams[ams_adr].filament[2].motion = (_filament_motion)(data_struct_ptr[4] & 0x7F);
            ams[ams_adr].filament[3].motion = (_filament_motion)(data_struct_ptr[5] & 0x7F);
            data_struct_ptr += 6;
        }
        break;
    }
    default:
        return;
    }

    // 响应包
    memcpy(ahubus_package_maker_buf_host, &ahubus_host_package_set_init, sizeof(ahubus_package_set_head));
    ((ahubus_package_set_head *)ahubus_package_maker_buf_host)->set_adr = ((ahubus_package_set_head *)buf)->set_adr;
    ((ahubus_package_set_head *)ahubus_package_maker_buf_host)->set_type = ((ahubus_package_set_head *)buf)->set_type;
    ((ahubus_package_set_head *)ahubus_package_maker_buf_host)->data_struct_count = 0;
    ((ahubus_package_set_head *)ahubus_package_maker_buf_host)->length = 0;
    *ahubus_package_data_length_host = ahubus_package_add_crc(ahubus_package_maker_buf_host); // 封包，添加CRC，并记录包长度
}

ahubus_package_type ahubus_run()
{
    ahubus_package_type package_type = ahubus_package_type::none;
    static uint64_t time_set = 0;
    uint64_t timex = get_time64();
    if ((bus_port_to_host.recv_data_len > 0) && (bus_port_to_host.bus_recv_data_ptr[0] == 0x33)) // 主机端口发来的数据包
    {

        uint8_t *package_recv_buf = bus_port_to_host.bus_recv_data_ptr;
        package_type = ahubus_get_package_type(package_recv_buf);
        int length = bus_port_to_host.recv_data_len;

        switch (package_type)
        {
        case ahubus_package_type::heartbeat:
            ahubus_slave_get_package_heartbeat(package_recv_buf);
            time_set = timex + 1000;
            break;
        case ahubus_package_type::query:
            ahubus_slave_get_package_query(package_recv_buf);
            break;
        case ahubus_package_type::set:
            ahubus_slave_get_package_set(package_recv_buf);
            break;
        case ahubus_package_type::none:
            break;
        default:
            break;
        }
        bus_port_to_host.recv_data_len = 0;
    }
    if (timex > time_set)
    {
        package_type = ahubus_package_type::error; // offline
    }
    return package_type;
}
