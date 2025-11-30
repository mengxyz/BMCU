#pragma once
#include "main.h"
#include "ams.h"
#include "_bus_hardware.h"
#define xMCU

enum class ahubus_package_type : uint8_t
{
    heartbeat = 0x01,
    query = 0x02,
    set = 0x03,
    none,
    error,
};
enum class ahubus_set_type : uint8_t
{
    filament_info = 0x02,
    dryer_stu = 0x05,
    all_filament_stu=0x06,
};

extern void ahubus_init();
extern ahubus_package_type ahubus_run();