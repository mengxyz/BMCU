#pragma once
#include "main.h"
#include "ams.h"
#include "_bus_hardware.h"

#ifdef AMS_TYPE_AMS_LITE
#define AMS_type_ams_lite
#else
#define AMS_type_ams
#endif

enum class bambubus_package_type 
{
    error = -1,
    none = 0,
    filament_motion_short,
    filament_motion_long,
    online_detect,
    REQx6,
    NFC_detect,
    set_filament_info,
    MC_online,
    read_filament_info,
    set_filament_info_type2,
    version,
    serial_number,
    heartbeat,
    ETC,

    read_cert,
    send_cert_verify,
    cert_datas_sync,


    __BambuBus_package_packge_type_size
};


extern bambubus_package_type bambubus_run();
extern uint16_t bambubus_ams_address;