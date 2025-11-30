#pragma once

#include "main.h"

extern void Motion_control_init();
extern void Motion_control_set_PWM(uint8_t CHx,int PWM);
extern void Motion_control_run(int error);
#ifdef BAMBU_BUS_AMS_NUM
#define motion_control_ams_num BAMBU_BUS_AMS_NUM
#else 
#define motion_control_ams_num 0
#endif
#ifdef DP1X_OUT_FILAMENT_METERS
#define motion_control_pull_back_distance DP1X_OUT_FILAMENT_METERS
#else
#define motion_control_pull_back_distance 0.2
#endif