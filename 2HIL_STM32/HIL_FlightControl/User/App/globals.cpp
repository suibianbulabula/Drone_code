#include "globals.h"

const float throttle = 803.5;
const float target_height = 0.3;
float motor[4] = {0};

Sensor_Att_t sensor_att_t = {0}; 
Sensor_Pos_t sensor_pos_t = {0};

Control_State_t state_t = {0};
Mode_e mode_e;

// 全局传感器数据
mavlink_hil_sensor_t sensor = {0};
mavlink_hil_gps_t gps = {0};

