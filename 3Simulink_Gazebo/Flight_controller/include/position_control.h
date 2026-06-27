#ifndef  POSITION_CONTROL_H__
#define POSITION_CONTROL_H__

#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>

#include "globals.h"
#include "sensor.h"
#include "attitude_control.h"

extern PID_t x_c_t;
extern PID_t y_c_t;
extern PID_t z_c_t;

void position_control(Sensor_Att_t *Datastruct_att, Sensor_Pos_t *Datastruct_pos, 
                                                float vel, float target_x, float target_y, float target_z, float target_yaw_deg, float dt);

#endif