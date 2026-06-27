#ifndef  ATTITUDE_CONTROL_H__
#define ATTITUDE_CONTROL_H__

#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>

#include "globals.h"
#include "sensor.h"

extern PID_t roll_c_t;
extern PID_t pitch_c_t;
extern PID_t yaw_c_t;

void motor_velocity_output(float vel, float roll_cmd, float pitch_cmd, float yaw_cmd);
void attitude_control(Sensor_Att_t *Datastruct, float vel, float target_roll_deg, float target_pitch_deg, float target_yaw_deg, float dt);

#endif
