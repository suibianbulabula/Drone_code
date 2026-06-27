#ifndef SENSOR_H__
#define SENSOR_H__


#include "globals.h"
#include <math.h>

typedef struct {
    // 姿态变量 (弧度)
    float roll_acc;
    float pitch_acc;
    float yaw_acc;
    float roll_gyro;
    float pitch_gyro;
    float yaw_gyro;
    float roll_mag;
    float pitch_mag;
    float yaw_mag;
}Attitude_t;

typedef struct {
    float gps_lat;
    float gps_lon;
    float gps_alt; 
    float pressure;
}Position_t;

extern Attitude_t attitude_t;
extern Position_t position_t;


void sensor_attitude_data(Sensor_Att_t *Datastruct , float dt);
void sensor_position_data(Sensor_Pos_t *Datastruct);

#endif
