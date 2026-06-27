#ifndef GLOBALS_H__
#define GLOBALS_H__

#include "mavlink.h"

#define PI 3.141592654
#define RAD_2_ANGLE(x) (x * 180 / PI)
#define ANGLE_2_RAD(x) (x * PI / 180)


typedef struct {
    float roll;
    float pitch;
    float yaw;
}Sensor_Att_t;

typedef struct {
    float x;
    float y;
    float z;
}Sensor_Pos_t;

typedef struct {
    float targrt;
    float current;
    float err;
    float last_err;

    float Kp;
    float Ki;
    float Kd;

    float integrator;
    float derivative;
    float output;
}PID_t;

typedef struct {
    bool baro_valid;
    bool flag;
}Control_State_t;

typedef enum{
    RemoteControl,
    Automatic
}Mode_e;


extern const float throttle;
extern const float target_height;
extern float motor[4];

extern Sensor_Att_t sensor_att_t; 
extern Sensor_Pos_t sensor_pos_t;

extern Control_State_t state_t;
extern Mode_e mode_e;

// 全局传感器数据
extern mavlink_hil_sensor_t sensor;
extern mavlink_hil_gps_t gps;

#endif
