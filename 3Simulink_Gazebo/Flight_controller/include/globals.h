#ifndef GLOBALS_H__
#define GLOBALS_H__

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "CommandMotorSpeed.pb.h"
#include "Groundtruth.pb.h"
#include "Imu.pb.h"          // sensor_msgs::msgs::Imu
#include "Pressure.pb.h"     // sensor_msgs::msgs::Pressure
#include "SITLGps.pb.h"    // sensor_msgs::msgs::SITLGps
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>
#include <gazebo/msgs/image.pb.h>
#include <opencv2/opencv.hpp>
#include <gazebo/msgs/image_stamped.pb.h> 

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
    Automatic,
    HIL
}Mode_e;

// 串口与 MAVLink 参数
extern const std::string SERIAL_PORT;
extern const unsigned int BAUD_RATE;
extern const int SYS_ID;
extern const int COMP_ID;
extern boost::asio::io_service io;
extern boost::asio::serial_port serial;

extern const float throttle;
extern const float target_height;
extern float motor[4];
extern float motor_mavlink[4] ;

extern Sensor_Att_t sensor_att_t; 
extern Sensor_Pos_t sensor_pos_t;

extern Control_State_t state_t;
extern Mode_e mode_e;

// 全局传感器数据
extern std::mutex data_mutex;
extern ignition::math::Vector3d gyro;
extern ignition::math::Vector3d acc;
extern ignition::math::Vector3d mag;
extern double gps_lat;
extern double gps_lon;
extern double gps_alt;
extern double pressure;


//相机数据
extern std::mutex image_mutex;
extern cv::Mat current_frame;
extern bool new_frame;
// 视觉处理结果
extern std::mutex vision_mutex;
extern bool target_found;
extern int vision_err_x;          // 水平像素误差
extern int vision_err_y;          // 垂直像素误差
extern cv::Mat display_frame;         // 用于显示的画面（带标注）
extern bool display_updated;  // 是否有新画面需要显示
extern std::atomic<bool> display_running;


#endif