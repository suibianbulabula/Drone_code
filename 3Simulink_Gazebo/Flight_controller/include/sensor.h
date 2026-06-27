#ifndef SENSOR_H__
#define SENSOR_H__

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

#include "globals.h"

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

void imu_callback(ConstIMUPtr &msg);
void mag_callback(ConstMagnetometerPtr &msg);
void baro_callback(const boost::shared_ptr<const sensor_msgs::msgs::Pressure>& msg);
void gps_callback(const boost::shared_ptr<const sensor_msgs::msgs::SITLGps>& msg) ;
void sensor_attitude_data(Sensor_Att_t *Datastruct , float dt);
void sensor_position_data(Sensor_Pos_t *Datastruct);

#endif