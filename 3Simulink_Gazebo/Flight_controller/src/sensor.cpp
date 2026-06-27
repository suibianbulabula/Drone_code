#include "sensor.h"

Attitude_t attitude_t = {0};
Position_t position_t = {0};

// 改为 Gazebo 原生 IMU 消息
void imu_callback(ConstIMUPtr &msg) {   // 直接使用 Gazebo 预定义的 ConstIMUPtr
    std::lock_guard<std::mutex> lock(data_mutex);
    gyro.X() = msg->angular_velocity().x();
    gyro.Y() = msg->angular_velocity().y();
    gyro.Z() = msg->angular_velocity().z();
    acc.X()  = msg->linear_acceleration().x();
    acc.Y()  = msg->linear_acceleration().y();
    acc.Z()  = msg->linear_acceleration().z();
}

// 磁力计回调 (Gazebo 原生消息)
void mag_callback(ConstMagnetometerPtr &msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    mag.X() = msg->field_tesla().x();
    mag.Y() = msg->field_tesla().y();
    mag.Z() = msg->field_tesla().z();
}

// 气压计回调 (PX4 自定义消息)
void baro_callback(const boost::shared_ptr<const sensor_msgs::msgs::Pressure>& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    pressure = msg->absolute_pressure();
    if (msg->absolute_pressure() > 500.0) { // 合理范围检查
        state_t.baro_valid = true;
    }
}

// GPS回调（PX4自定义消息）
void gps_callback(const boost::shared_ptr<const sensor_msgs::msgs::SITLGps>& msg) {
    static int gps_call_count = 0;
if (gps_call_count++ % 100 == 0) {
    std::cout << "GPS callback called, lat=" << msg->latitude_deg() << std::endl;
}
    std::lock_guard<std::mutex> lock(data_mutex);
    gps_lat = msg->latitude_deg();      
    gps_lon = msg->longitude_deg();
    gps_alt = msg->altitude();
}

void sensor_attitude_data(Sensor_Att_t *Datastruct , float dt)
{
    const float alpha = 0.0;
    const float home_yaw = 88.5;
    // 获取传感器数据
    ignition::math::Vector3d g, a, m;
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        g = gyro;
        a = acc;
        m = mag;
    }

    float acc_norm = sqrt(a.X()*a.X() + a.Y()*a.Y() + a.Z()*a.Z());
    // 数据有效，更新姿态角估计
    if (acc_norm > 7.0 && acc_norm < 13.0) {
        attitude_t.pitch_acc = RAD_2_ANGLE( atan2(-a.X(), sqrt(a.Y()*a.Y() + a.Z()*a.Z())) );
        attitude_t.roll_acc  = RAD_2_ANGLE( atan2( a.Y(), a.Z()) );
        attitude_t.yaw_mag = RAD_2_ANGLE( atan2(-mag.Y(), mag.X()) );
    }
    attitude_t.roll_gyro = RAD_2_ANGLE(g.X());
    attitude_t.pitch_gyro = RAD_2_ANGLE(g.Y());
    attitude_t.yaw_gyro = RAD_2_ANGLE(g.Z());

    // --- 姿态估计（互补滤波）---
    Datastruct->pitch = (1.0 - alpha) * (Datastruct->pitch +attitude_t.pitch_gyro * dt) + alpha * attitude_t.pitch_acc;
    Datastruct->roll  = (1.0 - alpha) * (Datastruct->roll  + attitude_t.roll_gyro * dt) + alpha * attitude_t.roll_acc;

    //初始偏行角转为0度
    Datastruct->yaw = attitude_t.yaw_mag - home_yaw;
    // 归一化到 -180° ~ 180°
    //注意这里的机头yaw与gazebo世界轴相反
    if (Datastruct->yaw > 180.0)  Datastruct->yaw -= 360.0;
    if (Datastruct->yaw < -180.0) Datastruct->yaw += 360.0;
}

void sensor_position_data(Sensor_Pos_t *Datastruct)
{
    // 首次锁定或手动设定
    const float home_lat = 47.397742;
    const float home_lon = 8.545594;  
    const float ground_pressure = 956.0;

    position_t.gps_lon = gps_lon;
    position_t.gps_lat = gps_lat;
    position_t.pressure = pressure;

    //计算x和y坐标
    Datastruct->y = -(position_t.gps_lon - home_lon) * 75650.0;
    Datastruct->x = (position_t.gps_lat - home_lat) * 111132.0;

    // 计算相对高度
    Datastruct->z = 44330.0 * (1.0 - pow(position_t.pressure / ground_pressure, 0.1903));
}