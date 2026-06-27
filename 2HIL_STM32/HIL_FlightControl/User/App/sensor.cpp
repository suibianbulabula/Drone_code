#include "sensor.h"

Attitude_t attitude_t = {0};
Position_t position_t = {0};

void sensor_attitude_data(Sensor_Att_t *Datastruct , float dt)
{
    const float alpha = 0.0;
    const float home_yaw = 88.5;
    // 获取传感器数据
    

    float acc_norm = sqrt(sensor.xacc * sensor.xacc + sensor.yacc * sensor.yacc + sensor.zacc * sensor.zacc);
    // 数据有效，更新姿态角估计
    if (acc_norm > 7.0 && acc_norm < 13.0) {
        attitude_t.pitch_acc = RAD_2_ANGLE( atan2(-sensor.xacc, sqrt(sensor.yacc * sensor.yacc + sensor.zacc * sensor.zacc)) );
        attitude_t.roll_acc  = RAD_2_ANGLE( atan2( sensor.yacc, sensor.zacc) );
        attitude_t.yaw_mag = RAD_2_ANGLE( atan2(-sensor.ymag, sensor.xmag) );
    }
    attitude_t.roll_gyro = RAD_2_ANGLE(sensor.xgyro);
    attitude_t.pitch_gyro = RAD_2_ANGLE(sensor.ygyro);
    attitude_t.yaw_gyro = RAD_2_ANGLE(sensor.zgyro);

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

    position_t.gps_lon = gps.lon;
    position_t.gps_lat = gps.lat;
    position_t.pressure = sensor.abs_pressure;

    //计算x和y坐标
    Datastruct->y = -(position_t.gps_lon - home_lon) * 75650.0;
    Datastruct->x = (position_t.gps_lat - home_lat) * 111132.0;

    // 计算相对高度
    Datastruct->z = 44330.0 * (1.0 - pow(position_t.pressure / ground_pressure, 0.1903));
}
