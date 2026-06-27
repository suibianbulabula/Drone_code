#include "attitude_control.h"

PID_t roll_c_t = {0};
PID_t pitch_c_t = {0};
PID_t yaw_c_t = {0};

void motor_velocity_output(float vel, float roll_cmd, float pitch_cmd, float yaw_cmd)
{
    // --- 混合矩阵 ---
    motor[0] = vel - roll_cmd -pitch_cmd + yaw_cmd; 
    motor[1] = vel +  roll_cmd + pitch_cmd + yaw_cmd; 
    motor[2] = vel +  roll_cmd - pitch_cmd - yaw_cmd; 
    motor[3] = vel - roll_cmd +pitch_cmd - yaw_cmd ; 

    // 限幅与保护
    for (int i = 0; i < 4; ++i) {
        if (motor[i] < 0) motor[i] = 0;
        if (motor[i] > 1000) motor[i] = 1000;
        if (!std::isfinite(motor[i])) motor[i] = vel;
    }
}

void attitude_control(Sensor_Att_t *Datastruct, float vel, float target_roll_deg, float target_pitch_deg, float target_yaw_deg, float dt)
{
    //sensor_attitude_data( Datastruct , dt);

    roll_c_t.Kp = 2.8;
    roll_c_t.Ki = 0;
    roll_c_t.Kd = 0.1;

    pitch_c_t.Kp = 2.8;
    pitch_c_t.Ki = 0;
    pitch_c_t.Kd = 0.1;

    yaw_c_t.Kp = 1;
    yaw_c_t.Ki = 0;
    yaw_c_t.Kd = 0;

    //pid -- roll
    roll_c_t.err  = target_roll_deg - Datastruct->roll;
    roll_c_t.derivative = -attitude_t.roll_gyro;
    roll_c_t.output  = roll_c_t.Kp * roll_c_t.err + roll_c_t.Kd * roll_c_t.derivative;

    //pid -- pitch
    pitch_c_t.err = target_pitch_deg - Datastruct->pitch;
    pitch_c_t.derivative = -attitude_t.pitch_gyro;
    pitch_c_t.output  = pitch_c_t.Kp * pitch_c_t.err + pitch_c_t.Kd * pitch_c_t.derivative;

    //pid -- yaw
    yaw_c_t.err = target_yaw_deg - Datastruct->yaw;
    yaw_c_t.derivative = -attitude_t.yaw_gyro;
    // 归一化到 -180 ~ 180
    if (yaw_c_t.err > 180.0)  yaw_c_t.err -= 360.0;
    if (yaw_c_t.err < -180.0) yaw_c_t.err += 360.0;
    yaw_c_t.output  = yaw_c_t.Kp * yaw_c_t.err + yaw_c_t.Kd * yaw_c_t.derivative;

    //电机速度输出
    motor_velocity_output(vel,roll_c_t.output, pitch_c_t.output, pitch_c_t.output);
    printf("roll : %f , pitch : %f , yaw : %f\n",Datastruct->roll , Datastruct->pitch , Datastruct->yaw);
}

