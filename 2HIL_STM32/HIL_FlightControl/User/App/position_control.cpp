#include "position_control.h"

PID_t x_c_t = {0};
PID_t y_c_t = {0};
PID_t z_c_t = {0};


void position_control(Sensor_Att_t *Datastruct_att, Sensor_Pos_t *Datastruct_pos, 
                                                float vel, float target_x, float target_y, float target_z, float target_yaw_deg, float dt)
{
//-----------------水平位置控制----------------
    float body_err_x = 0;
    float body_err_y = 0;

    //sensor_attitude_data(Datastruct_att, dt);
    //sensor_position_data(Datastruct_pos);

    float err_x =target_x - ( Datastruct_pos->x);  // 南北误差（米）
    float err_y =target_y - (Datastruct_pos->y);   // 东西误差（米）

    //防止尖峰数据
    if(err_x > 60)err_x = 0;
    if(err_y < -10)err_y = 0;

    //坐标变换，世界坐标系到机体，绕Z轴旋转
    body_err_x =  err_x * cos(ANGLE_2_RAD(Datastruct_att->yaw)) - err_y * sin(ANGLE_2_RAD(Datastruct_att->yaw));
    body_err_y =  err_x * sin(ANGLE_2_RAD(Datastruct_att->yaw)) + err_y * cos(ANGLE_2_RAD(Datastruct_att->yaw));
    
    x_c_t.Kp = 4;
    x_c_t.Ki = 0;
    x_c_t.Kd = 0.5;

    y_c_t.Kp = 4;
    y_c_t.Ki = 0;
    y_c_t.Kd = 0.5;    

    //pid -- x
    x_c_t.err = body_err_x;
        // 计算误差变化率 (米/秒)
    x_c_t.derivative = (x_c_t.err - x_c_t.last_err) / dt;
    x_c_t.output = -(x_c_t.Kp * x_c_t.err + x_c_t.Kd * x_c_t.derivative);
        // 存储本帧误差供下一帧使用
    x_c_t.last_err = x_c_t.err;

    //pid -- y
    y_c_t.err = body_err_y;
        // 计算误差变化率 (米/秒)
    y_c_t.derivative = (y_c_t.err - y_c_t.last_err) / dt;
    y_c_t.output = -(y_c_t.Kp * y_c_t.err + y_c_t.Kd * y_c_t.derivative);
        // 存储本帧误差供下一帧使用
    y_c_t.last_err = y_c_t.err;

    // 输出角度限幅
    if (x_c_t.output > 15.0) x_c_t.output = 15.0;
    if (x_c_t.output < -15.0) x_c_t.output = -15.0;
    if (y_c_t.output > 15.0) y_c_t.output = 15.0;
    if (y_c_t.output < -15.0) y_c_t.output = -15.0;

//-----------------高度位置控制----------------
    z_c_t.Kp = 1.8;
    z_c_t.Ki = 0;
    z_c_t.Kd = 0.8;
    z_c_t.err = target_z - Datastruct_pos->z;

    if (state_t.baro_valid) {
        //微分单独滤波
            // 强低通滤波，使高度值连续变化
            static float alt_filt = 0;
            alt_filt = Datastruct_pos->z;  // 首次初始化
            float filter_coeff = 0.05;       // 滤波系数，越小越平滑（0.01~0.1）
            alt_filt = (1.0 - filter_coeff) * alt_filt + filter_coeff * Datastruct_pos->z;
            // 使用滤波后的高度计算误差
            float d_alt_error = target_z - alt_filt;
            static float last_alt_error = 0;
            z_c_t.derivative = (d_alt_error - last_alt_error) / dt;
            last_alt_error = d_alt_error;

        //比例积分微分控制调整油门
        z_c_t.output = z_c_t.Kp * z_c_t.err + z_c_t.Ki * z_c_t.integrator  + z_c_t.Kd * z_c_t.derivative;

        //气压计高度输出简易滤波
        static float last_throttle_adj = 0;
        if(z_c_t.output - last_throttle_adj > 3)z_c_t.output = last_throttle_adj;
        else if(z_c_t.output - last_throttle_adj < -3)z_c_t.output = last_throttle_adj;
        last_throttle_adj = z_c_t.output;

        // 限制油门修正量，防止突变
        if (z_c_t.output > 100.0)  z_c_t.output = 100.0;
        if (z_c_t.output < -100.0) z_c_t.output = -100.0;
    }

    vel = vel + z_c_t.output ;
    attitude_control(Datastruct_att, vel,  x_c_t.output  ,  y_c_t.output  , target_yaw_deg, dt);
}
