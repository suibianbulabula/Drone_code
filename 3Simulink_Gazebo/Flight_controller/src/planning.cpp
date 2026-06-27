#include "planning.h"

void road_plan_circle(Sensor_Att_t *Datastruct_att, Sensor_Pos_t *Datastruct_pos, float vel, float target_z, float dt)
{
    static float theta = 0;

    float omega = 5.0;         // 角速度（度/秒），30 度/秒 → 12 秒一圈
    theta += omega * dt;         // dt ≈ 0.005s，每次增加约 0.025 度
    if (theta >= 360.0) theta -= 360.0;   // 归一化

    //圆心(0,-2)
    const float distance = 2;
    const float center_x  = 0.0;     // 圆心x
    const float center_y = -2.0;    // 圆心y（设为 -2 米，使 0° 时恰好经过 (0,0)）

    float target_x =  distance *  sin(ANGLE_2_RAD(theta)) + center_x;
    float target_y =  distance * cos(ANGLE_2_RAD(theta)) + center_y;

    float target_yaw_deg = theta;

    position_control(Datastruct_att, Datastruct_pos,vel,  target_x, target_y, target_z, target_yaw_deg, dt);
    printf("theta : %f\n",theta);
    printf("target_x : %f , target_y %f\n",target_x,target_y);
}