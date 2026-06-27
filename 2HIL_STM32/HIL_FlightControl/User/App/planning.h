#ifndef PLANNING_H__
#define PLANNING_H__

#include "globals.h"
#include "sensor.h"
#include "attitude_control.h"
#include "position_control.h"

void road_plan_circle(Sensor_Att_t *Datastruct_att, Sensor_Pos_t *Datastruct_pos, float vel, float target_z, float dt);

#endif
