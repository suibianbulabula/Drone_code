#ifndef MYTASK_H__
#define MYTASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include <stdio.h>

#include "mavlink.h"
#include "usbd_cdc_if.h"

#include "globals.h"
#include "sensor.h"
#include "attitude_control.h"
#include "position_control.h"
#include "planning.h"


void StartCDCReceiveTask(void *argument);
void FlightControlTask(void *argument);
void SendActuatorTask(void *argument);

void MyTask_Init(void);

#endif
