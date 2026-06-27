#include "MyTask.h"


int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

#define RX_BUF_SIZE 2048
extern USBD_HandleTypeDef hUsbDeviceFS;
extern volatile uint16_t rx_head, rx_tail;
extern uint8_t rx_buf[2048];

// 队列句柄
extern QueueHandle_t sensorQueue;
extern QueueHandle_t motorQueue;
extern QueueHandle_t gpsQueue;



void MyTask_Init(void)
{
    // ----- 静态任务所需资源 -----
    // 任务控制块
    static StaticTask_t xTaskCtrlCDC;
    static StaticTask_t xTaskCtrlFlight;
    static StaticTask_t xTaskCtrlMotor;

    // 任务栈（大小根据实际需求调整，单位：字）
    #define CDC_STACK_SIZE      configMINIMAL_STACK_SIZE * 4
    #define FLIGHT_STACK_SIZE   configMINIMAL_STACK_SIZE * 8
    #define MOTOR_STACK_SIZE    configMINIMAL_STACK_SIZE * 4

    static StackType_t xStackCDC[CDC_STACK_SIZE];
    static StackType_t xStackFlight[FLIGHT_STACK_SIZE];
    static StackType_t xStackMotor[MOTOR_STACK_SIZE];

    // ----- 静态队列所需资源 -----
    // 传感器队列
    #define SENSOR_QUEUE_LENGTH 5
    #define SENSOR_QUEUE_ITEM_SIZE sizeof(mavlink_hil_sensor_t)
    static StaticQueue_t xQctrlSensor;
    static uint8_t xQbufSensor[SENSOR_QUEUE_LENGTH * SENSOR_QUEUE_ITEM_SIZE];

    // 电机队列
    #define MOTOR_QUEUE_LENGTH 5
    #define MOTOR_QUEUE_ITEM_SIZE (sizeof(float) * 4)
    static StaticQueue_t xQctrlMotor;
    static uint8_t xQbufMotor[MOTOR_QUEUE_LENGTH * MOTOR_QUEUE_ITEM_SIZE];
    
    // GPS 队列静态资源
    #define GPS_QUEUE_LENGTH 2                    
    #define GPS_QUEUE_ITEM_SIZE sizeof(mavlink_hil_gps_t)
    static StaticQueue_t xQctrlGps;
    static uint8_t xQbufGps[GPS_QUEUE_LENGTH * GPS_QUEUE_ITEM_SIZE];



    // 创建静态队列
    sensorQueue = xQueueCreateStatic(
        SENSOR_QUEUE_LENGTH,
        SENSOR_QUEUE_ITEM_SIZE,
        xQbufSensor,
        &xQctrlSensor);

    motorQueue = xQueueCreateStatic(
        MOTOR_QUEUE_LENGTH,
        MOTOR_QUEUE_ITEM_SIZE,
        xQbufMotor,
        &xQctrlMotor);
    
    gpsQueue = xQueueCreateStatic(
        GPS_QUEUE_LENGTH,
        GPS_QUEUE_ITEM_SIZE,
        xQbufGps,
        &xQctrlGps); 

    // 创建静态任务
    xTaskCreateStatic(
        StartCDCReceiveTask,   // 任务函数
        "CDC_Recv",            // 任务名称
        CDC_STACK_SIZE,        // 栈大小（字）
        NULL,                  // 参数
        3,                     // 优先级（数字含义取决于配置，通常数值越小优先级越低）
        xStackCDC,             // 栈数组
        &xTaskCtrlCDC);        // 控制块

    xTaskCreateStatic(
        FlightControlTask,
        "FlightCtrl",
        FLIGHT_STACK_SIZE,
        NULL,
        4,                     // 最高优先级
        xStackFlight,
        &xTaskCtrlFlight);

    xTaskCreateStatic(
        SendActuatorTask,
        "MotorSend",
        MOTOR_STACK_SIZE,
        NULL,
        2,                     // 较低优先级
        xStackMotor,
        &xTaskCtrlMotor);

    // 启动 USB CDC 接收
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

}

extern QueueHandle_t sensorQueue;
extern QueueHandle_t motorQueue;

//接收解析任务
void StartCDCReceiveTask(void *argument) {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;

    for (;;) {
        if (rx_head != rx_tail) {
            byte = rx_buf[rx_tail];
            rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_HIL_SENSOR) {
//                    mavlink_hil_sensor_t sensor;
                    mavlink_msg_hil_sensor_decode(&msg, &sensor);
                    // 发送到飞控任务队列（非阻塞）
                    xQueueSend(sensorQueue, &sensor, 0);
                }
                else if (msg.msgid == MAVLINK_MSG_ID_HIL_GPS) {
//                    mavlink_hil_gps_t gps;
                    mavlink_msg_hil_gps_decode(&msg, &gps);
                    // 发送到飞控任务队列（非阻塞）
                    xQueueSend(gpsQueue, &gps, 0);
                }
            }
        } else {
            osDelay(1);// 缓冲区空，让出 CPU
        }
    }
}


//飞控任务（200 Hz）
void FlightControlTask(void *argument) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(5);  // 200 Hz

    uint8_t mode = 1;
    state_t.baro_valid = 0;
    
    float fc_motor[4] = {0};
    
    for (;;) {
        //vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // 非阻塞获取最新传感器
        if (xQueueReceive(sensorQueue, &sensor, 0) == pdTRUE) {
            //注意这里的dt与configTICK_RATE_HZ设置有关
            sensor_attitude_data(&sensor_att_t,xPeriod * 0.001);
        }
        // 非阻塞获取最新 GPS 数据
        if (xQueueReceive(gpsQueue, &gps, 0) == pdTRUE) {
            sensor_position_data(&sensor_pos_t);
        }
        
        switch(mode)
        {
            case RemoteControl :
                break;;

            case Automatic :
                road_plan_circle(&sensor_att_t, &sensor_pos_t, throttle, target_height, xPeriod * 0.001);
                for (int i = 0; i < 4; ++i) {
                    fc_motor[i] = motor[i];
                }
                break;

            default:
                break;
        }

        // 发送电机值到发送任务（非阻塞）
        xQueueSend(motorQueue, fc_motor, 0);
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


//发送任务
void SendActuatorTask(void *argument) {
    float sa_motor[4];
    for (;;) {
        // 等待电机值（阻塞）
        if (xQueueReceive(motorQueue, sa_motor, portMAX_DELAY) == pdTRUE) {
            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            uint64_t time_us = HAL_GetTick() * 1000;

            float controls[16] = {0};
            controls[0] = sa_motor[0];
            controls[1] = sa_motor[1];
            controls[2] = sa_motor[2];
            controls[3] = sa_motor[3];
            uint8_t mode = 0;
            uint64_t flags = 0;

            mavlink_msg_hil_actuator_controls_pack(
                1, 1, &msg, time_us, controls, mode, flags);

            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            CDC_Transmit_FS(buf, len);
        }
    }
}

