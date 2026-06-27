#include "globals.h"

const float throttle = 803.5;
const float target_height = 0.3;
float motor[4] = {0};
float motor_mavlink[4] = {0};

Sensor_Att_t sensor_att_t = {0}; 
Sensor_Pos_t sensor_pos_t = {0};

Control_State_t state_t = {0};
Mode_e mode_e;

// 串口与 MAVLink 参数
const std::string SERIAL_PORT = "/dev/ttyACM0";
const unsigned int BAUD_RATE = 921600;
const int SYS_ID = 1;
const int COMP_ID = 1;
boost::asio::io_service io;
boost::asio::serial_port serial(io, SERIAL_PORT);

// 全局传感器数据
std::mutex data_mutex;
ignition::math::Vector3d gyro(0,0,0);
ignition::math::Vector3d acc(0,0,0);
ignition::math::Vector3d mag(0,0,0);
double gps_lat = 0;
double gps_lon = 0;
double gps_alt = 0;
double pressure = 0;

//相机数据
std::mutex image_mutex;
cv::Mat current_frame;
bool new_frame = false;
// 视觉处理结果
std::mutex vision_mutex;
bool target_found = false;
int vision_err_x = 0;          // 水平像素误差
int vision_err_y = 0;          // 垂直像素误差
cv::Mat display_frame;         // 用于显示的画面（带标注）
bool display_updated = false;  // 是否有新画面需要显示
std::atomic<bool> display_running{true};