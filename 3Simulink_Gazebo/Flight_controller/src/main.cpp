#include "globals.h"
#include "sensor.h"
#include "attitude_control.h"
#include "position_control.h"
#include "vision_control.h"
#include "planning.h"
#include "my_mavlink.h"

int main(int argc, char **argv) {
    int mode = 1;
    state_t.baro_valid = false;

    // 初始化 Gazebo 客户端
    gazebo::client::setup(argc, argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // 订阅传感器（使用与之前相同的命名空间）
    //auto imu_sub = node->Subscribe<sensor_msgs::msgs::Imu>("/gazebo/runway/iris/imu", imu_callback);
    auto imu_sub = node->Subscribe("/gazebo/runway/iris/iris_D435i/imu_link/imu_sensor/imu", imu_callback);
    auto mag_sub = node->Subscribe("/gazebo/runway/iris/mag", mag_callback);
    auto baro_sub = node->Subscribe<sensor_msgs::msgs::Pressure>("/gazebo/runway/iris/baro", baro_callback);
    auto gps_sub = node->Subscribe<sensor_msgs::msgs::SITLGps>("/gazebo/runway/iris/link/gps0", gps_callback);
    //auto camera_sub = node->Subscribe<gazebo::msgs::ImageStamped>("/gazebo/runway/iris/iris_D435i/camera_link/camera/image", camera_callback);

    // 发布电机指令
    auto motor_pub = node->Advertise<mav_msgs::msgs::CommandMotorSpeed>(
        "/gazebo/runway/iris/gazebo/command/motor_speed");

    // 初始化串口
    serial.set_option(boost::asio::serial_port_base::baud_rate(BAUD_RATE));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));

    // 启动 MAVLink 收发线程
    std::thread send_th([&]() { mavlink_send_thread(io, serial); });
    std::thread recv_th([&]() { mavlink_recv_thread(io, serial, motor_pub); });

    //相机线程
    std::thread vision_thread([&node]() {vision_thread_func(node);});


    // 控制循环线程 (200 Hz)
    std::thread control_thread([&](){
        const int freq = 200;
        const float dt = 1.0 / freq;  // 0.005 秒
        auto next_loop = std::chrono::steady_clock::now() + std::chrono::microseconds(1000000 / freq);

        while (true) {

            switch(mode)
            {
                case RemoteControl :{
                    break;
                }
                    

                case Automatic :{
                    sensor_attitude_data(&sensor_att_t,dt);
                    sensor_position_data(&sensor_pos_t);
                    road_plan_circle(&sensor_att_t, &sensor_pos_t, throttle, target_height, dt);
                    // 发布电机指令
                    mav_msgs::msgs::CommandMotorSpeed motor_msg;
                    motor_msg.add_motor_speed(motor[0]);
                    motor_msg.add_motor_speed(motor[1]);
                    motor_msg.add_motor_speed(motor[2]);
                    motor_msg.add_motor_speed(motor[3]);
                    motor_pub->Publish(motor_msg);
                    break;
                }

                case HIL:{
                    // 发布到 Gazebo
                    mav_msgs::msgs::CommandMotorSpeed motor_msg;
                    motor_msg.add_motor_speed(motor_mavlink[0]);
                    motor_msg.add_motor_speed(motor_mavlink[1]);
                    motor_msg.add_motor_speed(motor_mavlink[2]);
                    motor_msg.add_motor_speed(motor_mavlink[3]);
                    motor_pub->Publish(motor_msg);
                    break;
                }
                default:{
                    break;
                }
            }



            static auto last_time = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            double dt_actual = std::chrono::duration<double>(now - last_time).count();
            last_time = now;

            printf("dt : %f\n",dt_actual);
            printf("alt : %f , throttle_adj : %f\n",sensor_pos_t.z, z_c_t.output);
            printf("---------------------------------------\n");
            printf("x : %f , y : %f\n",sensor_pos_t.x, sensor_pos_t.y);
            //printf("ex : %f , ey : %f\n",ex,ey);
            //printf("theta : %f , yaw_filt: %f , yaw_error : %f\n",theta,yaw,yaw_error);
            fflush(stdout);


            // 等待下一个周期
            std::this_thread::sleep_until(next_loop);
            next_loop += std::chrono::microseconds(1000000 / freq);
        }
    });

    // 主线程保持 Gazebo 消息循环
    while (true) {
        gazebo::common::Time::MSleep(100);
    }

    // 在vision程序结束前清理
    display_running = false;

    if (vision_thread.joinable())
        vision_thread.join();

    if(mode == HIL)
    {
        send_th.join();
        recv_th.join();
    }

    gazebo::client::shutdown();
    return 0;
}
