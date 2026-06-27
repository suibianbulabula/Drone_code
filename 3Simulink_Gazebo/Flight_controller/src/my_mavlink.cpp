#include "my_mavlink.h"

void mavlink_send_thread(boost::asio::io_service &io, boost::asio::serial_port &serial) {
    const int freq = 200;
    const auto period = std::chrono::microseconds(1000000 / freq);
    auto next_time = std::chrono::steady_clock::now() + period;
    static int gps_send_counter = 0;

    while (true) {
        // 获取传感器数据
        ignition::math::Vector3d g, a, m;
        double p, lat, lon, alt;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            g = gyro;
            a = acc;
            m = mag;
            p = pressure;
            lat = gps_lat;
            lon = gps_lon;
            alt = gps_alt;
        }

        // 组装 HIL_SENSOR
        uint64_t time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // ========== 1. 发送 HIL_SENSOR（每周期都发）==========
        {
            mavlink_message_t msg;
            mavlink_msg_hil_sensor_pack(
                SYS_ID, COMP_ID, &msg,
                time_us,
                a.X(), a.Y(), a.Z(),      // 加速度 m/s^2
                g.X(), g.Y(), g.Z(),      // 角速度 rad/s
                m.X(), m.Y(), m.Z(),      // 磁场 Tesla
                0.0f,                     // abs_pressure 不用
                0.0f,                     // diff_pressure
                static_cast<float>(p),    // 气压 hPa（修正：去掉 *0.01）
                25.0f,                    // 温度
                0, 0                      // fields_updated, id
            );
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            boost::system::error_code ec;
            write(serial, boost::asio::buffer(buf, len), ec);
            if (ec) std::cerr << "MAVLink HIL_SENSOR error: " << ec.message() << std::endl;
        }

        // ========== 2. 每 40 次循环发送一次 HIL_GPS（5 Hz）==========
        if (++gps_send_counter >= 40) {
            gps_send_counter = 0;

            mavlink_message_t gps_msg;
            mavlink_msg_hil_gps_pack(
                SYS_ID, COMP_ID, &gps_msg,
                time_us,                         // 时间戳 (微秒)
                3,                               // fix_type: 3D fix
                static_cast<int32_t>(lat * 1e7), // 纬度 (度 * 1e7)
                static_cast<int32_t>(lon * 1e7), // 经度 (度 * 1e7)
                static_cast<int32_t>(alt * 1000),// 海拔 (米 * 1000)
                0,                               // eph (水平精度, cm)
                0,                               // epv (垂直精度, cm)
                0,                               // vel (地速, cm/s)
                0,                               // vn (北向速度, cm/s)
                0,                               // ve (东向速度, cm/s)
                0,                               // vd (下降速度, cm/s)
                0,                               // cog (对地航向, 百分之一度)
                0,                               // satellites_visible
                0,                               // id (GPS ID)
                0                                // yaw (车辆偏航角，四旋翼可填0)
            );
            uint8_t gps_buf[MAVLINK_MAX_PACKET_LEN];
            uint16_t gps_len = mavlink_msg_to_send_buffer(gps_buf, &gps_msg);
            boost::system::error_code ec;
            write(serial, boost::asio::buffer(gps_buf, gps_len), ec);
            if (ec) std::cerr << "MAVLink HIL_GPS error: " << ec.message() << std::endl;
        }

        // ========== 等待下一周期 ==========
        std::this_thread::sleep_until(next_time);
        next_time += period;
    }
}


void mavlink_recv_thread(boost::asio::io_service &io, boost::asio::serial_port &serial,
                         gazebo::transport::PublisherPtr &motor_pub) {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;

    while (true) {
        boost::system::error_code ec;
        size_t len = serial.read_some(boost::asio::buffer(&byte, 1), ec);
        if (ec) { std::cerr << "Serial read error: " << ec.message() << std::endl; break; }
        if (len == 0) continue;

        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) {
                mavlink_hil_actuator_controls_t controls;
                mavlink_msg_hil_actuator_controls_decode(&msg, &controls);
                float motor_received[4];
                for (int i = 0; i < 4; i++) motor_received[i] = controls.controls[i];

                // 更新全局电机值（加锁）
                {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    for (int i = 0; i < 4; i++) motor_mavlink[i] = motor_received[i];
                }

            }
        }
    }
}
