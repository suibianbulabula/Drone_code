#include "vision_control.h"

//相机回调
void camera_callback(const boost::shared_ptr<const gazebo::msgs::ImageStamped>& msg) {
    const auto& image = msg->image();        // 获取真正的 Image 消息
    int width = image.width();
    int height = image.height();
    if (width <= 0 || height <= 0) {
        std::cerr << "Camera: invalid dimensions " << width << "x" << height << std::endl;
        return;
    }

    const std::string& data = image.data();
    if (data.size() != width * height * 3) {   // R8G8B8 → 3 bytes/pixel
        std::cerr << "Camera: data size mismatch" << std::endl;
        return;
    }

    cv::Mat img(height, width, CV_8UC3);
    memcpy(img.data, data.data(), data.size());
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);   // RGB → BGR (OpenCV 默认)

    std::lock_guard<std::mutex> lock(image_mutex);
    img.copyTo(current_frame);
    new_frame = true;
}


void vision_thread_func(gazebo::transport::NodePtr &node) {
    // 订阅相机话题（和之前一样）
    auto camera_sub = node->Subscribe<gazebo::msgs::ImageStamped>(
        "/gazebo/runway/iris/iris_D435i/camera_link/camera/image",
        [](const boost::shared_ptr<const gazebo::msgs::ImageStamped>& msg) {
            // 图像回调：仅负责将图像放入队列或直接处理
            const auto& image = msg->image();
            int width = image.width();
            int height = image.height();
            if (width <= 0 || height <= 0) return;

            const std::string& data = image.data();
            if (data.size() != width * height * 3) return;

            cv::Mat img(height, width, CV_8UC3);
            memcpy(img.data, data.data(), data.size());
            cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

            // 检测绿色方块
            cv::Mat hsv;
            cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
            cv::Scalar lower(35, 80, 80);
            cv::Scalar upper(85, 255, 255);
            cv::Mat mask;
            cv::inRange(hsv, lower, upper, mask);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            int err_x = 0, err_y = 0;
            bool found = false;
            if (!contours.empty()) {
                auto max_cnt = std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                        return cv::contourArea(a) < cv::contourArea(b);
                    });
                double area = cv::contourArea(*max_cnt);
                if (area > 500) {
                    cv::Rect bbox = cv::boundingRect(*max_cnt);
                    int cx = bbox.x + bbox.width / 2;
                    int cy = bbox.y + bbox.height / 2;
                    err_x = cx - img.cols / 2;
                    err_y = cy - (img.rows - 1);   // 底部为0
                    found = true;
                    cv::rectangle(img, bbox, cv::Scalar(0, 255, 0), 2);
                }
            }

            // 更新全局视觉数据
            {
                std::lock_guard<std::mutex> lock(vision_mutex);
                target_found = found;
                if (found) {
                    vision_err_x = err_x;
                    vision_err_y = err_y;
                }
                // 更新显示画面
                img.copyTo(display_frame);
                display_updated = true;
            }
        });
    
    // 显示循环（可选，独立线程或在此线程中处理）
    while (display_running) {
        cv::Mat frame_to_show;
        {
            std::lock_guard<std::mutex> lock(vision_mutex);
            if (display_updated) {
                display_frame.copyTo(frame_to_show);
                display_updated = false;
            }
        }
        if (!frame_to_show.empty()) {
            cv::imshow("Camera", frame_to_show);
            cv::waitKey(1);
            static int save_count = 0;
            if (++save_count % 30 == 0) {
                cv::imwrite("/tmp/debug.jpg", frame_to_show);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 避免忙等
        }
    }
}


//----------------视觉识别
void road_plan_circle_camara()
{
    int cur_err_x = 0, cur_err_y = 0;
    bool has_target = false;
    {
    std::lock_guard<std::mutex> lock(vision_mutex);
    if (target_found) {
        cur_err_x = vision_err_x;
        cur_err_y = vision_err_y;
        has_target = true;
    }
    }

    // 视觉水平误差 → 目标滚转角（根据之前的约定）
    if (has_target) {
        double K_vis_roll = 0.02;   // 像素误差到角度的增益
        double target_roll_vis = K_vis_roll * cur_err_x;
        // 限幅
        if (target_roll_vis > 5.0) target_roll_vis = 5.0;
        if (target_roll_vis < -5.0) target_roll_vis = -5.0;
        // target_roll_deg += target_roll_vis;
    }

    // 垂直误差 → 高度修正（如果你要用视觉高度）
    if (has_target) {
        double K_vis_alt = 0.005;   // 像素误差到高度修正 (m)
        double alt_correction = -K_vis_alt * cur_err_y;
        // 限制修正范围
        if (alt_correction > 0.5) alt_correction = 0.5;
        if (alt_correction < -0.5) alt_correction = -0.5;
        // 调整目标高度
        //target_height = target_height + alt_correction;
    }

}
