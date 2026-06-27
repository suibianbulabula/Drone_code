#ifndef  VISION_CONTROL_H__
#define VISION_CONTROL_H__

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "CommandMotorSpeed.pb.h"
#include "Groundtruth.pb.h"
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>
#include <gazebo/msgs/image.pb.h>
#include <opencv2/opencv.hpp>
#include <gazebo/msgs/image_stamped.pb.h>  

#include "globals.h"

void vision_thread_func(gazebo::transport::NodePtr &node);
void camera_callback(const boost::shared_ptr<const gazebo::msgs::ImageStamped>& msg);

#endif