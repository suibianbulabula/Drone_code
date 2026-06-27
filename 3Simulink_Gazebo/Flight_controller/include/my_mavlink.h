#ifndef MY_MAVLINK_H__
#define MY_MAVLINK_H__

#include <boost/asio.hpp>
#include <gazebo/transport/transport.hh>
#include <mutex>
#include <ignition/math/Vector3.hh>
#include "globals.h"
#include "common/mavlink.h"


void mavlink_send_thread(boost::asio::io_service &io, boost::asio::serial_port &serial) ;
void mavlink_recv_thread(boost::asio::io_service &io, boost::asio::serial_port &serial,
                         gazebo::transport::PublisherPtr &motor_pub);

#endif