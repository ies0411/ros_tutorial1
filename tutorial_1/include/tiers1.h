/**
 * @file tiers.h
 * @author EunsooLim(ies041196@gmail.com)
 * @brief header file of serial commuication
 * @version 0.1
 * @date 2022-01-31
 *
 * @copyright BSD
 *
 */
#ifndef __SERIAL_PUB__
#define __SERIAL_PUB__

/*
Write a simple node that echoes serial data to a String topic. Also write a
launch file that loads a config file with parameters. The parameters are (i)
serial port; (ii) baud rate; and (iii) output topic name. Provide the code within
a ROS package structure compressed in a ZIP file
*/

#include <assert.h>
#include <fcntl.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <string.h>
#include <termio.h>
#include <unistd.h>

#include <string>

class Serial {
    // ros variable
    ros::NodeHandle n_;
    ros::NodeHandle pram_handler_;
    ros::Publisher serial_data_pub_;
    std_msgs::String serial_msg_;

    // serial communication variable
    int32_t baudrate_;
    std::string serial_port_name_;
    std::string serial_topic_name_;
    int fd_;
    struct termios newtio_;

    // seiral setup function
    void serialInit();
    // parameter handler function
    void rosparamHandler();

   public:
    // get a seral data and send it as topic
    void serialDataPub();
    // convert baudrate to Hz
    u_int16_t baud2Hz() { return baudrate_ / 60.0f; }

    Serial();
    ~Serial() {
        close(fd_);
    }
};
Serial::Serial() {
    serialInit();
    rosparamHandler();
    serial_data_pub_ = n_.advertise<std_msgs::String>(serial_topic_name_, 10);
}
#endif
