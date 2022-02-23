/**
 * @file tiers.cpp
 * @author EunsooLim (ies041196@gmail.com)
 * @brief Get a string data from Serial Communication and send it to other node
 * @version 0.1
 * @date 2022-01-31
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "tiers1.h"

/**
 * @brief set parameter function
 *
 */
void Serial::rosparamHandler() {
    pram_handler_.param<int32_t>("BAUDRATE", baudrate_, 51200);
    pram_handler_.param<std::string>("SERIALPORT", serial_port_name_, "/dev/ttyUSB0");
    pram_handler_.param<std::string>("SERIAL_TOPIC_NAME", serial_topic_name_, "tiers1/serial_data");
}

/**
 * @brief set serial communication up
 *
 */
void Serial::serialInit() {
    fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(fd_ != -1);

    // newtio_ <-- serial port setting.
    memset(&newtio_, 0, sizeof(struct termios));
    newtio_.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio_.c_iflag = IGNPAR | ICRNL;
    newtio_.c_oflag = 0;
    newtio_.c_lflag = ~(ICANON | ECHO | ECHOE | ISIG);

    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &newtio_);
}

/**
 * @brief get a serialdata and send it as topic format
 *
 */
void Serial::serialDataPub() {
    char buf[BUFSIZ];
    read(fd_, buf, sizeof(buf));
    assert(fd_ != -1);
    std::string str(buf);
    serial_msg_.data = str;
    serial_data_pub_.publish(serial_msg_);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "tiers_1");
    Serial serial;
    // convert baudrate to Hz
    ros::Rate serial_rate(serial.baud2Hz());
    while (ros::ok()) {
        serial.serialDataPub();
        ros::spinOnce();
        serial_rate.sleep();
    }

    return 0;
}
