#pragma once

#include<libserial/SerialPort.h>
#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<Eigen/Eigen>
#include<memory>
#include<iostream>

using namespace std::chrono_literals;

enum decode_stage{
    STAGE_LOOKING_SOF = 0,
    STAGE_READING_HEADER,
    STAGE_READING_PACKET
};

class ImuSerialNode : public rclcpp::Node
{
    public:
        ImuSerialNode();
        void initserial();
        bool checksum(const uint8_t *data,size_t length);
        short hex_to_short(uint8_t high_byte,uint8_t low_byte);

        void timer_callback();
        void publish_imu(const sensor_msgs::msg::Imu& msg);

    private:
        std::string serial_port;

        decode_stage stage{};
        uint16_t remaining_byte{};
        uint8_t rx_buffer[14]{};

        double acc_x;
        double acc_y;
        double acc_z;

        double gym_x;
        double gym_y;
        double gym_z;

        double angle_row;
        double angle_pitch;
        double angle_yaw;

        uint16_t data_length{};
        uint16_t cmd_id{};
        std::shared_ptr<LibSerial::SerialPort> serial;
        rclcpp::TimerBase::SharedPtr __timer__;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr __imu_pub__;

};