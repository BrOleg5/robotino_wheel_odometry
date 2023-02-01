// Copyright 2023 BrOleg5

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robotino_wheel_odometry/WheelOdometry.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometry>());
    rclcpp::shutdown();

    return 0;
}
