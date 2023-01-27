#ifndef WHEELODOMETRY_HPP
#   define WHEELODOMETRY_HPP

#include <string>
#include <memory>
#include <chrono>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include "robotino2/RobotinoKinematics.hpp"

class WheelOdometry: public rclcpp::Node {
    public:
        WheelOdometry();

    private:
        void handle_wheel_pos_callback(const sensor_msgs::msg::JointState& msg);

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        geometry_msgs::msg::TransformStamped transform;
        tf2::Quaternion q;

        geometry_msgs::msg::PoseStamped robot_pose;
        nav_msgs::msg::Path path;

        RobotinoKinematics robotinoKinematics;
        std::array<float, 3> robot_position;
        std::array<float, 3> robot_speed;

        bool use_position;
        
        builtin_interfaces::msg::Time prev_stamp;
};

#endif