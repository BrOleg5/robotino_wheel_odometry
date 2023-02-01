// Copyright 2023 BrOleg5

#include "robotino_wheel_odometry/WheelOdometry.hpp"

WheelOdometry::WheelOdometry(): Node("wheel_odometry"), robotinoKinematics(0.04f, 0.130f, 16.f) {
    robot_position.fill(0.0f);
    robot_speed.fill(0.0f);

    this->declare_parameter<std::string>("parent_frame_id", "map");
    this->declare_parameter<std::string>("child_frame_id", "wheel_odom");

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    use_position = this->declare_parameter<bool>("use_position", true);

    transform.header.frame_id = this->get_parameter("parent_frame_id")
                                     .get_parameter_value().get<std::string>();
    transform.child_frame_id = this->get_parameter("child_frame_id")
                                    .get_parameter_value().get<std::string>();
    path.header.frame_id = this->get_parameter("parent_frame_id")
                                .get_parameter_value().get<std::string>();

    joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/robot/joint_state",
        10,
        std::bind(&WheelOdometry::handle_wheel_pos_callback, this, std::placeholders::_1)
    );

    std::string node_name = this->get_name();
    path_publisher = this->create_publisher<nav_msgs::msg::Path>("/" + node_name + "/path", 10);

    prev_stamp = this->get_clock()->now();
}

void WheelOdometry::handle_wheel_pos_callback(const sensor_msgs::msg::JointState& msg) {
    if(use_position) {
        robot_position = robotinoKinematics.direct(
            static_cast<float>(msg.position[0]),
            static_cast<float>(msg.position[1]),
            static_cast<float>(msg.position[2])
        );
    } else {
        robot_speed = robotinoKinematics.direct(
            static_cast<float>(msg.velocity[0]),
            static_cast<float>(msg.velocity[1]),
            static_cast<float>(msg.velocity[2])
        );
        double dt_sec = msg.header.stamp.sec - prev_stamp.sec + (
                            static_cast<double>(msg.header.stamp.nanosec) -
                            static_cast<double>(prev_stamp.nanosec)
                        ) / 1000000000.0;
        robot_position[0] += std::cos(robot_position[2]) * robot_speed[0] *
                             static_cast<float>(dt_sec) -
                             std::sin(robot_position[2]) * robot_speed[1] *
                             static_cast<float>(dt_sec);
        robot_position[1] += std::sin(robot_position[2]) * robot_speed[0] *
                             static_cast<float>(dt_sec) +
                             std::cos(robot_position[2]) * robot_speed[1] *
                             static_cast<float>(dt_sec);
        robot_position[2] += robot_speed[2] * static_cast<float>(dt_sec);
    }

    transform.header.stamp = this->get_clock()->now();

    transform.transform.translation.x = robot_position[0];
    transform.transform.translation.y = robot_position[1];
    transform.transform.translation.z = 0.0;

    q.setRPY(0, 0, robot_position[2]);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster->sendTransform(transform);

    robot_pose.header.stamp = msg.header.stamp;
    robot_pose.pose.position.x = transform.transform.translation.x;
    robot_pose.pose.position.y = transform.transform.translation.y;
    robot_pose.pose.position.z = transform.transform.translation.z;

    robot_pose.pose.orientation.x = transform.transform.rotation.x;
    robot_pose.pose.orientation.y = transform.transform.rotation.y;
    robot_pose.pose.orientation.z = transform.transform.rotation.z;
    robot_pose.pose.orientation.w = transform.transform.rotation.w;

    path.header.stamp = msg.header.stamp;
    path.poses.push_back(robot_pose);

    path_publisher->publish(path);

    prev_stamp = msg.header.stamp;
}
