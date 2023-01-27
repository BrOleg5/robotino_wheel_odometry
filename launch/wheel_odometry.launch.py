from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_name', default_value='wheel_odometry'
        ),
        DeclareLaunchArgument(
            'parent_frame_id', default_value='init_pose'
        ),
        DeclareLaunchArgument(
            'child_frame_id', default_value='wheel_odom'
        ),
        DeclareLaunchArgument(
            'use_position', default_value='True'
        ),
        DeclareLaunchArgument(
            'topic_name', default_value='/robot/joint_state'
        ),
        Node(
            package='robotino_wheel_odometry',
            executable='wheel_odometry_node',
            name=LaunchConfiguration('node_name'),
            output='screen',
            emulate_tty=True,
            parameters=[{
                'parent_frame_id': LaunchConfiguration('parent_frame_id'),
                'child_frame_id': LaunchConfiguration('child_frame_id'),
                'use_position': LaunchConfiguration('use_position')
            }],
            remappings=[
                ('/robot/joint_state', LaunchConfiguration('topic_name'))
            ]
        )
    ])