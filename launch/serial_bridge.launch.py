from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    
    send_node = Node(
        package='ros2serial_odom',
        executable='serial_send_node',
    )

    receive_node = Node(
        package='ros2serial_odom',
        executable='serial_receive_node',
        remappings=[('/serial_odom','/odom')],
    )

    ld = LaunchDescription()
    #ld.add_action(device_name_arg)
    ld.add_action(send_node)
    ld.add_action(receive_node)

    return ld