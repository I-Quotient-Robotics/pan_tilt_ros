from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port_name_arg = DeclareLaunchArgument('port_name', default_value='/dev/pan_tilt',
                                         description='usb bus name, e.g. /dev/pan_tilt')
    
    id_arg = DeclareLaunchArgument('ID',default_value='1')
    yaw_joint_name_arg = DeclareLaunchArgument('yaw_joint_name',default_value='pan_tilt_yaw_joint')
    pitch_joint_name_arg = DeclareLaunchArgument('pitch_joint_name',default_value='pan_tilt_pitch_joint')

    PanTiltDriver_Node = Node(
                        package='pan_tilt_driver',
                        executable='PanTiltDriverNode',
                        output='screen',
                        parameters=[{
                          'port_name': LaunchConfiguration('port_name'),
                          'ID': LaunchConfiguration('ID'),
                          'yaw_joint_name': LaunchConfiguration('yaw_joint_name'),
                          'pitch_joint_name': LaunchConfiguration('pitch_joint_name')
                        }]
                        )
    return LaunchDescription([
        port_name_arg,
        id_arg,
        yaw_joint_name_arg,
        pitch_joint_name_arg,
        PanTiltDriver_Node
    ])
