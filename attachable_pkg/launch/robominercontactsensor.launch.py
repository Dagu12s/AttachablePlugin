import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
  pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
  pkg = get_package_share_directory('rm2_simulation')
    
  ign_gazebo = IncludeLaunchDescription( PythonLaunchDescriptionSource(
  os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'))) #launch_arguments={'ign_args': os.path.join(pkg, 'worlds', 'cave_world.sdf')}.items(),)
            
  return LaunchDescription([
     DeclareLaunchArgument('ign_args',
     default_value=[os.path.join(pkg, 'worlds', 'test_worldContactSensor.sdf') +' --gui-config ' +
      os.path.join(pkg, 'ign', 'gui.config'), ''], description='Ignition Gazebo arguments'),
     ign_gazebo,
        #  spawn,
    # Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
    #     arguments=[urdf]),
     Node(
        package='ros_ign_bridge',
      executable='parameter_bridge',
        arguments=[
            '/AttacherContact/contact@std_msgs/msg/String@ignition.msgs.StringMsg',
            '/AttacherContact/touched@std_msgs/msg/Bool@ignition.msgs.Boolean'
            # '/box2/attach@std_msgs/msg/String@ignition.msgs.StringMsg', 
            # '/box2/detach@std_msgs/msg/Empty@ignition.msgs.Empty'
            ],
        output='screen'
        )


    ])

    