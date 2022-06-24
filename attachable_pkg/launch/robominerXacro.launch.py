import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf_file_name = 'myfirst.xacro'
  pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
  pkg = get_package_share_directory('rm2_simulation')


  print("urdf_file_name : {}".format(urdf_file_name))

  xacro_file = urdf_file_name #os.path.join("models", urdf_file_name)

  robot_description_config = xacro.process_file(xacro_file)
  robot_desc = robot_description_config.toxml()
  
  return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[#{'use_sim_time': use_sim_time},
                      {"robot_description": robot_desc}],),
          #arguments=[robot_desc]),

      Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/start@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/box2/attach@std_msgs/msg/String@ignition.msgs.StringMsg', 
            '/box2/detach@std_msgs/msg/Empty@ignition.msgs.Empty'],
        output='screen'
        )
      
      # Node(
      #     package='urdf_tutorial',
      #     executable='state_publisher',
      #     name='state_publisher',
      #     output='screen'),
  ])