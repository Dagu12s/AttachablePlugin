import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import attachable_pkg.mergeROSModels as merge


from launch_ros.actions import Node

import xacro

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
  pkg_rm2 = "/home/david/rm2_workspace/src/rm2_simulation"
  pkg_attach = "/home/david/rm2_workspace/src/attach"  #pkg = get_package_share_directory('attachable_pkg')
  

  file_name = "actual_model"
  xacro_file_name = 'myfirst.xacro'
  original_xacro_file = os.path.join(pkg_attach, 'models','xacro_files', xacro_file_name)
  xacro_file = os.path.join(pkg_attach, 'models', file_name + ".xacro")
  urdf_file = os.path.join(pkg_attach, 'models', file_name + ".urdf")
  

  ign_gazebo = IncludeLaunchDescription( PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'))
      ) 
  
  
  
  with open(original_xacro_file) as fisrtfile, open(xacro_file ,"w") as secondfile:
    for line in fisrtfile:
      secondfile.write(line)
  
  robot_description_config = xacro.process_file(xacro_file, pretty_print=True)
  robot_desc = robot_description_config.toprettyxml()
  
  with open(urdf_file ,"w") as open_file:
     open_file.write(robot_desc)



  return LaunchDescription([
    DeclareLaunchArgument('ign_args',
        default_value=[os.path.join(pkg_attach, 'worlds', 'cave_world.sdf') +' --gui-config ' +
        os.path.join(pkg_attach, 'ign', 'gui.config'), ''], 
        description='Ignition Gazebo arguments'),
        ign_gazebo,
           #  spawn,
    Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf_file]),
    Node(
        package='attachable_pkg',
        executable='rosActionServerMerger.py',
        name='AttachableJointActionServer',
        output='screen'
        ),


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
        ),



  


      #     package='urdf_tutorial',
      #     executable='state_publisher',
      #     name='state_publisher',
      #     output='screen'),
    ])