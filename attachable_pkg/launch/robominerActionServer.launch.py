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
    urdf_path = pkg + '/models/rm2/rm2.urdf'
    pkg_attach = get_package_share_directory('attachable_pkg') #"/home/david/rm2_workspace/src/attach"  #pkg = get_package_share_directory('attachable_pkg')
    
    ign_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
    )

    spawn1 = Node(
                  package='ros_ign_gazebo', executable='create',
                  arguments=[
                    '-name', 'rm2_1',
                    '-file',  os.path.join(pkg, 'models', 'rm2', 'rm2_sim', 'model.sdf'),
                    '-z', '-0.18',
                    '-y', '-0.65',
                    '-x', '3.93',
                    ],
                output='screen',
                )
    
    spawn2 = Node(
                  package='ros_ign_gazebo', executable='create',
                  arguments=[
                    '-name', 'rm2_2',
                    '-file',  os.path.join(pkg, 'models', 'rm2', 'rm2_sim2', 'model.sdf'),
                    '-z', '-0.18',
                    '-y', '-0.63',
                    '-x', '3.73',
                    ],
                output='screen',
                )

    state_publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
				output='screen',
				parameters = [
					{'ignore_timestamp': False},
					{'use_tf_static': True},
					{'robot_description': open(urdf_path).read()}],
				arguments = [urdf_path])


    ign_bridge = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
		    os.path.join(pkg, 'launch', 'bridge.launch.py'),),
        )
    
    attachable_pkg = Node(
                     package='attachable_pkg',
                     executable='rosActionServerMerger.py',
                     name='AttachableJointActionServer',
                     output='screen')

    attacher_bridge1 = Node(
                      package='ros_ign_bridge',
                      executable='parameter_bridge',
                      arguments = ['/AttacherContact/contact@std_msgs/msg/String@ignition.msgs.StringMsg'],
                      output='screen')

    attacher_bridge2 = Node(
                      package='ros_ign_bridge',
                      executable='parameter_bridge',
                      arguments = ['/AttacherContact/touched@std_msgs/msg/Bool@ignition.msgs.Boolean'],
                      output='screen')
                      
    attacher_bridge3 = Node(
                      package='ros_ign_bridge',
                      executable='parameter_bridge',
                      arguments = ['/AttachableJoint/attach@std_msgs/msg/String@ignition.msgs.StringMsg'],
                      output='screen')
    attacher_bridge4 = Node(
                      package='ros_ign_bridge',
                      executable='parameter_bridge',
                      arguments = ['/AttachableJoint/detach@std_msgs/msg/Empty@ignition.msgs.Empty'],
                      output='screen')
                      

    return LaunchDescription([
      DeclareLaunchArgument(
          'ign_args',
          default_value=[os.path.join(pkg, 'worlds', 'ground_world.sdf')]), #cave_world
        ign_gazebo,
        spawn1,
        spawn2,
        ign_bridge,
        state_publisher,
        attachable_pkg,
        attacher_bridge1,
        attacher_bridge2,
        attacher_bridge3,
        attacher_bridge4,        
    ])
