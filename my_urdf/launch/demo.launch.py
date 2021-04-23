import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf_file_name = 'robot.urdf.xacro.xml'

  print("urdf_file_name : {}".format(urdf_file_name))

  urdf = os.path.join(
      get_package_share_directory('my_urdf'),
      urdf_file_name)

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
          parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro', ' ', urdf])
            }]),
      Node(
          package='my_urdf',
          executable='state_publisher',
          name='state_publisher',
          output='screen'),
      Node(
          package='my_urdf',
          executable='nonkdl_dkin',
          name='nonkdl_dkin'),
       Node(
          package='my_urdf',
          executable='kdl_dkin',
          name='kdl_dkin'),
  ])
