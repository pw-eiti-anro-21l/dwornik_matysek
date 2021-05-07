
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  rviz2_file_name = 'robot.rviz'
  return LaunchDescription([
      Node(
          package='rviz2',
          executable='rviz2',
          name='my_rviz',
          output='screen',
          parameters=[{'use_sim_time': use_sim_time}],
          arguments=['-d', os.path.join(get_package_share_directory('my_urdf'),rviz2_file_name)])
     
  ])