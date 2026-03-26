import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Full disclosure: This file is currently a mishmash of components from other launch files. 
# It probably needs a lot of work, but I haven't figured out how to really test it from my machine.
# I'm also not 100% sure what needs to be included in this file so im using my best judgement.

def generate_launch_description():
  
  pkg_slam_toolbox = FindPackageShare('slam_toolbox')
  pkg_nav2_bringup = FindPackageShare('nav2_bringup')
  
  # Locate the URDF file
  robot_description_content = Command(
    [
      'xacro',
      ' ',
      PathSubstitution(FindPackageShare('description')),
      '/urdf/terrence.urdf.xacro',
      ' '
    ]
  )
  
  # Robot State Publisher
  rsp = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[{'robot_description': robot_description_content}],
  )
  
  # Controllers
  controller_manager = Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[
          PathSubstitution(FindPackageShare("control"))
          / "config"
          / "controllers.yaml"
      ],
      output="both",
  )
  
  joint_state_broadcaster_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["joint_state_broadcaster"],
  )

  terrence_controller_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["terrence_controller"],
  )

  # I don't know if we want foxglove bridge to start automatically. 
  # I'm leaving it commented out for now since I dont use it on my machine

  # foxglove = Node(
  #   package = 'foxglove_bridge',
  #   executable = 'foxglove_bridge',
  #   name = 'foxglove_bridge',
  # )

  # Calculates the map to odom transform
  slam_toolbox = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          [PathSubstitution(FindPackageShare("slam_toolbox")), "/launch/online_async_launch.py"]
      ),
  )
      
  # Launches the navigation stack (planner, controller, behavior trees)
  nav2_bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          [PathSubstitution(FindPackageShare("nav2_bringup")), "/launch/navigation_launch.py"]
      ),
      launch_arguments={
          'params_file': [PathSubstitution(FindPackageShare("bringup")), "/config/nav2_params.yaml"]
      }.items(),
  )

  # Converts point cloud to laser scan so nav2 can use it
  pointcloud_to_laserscan = Node(
    package = 'pointcloud_to_laserscan',
    executable = 'pointcloud_to_laserscan_node',
    name = 'pointcloud_to_laserscan',
    remappings = [
      ('cloud_in', '/camera/points'),
      ('scan', '/scan')
    ]
  )
  
  # Launches the RealSense (depth camera and IMU) using the camera package
  # camera = IncludeLaunchDescription(
  #   PythonLaunchDescriptionSource(
  #     [PathSubstitution(FindPackageShare("camera")), "/launch/realsense.launch.py"]
  #   )
  # )
  
  ekf_node = Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      parameters=[
          PathSubstitution(FindPackageShare("bringup"))
          / "config"
          / "ekf_params.yaml"
      ]
  )

  return LaunchDescription([
      rsp,
      controller_manager,
      joint_state_broadcaster_spawner,
      terrence_controller_spawner,
      # foxglove,
      slam_toolbox,
      nav2_bringup,
      pointcloud_to_laserscan,
      # camera
      ekf_node
  ])