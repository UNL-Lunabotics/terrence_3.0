import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchService


def generate_launch_description():
    realsense2_dir = get_package_share_directory('realsense2_camera')

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense2_dir, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                # Streams
                'camera_name': 'camera',
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',

                # Publish PointCloud2 natively — no manual deprojection needed
                'pointcloud.enable': 'true',

                # IMU — enable both sensors and fuse them into a single /camera/imu topic
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': 'linear_interpolation',
            }.items()
        )
    ])


def main():
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()