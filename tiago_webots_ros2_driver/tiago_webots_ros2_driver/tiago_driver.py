# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 TIAGo Iron driver."""

import rclpy
from webots_ros2_core.webots_differential_drive_node import WebotsDifferentialDriveNode

from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# robot device names must match `name` tags in `*.wbt` files placed in `worlds` directory
# `always_publish` does not check number of subscribers
DEVICE_CONFIG = {
    'robot': {
        'publish_base_footprint': True
    },
    # see `name` @ /usr/local/webots/projects/devices/hokuyo/protos/HokuyoUrg04lxug01.proto
    'Hokuyo URG-04LX-UG01': {
        'frame_id': 'base_laser_link',
        'topic_name': '/scan',
        'always_publish': True
    },
    'inertial_unit+accelerometer+gyro': {
        'frame_id': 'imu_link',
        'topic_name': '/imu',
        'always_publish': True
    },
    'Camera 2D': {
        'frame_id': 'camera_link',
        'topic_name': '/camera_head',
        'always_publish': True
    },
    'gps': {
        'frame_id': 'gps_link',
        'topic_name': '/gps',
        'always_publish': True
    },
    'base_sonar_01_link': {
        'frame_id': 'base_sonar_back_right_link',
        'topic_name': '/sonar_back_right',
        'always_publish': True
    },
    'base_sonar_02_link': {
        'frame_id': 'base_sonar_back_middle_link',
        'topic_name': '/sonar_back_middle',
        'always_publish': True
    },
    'base_sonar_03_link': {
        'frame_id': 'base_sonar_back_left_link',
        'topic_name': '/sonar_back_left',
        'always_publish': True
    },
    # "[tiago_driver]: Device `base_cover_link` is not considered!
    # The device doesn't exist or it is not supported."
    # 'base_cover_link': {
    #     'frame_id': 'base_bumper_link',
    #     'topic_name': '/bumper',
    #     'always_publish': True
    # }
}

class TiagoIronDriver(WebotsDifferentialDriveNode):
    def __init__(self, args):
        super().__init__(
            'tiago_driver',
            args,
            # names assigned to parameters must match the ones defined to robot's `.proto` file,
            # e.g., check "/usr/local/webots/projects/robots/pal_robotics/tiago_base/protos/TiagoBase.proto"
            left_encoder='wheel_left_joint_sensor',
            left_joint='wheel_left_joint',
            right_encoder='wheel_right_joint_sensor',
            right_joint='wheel_right_joint',
            robot_base_frame='base_link',
            wheel_distance=0.5,
            wheel_radius=0.12
            # few args left with default values:
            # * command_topic='/cmd_vel',
            # * odometry_topic='/odom',
            # * odometry_frame='odom',
        )
        self.start_device_manager(DEVICE_CONFIG)

        transform = TransformStamped()
        transform.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        # laser (must be rotated, because `webots` automatically
        # assigns `rotated` frame to messages)
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'base_laser_link'
        transform.transform.rotation.x = +0.5
        transform.transform.rotation.y = -0.5
        transform.transform.rotation.z = -0.5
        transform.transform.rotation.w = +0.5
        transform.transform.translation.x = 0.202
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = -0.004
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(transform)

        # IMU - same place as laser, no rotations
        transform.header.frame_id = 'base_link'
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        transform.child_frame_id = 'imu_link'
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(transform)

        # camera
        # TODO: fix orientation, can't debug properly without
        # working image view in `rviz2` or `rqt_image_view`
        transform.header.frame_id = 'head_2_link'
        transform.child_frame_id = 'camera_link'
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = 0.107
        transform.transform.translation.y = 0.0802
        transform.transform.translation.z = 0.0
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(transform)

        # TODO: `RobotModel` can't be visualized in `rviz2` due to lack
        # of transforms between `base_link` and:
        # * "caster_front_right_2_link"
        # * "caster_front_right_1_link"
        # * "caster_front_left_2_link"
        # * "caster_front_left_1_link"
        # * "caster_back_right_2_link"
        # * "caster_back_right_1_link"
        # * "caster_back_left_2_link"
        # * "caster_back_left_1_link"

def main(args=None):
    rclpy.init(args=args)
    driver = TiagoIronDriver(args=args)
    rclpy.spin(driver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
