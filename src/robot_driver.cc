#include <tiago_webots_ros2/robot_driver.h>

namespace tiago_webots_ros {

RobotDriver::RobotDriver() :
    Node("robot_driver"),
    clock(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)) {
  enableDevices();
  setTF();
}

RobotDriver::~RobotDriver() {}

void RobotDriver::setTF() {
  static tf2_ros::StaticTransformBroadcaster br(this);
  geometry_msgs::msg::TransformStamped transformStamped;
  
  transformStamped.header.stamp = clock->now();
  transformStamped.header.frame_id = "base_link";
  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.translation.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  
  transformStamped.child_frame_id = "Hokuyo_URG_04LX_UG01";
  br.sendTransform(transformStamped);

  transformStamped.child_frame_id = "camera_2D";
  br.sendTransform(transformStamped);

  transformStamped.child_frame_id = "laser_frame";
  br.sendTransform(transformStamped);
}

void RobotDriver::enableCamera() {
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera_2D/image_raw", rclcpp::SensorDataQoS(), 
    std::bind(&RobotDriver::updateImage, this, std::placeholders::_1));
}

void RobotDriver::enableRecognition() {
  recognition_sub_webots_ = this->create_subscription<
    webots_ros2_msgs::msg::WbCameraRecognitionObjects>(
    "/camera_2D/recognitions/webots", rclcpp::SensorDataQoS(), 
    std::bind(&RobotDriver::updateRecognizedObjectsWebots, this,
    std::placeholders::_1));
}

void RobotDriver::enableLidar() {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/Hokuyo_URG_04LX_UG01", rclcpp::SensorDataQoS(),
    std::bind(&RobotDriver::updateLaserScan, this, std::placeholders::_1));
  laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 
    rclcpp::SensorDataQoS());
}

void RobotDriver::enableWheel() {
  wheels_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(), 
    std::bind(&RobotDriver::updateJoints, this, std::placeholders::_1));
}

// void RobotDriver::enableGPS() {
//   gps_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/gps/gps",
//     rclcpp::SensorDataQoS(),
//     std::bind(&RobotDriver::updateGPS, this, std::placeholders::_1));
// }

void RobotDriver::updateJoints(const sensor_msgs::msg::JointState::SharedPtr
    joints) {
  wheels_ = *joints;
}

void RobotDriver::updateLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr
    scan) {
  sensor_msgs::msg::LaserScan msg = *scan;
  msg.header.frame_id = "laser_frame";
  std::reverse(msg.ranges.begin(), msg.ranges.end());
  laser_pub_->publish(msg);
}

void RobotDriver::updateImage(const sensor_msgs::msg::Image::SharedPtr image) {
  image_ = *image;
}

void RobotDriver::updateRecognizedObjectsWebots(
    const webots_ros2_msgs::msg::WbCameraRecognitionObjects::SharedPtr objects) {
  rec_objects_webots_ = *objects;
}

// void RobotTask::updateGPS(const geometry_msgs::msg::PointStamped::SharedPtr gps) {
//   gps_ = *gps;
// }

void RobotDriver::enableDevices() {
  enableCamera();
  enableRecognition();
  enableLidar();
  enableWheel();
  // enableGPS();
}

} // end namespace tiago_webots_ros
