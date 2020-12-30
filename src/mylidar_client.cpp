#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)
void scancb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;
  std::cout<<"[YDLIDAR INFO]: I heard a laser scan"<<scan->header.frame_id.c_str()<<":"<<count<<'\n';
  std::cout<<"[YDLIDAR INFO]: angle_range : "<<"["<<RAD2DEG(scan->angle_min)<<","<< RAD2DEG(scan->angle_max)<<"]"<<'\n';

  for (int i = 0; i < count; i++) 
  {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    std::cout<<"[YDLIDAR INFO]: angle-distance : "<<"["<< degree <<","<< scan->ranges[i]<<"]"<<'\n';
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scancb);

  rclcpp::spin(node);

  rclcpp::shutdown();


  return 0;
}