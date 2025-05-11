// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace smart_gap_follow
{
class SmartGapFollowNode : public rclcpp::Node
{
public:
  explicit SmartGapFollowNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    std::vector<int64_t> bev_origin{};
    double car_width{};
    double car_length{};
    double front_overhang{};
    double rear_overhang{};
    double wheel_base{};
    double steering_speed{};
    double scan_down_sample_scale{};
    double range_thresh_min{};
    double range_thresh_max{};
    double min_gap_size{};
    double goal_angle{};
    double wall_clearance_min{};
    double wall_clearance_max{};
    double wall_clearance_dist_thresh{};
    double min_speed{};
    double slow_speed{};
    double max_speed{};
    double max_speed_angle_thresh{};
    double boost_dist_thresh{};
    double boost_speed{};
    double acceleration{};
    double deceleration{};
    double small_angle_kp{};
    double large_angle_kp{};
  };

private:
  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_scan_{};

  // Callback
  void onLidarScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  // Data Buffer
  sensor_msgs::msg::LaserScan::ConstSharedPtr lidar_scan_{};

  // Publisher
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_sim_throttle_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_sim_steering_;

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  void findGap(const sensor_msgs::msg::LaserScan & scan);
  void findTargetGap(const sensor_msgs::msg::LaserScan & scan);
  void normalizeGapAngle(float & gap_angle);
  void calcMotionCmd(const sensor_msgs::msg::LaserScan & scan);
  void calcSpeedCmd(float target_speed, float acceleration, float deceleration);
  void downSampleLidarScan();

  // Parameter
  NodeParam node_param_{};

  sensor_msgs::msg::LaserScan lidar_scan;
  double range_thresh;
  std::vector<std::pair<int, int>> gap_indices;
  bool deadend;
  std::pair<int, int> target_gap_indices;
  std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> target_gap;
  rclcpp::Time velocitytimestamp;
  rclcpp::Time plannertimestamp;
  float target_speed;
  float target_speed_out;
  int gap_size_decrease_cnt;
  float last_gap_size;
};

}  // namespace classic_grass_detection
