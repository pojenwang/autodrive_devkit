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

#include "smart_gap_follow/smart_gap_follow_node.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <climits>

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}

}  // namespace

namespace smart_gap_follow
{

SmartGapFollowNode::SmartGapFollowNode(const rclcpp::NodeOptions & node_options)
: Node("smart_gap_follow", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&SmartGapFollowNode::onSetParam, this, std::placeholders::_1));

  // Node Parameter
  node_param_.car_width = declare_parameter<double>("car_width");
  node_param_.car_length = declare_parameter<double>("car_length");
  node_param_.front_overhang = declare_parameter<double>("front_overhang");
  node_param_.rear_overhang = declare_parameter<double>("rear_overhang");
  node_param_.wheel_base = declare_parameter<double>("wheel_base");
  node_param_.steering_speed = declare_parameter<double>("steering_speed");
  node_param_.scan_down_sample_scale = declare_parameter<double>("scan_down_sample_scale");
  node_param_.range_thresh_min = declare_parameter<double>("range_thresh_min");
  node_param_.range_thresh_max = declare_parameter<double>("range_thresh_max");
  node_param_.min_gap_size = declare_parameter<double>("min_gap_size");
  node_param_.goal_angle = declare_parameter<double>("goal_angle");
  node_param_.wall_clearance_min = declare_parameter<double>("wall_clearance_min");
  node_param_.wall_clearance_max = declare_parameter<double>("wall_clearance_max");
  node_param_.wall_clearance_dist_thresh = declare_parameter<double>("wall_clearance_dist_thresh");
  node_param_.min_speed = declare_parameter<double>("min_speed");
  node_param_.slow_speed = declare_parameter<double>("slow_speed");
  node_param_.max_speed = declare_parameter<double>("max_speed");
  node_param_.boost_speed = declare_parameter<double>("boost_speed");
  node_param_.max_speed_angle_thresh = declare_parameter<double>("max_speed_angle_thresh");
  node_param_.boost_dist_thresh = declare_parameter<double>("boost_dist_thresh");
  node_param_.acceleration = declare_parameter<double>("acceleration");
  node_param_.deceleration = declare_parameter<double>("deceleration");
  node_param_.small_angle_kp = declare_parameter<double>("small_angle_kp");
  node_param_.large_angle_kp = declare_parameter<double>("large_angle_kp");

  // Subscriber
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  sub_lidar_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "~/input/lidar_scan", qos, std::bind(&SmartGapFollowNode::onLidarScan, this, std::placeholders::_1));

  // Publisher
  //pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("~/out/drive", 10);
  pub_sim_throttle_ = this->create_publisher<std_msgs::msg::Float32>("~/out/sim_throttle", 10);
  pub_sim_steering_ = this->create_publisher<std_msgs::msg::Float32>("~/out/sim_steering", 10);
}

void SmartGapFollowNode::findGap(const sensor_msgs::msg::LaserScan & scan)
{
  const float min_gap_size = node_param_.min_gap_size;
  const float range_thresh_max = node_param_.range_thresh_max;
  const float range_thresh_min = node_param_.range_thresh_min;
  const float max_speed = node_param_.max_speed;
  const int scan_size = scan.ranges.size();
  const float angle_increment = scan.angle_increment;
  const float angle_min = scan.angle_min;
  const float angle_max = scan.angle_max;
  std::vector<float> scan_ranges = scan.ranges;
  std::vector<int> scan_ids(scan_size, 0);
  
  scan_ranges[0] = std::min(scan_ranges[0], 2 * min_gap_size);
  scan_ranges[scan_size - 1] = std::min(scan_ranges[scan_size - 1], 2 * min_gap_size);

  float range_scale = (target_speed_out / max_speed) > 1 ? 1 : target_speed / max_speed;
  range_thresh = range_thresh_max;// + (range_thresh_max - range_thresh_min) * range_scale;

  int new_id = 1;
  for(int i = 0; i < (int)scan_ranges.size(); i++){
    if(scan_ranges[i] > range_thresh)
        continue;

    if(scan_ids[i] == 0){
        scan_ids[i] = new_id;
        new_id++;
    }

    const float check_limit = min_gap_size < scan_ranges[i] ? 
          std::min(float(angle_min + i * angle_increment + asin(min_gap_size / scan_ranges[i])), angle_max) : angle_max;

    for(int j = i + 1; angle_min + j * angle_increment <= check_limit; j++){  
      if(scan_ranges[j] > range_thresh || scan_ids[i] == scan_ids[j])
        continue;
        
      const float dist = sqrt(pow(scan_ranges[i], 2) + pow(scan_ranges[j], 2) -  
                  2 * scan_ranges[i] * scan_ranges[j] * cos(abs(i - j) * angle_increment));
      //RCLCPP_INFO_STREAM(get_logger(), "dist:"<<dist);
      if(dist <= min_gap_size){
        if(scan_ids[j] == 0)
            scan_ids[j] = scan_ids[i];
        else{
          //cout<<"\nmerge:i:"<<i<<", j:"<<j;
          for(int k = 0; k <= j; k++){
            if(scan_ids[k] == scan_ids[i])
              scan_ids[k] = scan_ids[j]; //merge to the larger number cluster label
          }
        }
      }
    }
  }
  
  //reassign group numers in increasing order such that it no longer skip numbers after merging groups
  new_id = 1;
  int max_id = *max_element(scan_ids.begin(), scan_ids.end());
  for(int i = 1; i <= max_id; i++){
    bool id_found = false;
    for(int j = 0; j < (int)scan_ids.size(); j++){
      if(scan_ids[j] == i){
        scan_ids[j] = new_id;
        id_found = true;
      }
    }
    if(id_found)
      new_id++;
  }
  max_id = *max_element(scan_ids.begin(), scan_ids.end());
  
  //RCLCPP_INFO_STREAM(get_logger(), "num_group:"<<max_id);
  //for(int i = 0; i < (int)scan_ranges.size(); i++)
  //  RCLCPP_INFO_STREAM(get_logger(), i<<":["<<scan_ranges[i]<<","<<scan_ids[i]<<"]");
  
  std::unordered_map<int, std::pair<int, int>> group_map;
  std::unordered_map<int, float> group_min_dist_map;
  for(int i = 0; i < (int)scan_ranges.size(); i++){
    if(scan_ids[i] == 0)
      continue;
    if (group_map.find(scan_ids[i]) == group_map.end()){
      group_map[scan_ids[i]] = {i, i};
      group_min_dist_map[scan_ids[i]] = scan_ranges[i];
    }else{
      int curr_min_idx = group_map[scan_ids[i]].first;
      int curr_max_idx = group_map[scan_ids[i]].second;
      group_map[scan_ids[i]] = {std::min(curr_min_idx, i), std::max(curr_max_idx, i)};
      group_min_dist_map[scan_ids[i]] = std::min(group_min_dist_map[scan_ids[i]], scan_ranges[i]);
    }
  }
  
  /*
  cout<<"group_min_dist_map";
  for(auto i: group_min_dist_map){
    cout<<"\nid:"<<i.first<<" min_dist:"<< i.second;
  }
  for(auto i: group_map)
    cout<<"\ngroup id before:"<<i.first<<"["<<i.second.first<<","<<i.second.second<<"]";
    */
  
  std::vector<int> ids_to_remove;
  //remove the rear obstacle groups when two groups overlap each other
  for(int i = 1; i <= max_id; i++){
      for(int j = 1; j <= max_id; j++){
          if(i != j && std::max(group_map[i].first, group_map[j].first)
             <= std::min(group_map[i].second, group_map[j].second)){
              //cout<<"overlap!!!"<<i<<","<<j<<"\n";
              if(group_min_dist_map[i] < group_min_dist_map[j] && group_min_dist_map[i] != 0){
                  ids_to_remove.push_back(j);
                  group_map.erase(j);
                  //cout<<"erase id:"<<j;
              }
              if(group_min_dist_map[j] < group_min_dist_map[i] && group_min_dist_map[j] != 0){
                  ids_to_remove.push_back(i);
                  group_map.erase(i);
                  //cout<<"erase id:"<<i;
              }
          }
      }
  }
  //for(auto i:ids_to_remove)
  //  RCLCPP_INFO_STREAM(get_logger(), "remove:"<<i<<",");
  //for(auto i: group_map)
    //cout<<"\ngroup id after:"<<i.first<<"["<<i.second.first<<","<<i.second.second<<"]";

  std::vector<std::pair<int, int>> groups;
  for(auto it:group_map)
    if(find(ids_to_remove.begin(), ids_to_remove.end(), it.first) == ids_to_remove.end())
      groups.push_back(it.second);
  reverse(groups.begin(), groups.end());
  
  deadend = (groups[0].first == 0 && groups[0].second == 180);

  //cout<<"\ngroup:";
  //for(auto it: groups)
  //  cout<<"["<<it.first<<","<<it.second<<"]";
  
  gap_indices.clear();
  for(int i = 0; i < (int)groups.size()-1; i++)
    gap_indices.push_back({groups[i].second, groups[i+1].first});
  
  if(groups.empty())
    gap_indices.push_back({60, 120});

  //RCLCPP_INFO_STREAM(get_logger(), "gaps-----------------------------------");
  //for(auto it:gap_indices)
  //  RCLCPP_INFO_STREAM(get_logger(), "["<<it.first<<","<<it.second<<"]");
  
} 

void SmartGapFollowNode::findTargetGap(const sensor_msgs::msg::LaserScan & scan)
{
  const float angle_increment = scan.angle_increment;
  const float angle_min = scan.angle_min;
  //const float angle_max = scan.angle_max;
  const float goal_angle = node_param_.goal_angle;
  float min_angle_diff = std::numeric_limits<float>::max();
  for(auto it:gap_indices){
    float angle_diff_first = abs(angle_min + it.first * angle_increment - goal_angle);
    float angle_diff_second = abs(angle_min + it.second * angle_increment - goal_angle);
    if(std::min(angle_diff_first, angle_diff_second) < min_angle_diff){
      min_angle_diff = std::min(angle_diff_first, angle_diff_second);
      target_gap_indices = it;
    }
  }
}

void SmartGapFollowNode::normalizeGapAngle(float & gap_angle) {
  gap_angle = std::fmod(gap_angle + 3 * M_PI / 2, 2 * M_PI);
  if (gap_angle < 0)
      gap_angle += 2 * M_PI;
  gap_angle -= M_PI;
}

void SmartGapFollowNode::calcMotionCmd(const sensor_msgs::msg::LaserScan & scan)
{
  const float min_speed = node_param_.min_speed;
  const float slow_speed = node_param_.slow_speed;
  float max_speed = node_param_.max_speed;
  const float boost_speed = node_param_.boost_speed;
  const float max_speed_angle_thresh = node_param_.max_speed_angle_thresh;
  const float boost_dist_thresh = node_param_.boost_dist_thresh;
  float acceleration = node_param_.acceleration;
  float deceleration = node_param_.deceleration;
  const float small_angle_kp = node_param_.small_angle_kp;
  const float large_angle_kp = node_param_.large_angle_kp;
  const float angle_increment = scan.angle_increment;
  const float angle_min = scan.angle_min;
  const float wall_clearance_min = node_param_.wall_clearance_min;
  const float wall_clearance_max = node_param_.wall_clearance_max;
  const float wall_clearance_dist_thresh = node_param_.wall_clearance_dist_thresh;
  const float car_width = node_param_.car_width;

  float gap_angle_min = angle_min + target_gap_indices.first * angle_increment;
  float gap_angle_max = angle_min + target_gap_indices.second * angle_increment;
  float dist_to_gap = std::min(scan.ranges[target_gap_indices.first], scan.ranges[target_gap_indices.second]);
  float gap_angle_min_x = scan.ranges[target_gap_indices.first] * sin(gap_angle_min + M_PI / 2);
  float gap_angle_min_y = -scan.ranges[target_gap_indices.first] * cos(gap_angle_min + M_PI / 2);
  float gap_angle_max_x = scan.ranges[target_gap_indices.second] * sin(gap_angle_max + M_PI / 2);
  float gap_angle_max_y = -scan.ranges[target_gap_indices.second] * cos(gap_angle_max + M_PI / 2);
  float gap_size = sqrt(pow(gap_angle_min_x - gap_angle_max_x, 2) + pow(gap_angle_min_y - gap_angle_max_y,2));
  float dy = gap_angle_min_y - gap_angle_max_y;
  float dx = gap_angle_min_x - gap_angle_max_x;
  float gap_angle = scan.ranges[target_gap_indices.first] > scan.ranges[target_gap_indices.second] 
    ? std::atan2(dx, -dy)
    : std::atan2(-dx, dy);
  normalizeGapAngle(gap_angle);
  
  float wall_clearance = (dist_to_gap < wall_clearance_dist_thresh)
    ? wall_clearance_min
    : wall_clearance_max;
  
  // add buffer to accommodate vehicle size
  float steer_angle_min = gap_angle_min + atan((car_width / 2 + wall_clearance) / scan.ranges[target_gap_indices.first]);
  float steer_angle_max = gap_angle_max - atan((car_width / 2 + wall_clearance) / scan.ranges[target_gap_indices.second]);
  
  // if the min and max steer angles crosses over because of the added buffer, set target steer_angle to the closer obstacle
  // edge, otherwise, set steer angle equle the average
  float steer_angle;
  if(steer_angle_min > steer_angle_max){
    steer_angle = scan.ranges[target_gap_indices.first] > scan.ranges[target_gap_indices.second] ? 
      steer_angle_max : steer_angle_min;
  }
  else
    steer_angle = (steer_angle_min + steer_angle_max) / 2;

  RCLCPP_INFO_STREAM(get_logger(), "steer_angle_min:"<<steer_angle_min * 180 / M_PI);
  RCLCPP_INFO_STREAM(get_logger(), "steer_angle_max:"<<steer_angle_max * 180 / M_PI); 

  const float small_steer_angle_thresh = 20 * M_PI / 180;
  steer_angle *= abs(steer_angle) < small_steer_angle_thresh ? small_angle_kp : large_angle_kp;
  
  const float steer_angle_limit = 60 * M_PI / 180;
  steer_angle = std::min(steer_angle, steer_angle_limit);
  steer_angle = std::max(steer_angle, -steer_angle_limit);

  if(dist_to_gap < 2.45 && gap_size < 3.8){
    max_speed = slow_speed;
    RCLCPP_INFO_STREAM(get_logger(), "slow!!!");
  }

  // target speed based on steer angle
  float target_speed_steer;
  float threshold_angle = max_speed_angle_thresh * M_PI / 180;
  if (abs(steer_angle) <= threshold_angle) {
      target_speed_steer = max_speed;
  }else if(abs(steer_angle) >= steer_angle_limit) {
      target_speed_steer = min_speed;
  }else{
      float scale = (abs(steer_angle) - threshold_angle) / (steer_angle_limit - threshold_angle);
      target_speed_steer = max_speed - (max_speed - min_speed) * scale;
  }

  // target speed based on gap orientation
  float target_speed_gap = min_speed + (max_speed - min_speed) * pow(cos(abs(gap_angle)), 2.0);

  std_msgs::msg::Float32 steer_angle_msg;
  steer_angle_msg.data = steer_angle;
  pub_sim_steering_ ->publish(steer_angle_msg);

  target_speed = 0.5 * target_speed_gap + 0.5 * target_speed_steer;

  if(dist_to_gap < 1.1 || (dist_to_gap < 1.4 && abs(steer_angle) * 180 / M_PI > 3)){
    target_speed = 1.45;
    RCLCPP_INFO_STREAM(get_logger(), "more slow!!!");
  }
  if(dist_to_gap > boost_dist_thresh && abs(steer_angle) * 180 / M_PI < 5){
    target_speed = boost_speed;
    acceleration *= 1.5;
    RCLCPP_INFO_STREAM(get_logger(), "boost!!!");
  }

  calcSpeedCmd(target_speed, acceleration, deceleration);
  std_msgs::msg::Float32 throttle_msg;
  throttle_msg.data = target_speed_out * 0.05;
  pub_sim_throttle_ ->publish(throttle_msg);
  
  RCLCPP_INFO_STREAM(get_logger(), "gap_size: "<<gap_size<< " m");
  RCLCPP_INFO_STREAM(get_logger(), "dist_to_gap: "<<dist_to_gap<< " m");
  RCLCPP_INFO_STREAM(get_logger(), "wall_clearance: "<<wall_clearance<< " m");
  RCLCPP_INFO_STREAM(get_logger(), "gap angle:"<<gap_angle * 180 / M_PI);
  RCLCPP_INFO_STREAM(get_logger(), "target_speed_steer: "<<target_speed_steer << " m/s");
  RCLCPP_INFO_STREAM(get_logger(), "target_speed_gap: "<<target_speed_gap << " m/s");
  RCLCPP_INFO_STREAM(get_logger(), "target_speed: "<<target_speed_out << " m/s"); 
  RCLCPP_INFO_STREAM(get_logger(), "steer_angle: "<<steer_angle * 180 / M_PI << " deg\n");
  
  /*
  ackermann_msgs::msg::AckermannDriveStamped gap_follow_drive_msg;
  gap_follow_drive_msg.drive.speed = target_speed;
  gap_follow_drive_msg.drive.steering_angle = steer_angle;
  gap_follow_drive_msg.drive.steering_angle_velocity = 1.0;
  gap_follow_drive_msg.drive.acceleration = 4.0;
  pub_drive_->publish(gap_follow_drive_msg);
  */
}

void SmartGapFollowNode::calcSpeedCmd(float target_speed, float acceleration, float deceleration){
  const auto duration = rclcpp::Clock().now() - timestamp;
  const float time_lapse = duration.seconds();
  RCLCPP_INFO_STREAM(get_logger(), "time_lapse: "<<time_lapse << " sec");
  if(time_lapse < 0.02)
    return; 
  timestamp = rclcpp::Clock().now();
  
  double speed_diff = target_speed - target_speed_out;
  double max_delta = time_lapse * acceleration;
  
  if(speed_diff > 0){
    double max_delta = time_lapse * acceleration;
    target_speed_out += (speed_diff > max_delta) ? max_delta : speed_diff;
  }else{
    double max_delta = time_lapse * deceleration;
    target_speed_out += (speed_diff > max_delta) ? max_delta : speed_diff;
  }
}

void SmartGapFollowNode::downSampleLidarScan()
{
  const float scan_down_sample_scale = node_param_.scan_down_sample_scale;
  lidar_scan.angle_increment = lidar_scan.angle_increment * scan_down_sample_scale;
  sensor_msgs::msg::LaserScan lidar_scan_copy = lidar_scan;
  lidar_scan.ranges = {};
  for (int i = 0; i < lidar_scan_copy.ranges.size(); i += scan_down_sample_scale)
    lidar_scan.ranges.push_back(lidar_scan_copy.ranges[i]);
}

void SmartGapFollowNode::onLidarScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr lidar_scan_)
{
  lidar_scan.angle_increment = lidar_scan_->angle_increment;
  lidar_scan.angle_min = lidar_scan_->angle_min;
  lidar_scan.angle_max = lidar_scan_->angle_max;
  lidar_scan.range_min = lidar_scan_->range_min;
  lidar_scan.range_max = lidar_scan_->range_max;
  lidar_scan.ranges = lidar_scan_->ranges;

  findGap(lidar_scan);
  findTargetGap(lidar_scan);
  calcMotionCmd(lidar_scan);
}

rcl_interfaces::msg::SetParametersResult SmartGapFollowNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      //update_param(params, "px_dist", p.px_dist);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}
}  // namespace 

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smart_gap_follow::SmartGapFollowNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
