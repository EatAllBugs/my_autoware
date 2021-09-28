/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ASTAR_AVOID_H
#define ASTAR_AVOID_H

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/LaneArray.h>

#include "waypoint_follower/libwaypoint_follower.h"
#include "astar_search/astar_search.h"

class AstarAvoid
{
public:
  typedef enum STATE
  {
    INITIALIZING = -1,
    RELAYING = 0,
    STOPPING = 1,
    PLANNING = 2,
    AVOIDING = 3
  } State;

  AstarAvoid();
  ~AstarAvoid();
  void run();

private:
  // ros
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher safety_waypoints_pub_; //发布安全路径点
  ros::Subscriber costmap_sub_; //订阅代价地图
  ros::Subscriber current_pose_sub_; //当前位姿
  ros::Subscriber current_velocity_sub_;
  ros::Subscriber base_waypoints_sub_;
  ros::Subscriber closest_waypoint_sub_;
  ros::Subscriber obstacle_waypoint_sub_;
  ros::Subscriber state_sub_;
  ros::Rate *rate_;
  tf::TransformListener tf_listener_;

  // params
  int safety_waypoints_size_;   // output waypoint size [-]
  double update_rate_;          // publishing rate [Hz]
  bool enable_avoidance_;           // enable avoidance mode 模式选择　是否避障
  double avoid_waypoints_velocity_; // constant velocity on planned waypoints [m/s]  避障速度
  double avoid_start_velocity_;     // self velocity for staring avoidance behavior [m/s]
  double replan_interval_;          // replan interval for avoidance planning [Hz]
  int search_waypoints_size_;       // range of waypoints for incremental search [-]
  int search_waypoints_delta_;      // skipped waypoints for incremental search [-]
  int closest_search_size_;         // search closest waypoint around your car [-]

  // classes 
  AstarSearch astar_;
  State state_;

  // threads 
  std::thread publish_thread_;
  std::mutex mutex_;

  // variables
  bool terminate_thread_;
  bool found_avoid_path_;  // 有无找到绕障路径
  int closest_waypoint_index_; //最近的waypoint
  int obstacle_waypoint_index_; // 障碍物点的数量
  nav_msgs::OccupancyGrid costmap_; //代价地图
  autoware_msgs::Lane base_waypoints_;
  autoware_msgs::Lane avoid_waypoints_;
  autoware_msgs::Lane safety_waypoints_;
  geometry_msgs::PoseStamped current_pose_local_, current_pose_global_; // 局部位姿、全局位姿？
  geometry_msgs::PoseStamped goal_pose_local_, goal_pose_global_;
  geometry_msgs::TwistStamped current_velocity_; //当前机器自身速度
  tf::Transform local2costmap_;  // local frame (e.g. velodyne) -> costmap origin
  //布尔量 是否初始化
  bool costmap_initialized_;
  bool current_pose_initialized_;
  bool current_velocity_initialized_;
  bool base_waypoints_initialized_;
  bool closest_waypoint_initialized_;

  //回调函数
  void costmapCallback(const nav_msgs::OccupancyGrid& msg); //代价地图
  void currentPoseCallback(const geometry_msgs::PoseStamped& msg); //当前位姿
  void currentVelocityCallback(const geometry_msgs::TwistStamped& msg); //当前速度
  void baseWaypointsCallback(const autoware_msgs::Lane& msg);
  void closestWaypointCallback(const std_msgs::Int32& msg);
  void obstacleWaypointCallback(const std_msgs::Int32& msg);

  //检查是否初始化
  bool checkInitialized();
  bool planAvoidWaypoints(int& end_of_avoid_index); //发布绕障路径点
  void mergeAvoidWaypoints(const nav_msgs::Path& path, int& end_of_avoid_index); //整合
  void publishWaypoints(); //发布路径
  tf::Transform getTransform(const std::string& from, const std::string& to); //TF变换
  int getLocalClosestWaypoint(const autoware_msgs::Lane& waypoints, const geometry_msgs::Pose& pose, const int& search_size);
};

#endif
