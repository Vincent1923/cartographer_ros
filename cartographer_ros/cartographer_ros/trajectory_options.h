/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"

namespace cartographer_ros {

// Trajectory 的参数配置
struct TrajectoryOptions {
  // trajectory_builder_options 的类型为 TrajectoryBuilderOptions，这是一个 ProtocolBuffer 消息类型，
  // 消息类型定义在“cartographer/cartographer/mapping/proto/trajectory_builder_options.proto”文件中。
  // trajectory_builder_options 主要是配置有关 Local SLAM 的参数，其中包括2d的 trajectory_builder_2d_options 以及
  // 3d的 trajectory_builder_3d_options 的参数设置，并且还包括是否使用定位的 pure_localization 参数。
  ::cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options;
  std::string tracking_frame;              // The ROS frame ID of the frame that is tracked by the SLAM algorithm
  std::string published_frame;             // The ROS frame ID to use as the child frame for publishing poses
  std::string odom_frame;                  // 仅在“provide_odom_frame”为true时使用，位于“published_frame”和“map_frame”之间，用来发布本地SLAM结果（非闭环），通常是“odom”
  bool provide_odom_frame;                 // 如果启用，这个局部的，非闭环的，连续的“odom_frame”在“map_frame”中的位置将被发布
  bool use_odometry;                       // 如果启用，将在 topic “odom” 上订阅 nav_msgs/Odometry 类型数据
  bool use_nav_sat;                        // 如果启用，将在 topic “fix” 上订阅 sensor_msgs/NavSatFix 类型数据
  bool use_landmarks;                      // 如果启用，将在 topic “landmarks” 上订阅 cartographer_ros_msgs/LandmarkList 类型数据
  bool publish_frame_projected_to_2d;      // 如果启用，则已发布的位姿将限制为纯2D姿势（无滚动，俯仰或z偏移）
  int num_laser_scans;                     // 订阅的 laser scan topics 的数量
  int num_multi_echo_laser_scans;          // 订阅的 multi-echo laser scan topics 的数量
  int num_subdivisions_per_laser_scan;     // 每个接收到的 (multi-echo) laser scan 中分离出来的点云的数量
  int num_point_clouds;                    // 订阅的 point cloud topics 的数量
  double rangefinder_sampling_ratio;       // 测距仪消息的采样率
  double odometry_sampling_ratio;          // 里程计消息的采样率
  double fixed_frame_pose_sampling_ratio;
  double imu_sampling_ratio;               // IMU消息的采样率
  double landmarks_sampling_ratio;         // landmarks 消息的采样率
};

::cartographer::mapping::proto::InitialTrajectoryPose
CreateInitialTrajectoryPose(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary,
    ::cartographer::common::LuaParameterDictionary* initial_trajectory_pose);

/**
 * @brief FromRosMessage  尝试将"msg"转换为"options"，失败时返回false
 * @param msg             用msg的消息类型表示的跟trajectory相关的参数配置
 * @param options         跟trajectory相关的参数配置，如tracking_frame，published_frame等等
 * @return
 */
// Try to convert 'msg' into 'options'. Returns false on failure.
// 尝试将"msg"转换为"options"，失败时返回false。
bool FromRosMessage(const cartographer_ros_msgs::TrajectoryOptions& msg,
                    TrajectoryOptions* options);

// Converts 'trajectory_options' into a ROS message.
cartographer_ros_msgs::TrajectoryOptions ToRosMessage(
    const TrajectoryOptions& trajectory_options);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
