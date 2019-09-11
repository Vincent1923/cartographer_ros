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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H

#include <string>
#include <tuple>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer_ros/trajectory_options.h"

namespace cartographer_ros {

// Top-level options of Cartographer's ROS integration.
// Cartographer ROS集成的最上层选项
struct NodeOptions {
  // map_builder_options 的类型为 MapBuilderOptions，这是一个 ProtocolBuffer 消息类型，用于做串行化的数据结构信息，
  // 消息类型定义在“cartographer/cartographer/mapping/proto/map_builder_options.proto”文件中。
  // map_builder_options 主要是配置使用2d还是3d构图，以及配置有关 PoseGraph，即 Global SLAM的参数。
  ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
  std::string map_frame;                 // 地图坐标系名字
  double lookup_transform_timeout_sec;   // 使用tf2进行转换搜素的超时时间的秒数
  double submap_publish_period_sec;      // 发布submap位置的间隔秒数，如0.3s
  double pose_publish_period_sec;        // 发布位置的间隔秒数，如5e-3对应200Hz
  double trajectory_publish_period_sec;  // 发布轨迹标记的间隔秒数，如30e-3对应30ms
};

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H
