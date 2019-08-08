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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "cartographer/common/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

class MapBuilderBridge {
 public:
  /*
   * 一个结构体TrajectoryState，该结构体存储的是local SLAM处理后的结果。
   * 由range_data_in_local中已经估计出了在时刻time时的当前local_pose。
   */
  struct TrajectoryState {
    // Contains the trajectory state data received from local SLAM, after
    // it had processed accumulated 'range_data_in_local' and estimated
    // current 'local_pose' at 'time'.
    // 包含从local SLAM接收的轨迹状态数据，在处理累积的‘range_data_in_local’并且在‘time’估计当前‘local_pose’之后
    struct LocalSlamData {                                    // 结构体LocalSlamData
      ::cartographer::common::Time time;                      // 时间
      ::cartographer::transform::Rigid3d local_pose;          // 优化匹配出来的local_pose——在submap这个局部坐标系中的位姿
      ::cartographer::sensor::RangeData range_data_in_local;  // 激光数据
    };
    std::shared_ptr<const LocalSlamData> local_slam_data;     // local SLAM的数据
    cartographer::transform::Rigid3d local_to_map;            // submap到global map的坐标变换关系
    // 猜测是要输入PoseExtrapolator中与IMU、里程计等数据融合来估计位姿的
    // （PoseExtrapolator详见“Cartographer源码阅读1——整体框架介绍”中的第一个图）
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
    TrajectoryOptions trajectory_options;                     // 配置参数
  };

  // 构造函数
  MapBuilderBridge(
      const NodeOptions& node_options,
      std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
      tf2_ros::Buffer* tf_buffer);

  MapBuilderBridge(const MapBuilderBridge&) = delete;
  MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;  //重载了赋值操作

  /**
   * @brief LoadState          调用了map_builder_的成员函数LoadState来加载一个.pbstream文件
   * @param state_filename     .pbstream文件的路径
   * @param load_frozen_state  默认值为true
   * @return
   */
  void LoadState(const std::string& state_filename, bool load_frozen_state);
  /**
   * @brief AddTrajectory        添加一条trajectory
   * @param expected_sensor_ids  一条trajectory所期望的SensorIds集合
   * @param trajectory_options   跟trajectory相关的参数配置，如tracking_frame，published_frame等等
   * @return                     增加的trajectory的id
   */
  int AddTrajectory(
      const std::set<
          ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& trajectory_options);
  void FinishTrajectory(int trajectory_id);
  void RunFinalOptimization();
  bool SerializeState(const std::string& filename);

  void HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);

  std::set<int> GetFrozenTrajectoryIds();
  cartographer_ros_msgs::SubmapList GetSubmapList();
  std::unordered_map<int, TrajectoryState> GetTrajectoryStates()
      EXCLUDES(mutex_);
  visualization_msgs::MarkerArray GetTrajectoryNodeList();
  visualization_msgs::MarkerArray GetLandmarkPosesList();  // 获取landmark的pose列表
  visualization_msgs::MarkerArray GetConstraintList();

  SensorBridge* sensor_bridge(int trajectory_id);

 private:
  // 成员函数
  void OnLocalSlamResult(
      const int trajectory_id, const ::cartographer::common::Time time,
      const ::cartographer::transform::Rigid3d local_pose,
      ::cartographer::sensor::RangeData range_data_in_local,
      const std::unique_ptr<const ::cartographer::mapping::
                                TrajectoryBuilderInterface::InsertionResult>
          insertion_result) EXCLUDES(mutex_);

  // 成员变量
  cartographer::common::Mutex mutex_;
  const NodeOptions node_options_;
  // 几个Unordered map的container
  std::unordered_map<int, std::shared_ptr<const TrajectoryState::LocalSlamData>>
      trajectory_state_data_ GUARDED_BY(mutex_);
  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
  tf2_ros::Buffer* const tf_buffer_;

  // 跟landmark相关，其中std::string变量表征landmark的ID
  std::unordered_map<std::string /* landmark ID */, int> landmark_to_index_;

  // These are keyed with 'trajectory_id'.
  std::unordered_map<int, TrajectoryOptions> trajectory_options_;
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;  //元素为SensorBridge成员的一个unordered map
  std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
