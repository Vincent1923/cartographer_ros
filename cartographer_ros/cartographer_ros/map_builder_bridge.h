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

  /**
   * @brief MapBuilderBridge  构造函数。
   *                          对象 map_builder_bridge_ 主要完成 ROS 系统与 Cartographer 内核之间的信息交换。
   * @param node_options      从配置文件中加载的配置项
   * @param map_builder       Cartographer 的地图构建器
   * @param tf_buffer         ROS 系统中坐标变换库 tf2 的监听缓存
   */
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

  /**
   * @brief HandleSubmapQuery  处理submap查询的，在cartographer_node中被kSubmapQueryServiceName这个Service调用
   * @return
   */
  void HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);

  std::set<int> GetFrozenTrajectoryIds();
  /**
   * @brief GetSubmapList  获取Submap的列表，在往kSubmapListTopic这个Topic上发布数据时，被Node::PublishSubmapList调用的
   * @return               Submap的列表
   */
  cartographer_ros_msgs::SubmapList GetSubmapList();
  /**
   * @brief GetTrajectoryStates  返回一个TrajectoryStates变量组成的unordered_map这个容器
   * @return
   */
  std::unordered_map<int, TrajectoryState> GetTrajectoryStates()
      EXCLUDES(mutex_);
  visualization_msgs::MarkerArray GetTrajectoryNodeList();
  visualization_msgs::MarkerArray GetLandmarkPosesList();  // 获取landmark的pose列表
  visualization_msgs::MarkerArray GetConstraintList();

  /**
   * @brief sensor_bridge  返回 map 容器中 trajectory_id 所对应的 SensorBridge 对象
   * @param trajectory_id  轨迹索引
   * @return               map 容器中 trajectory_id 所对应的 SensorBridge 对象
   */
  SensorBridge* sensor_bridge(int trajectory_id);

 private:
  // 成员函数
  /**
   * @brief OnLocalSlamResult    函数的目的就是记录下轨迹状态
   * @param trajectory_id        轨迹索引
   * @param time                 更新子图的时间
   * @param local_pose           子图的参考位置
   * @param range_data_in_local  参考位置下的扫描数据
   * @param insertion_result     这是一个指针似乎没有用到
   */
  void OnLocalSlamResult(
      const int trajectory_id, const ::cartographer::common::Time time,
      const ::cartographer::transform::Rigid3d local_pose,
      ::cartographer::sensor::RangeData range_data_in_local,
      const std::unique_ptr<const ::cartographer::mapping::
                                TrajectoryBuilderInterface::InsertionResult>
          insertion_result) EXCLUDES(mutex_);

/*************************************** 成员变量 ***************************************/
  cartographer::common::Mutex mutex_;                                             // 互斥信号量
  const NodeOptions node_options_;                                                // ROS 节点 cartographer_node 的配置
  std::unordered_map<int, std::shared_ptr<const TrajectoryState::LocalSlamData>>
      trajectory_state_data_ GUARDED_BY(mutex_);                                  // 轨迹状态数据容器
  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;       // Cartographer 的地图构建器
  tf2_ros::Buffer* const tf_buffer_;                                              // ROS 系统坐标变换缓存

  // 跟 landmark 相关，其中 std::string 变量表征 landmark 的 ID
  std::unordered_map<std::string /* landmark ID */, int> landmark_to_index_;  // 路标名称与索引之间的映射

  // These are keyed with 'trajectory_id'.
  // 以下的变量都以 "trajectory_id" 为键，即变量都跟 "trajectory_id" 关联。
  std::unordered_map<int, TrajectoryOptions> trajectory_options_;          // 轨迹跟踪器的配置容器
  // sensor_bridges_ 为 SensorBridge 成员的一个 unordered map，
  // 在函数 MapBuilderBridge::AddTrajectory 中进行初始化。
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;  // 传感器数据转换器容器
  std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;        // 轨迹与路标之间的对应关系
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
