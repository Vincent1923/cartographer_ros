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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/SensorTopics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"
#include "cartographer_ros_msgs/WriteState.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"

namespace cartographer_ros {

// Wires up ROS topics to SLAM.
class Node {
 public:
  /**
   * @brief Node          构造函数
   * @param node_options  node 对象的各种配置参数，在 "node_main.cc" 中由函数 Run 中从配置文件中获取
   * @param map_builder   Cartographer 用于建图的对象
   * @param tf_buffer     ROS 系统中常用的坐标变换库 tf2 的缓存对象
   */
  Node(const NodeOptions& node_options,
       std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
       tf2_ros::Buffer* tf_buffer);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  // Finishes all yet active trajectories.
  void FinishAllTrajectories();
  // Finishes a single given trajectory. Returns false if the trajectory did not
  // exist or was already finished.
  bool FinishTrajectory(int trajectory_id);

  // Runs final optimization. All trajectories have to be finished when calling.
  void RunFinalOptimization();

  // Starts the first trajectory with the default topics.
  // 使用默认的主题开始第一条轨迹。
  /**
   * @brief StartTrajectoryWithDefaultTopics  使用系统默认的订阅主题来开始轨迹跟踪，
   *                                          这里默认的订阅主题名称集合通过函数 DefaultSensorTopics() 获取。
   * @param options                           轨迹的配置参数，在 "node_main.cc" 中由函数 Run 从配置文件中获取
   */
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

  // Returns unique SensorIds for multiple input bag files based on
  // their TrajectoryOptions.
  // 'SensorId::id' is the expected ROS topic name.
  std::vector<
      std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
  ComputeDefaultSensorIdsForMultipleBags(
      const std::vector<TrajectoryOptions>& bags_options) const;

  // Adds a trajectory for offline processing, i.e. not listening to topics.
  int AddOfflineTrajectory(
      const std::set<
          cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& options);

  // The following functions handle adding sensor data to a trajectory.
  void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                             const nav_msgs::Odometry::ConstPtr& msg);
  void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::NavSatFix::ConstPtr& msg);
  void HandleLandmarkMessage(
      int trajectory_id, const std::string& sensor_id,
      const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);
  void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                        const sensor_msgs::Imu::ConstPtr& msg);
  /**
   * @brief HandleLaserScanMessage  订阅sensor_msgs::LaserScan的消息处理函数
   * @param trajectory_id           trajectory的id
   * @param sensor_id               sensor_msgs::LaserScan消息的topic名字
   * @param msg                     sensor_msgs::LaserScan消息
   * @return
   */
  void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      int trajectory_id, const std::string& sensor_id,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::PointCloud2::ConstPtr& msg);

  // Serializes the complete Node state.
  void SerializeState(const std::string& filename);

  // Loads a serialized SLAM state from a .pbstream file.
  void LoadState(const std::string& state_filename, bool load_frozen_state);

  ::ros::NodeHandle* node_handle();

 private:
  struct Subscriber {
    ::ros::Subscriber subscriber;

    // ::ros::Subscriber::getTopic() does not necessarily return the same
    // std::string
    // it was given in its constructor. Since we rely on the topic name as the
    // unique identifier of a subscriber, we remember it ourselves.
    std::string topic;
  };

  /**
   * @brief HandleSubmapQuery  Service kSubmapQueryServiceName 绑定的函数句柄，
   *                           主要工作是根据请求的 trajectory_id 和 submap_index，查询对应的 Submap。
   * @return true              查询 Submap 成功
   *         false             查询 Submap 失败
   */
  bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);

  /**
   * @brief HandleStartTrajectory  Service kStartTrajectoryServiceName绑定的函数句柄，
   *                               前面一些异常情况的处理，正常情况下调用AddTrajectory函数，增加一条trajectory
   * @return true                  增加一条trajectory成功
   *         false                 增加一条trajectory失败
   */
  bool HandleStartTrajectory(
      cartographer_ros_msgs::StartTrajectory::Request& request,
      cartographer_ros_msgs::StartTrajectory::Response& response);

  /**
   * @brief HandleFinishTrajectory  Service kFinishTrajectoryServiceName绑定的函数句柄，
   *                                根据请求的trajectory_id，结束该trajectory
   * @return true                   结束一条trajectory成功
   *         false                  结束一条trajectory失败
   */
  bool HandleFinishTrajectory(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);

  /**
   * @brief HandleWriteState  Service kWriteStateServiceName绑定的函数句柄，
   *                          写状态，根据请求的request.filename文件名，把构建的地图数据保存为后缀名为“.pbstream”的文件
   * @return true             保存地图数据文件成功
   *         false            保存地图数据文件失败
   */
  bool HandleWriteState(cartographer_ros_msgs::WriteState::Request& request,
                        cartographer_ros_msgs::WriteState::Response& response);
  /**
   * @brief ComputeExpectedSensorIds  返回一条trajectory所期望的SensorIds集合，‘SensorId::id’是期望的传感器的ROS topic名称
   * @param options                   跟trajectory相关的参数配置，如tracking_frame，published_frame等等
   * @param topics                    输入的传感器数据的topic名称
   * @return                          一条trajectory所期望的SensorIds集合
   *                                  SensorId会把SensorType（传感器类型）和传感器数据的topic名称（类型为std::string）绑定在一起
   */
  // Returns the set of SensorIds expected for a trajectory.
  // 'SensorId::id' is the expected ROS topic name.
  // 返回一条trajectory所期望的SensorIds集合，‘SensorId::id’是期望的ROS topic名称
  std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
  ComputeExpectedSensorIds(
      const TrajectoryOptions& options,
      const cartographer_ros_msgs::SensorTopics& topics) const;
  /**
   * @brief AddTrajectory  开始一条新的轨迹跟踪，并返回新建轨迹的索引
   * @param options        轨迹的配置参数，在 "node_main.cc" 中由函数 Run 从配置文件中获取
   * @param topics         Cartographer ROS 默认的订阅主题名称集合
   * @return               新建轨迹的索引
   */
  int AddTrajectory(const TrajectoryOptions& options,
                    const cartographer_ros_msgs::SensorTopics& topics);
  /**
   * @brief LaunchSubscribers  根据配置订阅需要的主题
   * @param options            轨迹的配置参数，在 "node_main.cc" 中由函数 Run 从配置文件中获取
   * @param topics             Cartographer ROS 默认的订阅主题名称集合
   * @param trajectory_id      轨迹的索引
   */
  void LaunchSubscribers(const TrajectoryOptions& options,
                         const cartographer_ros_msgs::SensorTopics& topics,
                         int trajectory_id);
  /**
   * @brief PublishSubmapList  在Topic kSubmapListTopic上发布::cartographer_ros_msgs::SubmapList类型的消息
   * @param timer_event
   * @return
   */
  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);
  void PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event);
  /**
   * @brief PublishTrajectoryNodeList  在Topic kTrajectoryNodeListTopic上发布::visualization_msgs::MarkerArray类型的消息
   * @param timer_event
   * @return
   */
  void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
  /**
   * @brief PublishLandmarkPosesList  在Topic kLandmarkPosesListTopic上发布::visualization_msgs::MarkerArray类型的消息
   * @param timer_event
   * @return
   */
  void PublishLandmarkPosesList(const ::ros::WallTimerEvent& timer_event);
  /**
   * @brief PublishConstraintList  在Topic kConstraintListTopic上发布::visualization_msgs::MarkerArray类型的消息
   * @param timer_event
   * @return
   */
  void PublishConstraintList(const ::ros::WallTimerEvent& timer_event);
  void SpinOccupancyGridThreadForever();
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const ::cartographer_ros_msgs::SensorTopics& topics,
                          const TrajectoryOptions& options);
  /**
   * @brief FinishTrajectoryUnderLock               根据trajectory_id，结束该trajectory
   * @return cartographer_ros_msgs::StatusResponse  结束一条指定id的trajectory后的反馈结果
   */
  cartographer_ros_msgs::StatusResponse FinishTrajectoryUnderLock(
      int trajectory_id) REQUIRES(mutex_);

/*************************************** 成员变量 ***************************************/
  const NodeOptions node_options_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  cartographer::common::Mutex mutex_;
  // 变量带保护锁声明，GUARDED_BY(mutex_) 表示 map_builder_bridge_ 变量被 mutex_ 保护，
  // 在访问变量 map_builder_bridge_ 时会检查是否占有 mutex_。
  // guarded_by 属性是为了保证线程安全，使用该属性后，线程要使用相应变量，必须先锁定 mutex_。
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

  // Publishers，ServiceServers
  ::ros::NodeHandle node_handle_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::Publisher trajectory_node_list_publisher_;
  ::ros::Publisher landmark_poses_list_publisher_;
  ::ros::Publisher constraint_list_publisher_;
  // These ros::ServiceServers need to live for the lifetime of the node.
  std::vector<::ros::ServiceServer> service_servers_;
  ::ros::Publisher scan_matched_point_cloud_publisher_;

  struct TrajectorySensorSamplers {
    TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                             const double odometry_sampling_ratio,
                             const double fixed_frame_pose_sampling_ratio,
                             const double imu_sampling_ratio,
                             const double landmark_sampling_ratio)
        : rangefinder_sampler(rangefinder_sampling_ratio),
          odometry_sampler(odometry_sampling_ratio),
          fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
          imu_sampler(imu_sampling_ratio),
          landmark_sampler(landmark_sampling_ratio) {}

    ::cartographer::common::FixedRatioSampler rangefinder_sampler;
    ::cartographer::common::FixedRatioSampler odometry_sampler;
    ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
    ::cartographer::common::FixedRatioSampler imu_sampler;
    ::cartographer::common::FixedRatioSampler landmark_sampler;
  };

  // These are keyed with 'trajectory_id'.
  // 以下的变量都以“trajectory_id”为键，即变量都跟“trajectory_id”关联
  std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;  // 对 IMU、odom 数据进行融合，估计机器人的实时位姿
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
  std::unordered_map<int, std::vector<Subscriber>> subscribers_;            // 路径 trajectory_id 下的 Subscriber
  std::unordered_set<std::string> subscribed_topics_;                       // 订阅的传感器 topic 名称的集合
  std::unordered_map<int, bool> is_active_trajectory_ GUARDED_BY(mutex_);   // 路径 trajectory_id 是否为激活状态

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
