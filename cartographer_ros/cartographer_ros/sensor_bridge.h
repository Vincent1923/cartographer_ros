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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H

#include <memory>

#include "cartographer/common/optional.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"

namespace cartographer_ros {

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
// 在MapBuilder的tracking frame中将ROS消息转换为SensorData。
class SensorBridge {
 public:
  /**
   * @brief SensorBridge                     构造函数
   * @param num_subdivisions_per_laser_scan  激光雷达数据分段数量
   * @param tracking_frame                   参考坐标系
   * @param lookup_transform_timeout_sec     tf 坐标变换查询超时设置
   * @param tf_buffer                        tf 坐标变换缓存
   * @param trajectory_builder               轨迹构建器
   */
  explicit SensorBridge(
      int num_subdivisions_per_laser_scan, const std::string& tracking_frame,
      double lookup_transform_timeout_sec, tf2_ros::Buffer* tf_buffer,
      ::cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder);

  SensorBridge(const SensorBridge&) = delete;
  SensorBridge& operator=(const SensorBridge&) = delete;

  /**
   * @brief ToOdometryData  一个预处理的工具函数，返回的是经过变换后的里程计数据，它是相对于tracking_frame坐标系下的
   * @param msg             里程计数据
   * @return                相对于tracking_frame坐标系下的里程计数据
   */
  std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(
      const nav_msgs::Odometry::ConstPtr& msg);
  /**
   * @brief HandleOdometryMessage  处理里程计消息
   * @param sensor_id              nav_msgs::Odometry消息的topic名字
   * @param msg                    里程计数据
   * @return
   */
  void HandleOdometryMessage(const std::string& sensor_id,
                             const nav_msgs::Odometry::ConstPtr& msg);
  void HandleNavSatFixMessage(const std::string& sensor_id,
                              const sensor_msgs::NavSatFix::ConstPtr& msg);
  void HandleLandmarkMessage(
      const std::string& sensor_id,
      const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);

  /**
   * @brief ToImuData  数据预处理函数，处理IMU数据，返回的是经过变换后的IMU数据
   * @param msg        IMU数据
   * @return           相对于tracking_frame坐标系下的IMU数据
   */
  std::unique_ptr<::cartographer::sensor::ImuData> ToImuData(
      const sensor_msgs::Imu::ConstPtr& msg);
  /**
   * @brief HandleImuMessage  处理IMU消息
   * @param sensor_id         sensor_msgs::Imu消息的topic名字
   * @param msg               IMU数据
   * @return
   */
  void HandleImuMessage(const std::string& sensor_id,
                        const sensor_msgs::Imu::ConstPtr& msg);
  /**
   * @brief HandleLaserScanMessage  处理 LaserScan 单线激光扫描消息，消息类型为 sensor_msgs::LaserScan。
   *                                把 ROS 消息(sensor_msgs::LaserScan)转换成 Cartographer 中的
   *                                传感器数据类型(carto::sensor::PointCloudWithIntensities)。
   * @param sensor_id               单线激光扫描消息 laser_scan 的主题，默认名称为 "scan"
   * @param msg                     单线激光扫描的消息
   */
  void HandleLaserScanMessage(const std::string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  /**
   * @brief HandleMultiEchoLaserScanMessage  处理 MultiEchoLaserScan 多线激光扫描消息，消息类型为 sensor_msgs::MultiEchoLaserScan。
   *                                         把 ROS 消息(sensor_msgs::MultiEchoLaserScan)转换成 Cartographer 中的
   *                                         传感器数据类型(carto::sensor::PointCloudWithIntensities)。
   * @param sensor_id                        多线激光扫描消息 multi_echo_laser_scan 的主题，默认名称为 "echoes"
   * @param msg                              多线激光扫描的消息
   */
  void HandleMultiEchoLaserScanMessage(
      const std::string& sensor_id,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  /**
   * @brief HandlePointCloud2Message  把sensor_msgs::PointCloud2类型的数据转化成carto::sensor::TimedPointCloud类型
   * @param sensor_id                 sensor_msgs::PointCloud2消息的topic名字
   * @param msg                       sensor_msgs::PointCloud2数据
   * @return
   */
  void HandlePointCloud2Message(const std::string& sensor_id,
                                const sensor_msgs::PointCloud2::ConstPtr& msg);

  const TfBridge& tf_bridge() const;

 private:
  /**
   * @brief HandleLaserScan  根据 num_subdivisions_per_laser_scan_ 的大小，对一帧点云数据 points 进行划分。
   *                         把 carto::sensor::PointCloudWithIntensities 类型的数据转化成 carto::sensor::TimedPointCloud 类型
   * @param sensor_id        LaserScan 消息的 topic 名字
   * @param start_time       一帧点云数据 points 的最后一个点的时间
   * @param frame_id         LaserScan 消息的 frame_id
   * @param points           LaserScan 消息转化成 carto::sensor::PointCloudWithIntensities 类型后的数据，输入的一帧点云数据
   * @return
   */
  void HandleLaserScan(
      const std::string& sensor_id, ::cartographer::common::Time start_time,
      const std::string& frame_id,
      const ::cartographer::sensor::PointCloudWithIntensities& points);
  /**
   * @brief HandleRangefinder  处理测距仪数据
   * @param sensor_id          测距仪消息的 topic 名字
   * @param time               输入的一帧点云数据 ranges 的最后一个点的时间
   * @param frame_id           测距仪消息的 frame_id
   * @param ranges             输入的一帧点云数据
   * @return
   */
  void HandleRangefinder(const std::string& sensor_id,
                         ::cartographer::common::Time time,
                         const std::string& frame_id,
                         const ::cartographer::sensor::TimedPointCloud& ranges);

/*************************************** 成员变量 ***************************************/
  const int num_subdivisions_per_laser_scan_;                 // 激光传感器的分段数量
  std::map<std::string, cartographer::common::Time>
      sensor_to_previous_subdivision_time_;                   // 记录了各个传感器最新数据的时间
  const TfBridge tf_bridge_;                                  // cartographer_ros 中关于 ROS 坐标变换的封装
  ::cartographer::mapping::TrajectoryBuilderInterface* const
      trajectory_builder_;                                    // Cartographer 的核心对象 map_builder_ 提供的轨迹跟踪器

  ::cartographer::common::optional<::cartographer::transform::Rigid3d>
      ecef_to_local_frame_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
