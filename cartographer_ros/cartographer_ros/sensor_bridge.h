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
  // 构造函数
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
   * @brief HandleLaserScanMessage  把sensor_msgs::LaserScan类型的数据转化成carto::sensor::PointCloudWithIntensities类型
   * @param sensor_id               sensor_msgs::LaserScan消息的topic名字
   * @param msg                     sensor_msgs::LaserScan数据
   * @return
   */
  void HandleLaserScanMessage(const std::string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  /**
   * @brief HandleMultiEchoLaserScanMessage  把sensor_msgs::MultiEchoLaserScan类型的数据转化成carto::sensor::PointCloudWithIntensities类型
   * @param sensor_id                        sensor_msgs::MultiEchoLaserScan消息的topic名字
   * @param msg                              sensor_msgs::MultiEchoLaserScan数据
   * @return
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
   * @brief HandleLaserScan  把carto::sensor::PointCloudWithIntensities类型的数据转化成carto::sensor::TimedPointCloud类型
   * @param sensor_id        LaserScan消息的topic名字
   * @param start_time
   * @param frame_id         LaserScan消息的frame_id
   * @param points           LaserScan消息转化成carto::sensor::PointCloudWithIntensities类型后的数据
   * @return
   */
  void HandleLaserScan(
      const std::string& sensor_id, ::cartographer::common::Time start_time,
      const std::string& frame_id,
      const ::cartographer::sensor::PointCloudWithIntensities& points);
  /**
   * @brief HandleRangefinder  处理测距仪数据
   * @param sensor_id          测距仪消息的topic名字
   * @param time
   * @param frame_id           测距仪消息的frame_id
   * @param ranges             测距仪数据
   * @return
   */
  void HandleRangefinder(const std::string& sensor_id,
                         ::cartographer::common::Time time,
                         const std::string& frame_id,
                         const ::cartographer::sensor::TimedPointCloud& ranges);

  const int num_subdivisions_per_laser_scan_;
  std::map<std::string, cartographer::common::Time>
      sensor_to_previous_subdivision_time_;
  const TfBridge tf_bridge_;
  ::cartographer::mapping::TrajectoryBuilderInterface* const
      trajectory_builder_;

  ::cartographer::common::optional<::cartographer::transform::Rigid3d>
      ecef_to_local_frame_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
