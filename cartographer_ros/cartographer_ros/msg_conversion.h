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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H

#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/PointCloud2.h"

namespace cartographer_ros {

sensor_msgs::PointCloud2 ToPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud);

geometry_msgs::Transform ToGeometryMsgTransform(
    const ::cartographer::transform::Rigid3d& rigid3d);

geometry_msgs::Pose ToGeometryMsgPose(
    const ::cartographer::transform::Rigid3d& rigid3d);

geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d);

// Converts ROS message to point cloud. Returns the time when the last point
// was acquired (different from the ROS timestamp). Timing of points is given in
// the fourth component of each point relative to `Time`.
// 将 ROS 消息转换为点云。返回获取最后一点的时间（与 ROS 时间戳不同）。相对于 "Time"，每个点的第四部分给出了点的计时。
/**
 * @brief ToPointCloudWithIntensities  处理 LaserScan 单线激光扫描消息。消息类型为 sensor_msgs::LaserScan。
 *                                     把 ROS 消息(sensor_msgs::LaserScan)转换成 Cartographer 中的
 *                                     传感器数据类型(carto::sensor::PointCloudWithIntensities)。
 *                                     主要工作就是根据 ROS 的消息内容，计算扫描到的障碍物在工作空间中的坐标位置，并将其保存在一个特定的数据结构中。
 *                                     并且返回获取最后一个扫描点的时间（与 ROS 时间戳不同）。
 * @param msg                          单线激光扫描的消息
 * @return PointCloudWithIntensities   转换之后的点云数据，Cartographer 定义的点云数据，带相对测量时间和强度
 *         Time                        转换之后的时间戳，这是一帧雷达消息 msg 中获取最后一个扫描点的时间
 */
std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::LaserScan& msg);

/**
 * @brief ToPointCloudWithIntensities  处理 MultiEchoLaserScan 多线激光扫描消息。消息类型为 sensor_msgs::MultiEchoLaserScan。
 *                                     把 ROS 消息(sensor_msgs::MultiEchoLaserScan)转换成 Cartographer 中的
 *                                     传感器数据类型(carto::sensor::PointCloudWithIntensities)。
 *                                     主要工作就是根据 ROS 的消息内容，计算扫描到的障碍物在工作空间中的坐标位置，并将其保存在一个特定的数据结构中。
 *                                     并且返回获取最后一个扫描点的时间（与 ROS 时间戳不同）。
 * @param msg                          多线激光扫描的消息
 * @return PointCloudWithIntensities   转换之后的点云数据，Cartographer 定义的点云数据，带相对测量时间和强度
 *         Time                        转换之后的时间戳，这是一帧雷达消息 msg 中获取最后一个扫描点的时间
 */
std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::MultiEchoLaserScan& msg);

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::PointCloud2& message);

::cartographer::sensor::LandmarkData ToLandmarkData(
    const cartographer_ros_msgs::LandmarkList& landmark_list);

::cartographer::transform::Rigid3d ToRigid3d(
    const geometry_msgs::TransformStamped& transform);

::cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose);

Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3);

Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion);

// Converts from WGS84 (latitude, longitude, altitude) to ECEF.
Eigen::Vector3d LatLongAltToEcef(double latitude, double longitude,
                                 double altitude);

// Returns a transform that takes ECEF coordinates from nearby points to a local
// frame that has z pointing upwards.
cartographer::transform::Rigid3d ComputeLocalFrameFromLatLong(double latitude,
                                                              double longitude);

// Points to an occupancy grid message at a specific resolution from painted
// submap slices obtained via ::cartographer::io::PaintSubmapSlices(...).
std::unique_ptr<nav_msgs::OccupancyGrid> CreateOccupancyGridMsg(
    const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const ros::Time& time);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MSG_CONVERSION_H
