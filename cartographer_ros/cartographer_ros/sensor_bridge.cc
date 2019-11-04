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

#include "cartographer_ros/sensor_bridge.h"

#include "cartographer/common/make_unique.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

/*
 * （1）构造函数做的工作就是把参数表赋值给成员变量
 * （2）但需要注意的是成员变量中有一个TrajectoryBuilderInterface型的一个指针变量。
 *     继续跟踪代码我们可以发现，cartographer中各种消息都统一调用了这个成员类的虚函数AddSensorData()。
 * （3）而CollatedTrajectoryBuilder继承了这个类并实现了AddSensorData()函数。
 *     这两个类都定义在cartographer中的mapping文件夹下。
 *     CollatedTrajectoryBuilder的构造函数说明通过统一调用HandleCollatedSensorData()函数，
 *     来轮询处理kImu(IMU消息)、kRangefinder(测距消息，不仅仅是激光)、kOdometer(里程计消息)等。
 */
SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, tf2_ros::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),    // 初始化 num_subdivisions_per_laser_scan_
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),  // 初始化 tf_bridge_
      trajectory_builder_(trajectory_builder) {}                            // 初始化 trajectory_builder_

/*
 * （1）一个预处理的工具函数，并非SensorBridge的成员变量；其参数类型是nav_msgs::Odometry::ConstPtr&。
 * （2）这个需要注意的是TfBridge这个类，他的实例化对象是tf_bridge_，tf_bridge_是SensorBridge的一个成员变量。
 *     不管什么传感器，都会对每一帧数据的位姿进行估计，而这个历史的位姿数据就以TfBridge的形式存起来。
 *     这样在使用的时候也可以通过TfBridge查询某一个传感器在某一个历史时刻的位姿。
 * （3）这里同样也是，代码通过TfBridge查询了一下历史数据，然后把这个作为参数传给了TrajectoryBuilder的AddSensorData来做处理。
 *     前面我们已经介绍过了，TrajectoryBuilder也是为不同的传感器提供了统一的处理接口。
 *     我们可以在TrajectoryBuilder的具体实现里再看具体做了什么操作。
 *     但我们可以合理猜测，比如，里程计的数据是在原来数据的基础上再做一个累加，作为新的值同样保存到TfBridge里。
 */
std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
    const nav_msgs::Odometry::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  /*
   * 函数tf_bridge_.LookupToTracking()作用是查询一帧里程计数据相对于tracking_frame的变换矩阵，
   * 它的返回类型为指向::cartographer::transform::Rigid3d的指针，这是一个变换矩阵。
   * 所以该函数返回的是一个变换矩阵，查询的是某时刻某一帧数据的变换估计，估计要用来做累加。
   */
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  return carto::common::make_unique<carto::sensor::OdometryData>(
      carto::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
}

// 处理里程计消息
void SensorBridge::HandleOdometryMessage(
    const std::string& sensor_id, const nav_msgs::Odometry::ConstPtr& msg) {
  // odometry_data是在tracking_frame坐标系下的里程计数据
  std::unique_ptr<carto::sensor::OdometryData> odometry_data =
      ToOdometryData(msg);
  if (odometry_data != nullptr) {
    // 调用trajectory_builder_->AddSensorData()函数对里程计数据进行处理
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
}

void SensorBridge::HandleNavSatFixMessage(
    const std::string& sensor_id, const sensor_msgs::NavSatFix::ConstPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::FixedFramePoseData{
                       time, carto::common::optional<Rigid3d>()});
    return;
  }

  if (!ecef_to_local_frame_.has_value()) {
    ecef_to_local_frame_ =
        ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);
    LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
              << msg->latitude << ", long = " << msg->longitude << ".";
  }

  trajectory_builder_->AddSensorData(
      sensor_id,
      carto::sensor::FixedFramePoseData{
          time, carto::common::optional<Rigid3d>(Rigid3d::Translation(
                    ecef_to_local_frame_.value() *
                    LatLongAltToEcef(msg->latitude, msg->longitude,
                                     msg->altitude)))});
}

void SensorBridge::HandleLandmarkMessage(
    const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  trajectory_builder_->AddSensorData(sensor_id, ToLandmarkData(*msg));
}

// 数据预处理函数，处理IMU数据，返回的是经过变换后的IMU数据。并非SensorBridge的成员函数。
std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
    const sensor_msgs::Imu::ConstPtr& msg) {
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
      << "The IMU frame must be colocated with the tracking frame. "
         "Transforming linear acceleration into the tracking frame will "
         "otherwise be imprecise.";
  return carto::common::make_unique<carto::sensor::ImuData>(
      carto::sensor::ImuData{
          time,
          sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
          sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
}

// 处理IMU数据
void SensorBridge::HandleImuMessage(const std::string& sensor_id,
                                    const sensor_msgs::Imu::ConstPtr& msg) {
  std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
  if (imu_data != nullptr) {
    // 最终，将线加速度和角加速度传入trajectory_builder_->AddSensorData做处理
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                               imu_data->angular_velocity});
  }
}

// 处理数据类型为 sensor_msgs::LaserScan 的 激光雷达数据，
// 把sensor_msgs::LaserScan类型的数据转化成carto::sensor::PointCloudWithIntensities类型，
// 并调用SensorBridge::HandleLaserScan()函数来做处理。
void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;  // 点云数据，包含3D位置，时间，以及 intensities
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);  // 将ROS消息转换为点云
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

// 处理数据类型为 sensor_msgs::MultiEchoLaserScan 的激光雷达数据，
// 把sensor_msgs::MultiEchoLaserScan类型的数据转化成carto::sensor::PointCloudWithIntensities类型，
// 并调用SensorBridge::HandleLaserScan()函数来做处理。
void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;  // 点云数据，包含3D位置，时间，以及 intensities
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);  // 将ROS消息转换为点云
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

// 把sensor_msgs::PointCloud2类型的数据转化成carto::sensor::TimedPointCloud类型，
// 并调用SensorBridge::HandleRangefinder()函数来做处理。
void SensorBridge::HandlePointCloud2Message(
    const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
  pcl::fromROSMsg(*msg, pcl_point_cloud);
  carto::sensor::TimedPointCloud point_cloud;
  for (const auto& point : pcl_point_cloud) {
    point_cloud.emplace_back(point.x, point.y, point.z, 0.f);
  }
  HandleRangefinder(sensor_id, FromRos(msg->header.stamp), msg->header.frame_id,
                    point_cloud);
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

// 把carto::sensor::PointCloudWithIntensities类型的数据转化成carto::sensor::TimedPointCloud类型，
// 并调用SensorBridge::HandleRangefinder()函数来做处理。
void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) {
  if (points.points.empty()) {
    return;
  }
  CHECK_LE(points.points.back()[3], 0);
  // TODO(gaschler): Use per-point time instead of subdivisions.
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    carto::sensor::TimedPointCloud subdivision(
        points.points.begin() + start_index, points.points.begin() + end_index);
    if (start_index == end_index) {
      continue;
    }
    const double time_to_subdivision_end = subdivision.back()[3];
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        it->second >= subdivision_time) {
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;
    }
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    for (Eigen::Vector4f& point : subdivision) {
      point[3] -= time_to_subdivision_end;
    }
    CHECK_EQ(subdivision.back()[3], 0);
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  }
}

// 处理测距仪数据。
// HandleRangefinder函数调用了trajectory_builder_->AddSensorData来处理。
// 所以这里相当于做了一层抽象。我们的Rangefinder可以不一样是激光，也可以是其他类型的传感器，比如Kinect。
// 这样，以后如果要扩展或修改，我们可以不改之前的代码，而只需要多写一个处理Kinect的代码就可以。这也是封装的好处。
void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) {
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                       time, sensor_to_tracking->translation().cast<float>(),
                       carto::sensor::TransformTimedPointCloud(
                           ranges, sensor_to_tracking->cast<float>())});
  }
}

}  // namespace cartographer_ros
