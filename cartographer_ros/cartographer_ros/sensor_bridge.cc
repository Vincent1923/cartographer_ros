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

/**
 * 1. 构造函数，函数主体是空的没有任何操作，主要工作就是把参数表赋值给成员变量。
 * 2. 有五个输入参数，前面四个参数是从配置文件中加载的配置，第五个参数 trajectory_builder_ 则是 Cartographer 的一个核心对象，
 *    通过 sensor_bridge 对象转换后的数据都是通过它喂给 Cartographer 的，在 map_builder_bridge_ 的成员函数
 *    AddTrajectory() 中通过 map_builder_->GetTrajectoryBuilder(trajectory_id) 获得。
 */
SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,  // 激光雷达数据分段数量
    const std::string& tracking_frame,          // 参考坐标系
    const double lookup_transform_timeout_sec,  // tf 坐标变换查询超时设置
    tf2_ros::Buffer* const tf_buffer,           // tf 坐标变换缓存
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)  // 轨迹构建器
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

// 把 ROS 的 IMU 消息转换成 Cartographer 的 ImuData 数据类型，最后返回的 ImuData 数据是相对于机器人坐标系下
std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
    const sensor_msgs::Imu::ConstPtr& msg) {
  // 先检查线加速度和角速度的协方差矩阵的第一个元素。
  // 根据 ROS 系统的定义，如果协方差矩阵的第一个元素为 -1 意味着所对应的传感器数据无效。
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

  // 将 ROS 消息头中的时间戳转换成 Cartographer 中的时间
  const carto::common::Time time = FromRos(msg->header.stamp);
  // 通过坐标变换对象 tf_bridge_ 从 ROS 系统中查询 IMU 传感器相对于机器人坐标系的坐标变换关系
  // 返回的变换关系 sensor_to_tracking 的数据类型是 cartographer::transform::Rigid3d 的指针对象
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  // 检查 IMU 坐标系相对于机器人坐标系的平移距离，以保证 IMU 坐标系原点应当尽量与机器人坐标系重合。
  // 否则，将传感器测量的线加速度转换到机器人坐标系下就会出现比较大的偏差，对于位姿估计而言是不利的。
  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
      << "The IMU frame must be colocated with the tracking frame. "
         "Transforming linear acceleration into the tracking frame will "
         "otherwise be imprecise.";
  // 检查通过之后，就可以构建一个 ImuData 的对象，将经过坐标变换的传感器数据填充到该对象中，并返回。
  return carto::common::make_unique<carto::sensor::ImuData>(
      carto::sensor::ImuData{
          time,
          // 函数 sensor_to_tracking->rotation() 返回变换关系的旋转矩阵，类型为 Eigen::Quaternion<double>
          // 左乘旋转矩阵是把 IMU 传感器坐标系下的线加速度和角加速度变换到机器人坐标系下。
          // 函数 ToEigen() 把 ROS 的消息类型 geometry_msgs/Vector3 转换为 Eigen 库的类型 Eigen::Vector3d
          sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
          sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
}

// 处理 IMU 消息，消息类型为 sensor_msgs::Imu。
// 首先把 ROS 的 IMU 消息转换成 Cartographer 的 ImuData 数据类型，最后把转换后的数据喂给轨迹跟踪器。
void SensorBridge::HandleImuMessage(const std::string& sensor_id,
                                    const sensor_msgs::Imu::ConstPtr& msg) {
  // 调用函数 ToImuData() 把 ROS 的 IMU 消息转换成 Cartographer 的 ImuData 数据类型
  std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
  if (imu_data != nullptr) {
    // 直接通过轨迹跟踪器的接口 AddSensorData() 填塞数据
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                               imu_data->angular_velocity});
  }
}

// 处理 LaserScan 单线激光扫描消息，消息类型为 sensor_msgs::LaserScan。
// 把 ROS 消息(sensor_msgs::LaserScan)转换成 Cartographer 中的传感器数据类型
// (carto::sensor::PointCloudWithIntensities)。最后调用成员函数 HandleLaserScan() 来处理转换后的点云数据。
void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) {
  // 首先定义两个临时变量分别用于记录转换之后的点云数据和时间戳
  carto::sensor::PointCloudWithIntensities point_cloud;  // 转换之后的点云数据，包含3D位置，相对测量时间和强度
  carto::common::Time time;                              // 转换之后的时间戳，这是获取最后一个扫描点的时间
  // 通过函数 ToPointCloudWithIntensities()，将 ROS 的消息转换成 Cartographer 定义的点云数据。
  // 并且返回获取最后一个扫描点的时间（与 ROS 时间戳不同）。
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  // 通过成员函数 HandleLaserScan() 来处理点云数据
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

// 处理 MultiEchoLaserScan 多线激光扫描消息，消息类型为 sensor_msgs::MultiEchoLaserScan。
// 把 ROS 消息(sensor_msgs::MultiEchoLaserScan)转换成 Cartographer 中的
// 传感器数据类型(carto::sensor::PointCloudWithIntensities)。
// 最后调用成员函数 HandleLaserScan() 来处理转换后的点云数据。
void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  // 首先定义了两个临时变量分别用于记录转换之后的点云数据和时间戳。
  carto::sensor::PointCloudWithIntensities point_cloud;  // 转换之后的点云数据，包含3D位置，相对测量时间和强度
  carto::common::Time time;                              // 转换之后的时间戳，这是获取最后一个扫描点的时间
  // 通过函数 ToPointCloudWithIntensities()，将 ROS 的消息转换成 Cartographer 定义的点云数据。
  // 并且返回获取最后一个扫描点的时间（与 ROS 时间戳不同）。
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  // 通过成员函数 HandleLaserScan() 来处理点云数据
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

// 处理从 ROS 激光扫描消息转换过来的 Cartographer 定义的点云数据，将转换后的数据喂给 Cartographer 进行后序的处理。
// 根据 num_subdivisions_per_laser_scan_ 的大小，对一帧点云数据 points 进行划分。
// 把 carto::sensor::PointCloudWithIntensities 类型的数据转换成 carto::sensor::TimedPointCloud 类型。
void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) {
  // 先确认一下输入的点云非空
  if (points.points.empty()) {
    return;
  }
  // 检查一帧点云中最后一个点的时间是否小于或等于0
  CHECK_LE(points.points.back()[3], 0);
  // TODO(gaschler): Use per-point time instead of subdivisions.
  // 根据配置变量 num_subdivisions_per_laser_scan_ 在一个 for 循环中将点云数据拆分为若干段。
  // 对于 sensor_msgs::LaserScan 消息，num_subdivisions_per_laser_scan_ = 1
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    // 计算分段的起始索引
    const size_t start_index =  // 对于 sensor_msgs::LaserScan 消息，start_index = 0
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    // 计算分段的结束索引
    const size_t end_index =  // 对于 sensor_msgs::LaserScan 消息，end_index = 360，等于 range.size()
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    // 构建分段数据。临时变量 subdivision 记录经过划分后的包含时间信息的一帧点云数据。
    // 若 num_subdivisions_per_laser_scan_ = 1，则 subdivision 为输入的一帧点云数据。
    carto::sensor::TimedPointCloud subdivision(
        points.points.begin() + start_index, points.points.begin() + end_index);
    // 以下是为了跳过同一个分段中的元素
    if (start_index == end_index) {
      continue;
    }
    // 接着参考分段中最后一个数据的时间调整其他数据的时间，但在该操作之前需要先确认当前的数据没有过时。
    // time_to_subdivision_end 记录分段数据 subdivision 最后一个点的时间，这个时间是相对于输入的一帧点云数据的最后一个点的相对时间。
    // 若 num_subdivisions_per_laser_scan_ = 1，则 time_to_subdivision_end = 0
    const double time_to_subdivision_end = subdivision.back()[3];
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    // 计算分段数据 subdivision 最后一个点的时间戳。
    // subdivision_time 记录 subdivision 最后一个点的时间戳，这个时间为实际时间，不是相对时间。
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);
    // 检查当前的数据是否过时。
    // 成员容器 sensor_to_previous_subdivision_time_ 中以 sensor_id 为索引记录了最新的数据产生时间，
    // 如果分段的时间落后于记录值，将抛弃所对应的数据。
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        it->second >= subdivision_time) {
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;
    }
    // 更新传感器 sensor_id 最新数据的时间
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    // 调整分段数据 subdivision 每一个点的时间。
    // 即把分段数据 subdivision 最后一个点的时间变成0，其它点的时间换算成相对于 subdivision 的最后一个点的相对时间。
    for (Eigen::Vector4f& point : subdivision) {
      point[3] -= time_to_subdivision_end;
    }
    CHECK_EQ(subdivision.back()[3], 0);  // 检查点云 subdivision 的最后一个点的时间是否为0
    // 调用函数 HandleRangefinder() 将分段数据喂给 Cartographer
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  }
}

// 处理测距仪数据，将数据喂给 Cartographer 进行后序的处理。
// HandleRangefinder() 函数调用了 trajectory_builder_->AddSensorData() 来处理。
// 所以这里相当于做了一层抽象。我们的 Rangefinder 可以不一样是激光，也可以是其他类型的传感器，比如 Kinect。
// 这样，以后如果要扩展或修改，我们可以不改之前的代码，而只需要多写一个处理 Kinect 的代码就可以。这也是封装的好处。
void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) {
  // 通过 tf_bridge_ 对象查询传感器坐标系相对于机器人坐标系之间的坐标变换，记录在对象 sensor_to_tracking 中。
  // frame_id 是传感器坐标系名称，例如单线激光扫描消息一般为 "laser_link"。
  // tracking_frame_ 是机器人坐标系名称，一般为 "base_link" 或 "base_footprint"。
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
  if (sensor_to_tracking != nullptr) {
    // 调用 trajectory_builder_->AddSensorData() 函数把点云数据喂给 Cartographer 进行后序的处理。
    // TransformTimedPointCloud() 函数把传感器坐标系(frame_id)下的点云数据 ranges
    // 变换到机器人坐标系(tracking_frame_)下。
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                       time, sensor_to_tracking->translation().cast<float>(),
                       carto::sensor::TransformTimedPointCloud(
                           ranges, sensor_to_tracking->cast<float>())});
  }
}

}  // namespace cartographer_ros
