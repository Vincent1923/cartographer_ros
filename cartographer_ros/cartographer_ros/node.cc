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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

namespace {

// 获取系统默认的订阅主题名称集合
cartographer_ros_msgs::SensorTopics DefaultSensorTopics() {
  cartographer_ros_msgs::SensorTopics topics;
  // 这里以前缀 "k" 开始的变量都是在 "node_constants.h" 中定义的常数
  topics.laser_scan_topic = kLaserScanTopic;
  topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  topics.point_cloud2_topic = kPointCloud2Topic;
  topics.imu_topic = kImuTopic;
  topics.odometry_topic = kOdometryTopic;
  topics.nav_sat_fix_topic = kNavSatFixTopic;
  topics.landmark_topic = kLandmarkTopic;
  return topics;
}

// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
// 使用 "node_handle"为 "trajectory_id" 的路径订阅 "topic"，并在 "node" 上调用 "handler" 来处理消息。返回 subscriber。
/**
 * @brief SubscribeWithHandler  订阅传感器的主题，这是一个模板函数，模板为传感器的消息类型，
 *                              例如，单线激光数据 laser_scan 的消息类型为 sensor_msgs::LaserScan
 * @param trajectory_id         轨迹的索引
 * @param topic                 传感器的主题，例如单线激光数据 laser_scan 的系统默认主题为 "scan"
 * @param node_handle           ROS 的节点句柄
 * @param node
 * @return                      传感器的订阅器，数据类型为 ::ros::Subscriber
 */
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    // 这里用函数指针 void (Node::*handler)(int, const std::string&, const typename MessageType::ConstPtr&)
    // 作为函数 SubscribeWithHandler 的第一个形式参数。
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstPtr&),
    const int trajectory_id, const std::string& topic,
    ::ros::NodeHandle* const node_handle, Node* const node) {
  return node_handle->subscribe<MessageType>(
      topic, kInfiniteSubscriberQueueSize,
      boost::function<void(const typename MessageType::ConstPtr&)>(
          [node, handler, trajectory_id,
           topic](const typename MessageType::ConstPtr& msg) {
            (node->*handler)(trajectory_id, topic, msg);
          }));
}

}  // namespace

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

/**
 * 构造函数
 * node_options 是 node 对象的各种配置参数，在 "node_main.cc" 中由函数 Run 中从配置文件中获取。
 * map_builder 是 Cartographer 用于建图的对象，而 tf_buffer 是 ROS 系统中常用的坐标变换库 tf2 的缓存对象。
 * 函数后面的冒号部分是 C++ 标准的构造时对成员变量进行初始化的方法。
 */
Node::Node(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer) {  // 构造函数的主体
  // 在构造函数一开始，先对互斥量 mutex_ 加锁，防止因为多线程等并行运算的方式产生异常的行为。
  // 这里定义的是局部变量 lock，会在其构造函数中对 mutex_ 加锁，当构造函数运行结束之后会销毁该变量，在其析构函数中释放 mutex_。
  carto::common::MutexLocker lock(&mutex_);  // 设置一个互斥锁

  /**
   * Publisher
   * 通过 ROS 的节点句柄 node_handle_ 注册发布了一系列主题。
   * 这里以前缀 "k" 开始的变量都是在 "node_constants.h" 中定义的常数，它们都是默认的发布消息的名称和消息队列大小。
   */
  // 构建好的子图列表。主题名称为 kSubmapListTopic[] = "submap_list"
  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  // 跟踪轨迹路径点列表，用于在 rviz 上显示。主题名称为 kTrajectoryNodeListTopic[] = "trajectory_node_list"
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  // 路标点位姿列表，用于在 rviz 上显示。主题名称为 kLandmarkPosesListTopic[] = "landmark"
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  // 优化约束，用于在 rviz 上显示。主题名称为 kConstraintListTopic[] = "constraint_list"
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);

  /**
   * Service Server
   * 注册服务，并保存在容器 service_servers_ 中。
   * 同样的前缀 "k" 开始的变量都是在 node_constants.h 中定义的常数，它们是默认的服务名称。
   * 在注册服务的时候还需要提供一个响应该服务的回调函数。
   */
  // 查询 Submap。服务名称为 kSubmapQueryServiceName[] = "submap_query"
  service_servers_.push_back(node_handle_.advertiseService(                 
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  // 开始一条新的轨迹。服务名称为 kStartTrajectoryServiceName[] = "start_trajectory"
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  // 结束一条轨迹。服务名称为 kFinishTrajectoryServiceName[] = "finish_trajectory"
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  // 写状态，把构建的地图数据保存为后缀名为 ".pbstream" 的文件。服务名称为 kWriteStateServiceName[] = "write_state"
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));

  // Publisher
  // 发布匹配的 2D 点云数据。kScanMatchedPointCloudTopic[] = "scan_matched_points2"
  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

  /**
   * ::ros::WallTimer
   * 创建了一系列计时器保存在容器 wall_timers_ 中。
   * 这些计时器的作用是通过超时回调函数定时的发布对应主题的消息。计时器的周期由 node_options_ 确定，通过配置文件获取。
   */
  // 计时器往主题 "submap_list" 定时发布消息，周期为 node_options_.submap_publish_period_sec
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  // 计时器往主题 "scan_matched_points2" 定时发布消息，周期为 node_options_.pose_publish_period_sec
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.pose_publish_period_sec),
      &Node::PublishTrajectoryStates, this));
  // 计时器往主题 "trajectory_node_list" 定时发布消息，周期为 node_options_.trajectory_publish_period_sec
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishTrajectoryNodeList, this));
  // 计时器往主题 "landmark" 定时发布消息，周期为 node_options_.trajectory_publish_period_sec
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishLandmarkPosesList, this));
  // 计时器往主题 "constraint_list" 定时发布消息，周期为 kConstraintPublishPeriodSec
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &Node::PublishConstraintList, this));
}

Node::~Node() { FinishAllTrajectories(); }

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

// "/submap_query" 的服务响应函数，根据请求的轨迹索引 trajectory_id 和子图索引 submap_index，
// 向 Cartographer 请求子图。
bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  // 先对互斥量 mutex_ 加锁，防止因为多线程等并行运算的方式产生异常的行为。
  carto::common::MutexLocker lock(&mutex_);
  // 通过 map_builder_bridge_ 对象的 函数 HandlesSubmapQuery() 请求子图
  map_builder_bridge_.HandleSubmapQuery(request, response);
  return true;
}

// 调用 map_builder_bridge_.GetSubmapList() 函数获取 submap 的 list，然后用 ros 的 publish 函数向 Topic 上广播这个消息。
// 发布的 topic 名字为 kSubmapListTopic[] = "submap_list"，在节点 cartographer_occupancy_grid_node 上进行接受。
void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
}

void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

void Node::PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    const auto& trajectory_state = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_state.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      if (scan_matched_point_cloud_publisher_.getNumSubscribers() > 0) {
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;
        point_cloud.reserve(trajectory_state.local_slam_data
                                ->range_data_in_local.returns.size());
        for (const Eigen::Vector3f point :
             trajectory_state.local_slam_data->range_data_in_local.returns) {
          Eigen::Vector4f point_time;
          point_time << point, 0.f;
          point_cloud.push_back(point_time);
        }
        scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_state.local_slam_data->time),
            node_options_.map_frame,
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_state.local_to_map.cast<float>())));
      }
      extrapolator.AddPose(trajectory_state.local_slam_data->time,
                           trajectory_state.local_slam_data->local_pose);
    }

    geometry_msgs::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = std::max(
        FromRos(ros::Time::now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp = ToRos(now);

    const Rigid3d tracking_to_local = [&] {
      if (trajectory_state.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(extrapolator.ExtrapolatePose(now)));
      }
      return extrapolator.ExtrapolatePose(now);
    }();

    const Rigid3d tracking_to_map =
        trajectory_state.local_to_map * tracking_to_local;

    if (trajectory_state.published_to_tracking != nullptr) {
      if (trajectory_state.trajectory_options.provide_odom_frame) {
        std::vector<geometry_msgs::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_state.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_state.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        tf_broadcaster_.sendTransform(stamped_transforms);
      } else {
        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_map * (*trajectory_state.published_to_tracking));
        tf_broadcaster_.sendTransform(stamped_transform);
      }
    }
  }
}

void Node::PublishTrajectoryNodeList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
    carto::common::MutexLocker lock(&mutex_);
    trajectory_node_list_publisher_.publish(
        map_builder_bridge_.GetTrajectoryNodeList());
  }
}

void Node::PublishLandmarkPosesList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (landmark_poses_list_publisher_.getNumSubscribers() > 0) {
    carto::common::MutexLocker lock(&mutex_);
    landmark_poses_list_publisher_.publish(
        map_builder_bridge_.GetLandmarkPosesList());
  }
}

void Node::PublishConstraintList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (constraint_list_publisher_.getNumSubscribers() > 0) {
    carto::common::MutexLocker lock(&mutex_);
    constraint_list_publisher_.publish(map_builder_bridge_.GetConstraintList());
  }
}

// 返回一条trajectory所期望的SensorIds集合，‘SensorId::id’是期望的ROS topic名称
/*
 * cartographer_ros_msgs::SensorTopics定义：
 *   string laser_scan_topic
 *   string multi_echo_laser_scan_topic
 *   string point_cloud2_topic
 *   string imu_topic
 *   string odometry_topic
 *   string nav_sat_fix_topic
 *   string landmark_topic
 */
std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(
    const TrajectoryOptions& options,
    const cartographer_ros_msgs::SensorTopics& topics) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  // SensorId会把SensorType（传感器类型）和传感器数据的topic名称（类型为std::string）绑定在一起。
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  // 订阅所有laser scan，multi echo laser scan和point cloud主题。
  LOG(WARNING) << "Expected topics:";
  // 函数 ComputeRepeatedTopicNames 主要作用是对于多个 topics，把数字添加到 topic 的名字并返回列表。
  // 以 scan 为例，若数据只有一个，则直接返回 scan。若数据有多个，则返回 (scan_1，scan_2,...) 的列表。
  for (const std::string& topic : ComputeRepeatedTopicNames(     // 把 laser_scan_topic 的 topics 名称跟 SensorType::RANGE 进行绑定
           topics.laser_scan_topic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
    std::cout << "topics.laser_scan_topic: " << topic << std::endl;
  }
  for (const std::string& topic :                                     // 把 multi_echo_laser_scan_topic 的 topics 名称跟 SensorType::RANGE 进行绑定
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
    std::cout << "topics.multi_echo_laser_scan_topic: " << topic << std::endl;
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(        // 把 point_cloud2_topic 的 topics 名称跟 SensorType::RANGE 进行绑定
           topics.point_cloud2_topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
    std::cout << "topics.point_cloud2_topic: " << topic << std::endl;
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  // 对于2D SLAM，如果我们需要IMU，就订阅。对于3D SLAM，需要订阅IMU。
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||   // 把 imu_topic 的 topics 名称跟 SensorType::IMU 进行绑定
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, topics.imu_topic});
    std::cout << "topics.imu_topic: " << topics.imu_topic << std::endl;
  }
  // Odometry is optional.
  // Odometry（里程计数据）是可选的。
  if (options.use_odometry) {  // 把 odometry_topic 的 topics 名称跟 SensorType::ODOMETRY 进行绑定
    expected_topics.insert(
        SensorId{SensorType::ODOMETRY, topics.odometry_topic});
    std::cout << "topics.odometry_topic: " << topics.odometry_topic << std::endl;
  }
  // NavSatFix is optional.
  // NavSatFix 是可选的。
  if (options.use_nav_sat) {  // 把 nav_sat_fix_topic 的 topics 名称跟 SensorType::FIXED_FRAME_POSE 进行绑定
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, topics.nav_sat_fix_topic});
    std::cout << "topics.nav_sat_fix_topic: " << topics.nav_sat_fix_topic << std::endl;
  }
  // Landmark is optional.
  // Landmark 是可选的。
  if (options.use_landmarks) {  // 把 kLandmarkTopic 的 topics 名称跟 SensorType::LANDMARK 进行绑定
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
    std::cout << "kLandmarkTopic: " << kLandmarkTopic << std::endl;
  }
  return expected_topics;
}

// 开始一条新的轨迹跟踪，并返回新建轨迹的索引
int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::SensorTopics& topics) {
  // 通过函数 ComputeExpectedSensorIds() 根据配置选项 options 获取 SendorId。
  // 所谓的 SensorId 是定义在 "trajectory_builder_interface.h" 中的一个结构体，它一共有两个字段，
  // type 通过枚举描述了传感器的类型，id 是一个字符串记录了传感器所对应的 ROS 主题名称。
  // expected_sensor_ids 为订阅的传感器主题名称的集合。
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options, topics);
  // 通过接口 map_builder_bridge_ 向 Cartographer 添加一条新的轨迹并获取轨迹的索引。
  // 调用 map_builder_bridge_ 的函数 AddTrajectory() 来处理
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  LOG(WARNING) << "trajectory_id: " << trajectory_id;
  // 以新添加的轨迹索引 trajectory_id 为键值，通过函数 AddExtrapolator() 和 AddSensorSamplers()
  // 添加用于位姿插值(PoseExtrapolator)和传感器采样(TrajectorySensorSamplers)的对象。
  AddExtrapolator(trajectory_id, options);    // 添加位姿估计
  AddSensorSamplers(trajectory_id, options);  // 设置传感器
  // 然后调用成员函数 LaunchSubscribers() 完成传感器消息的订阅。
  // 这是一个关键的步骤，只有订阅了传感器消息，才能够在传感器数据的作用下驱动系统运转，进而完成位姿估计和建图的任务。
  LaunchSubscribers(options, topics, trajectory_id);
  is_active_trajectory_[trajectory_id] = true;  // 记录当前正在更新的轨迹
  for (const auto& sensor_id : expected_sensor_ids) {  // 记录当前轨迹订阅的主题
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;  // 返回新建的轨迹索引
}

// 根据配置订阅需要的主题
void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::SensorTopics& topics,
                             const int trajectory_id) {
  LOG(WARNING) << "Subscribe topics:";
  // 订阅单线激光数据 laser_scan 的主题，默认名称为 "scan"，消息类型为 sensor_msgs::LaserScan
  // for 循环写法是 C++11 标准中新增的 "for range" 形式，函数 ComputeRepeatedTopicNames() 是用来处理有多个相同类型的传感器的。
  // 比如这里的单线激光 laser_scan，通过在配置文件中的 num_laser_scans 字段指定单线激光的数量。
  // 假如输入参数 topics.laser_scan_topic 中对应的主题名称是 "scan"，那么如果只有一个单线激光，
  // 就是用 scan 作为订阅主题名称，如果有多个单线激光则在主题名称之后添加数字予以区别，即 scan_1, scan_2, ...
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    std::cout << "topics.laser_scan_topic: " << topic << std::endl;
    // 通过定义在 "node.cc" 中的模板函数 SubscribeWithHandler() 调用 ROS 的节点句柄 node_handle_ 来订阅传感器的主题。
    // subscribers_ 的类型为 std::unordered_map<int, std::vector<Subscriber>>，
    /**
     * 1. 通过定义在 "node.cc" 中的模板函数 SubscribeWithHandler() 调用 ROS 的节点句柄 node_handle_ 来订阅传感器的主题。
     * 2. 容器对象 subscribers_ 的类型为 std::unordered_map<int, std::vector<Subscriber>>，
     *    这是一个 std::unordered_map 的容器，容器的 key 为 int 类型，表示轨迹索引，
     *    而 std::vector<Subscriber> 表示一条轨迹中所有的传感器的订阅器集合，
     *    所以 subscribers_ 保存的是轨迹索引为 trajectory_id 的轨迹下的所有构建的订阅器集合，
     *    即它把轨迹索引 trajectory_id 和该轨迹下所有订阅传感器主题的订阅器 Subscriber 绑定在一起，
     *    其中元素的数据类型是 Node 类型中定义的私有结构体 Subscriber，有两个字段为 subscriber 和 topic，分别记录了订阅器和传感器主题。
     * 3. 订阅之后的处理是在函数 Node::HandleLaserScanMessage()，查看该代码可以发现最后交给了 map_builder_bridge_ 去处理。
     *    其中这里用函数指针 &Node::HandleLaserScanMessage 作为函数 SubscribeWithHandler() 的其中一个实际参数。
     */
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
  // 订阅多线激光扫描数据 multi_echo_laser_scan 的主题，默认名称为 "echoes"，消息类型为 sensor_msgs::MultiEchoLaserScan
  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    std::cout << "topics.multi_echo_laser_scan_topic: " << topic << std::endl;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }
  // 订阅点云数据 point_clouds 的主题，默认名称为 "points2"，消息类型为 sensor_msgs::PointCloud2
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    std::cout << "topics.point_cloud2_topic: " << topic << std::endl;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  // 订阅 IMU 传感器主题，默认名称为 "imu"，消息类型为 sensor_msgs::Imu
  // 如果是三维建图就必须使用 IMU，二维建图可以通过配置文件中的 use_imu_data 字段设置。
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    std::string topic = topics.imu_topic;
    std::cout << "topics.imu_topic: " << topic << std::endl;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, topic,
                                                &node_handle_, this),
         topic});
  }

  // 订阅里程计 Odometry 传感器主题，默认名称为 "odom"，消息类型为 nav_msgs::Odometry
  if (options.use_odometry) {
    std::string topic = topics.odometry_topic;
    std::cout << "topics.odometry_topic: " << topic << std::endl;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, topic,
                                                  &node_handle_, this),
         topic});
  }
  // 订阅导航卫星 NavSatFix 传感器主题，默认名称为 "fix"，消息类型为 sensor_msgs::NavSatFix
  if (options.use_nav_sat) {
    std::string topic = topics.nav_sat_fix_topic;
    std::cout << "topics.nav_sat_fix_topic: " << topic << std::endl;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
  // 订阅路标 Landmark 传感器主题，默认名称为 "landmark"，消息类型为 cartographer_ros_msgs::LandmarkList
  if (options.use_landmarks) {
    std::string topic = topics.landmark_topic;
    std::cout << "topics.landmark_topic: " << topic << std::endl;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(
    const ::cartographer_ros_msgs::SensorTopics& topics,
    const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options, topics)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

/*
 * （1）返回值类型是cartographer_ros_msgs::StatusResponse,
 *     在/src/cartographer_ros/cartographer_ros_msgs/msg/StatusResponse.msg中定义。
 * （2）前面检查了一下是否可以关掉，指定id是否存在，是否已经被Finished了等情况后，
 *     如果一切正常，则停止订阅Topic、清除id及其他与该trajectory相关的量。
 * （3）最后调用map_builder_bridge_中的FinishTrajectory函数。
 */
cartographer_ros_msgs::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::StatusResponse status_response;

  // First, check if we can actually finish the trajectory.
  // 首先，检查我们是否能够真正结束trajectory。
  // 检查id为 trajectory_id 的 trajectory 是否为 frozen 状态。
  // 在进行定位 pure localization 时，用来定位的底图的 trajectory_id = 0，此时它的 trajectory 状态就默认为 frozen。
  if (map_builder_bridge_.GetFrozenTrajectoryIds().count(trajectory_id)) {
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " is frozen.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    status_response.message = error;
    return status_response;
  }
  // 检查id为 trajectory_id 的 trajectory 是否已经创建
  if (is_active_trajectory_.count(trajectory_id) == 0) {
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " is not created yet.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    status_response.message = error;
    return status_response;
  }
  // 检查id为 trajectory_id 的 trajectory 是否已经被 Finished
  if (!is_active_trajectory_[trajectory_id]) {
    const std::string error = "Trajectory " + std::to_string(trajectory_id) +
                              " has already been finished.";
    LOG(ERROR) << error;
    status_response.code =
        cartographer_ros_msgs::StatusCode::RESOURCE_EXHAUSTED;
    status_response.message = error;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  // 关闭此trajectory的subscribers。
  for (auto& entry : subscribers_[trajectory_id]) {
    entry.subscriber.shutdown();
    subscribed_topics_.erase(entry.topic);  // 删除 subscribed_topics_ 中值为 entry.topic 的元素
    LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
  }
  CHECK_EQ(subscribers_.erase(trajectory_id), 1);  // 删除 subscribers_ 中键值为 trajectory_id 的元素
  CHECK(is_active_trajectory_.at(trajectory_id));
  // 调用 map_builder_bridge_ 的 FinishTrajectory 函数来结束 id 为 trajectory_id 的路径
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
  const std::string message =
      "Finished trajectory " + std::to_string(trajectory_id) + ".";
  status_response.code = cartographer_ros_msgs::StatusCode::OK;
  status_response.message = message;
  return status_response;
}

// "/start_trajectory" 的服务响应函数，开启一个路径跟踪
bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  // 先对互斥量 mutex_ 加锁，防止因为多线程等并行运算的方式产生异常的行为。
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;  // 轨迹的配置参数
  // 通过两个条件语句来检查输入参数 request
  if (!FromRosMessage(request.options, &options) ||
      !ValidateTrajectoryOptions(options)) {
    // 输入参数 request.options 不合法，报错返回。
    // FromRosMessage() 函数将 "msg" 转换为 "options"，成功时返回 true，失败时返回 false。
    // ValidateTrajectoryOptions() 函数判断请求的 options 参数配置是否合法。
    const std::string error = "Invalid trajectory options.";
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message = error;
  } else if (!ValidateTopicNames(request.topics, options)) {
    // 输入参数 request.topics 不合法，报错返回。
    // ValidateTopicNames() 函数判断请求的订阅主题名称 request.topics 是否合法。
    const std::string error = "Invalid topics.";
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message = error;
  } else {
    // 如果输入参数合法则通过函数 AddTrajectory() 按照服务请求的订阅主题开启一个路径跟踪，并填充返回消息的相关字段
    response.trajectory_id = AddTrajectory(options, request.topics);  // 返回新添加轨迹的索引
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message = "Success.";
  }
  return true;
}

// 使用系统默认的订阅主题来开始轨迹跟踪。
// 输入参数 options 为轨迹的配置参数，在 "node_main.cc" 中由函数 Run 从配置文件中获取。
void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  // 先对互斥量 mutex_ 加锁，防止因为多线程等并行运算的方式产生异常的行为。
  carto::common::MutexLocker lock(&mutex_);
  // 检查输入的配置是否合法
  CHECK(ValidateTrajectoryOptions(options));
  // 调用函数 AddTrajectory() 开始轨迹跟踪。
  // 函数 DefaultSensorTopics() 获取系统默认的订阅主题名称集合，返回的数据类型为 cartographer_ros_msgs::SensorTopics
  AddTrajectory(options, DefaultSensorTopics());
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id :
         ComputeExpectedSensorIds(bags_options.at(i), DefaultSensorTopics())) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  is_active_trajectory_[trajectory_id] = true;
  return trajectory_id;
}

// "/finish_trajectory" 的服务响应函数，根据请求的轨迹索引 trajectory_id 停止路径跟踪
bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  // 先对互斥量 mutex_ 加锁，防止因为多线程等并行运算的方式产生异常的行为。
  carto::common::MutexLocker lock(&mutex_);
  // 调用函数 FinishTrajectoryUnderLock() 停止路径跟踪
  response.status = FinishTrajectoryUnderLock(request.trajectory_id);
  return true;
}

// "/write_state" 的服务响应函数，向 Cartographer 请求状态，并把结果序列化到指定的文件中，
// 文件名字为请求的名字 filename，后缀为 ".pbstream"。
bool Node::HandleWriteState(
    ::cartographer_ros_msgs::WriteState::Request& request,
    ::cartographer_ros_msgs::WriteState::Response& response) {
  // 先对互斥量 mutex_ 加锁，防止因为多线程等并行运算的方式产生异常的行为。
  carto::common::MutexLocker lock(&mutex_);
  // 通过 map_builder_bridge_ 对象的函数 SerializeState() 向 Cartographer 请求状态，
  // 并把结果序列化到指定的文件 request.filename 中。
  if (map_builder_bridge_.SerializeState(request.filename)) {
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message = "State written to '" + request.filename + "'.";
  } else {
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message = "Failed to write '" + request.filename + "'.";
  }
  return true;
}

void Node::FinishAllTrajectories() {
  carto::common::MutexLocker lock(&mutex_);
  for (auto& entry : is_active_trajectory_) {
    const int trajectory_id = entry.first;
    if (entry.second) {
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::StatusCode::OK);
    }
  }
}

bool Node::FinishTrajectory(const int trajectory_id) {
  carto::common::MutexLocker lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::StatusCode::OK;
}

void Node::RunFinalOptimization() {
  {
    carto::common::MutexLocker lock(&mutex_);
    for (const auto& entry : is_active_trajectory_) {
      CHECK(!entry.second);
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_.RunFinalOptimization();
}

void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::NavSatFix::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

// 处理 IMU 的消息回调函数
void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
  // 先对互斥量 mutex_ 加锁，防止因为多线程等并行运算的方式产生异常的行为。
  carto::common::MutexLocker lock(&mutex_);
  // 通过采样器对传感器的数据进行降采样。
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    // 将 IMU 数据喂给位姿估计器 extrapolators_
    // 位姿估计器的数据类型是定义在 cartographer 的 PoseExtrapolator，
    // 在函数 AddTrajectory() 中通过调用 AddExtrapolator() 完成初始化操作。
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  // 通过 map_builder_bridge_ 将传感器数据喂给 Cartographer
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

// 订阅sensor_msgs::LaserScan的消息处理函数，最后依然交给了map_builder_bridge_去处理。
// 实际调用的是map_builder_bridge_中的一个成员类sensor_bridge的函数来处理：
// map_builder_bridge_.sensor_bridge(trajectory_id)->HandleLaserScanMessage(sensor_id, msg)。
void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

// 处理多线激光扫描数据 multi_echo_laser_scan 的消息回调函数
void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  // 先对互斥量 mutex_ 加锁，防止因为多线程等并行运算的方式产生异常的行为。
  carto::common::MutexLocker lock(&mutex_);
  // 通过采样器对传感器的数据进行降采样。
  // 采样器的数据类型是定义在 "node.h" 中的结构体 TrajectorySensorSamplers，在函数 AddTrajectory() 中
  // 通过调用 AddSensorSamplers() 完成初始化操作。
  // 其本质是对 cartographer 中的采样器(fixed_ratio_sampler.h, fixed_ratio_sampler.cc)的封装，
  // 用一个计数器来按照一个指定的频率对原始的数据进行降采样，采样频率可以通过轨迹参数文件来配置。
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  // 通过 map_builder_bridge_ 将传感器数据喂给 Cartographer。
  // 通过轨迹索引 trajectory_id 从 map_builder_bridge_ 对象中查询获得对应轨迹的 SensorBridge 对象，
  // 并通过该对象的 HandleMultiEchoLaserScanMessage() 函数来处理雷达数据。
  // 值得注意的是，map_builder_bridge_ 的成员变量为 sensor_bridges_，它在函数 AddTrajectory() 完成初始化。
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(map_builder_bridge_.SerializeState(filename))
      << "Could not write state.";
}

void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.LoadState(state_filename, load_frozen_state);
}

}  // namespace cartographer_ros
