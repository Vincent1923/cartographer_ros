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

cartographer_ros_msgs::SensorTopics DefaultSensorTopics() {
  cartographer_ros_msgs::SensorTopics topics;
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
// 使用'node_handle'为'trajectory_id'的路径订阅'topic'，并在'node'上调用'handler'来处理消息。返回subscriber。
// 这里用函数指针void (Node::*handler)(int, const std::string&, const typename MessageType::ConstPtr&)
// 作为函数SubscribeWithHandler的其中一个形式参数。
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
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

Node::Node(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer)  // 函数后面的冒号表示一个实例的赋值
    : node_options_(node_options),
      map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer) {  // 构造函数的主体
  carto::common::MutexLocker lock(&mutex_);  // 设置一个互斥锁

  /*
   * Publisher
   * cartographer_ros发布的topics。
   * 告知master节点，我们将要向kSubmapListTopic这个Topic上发布一个::cartographer_ros_msgs::SubmapList型的message，
   * 而第二个参数kLatestOnlyPublisherQueueSize是publishing的缓存大小；发布的该Topic即可允许其他节点获取到我们构建的Submap的信息。
   * kSubmapListTopic 等常量定义在cartographer_ros/node_constants.h
   */
  submap_list_publisher_ =                                           // 发布构建的 submap 的list
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(   // kLatestOnlyPublisherQueueSize = 1
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);          // kSubmapListTopic[] = "submap_list"
  trajectory_node_list_publisher_ =                                  // 发布 trajectory 的list，在rviz上显示
      node_handle_.advertise<::visualization_msgs::MarkerArray>(     // kTrajectoryNodeListTopic[] = "trajectory_node_list"
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =                                   // 发布 Landmark 的位姿list，在rviz上显示
      node_handle_.advertise<::visualization_msgs::MarkerArray>(     // kLandmarkPosesListTopic[] = "landmark"
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =                                       // 发布 constraint list，在rviz上显示
      node_handle_.advertise<::visualization_msgs::MarkerArray>(     // kConstraintListTopic[] = "constraint_list"
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);

  /*
   * Service Server
   * 注册一个Service，Service的名字由kSubmapQueryServiceName给出。
   * 第二个参数HandleSubmapQuery是该Service绑定的函数句柄,即当有一个service的request时，由该函数进行response。
   * 注册的第一个service就对应了"submap_query"这个service。这是cartographer_node可以提供的一个service。
   */
  service_servers_.push_back(node_handle_.advertiseService(                 // 查询 Submap
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));            // kSubmapQueryServiceName[] = "submap_query";
  service_servers_.push_back(node_handle_.advertiseService(                 // 开始一段 trajectory
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));    // kStartTrajectoryServiceName[] = "start_trajectory";
  service_servers_.push_back(node_handle_.advertiseService(                 // 结束一段 trajectory
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));  // kFinishTrajectoryServiceName[] = "finish_trajectory";
  service_servers_.push_back(node_handle_.advertiseService(                 // 写状态，把构建的地图数据保存为后缀名为“.pbstream”的文件
      kWriteStateServiceName, &Node::HandleWriteState, this));              // kWriteStateServiceName[] = "write_state";

  // 发布了一个跟点云相关的Topic
  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);      // kScanMatchedPointCloudTopic[] = "scan_matched_points2";

  /*
   * （1）wall_timers在node.h中定义，是一个存储::ros::WallTimer类型的vector，
   *     以下通过vector的push_back操作依次将五个::ros::WallTimer型对象插入这个vector的末尾。
   * （2）::ros::WallTimer这个类参见如下链接：http://docs.ros.org/jade/api/roscpp/html/classros_1_1WallTimer.html
   *     简单说，这是一个定时器，这里分别为如下的五个函数设置了定时器。定时器的参数就是node_options_里的各项参数。
   * （3）接下来是为几个Topic设置了定时器，以及定时器函数。猜测这几个定时器函数里就是定时往Topic上发布消息。
   */
  wall_timers_.push_back(node_handle_.createWallTimer(                   // 往Topic kSubmapListTopic上发布消息的定时器
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.pose_publish_period_sec),
      &Node::PublishTrajectoryStates, this));
  wall_timers_.push_back(node_handle_.createWallTimer(                   // 往Topic kTrajectoryNodeListTopic上发布消息的定时器
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(                   // 往Topic kLandmarkPosesListTopic上发布消息的定时器
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishLandmarkPosesList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(                   // 往Topic kConstraintListTopic上发布消息的定时器
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &Node::PublishConstraintList, this));
}

Node::~Node() { FinishAllTrajectories(); }

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

// Service kSubmapQueryServiceName 绑定的函数句柄，主要工作是根据请求的 trajectory_id 和 submap_index，查询对应的 Submap。
// 函数有两个参数，
// 一个参数是一个 ::cartographer_ros_msgs::SubmapQuery::Request 型的变量，对应是请求服务时的输入参数，
// 另外一个参数是 ::cartographer_ros_msgs::SubmapQuery::Response 型的变量，对应的是服务响应后的返回值。
bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  carto::common::MutexLocker lock(&mutex_);  // 设置互斥锁
  // map_builder_bridge_ 在node.h 定义，是一个 MapBuilderBridge 型的变量。
  // map_builder_bridge_ 本质上的功能是由 map_builder.cc 决定的。
  map_builder_bridge_.HandleSubmapQuery(request, response);  // 调用了 map_builder_bridge_ 的 HandlesSubmapQuery 来做处理
  return true;
}

// 调用map_builder_bridge_.GetSubmapList()函数获取submap的list，
// 然后用ros的publish函数向Topic上广播这个消息。
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
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  // 订阅所有laser scan，multi echo laser scan和point cloud主题。
  // 函数 ComputeRepeatedTopicNames 定义在node_constants.h
  // SensorId会把SensorType（传感器类型）和传感器数据的topic名称（类型为std::string）绑定在一起。
  LOG(WARNING) << "Expected topics:";
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
    std::cout << "topics.laser_scan_topic: " << topic << std::endl;
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
    std::cout << "topics.multi_echo_laser_scan_topic: " << topic << std::endl;
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
    std::cout << "topics.point_cloud2_topic: " << topic << std::endl;
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  // 对于2D SLAM，如果我们需要IMU，就订阅。对于3D SLAM，需要订阅IMU。
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, topics.imu_topic});
    std::cout << "topics.imu_topic: " << topics.imu_topic << std::endl;
  }
  // Odometry is optional.
  // Odometry（里程计数据）是可选的。
  if (options.use_odometry) {
    expected_topics.insert(
        SensorId{SensorType::ODOMETRY, topics.odometry_topic});
    std::cout << "topics.odometry_topic: " << topics.odometry_topic << std::endl;
  }
  // NavSatFix is optional.
  // NavSatFix 是可选的。
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, topics.nav_sat_fix_topic});
    std::cout << "topics.nav_sat_fix_topic: " << topics.nav_sat_fix_topic << std::endl;
  }
  // Landmark is optional.
  // Landmark 是可选的。
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
    std::cout << "kLandmarkTopic: " << kLandmarkTopic << std::endl;
  }
  return expected_topics;
}

/*
 * 同样，AddTrajectory函数也是通过调用map_builder_bridge_中的AddTrajectory来处理。
 * 同时，每增加一条轨迹，都需要给该轨迹增加必要的处理，比如添加位姿估计的AddExtrapolator，
 * 设置传感器的AddSensorSamplers，用来订阅必要的Topic以接收数据的LaunchSubscribers等。
 */
int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::SensorTopics& topics) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options, topics);  // expected_sensor_ids为我们要订阅的传感器topics名称的集合
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);  // 调用 map_builder_bridge_ 中的 AddTrajectory 来处理
  LOG(WARNING) << "trajectory_id: " << trajectory_id;
  AddExtrapolator(trajectory_id, options);            // 添加位姿估计
  AddSensorSamplers(trajectory_id, options);          // 设置传感器
  LaunchSubscribers(options, topics, trajectory_id);  // 订阅必要的 Topic 以接收数据
  is_active_trajectory_[trajectory_id] = true;
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

// 订阅传感器发布的消息
void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::SensorTopics& topics,
                             const int trajectory_id) {
  LOG(WARNING) << "Subscribe topics:";
  // subscribers_的类型为std::unordered_map<int, std::vector<Subscriber>>，
  // 它把trajectory id和该路径下所有传感器数据订阅的Subscriber绑定在一起。
  /*
   * （1）subscribers_ 的类型为 std::unordered_map<int, std::vector<Subscriber>>，
   *     这是一个 std::unordered_map 的容器，容器的 key 为 int 类型，表示 trajectory 的下标，
   *     而 std::vector<Subscriber> 表示一条 trajectory 中所有订阅传感器数据的 Subscriber 集合，
   *     它把 trajectory id 和该路径下所有传感器数据订阅的Subscriber绑定在一起。
   * （2）SubscribeWithHandler()函数主要作用就是订阅一个以topic为名字的Topic，
   *     不同的传感器中的topic这个变量是for循环体中的这一句代码赋值的
   *     const std::string& topic:ComputeRepeatedTopicNames(topics.laser_scan_topic, options.num_laser_scans)，
   *     然后返回了一个node_handle->subscribe<MessageType>，即返回值类型为::ros::Subscriber。
   *     在LaunchSubscribers函数里把这个返回值压入了subscribers_[trajectory_id]列表中。
   * （3）订阅之后的处理是在Node::HandleLaserScanMessage，查看该代码就可以发现最后依然交给了map_builder_bridge_去处理。
   *     其中这里用函数指针&Node::HandleLaserScanMessage作为函数SubscribeWithHandler的其中一个实际参数。
   * （4）这里用函数名&Node::HandleLaserScanMessage作为函数SubscribeWithHandler的其中一个输入参数。
   */
  for (const std::string& topic : ComputeRepeatedTopicNames(     // 订阅sensor_msgs::LaserScan
           topics.laser_scan_topic, options.num_laser_scans)) {
    std::cout << "topics.laser_scan_topic: " << topic << std::endl;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
  for (const std::string& topic :                                        // 订阅sensor_msgs::MultiEchoLaserScan
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    std::cout << "topics.multi_echo_laser_scan_topic: " << topic << std::endl;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(        // 订阅sensor_msgs::PointCloud2
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
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||     // 订阅sensor_msgs::Imu
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

  if (options.use_odometry) {                                                    // 订阅nav_msgs::Odometry
    std::string topic = topics.odometry_topic;
    std::cout << "topics.odometry_topic: " << topic << std::endl;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, topic,
                                                  &node_handle_, this),
         topic});
  }
  if (options.use_nav_sat) {                                                      // 订阅sensor_msgs::NavSatFix
    std::string topic = topics.nav_sat_fix_topic;
    std::cout << "topics.nav_sat_fix_topic: " << topic << std::endl;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
  if (options.use_landmarks) {                                                   // 订阅cartographer_ros_msgs::LandmarkList
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
  if (map_builder_bridge_.GetFrozenTrajectoryIds().count(trajectory_id)) {     // 检查id为trajectory_id的trajectory是否为frozen状态
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " is frozen.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    status_response.message = error;
    return status_response;
  }
  if (is_active_trajectory_.count(trajectory_id) == 0) {                       // 检查id为trajectory_id的trajectory是否已经创建
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " is not created yet.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    status_response.message = error;
    return status_response;
  }
  if (!is_active_trajectory_[trajectory_id]) {                                 // 检查id为trajectory_id的trajectory是否已经被Finished
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
    subscribed_topics_.erase(entry.topic);
    LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
  }
  CHECK_EQ(subscribers_.erase(trajectory_id), 1);
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

// 前面一些异常情况的处理，正常情况下调用AddTrajectory函数，增加一条trajectory。
/*
 * cartographer_ros_msgs::StartTrajectory定义：
 *   cartographer_ros_msgs/TrajectoryOptions options
 *   cartographer_ros_msgs/SensorTopics topics
 *   ---
 *   cartographer_ros_msgs/StatusResponse status
 *   int32 trajectory_id
 */
bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;
  // FromRosMessage() 函数将"msg"转换为"options"，成功时返回true，失败时返回false。
  // ValidateTrajectoryOptions() 函数判断请求的 options 参数配置是否合法。
  if (!FromRosMessage(request.options, &options) ||           // 异常情况的处理
      !ValidateTrajectoryOptions(options)) {
    const std::string error = "Invalid trajectory options.";
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message = error;
  } else if (!ValidateTopicNames(request.topics, options)) {  // 异常情况的处理，ValidateTopicNames() 函数判断请求的 topics 名称是否合法。
    const std::string error = "Invalid topics.";
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message = error;
  } else {                                                    // 正常情况下调用AddTrajectory函数，增加一条trajectory
    response.trajectory_id = AddTrajectory(options, request.topics);  // 返回trajectory的id
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message = "Success.";
  }
  return true;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
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

// 根据请求的trajectory_id，结束该trajectory
/*
 * cartographer_ros_msgs::FinishTrajectory定义：
 *   int32 trajectory_id
 *   ---
 *   cartographer_ros_msgs/StatusResponse status
 */
bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  response.status = FinishTrajectoryUnderLock(request.trajectory_id);
  return true;
}

// 写状态，根据请求的request.filename文件名，把构建的地图数据保存为后缀名为“.pbstream”的文件
/*
 * cartographer_ros_msgs::WriteState定义：
 *   string filename
 *   ---
 *   cartographer_ros_msgs/StatusResponse status
 */
bool Node::HandleWriteState(
    ::cartographer_ros_msgs::WriteState::Request& request,
    ::cartographer_ros_msgs::WriteState::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  // 调用了 map_builder_bridge_.SerializeState(request.filename) 函数进行写文件的操作
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

void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
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

void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
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
