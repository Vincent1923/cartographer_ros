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

#include "cartographer_ros/map_builder_bridge.h"

#include "cartographer/common/make_unique.h"
#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"

namespace cartographer_ros {
namespace {

using ::cartographer::transform::Rigid3d;

constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kLandmarkMarkerScale = 0.3;
constexpr double kConstraintMarkerScale = 0.025;

::std_msgs::ColorRGBA ToMessage(const cartographer::io::FloatColor& color) {
  ::std_msgs::ColorRGBA result;
  result.r = color[0];
  result.g = color[1];
  result.b = color[2];
  result.a = 1.f;
  return result;
}

visualization_msgs::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                  const std::string& frame_id) {
  visualization_msgs::Marker marker;
  marker.ns = "Trajectory " + std::to_string(trajectory_id);
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.header.stamp = ::ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
  marker.scale.x = kTrajectoryLineStripMarkerScale;
  marker.pose.orientation.w = 1.;
  marker.pose.position.z = 0.05;
  return marker;
}

// 返回一个指定id的landmark在container中的索引值；该函数也并非MapBuilderBridge的成员函数
int GetLandmarkIndex(
    const std::string& landmark_id,
    std::unordered_map<std::string, int>* landmark_id_to_index) {
  auto it = landmark_id_to_index->find(landmark_id);
  if (it == landmark_id_to_index->end()) {
    const int new_index = landmark_id_to_index->size();
    landmark_id_to_index->emplace(landmark_id, new_index);
    return new_index;
  }
  return it->second;
}

// 创建一个landmark
visualization_msgs::Marker CreateLandmarkMarker(int landmark_index,
                                                const Rigid3d& landmark_pose,
                                                const std::string& frame_id) {
  visualization_msgs::Marker marker;
  marker.ns = "Landmarks";
  marker.id = landmark_index;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.header.stamp = ::ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.scale.x = kLandmarkMarkerScale;
  marker.scale.y = kLandmarkMarkerScale;
  marker.scale.z = kLandmarkMarkerScale;
  marker.color = ToMessage(cartographer::io::GetColor(landmark_index));
  marker.pose = ToGeometryMsgPose(landmark_pose);
  return marker;
}

void PushAndResetLineMarker(visualization_msgs::Marker* marker,
                            std::vector<visualization_msgs::Marker>* markers) {
  markers->push_back(*marker);
  ++marker->id;
  marker->points.clear();
}

}  // namespace

/**
 * 1. 构造函数。只是为必要的变量提供初值，除此之外没有任何其他的操作。
 * 2. 这个函数的三个输入变量都是 cartographer_node 在系统运行之初构建的。
 *    node_options 是从配置文件中加载的配置项，
 *    map_builder 则是 Cartographer 的地图构建器，
 *    tf_buffer 是 ROS 系统中坐标变换库 tf2 的监听缓存。
 * 3. map_builder_bridge_ 对象是 node 对象的一个成员，在 node 对象的构造函数中构建。
 *    而它的初始化需要在 node 对象调用它的成员函数 AddTrajectory() 之后才能触发。
 */
MapBuilderBridge::MapBuilderBridge(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),           // 初始化 node_options_
      map_builder_(std::move(map_builder)),  // 初始化 map_builder_
      tf_buffer_(tf_buffer) {}               // 初始化 tf_buffer_

// 调用了map_builder_的成员函数LoadState来加载一个.pbstream文件。
// map_builder_是接口MapBuilderInterface的实例化对象，而根据是2d还是3d情况，其具体实现会略有不同。
void MapBuilderBridge::LoadState(const std::string& state_filename,
                                 bool load_frozen_state) {
  // Check if suffix of the state file is ".pbstream".
  // 检查状态文件的后缀是否为“.pbstream”。
  const std::string suffix = ".pbstream";
  CHECK_EQ(state_filename.substr(
               std::max<int>(state_filename.size() - suffix.size(), 0)),
           suffix)
      << "The file containing the state to be loaded must be a "
         ".pbstream file.";
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  cartographer::io::ProtoStreamReader stream(state_filename);
  map_builder_->LoadState(&stream, load_frozen_state);
}

// 开始一条新的轨迹跟踪，并返回新建轨迹的索引
int MapBuilderBridge::AddTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
  /**
   * 1. 通知 map_builder_ 对象添加一个轨迹跟踪器，同时将构建成功的索引返回保存在局部变量 trajectory_id 中。
   * 2. 有三个输入参数：
   *    expected_sensor_ids 订阅的所有传感器主题名称和类型的集合；
   *    trajectory_options.trajectory_builder_options 是轨迹跟踪器的配置信息；
   *    第三个参数比较重要，看字面意思，它相当于注册了一个回调函数 OnLocalSlamResult，
   *    用于响应 map_builder_ 完成一个局部 SLAM 或者说是成功构建了一个子图的事件。
   * 3. std::bind 用来将可调用对象与其参数进行绑定。绑定之后的结果可以使用 std::function 进行保存，
   *     并延迟调用到任何需要的时候。一般来讲，它主要有两大作用：
   *    （1）将可调用对象与其参数一起绑定成为一个仿函数；
   *    （2）将多元可调用对象转换成为1元或是(n-1)元调用对象，既只是绑定部分参数。
   *    实际上 std::bind 的返回类型是一个 std 内部定义的仿函数类型，在这里就只需要知道它是一个仿函数，
   *    可以赋值给一个 std::function，这里直接用 std::function 类型来保存 std::bind 的返回值也是可以的。
   *    其中 std::placeholders::_1 是一个占位符，代表这个位置将在函数调用时，被传入的第一个参数代替。
   * 4. 所以 map_builder_->AddTrajectoryBuilder 这个函数的第三个参数是一个 std:function 型的。
   *    这样，在 map_builder_->AddTrajectoryBuilder 内部可以通过如下方式调用 MapBuilderBridge::OnLocalSlamResult
   *    call_func(para1,para2,..., std::function);
   */
  const int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options,
      ::std::bind(&MapBuilderBridge::OnLocalSlamResult, this,
                  ::std::placeholders::_1, ::std::placeholders::_2,
                  ::std::placeholders::_3, ::std::placeholders::_4,
                  ::std::placeholders::_5));
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  // Make sure there is no trajectory with 'trajectory_id' yet.
  // 检查 trajectory_id 确保之前没有使用过
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  // 为 trajectory_id 构建一个 SensorBridge 对象。
  // SensorBridge 是 cartographer_ros 中对于传感器的一个封装，用于将 ROS 的消息转换成 Cartographer 中的传感器数据类型。
  sensor_bridges_[trajectory_id] =
      cartographer::common::make_unique<SensorBridge>(
          trajectory_options.num_subdivisions_per_laser_scan,
          trajectory_options.tracking_frame,
          node_options_.lookup_transform_timeout_sec, tf_buffer_,
          map_builder_->GetTrajectoryBuilder(trajectory_id));
  // 将轨迹相关的配置保存到容器对象 trajectory_options_ 中并检查
  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;  // 返回刚刚生成的索引 trajectory_id
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 1);
  map_builder_->FinishTrajectory(trajectory_id);
  sensor_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::RunFinalOptimization() {
  LOG(INFO) << "Running final trajectory optimization...";
  map_builder_->pose_graph()->RunFinalOptimization();
}

bool MapBuilderBridge::SerializeState(const std::string& filename) {
  cartographer::io::ProtoStreamWriter writer(filename);
  map_builder_->SerializeState(&writer);
  return writer.Close();
}

/*
 * （1）处理submap查询的，在cartographer_node中被kSubmapQueryServiceName这个Service调用。
 *     这里基本上就是利用/src/cartographer/cartographer/mapping下的工具来查询指定id的submap的一些信息。
 * （2）proto是Google提供的一个ProtoBuf库的工具Google Protobuf库，用来实现数据的序列化和反序列化。
 * （3）整体上而言，当请求一个查询submap的service时，程序是在这里处理，能够返回一个submap的相关信息。
 *     最终的处理还是要靠cartographer这个包里面的东西处理，cartographer_ros只是做了一层ROS封装也在这里体现了出来。
 */
void MapBuilderBridge::HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response) {
  // response_proto 是一个 protobuf 数据类型，这里通过这种类型的数据传输 submap 信息。
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  // submap_id 的数据类型为一个结构体，对 submap 的 id 进行标识，这是通过 trajectory ID 和 submap 的索引来共同标识的。
  cartographer::mapping::SubmapId submap_id{request.trajectory_id,
                                            request.submap_index};
  // 调用map_builder_->SubmapToProto(submap_id, &response_proto)这个函数查询信息，结果放到response_proto这个变量中。
  // 如果返回的 error 信息为空，则表示查询 submap 成功；如果返回信息不为空，则表示查询出错，找不到查询的 submap。
  const std::string error =
      map_builder_->SubmapToProto(submap_id, &response_proto);
  if (!error.empty()) {
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    response.status.message = error;
    return;
  }

  response.submap_version = response_proto.submap_version();
  for (const auto& texture_proto : response_proto.textures()) {
    response.textures.emplace_back();
    auto& texture = response.textures.back();
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    // ToGeometryMsgPose() 函数把 transform::Rigid3d 类型的数据转换成 geometry_msgs::Pose 类型。
    // ToRigid3() 函数把 proto::Rigid3d 类型的数据转换成 transform::Rigid3d 类型。
    texture.slice_pose = ToGeometryMsgPose(
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  }
  response.status.message = "Success.";
  response.status.code = cartographer_ros_msgs::StatusCode::OK;
}

// GetFrozenTrajectoryIds这个函数从名字看是冻结一个trajectory.不太明白具体是什么意思，
// 个人猜测是一条trajectory已经构建完毕，不再继续扩展之后是不是就把其Frozen并存起来。
std::set<int> MapBuilderBridge::GetFrozenTrajectoryIds() {
  std::set<int> frozen_trajectory_ids;
  // 调用map_builder_->pose_graph()->GetTrajectoryNodePoses()来处理。
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  for (const int trajectory_id : node_poses.trajectory_ids()) {
    // 调用map_builder_->pose_graph()->IsTrajectoryFrozen(trajectory_id)来处理。
    if (map_builder_->pose_graph()->IsTrajectoryFrozen(trajectory_id)) {
      frozen_trajectory_ids.insert(trajectory_id);
    }
  }
  return frozen_trajectory_ids;
}

/*
 * GetSubmapList() 函数主要用来获取 Submap 的列表。
 * 它是在往 kSubmapListTopic = "submap_list" 这个 Topic 上发布数据时，被 Node::PublishSubmapList 调用的，
 * 发布的 submap 列表信息在节点 cartographer_occupancy_grid_node 上进行接受。
 */
cartographer_ros_msgs::SubmapList MapBuilderBridge::GetSubmapList() {
  cartographer_ros_msgs::SubmapList submap_list;
  submap_list.header.stamp = ::ros::Time::now();
  submap_list.header.frame_id = node_options_.map_frame;
  // 通过调用 map_builder_->pose_graph()->GetAllSubmapPoses() 来获取列表信息
  for (const auto& submap_id_pose :
       map_builder_->pose_graph()->GetAllSubmapPoses()) {
    cartographer_ros_msgs::SubmapEntry submap_entry;
    submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
    submap_entry.submap_index = submap_id_pose.id.submap_index;
    submap_entry.submap_version = submap_id_pose.data.version;
    submap_entry.pose = ToGeometryMsgPose(submap_id_pose.data.pose);
    submap_list.submap.push_back(submap_entry);
  }
  return submap_list;
}

/*
 * GetTrajectoryStates 函数用来返回一个 TrajectoryStates 变量组成的 unordered_map 这个容器。
 * 它是在往 kScanMatchedPointCloudTopic[] = "scan_matched_points2" 这个 Topic 上发布数据时，
 * 被 Node::PublishTrajectoryStates 调用的。发布的 topic 是在 rviz 上看到的绿色的雷达信息。
 * 这里稍微注意一下，Unordered Map 跟我们程序里 MapBuilder 里的 map 不是同一个概念。
 * Unordered Map 只是 c++ 中的一种 container。
 */
std::unordered_map<int, MapBuilderBridge::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
  std::unordered_map<int, TrajectoryState> trajectory_states;  // 变量用来存返回结果
  // 一个循环，依次取出来 TrajectoryState；这里以 SensorBridge 为索引来取，还不知道为什么
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;

    // TrajectoryState 结构体中的第一个成员 local_slam_data，注意这是一个智能指针
    std::shared_ptr<const TrajectoryState::LocalSlamData> local_slam_data;
    {
      cartographer::common::MutexLocker lock(&mutex_);
      // 检查 trajectory_state_data_ 容器中时候是否存在 trajectory_id 的路径
      if (trajectory_state_data_.count(trajectory_id) == 0) {  // 不存在直接跳过这次循环
        continue;
      }
      // 存在则获取路径 trajectory_id 的 local_slam_data 数据
      local_slam_data = trajectory_state_data_.at(trajectory_id);
    }

    // Make sure there is a trajectory with 'trajectory_id'.
    CHECK_EQ(trajectory_options_.count(trajectory_id), 1);
    /*
     * 第 trajectory_id 个 TrajectoryState 存入返回变量中：
     * （1）第一个成员变量 std::shared_ptr<const LocalSlamData> local_slam_data 为 local SLAM 的数据，
     *     是通过上面的 trajectory_state_data_.at(trajectory_id) 获取的。
     * （2）第二个成员变量 cartographer::transform::Rigid3d local_to_map 为 submap 到 global map 的坐标变换关系，
     *     是通过 map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) 获取的。
     * （3）第三个成员变量 std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking
     *     是通过 sensor_bridge.tf_bridge().LookupToTracking() 获取的。
     * （4）第四个成员变量 TrajectoryOptions trajectory_options 为路径配置参数，
     *     是通过 trajectory_options_[trajectory_id] 获取的。
     */
    trajectory_states[trajectory_id] = {
        local_slam_data,
        map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id),
        sensor_bridge.tf_bridge().LookupToTracking(
            local_slam_data->time,
            trajectory_options_[trajectory_id].published_frame),
        trajectory_options_[trajectory_id]};
  }
  return trajectory_states;
}

visualization_msgs::MarkerArray MapBuilderBridge::GetTrajectoryNodeList() {
  visualization_msgs::MarkerArray trajectory_node_list;
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  // Find the last node indices for each trajectory that have either
  // inter-submap or inter-trajectory constraints.
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_submap_constrained_node;
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_trajectory_constrained_node;
  for (const int trajectory_id : node_poses.trajectory_ids()) {
    trajectory_to_last_inter_submap_constrained_node[trajectory_id] = 0;
    trajectory_to_last_inter_trajectory_constrained_node[trajectory_id] = 0;
  }
  const auto constraints = map_builder_->pose_graph()->constraints();
  for (const auto& constraint : constraints) {
    if (constraint.tag ==
        cartographer::mapping::PoseGraph::Constraint::INTER_SUBMAP) {
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        trajectory_to_last_inter_submap_constrained_node[constraint.node_id
                                                             .trajectory_id] =
            std::max(trajectory_to_last_inter_submap_constrained_node.at(
                         constraint.node_id.trajectory_id),
                     constraint.node_id.node_index);
      } else {
        trajectory_to_last_inter_trajectory_constrained_node
            [constraint.node_id.trajectory_id] =
                std::max(trajectory_to_last_inter_submap_constrained_node.at(
                             constraint.node_id.trajectory_id),
                         constraint.node_id.node_index);
      }
    }
  }

  for (const int trajectory_id : node_poses.trajectory_ids()) {
    visualization_msgs::Marker marker =
        CreateTrajectoryMarker(trajectory_id, node_options_.map_frame);
    int last_inter_submap_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_submap_constrained_node.at(trajectory_id));
    int last_inter_trajectory_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_trajectory_constrained_node.at(trajectory_id));
    last_inter_submap_constrained_node =
        std::max(last_inter_submap_constrained_node,
                 last_inter_trajectory_constrained_node);

    if (map_builder_->pose_graph()->IsTrajectoryFrozen(trajectory_id)) {
      last_inter_submap_constrained_node =
          (--node_poses.trajectory(trajectory_id).end())->id.node_index;
      last_inter_trajectory_constrained_node =
          last_inter_submap_constrained_node;
    }

    marker.color.a = 1.0;
    for (const auto& node_id_data : node_poses.trajectory(trajectory_id)) {
      if (!node_id_data.data.constant_pose_data.has_value()) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        continue;
      }
      const ::geometry_msgs::Point node_point =
          ToGeometryMsgPoint(node_id_data.data.global_pose.translation());
      marker.points.push_back(node_point);

      if (node_id_data.id.node_index ==
          last_inter_trajectory_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.5;
      }
      if (node_id_data.id.node_index == last_inter_submap_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.25;
      }
      // Work around the 16384 point limit in RViz by splitting the
      // trajectory into multiple markers.
      if (marker.points.size() == 16384) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        // Push back the last point, so the two markers appear connected.
        marker.points.push_back(node_point);
      }
    }
    PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
    size_t current_last_marker_id = static_cast<size_t>(marker.id - 1);
    if (trajectory_to_highest_marker_id_.count(trajectory_id) == 0) {
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    } else {
      marker.action = visualization_msgs::Marker::DELETE;
      while (static_cast<size_t>(marker.id) <=
             trajectory_to_highest_marker_id_[trajectory_id]) {
        trajectory_node_list.markers.push_back(marker);
        ++marker.id;
      }
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    }
  }
  return trajectory_node_list;
}

// 获取landmark的pose列表
visualization_msgs::MarkerArray MapBuilderBridge::GetLandmarkPosesList() {
  visualization_msgs::MarkerArray landmark_poses_list;
  const std::map<std::string, Rigid3d> landmark_poses =
      map_builder_->pose_graph()->GetLandmarkPoses();
  for (const auto& id_to_pose : landmark_poses) {
    landmark_poses_list.markers.push_back(CreateLandmarkMarker(
        GetLandmarkIndex(id_to_pose.first, &landmark_to_index_),
        id_to_pose.second, node_options_.map_frame));
  }
  return landmark_poses_list;
}

visualization_msgs::MarkerArray MapBuilderBridge::GetConstraintList() {
  visualization_msgs::MarkerArray constraint_list;
  int marker_id = 0;
  visualization_msgs::Marker constraint_intra_marker;
  constraint_intra_marker.id = marker_id++;
  constraint_intra_marker.ns = "Intra constraints";
  constraint_intra_marker.type = visualization_msgs::Marker::LINE_LIST;
  constraint_intra_marker.header.stamp = ros::Time::now();
  constraint_intra_marker.header.frame_id = node_options_.map_frame;
  constraint_intra_marker.scale.x = kConstraintMarkerScale;
  constraint_intra_marker.pose.orientation.w = 1.0;

  visualization_msgs::Marker residual_intra_marker = constraint_intra_marker;
  residual_intra_marker.id = marker_id++;
  residual_intra_marker.ns = "Intra residuals";
  // This and other markers which are less numerous are set to be slightly
  // above the intra constraints marker in order to ensure that they are
  // visible.
  residual_intra_marker.pose.position.z = 0.1;

  visualization_msgs::Marker constraint_inter_same_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_same_trajectory_marker.id = marker_id++;
  constraint_inter_same_trajectory_marker.ns =
      "Inter constraints, same trajectory";
  constraint_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::Marker residual_inter_same_trajectory_marker =
      constraint_intra_marker;
  residual_inter_same_trajectory_marker.id = marker_id++;
  residual_inter_same_trajectory_marker.ns = "Inter residuals, same trajectory";
  residual_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::Marker constraint_inter_diff_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_diff_trajectory_marker.id = marker_id++;
  constraint_inter_diff_trajectory_marker.ns =
      "Inter constraints, different trajectories";
  constraint_inter_diff_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::Marker residual_inter_diff_trajectory_marker =
      constraint_intra_marker;
  residual_inter_diff_trajectory_marker.id = marker_id++;
  residual_inter_diff_trajectory_marker.ns =
      "Inter residuals, different trajectories";
  residual_inter_diff_trajectory_marker.pose.position.z = 0.1;

  const auto trajectory_node_poses =
      map_builder_->pose_graph()->GetTrajectoryNodePoses();
  const auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  const auto constraints = map_builder_->pose_graph()->constraints();

  for (const auto& constraint : constraints) {
    visualization_msgs::Marker *constraint_marker, *residual_marker;
    std_msgs::ColorRGBA color_constraint, color_residual;
    if (constraint.tag ==
        cartographer::mapping::PoseGraph::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;
      // Color mapping for submaps of various trajectories - add trajectory id
      // to ensure different starting colors. Also add a fixed offset of 25
      // to avoid having identical colors as trajectories.
      color_constraint = ToMessage(
          cartographer::io::GetColor(constraint.submap_id.submap_index +
                                     constraint.submap_id.trajectory_id + 25));
      color_residual.a = 1.0;
      color_residual.r = 1.0;
    } else {
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        constraint_marker = &constraint_inter_same_trajectory_marker;
        residual_marker = &residual_inter_same_trajectory_marker;
        // Bright yellow
        color_constraint.a = 1.0;
        color_constraint.r = color_constraint.g = 1.0;
      } else {
        constraint_marker = &constraint_inter_diff_trajectory_marker;
        residual_marker = &residual_inter_diff_trajectory_marker;
        // Bright orange
        color_constraint.a = 1.0;
        color_constraint.r = 1.0;
        color_constraint.g = 165. / 255.;
      }
      // Bright cyan
      color_residual.a = 1.0;
      color_residual.b = color_residual.g = 1.0;
    }

    for (int i = 0; i < 2; ++i) {
      constraint_marker->colors.push_back(color_constraint);
      residual_marker->colors.push_back(color_residual);
    }

    const auto submap_it = submap_poses.find(constraint.submap_id);
    if (submap_it == submap_poses.end()) {
      continue;
    }
    const auto& submap_pose = submap_it->data.pose;
    const auto node_it = trajectory_node_poses.find(constraint.node_id);
    if (node_it == trajectory_node_poses.end()) {
      continue;
    }
    const auto& trajectory_node_pose = node_it->data.global_pose;
    const Rigid3d constraint_pose = submap_pose * constraint.pose.zbar_ij;

    constraint_marker->points.push_back(
        ToGeometryMsgPoint(submap_pose.translation()));
    constraint_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));

    residual_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));
    residual_marker->points.push_back(
        ToGeometryMsgPoint(trajectory_node_pose.translation()));
  }

  constraint_list.markers.push_back(constraint_intra_marker);
  constraint_list.markers.push_back(residual_intra_marker);
  constraint_list.markers.push_back(constraint_inter_same_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_same_trajectory_marker);
  constraint_list.markers.push_back(constraint_inter_diff_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_diff_trajectory_marker);
  return constraint_list;
}

// 返回 map 容器中 trajectory_id 所对应的 SensorBridge 对象
SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

// OnLocalSlamResult 的目的就是记录下轨迹状态，它有5个参数。前四个参数的意义如注释所示，第五个参数是一个指针似乎没有用到。
// 其中第四个参数的数据类型 RangeData 是 Cartographer 定义的激光雷达传感器测量数据的存储结构，
// 它有三个字段，origin 描述了传感器的坐标，returns 和 misses 都是点云数据，分别表示有物体反射和空闲区域。
void MapBuilderBridge::OnLocalSlamResult(
    const int trajectory_id,                                // 轨迹索引
    const ::cartographer::common::Time time,                // 更新子图的时间
    const Rigid3d local_pose,                               // 子图的参考位置
    ::cartographer::sensor::RangeData range_data_in_local,  // 参考位置下的扫描数据
    const std::unique_ptr<const ::cartographer::mapping::
                              TrajectoryBuilderInterface::InsertionResult>) {
  // 根据输入的参数构建对象 local_slam_data
  // 它的数据类型 LocalSlamData 是定义在 MapBuilderBridge 内部的结构体，用于记录局部 SLAM 反馈的状态。
  std::shared_ptr<const TrajectoryState::LocalSlamData> local_slam_data =
      std::make_shared<TrajectoryState::LocalSlamData>(
          TrajectoryState::LocalSlamData{time, local_pose,
                                         std::move(range_data_in_local)});
  cartographer::common::MutexLocker lock(&mutex_);  // 加锁
  // 把 local_slam_data 写入容器 trajectory_state_data_ 中
  trajectory_state_data_[trajectory_id] = std::move(local_slam_data);
}

}  // namespace cartographer_ros
