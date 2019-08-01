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

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

#include <iostream>        // 输出变量信息进行调试
#include "glog/logging.h"  // glog 头文件

/*
 * 搜索配置文件的第一个目录，第二个是Cartographer安装，以允许包含文件。
 * configuration_directory = /home/aicrobo/catkin_ws/install_isolated/share/cartographer_ros/configuration_files
 * 这是cartographer_ros的lua配置文件的路径，使用前需在变量前面加上FLAGS_。
 */
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
/*
 * cartographer_ros的lua配置文件名，不包含配置文件的任何目录前缀。
 * 例如，configuration_basename = backpack_2d.lua
 */
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
/*
 * 如果非空，则加载.pbstream文件的文件名，包含已保存的SLAM状态。
 */
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
/*
 * 将已保存状态加载为frozen（非优化）轨迹。
 */
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
/*
 * 启用以使用默认主题立即启动第一个轨迹。
 */
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
/*
 * 如果非空，则序列化状态并在关闭之前将其写入磁盘。
 */
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;  // kTfBufferCacheTimeInSeconds表示缓冲区的时间间隔数值
  /*
   * tf2_ros::Buffer是tf2 library的主要工具。Its main public API is defined by tf2_ros::BufferInterface. 
   * ::ros::Duration表示时间间隔
   */
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);  // subscribes to a appropriate topics to receive the transformation.
  /*
   * （1）NodeOptions在/cartographer_ros/cartographer_ros/cartographer/node_options.h中定义；
   *     该struct中包含了对一些基本参数的设置，比如接收tf的timeout时间设置、子图发布周期设置等。
   * （2）TrajectoryOptions在/cartographer_ros/cartographer_ros/cartographer/trajectory_options.h中定义。
   */
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  /*
   * （1）std::tie和std::tuple配合使用，用于解包（unpack），这里表示拆分node_options和trajectory_options。
   * （2）LoadOptions函数把获取到的参数值分别赋给node_options和trajectory_options。
   *     LoadOptions函数在node_options.h中定义。
   */
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  //查看 node_options.map_builder_options.use_trajectory_builder_2d 获取的数值
  ROS_WARN("node_options:");
  std::cout << "node_options.map_builder_options.use_trajectory_builder_2d: " << node_options.map_builder_options.use_trajectory_builder_2d() << std::endl;
  std::cout << "node_options.map_builder_options.use_trajectory_builder_3d: " << node_options.map_builder_options.use_trajectory_builder_3d() << std::endl;
  std::cout << "node_options.map_builder_options.num_background_threads: " << node_options.map_builder_options.num_background_threads() << std::endl;
  std::cout << "node_options.map_frame: " << node_options.map_frame << std::endl;
  std::cout << "node_options.lookup_transform_timeout_sec: " << node_options.lookup_transform_timeout_sec << std::endl;
  std::cout << "node_options.submap_publish_period_sec: " << node_options.submap_publish_period_sec << std::endl;
  std::cout << "node_options.pose_publish_period_sec: " << node_options.pose_publish_period_sec << std::endl;
  std::cout << "node_options.trajectory_publish_period_sec: " << node_options.trajectory_publish_period_sec << std::endl;

  ROS_WARN("trajectory_options:");
  std::cout << "linear_search_window: " << trajectory_options.trajectory_builder_options.trajectory_builder_2d_options().
    real_time_correlative_scan_matcher_options().linear_search_window() << std::endl;
  std::cout << "angular_search_window: " << trajectory_options.trajectory_builder_options.trajectory_builder_2d_options().
    real_time_correlative_scan_matcher_options().angular_search_window() << std::endl;

  /*
   * （1）map_builder为一个std::unique_ptr指针，指向的对象类型为cartographer::mapping::MapBuilder，
   *     而对象的构造函数参数为node_options.map_builder_options，这是一个ProtocolBuffer消息类型。
   *     cartographer::common::make_unique定义在common文件夹下的make_unique.h文件中。
   * （2）node_options.map_builder_options的成员判断是否使用局部SLAM的配置文件以及全局SLAM的配置文件。
   *     例如局部SLAM由use_trajectory_builder_2d和use_trajectory_builder_3d确定，全局SLAM由pose_graph_options确定。
   * （3）auto关键字：auto可以在声明变量的时候根据变量初始值的类型自动为此变量选择匹配的类型，类似的关键字还有decltype。
   *     举个例子：int a = 10;auto au_a = a;  //自动类型推断，au_a为int类型； cout << typeid(au_a).name() << endl;
   */
  auto map_builder =
      cartographer::common::make_unique<cartographer::mapping::MapBuilder>(
          node_options.map_builder_options);
  /*
   * Node在/cartographer_ros/cartographer_ros/cartographer/node.h中定义；
   * 在该构造函数中订阅了很多传感器的topic，收集传感器数据。
   */
  Node node(node_options, std::move(map_builder), &tf_buffer);
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);  // 加载数据包数据
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  /**
   * ros::spin() 将会进入循环，一直调用回调函数chatterCallback()
   */
  ::ros::spin();

  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);                 // 初始化glog
  google::ParseCommandLineFlags(&argc, &argv, true);  // 解析命令行参数，一般都放在main函数中开始位置

  // 访问参数变量，变量前面需要加上前缀 FLAGS_
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  // 不知道为什么这里Google的log信息没法在终端中打印出来
  LOG(INFO) << "configuration_directory: " << FLAGS_configuration_directory << std::endl;
  // 显示输入参数的具体结果
  ROS_WARN("configuration_directory: %s", FLAGS_configuration_directory.c_str());
  ROS_WARN("configuration_basename: %s", FLAGS_configuration_basename.c_str());
  ROS_WARN("load_state_filename: %s", FLAGS_load_state_filename.c_str());
  ROS_WARN("load_frozen_state: %d", FLAGS_load_frozen_state);
  ROS_WARN("start_trajectory_with_default_topics: %d", FLAGS_start_trajectory_with_default_topics);
  ROS_WARN("save_state_filename: %s", FLAGS_save_state_filename.c_str());

  ::ros::init(argc, argv, "cartographer_node");  // 初始化ROS，定义一个节点名为“cartographer_node”
  ::ros::start();  // 启动ROS

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
