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

// cartographer_node 在 main 函数中使用 gflags 解析运行参数，在使用函数 ParseCommandLineFlags 初始化运行参数之前，
// 需要先通过 gflags 的宏来定义参数对象。
/**
 * 搜索配置文件的第一个目录，第二个是 Cartographer 安装，以允许包含文件。
 * 例如，configuration_directory = /home/aicrobo/catkin_ws/install_isolated/share/cartographer_ros/configuration_files
 * 这是 cartographer_ros 的 lua 配置文件的路径，使用前需在变量前面加上 FLAGS_。
 */
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
// cartographer_ros 的 lua 配置文件名，不包含配置文件的任何目录前缀。
// 例如，configuration_basename = backpack_2d.lua
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
// 如果非空，则加载 ".pbstream" 文件的文件名，包含已保存的 SLAM 状态。
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
// 将已保存状态加载为 frozen（非优化）轨迹。
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
// 启用以使用默认主题立即启动第一个轨迹。
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
// 如果非空，则序列化状态并在关闭之前将其写入磁盘。
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  /**
   * 首先使用 tf2 定义了 ROS 的坐标系统监听器，它用于监听 ROS 系统中发布的坐标变换(tf)消息，并将之缓存在 tf_buffer 中。
   * tf 是 ROS 系统中常用的坐标变换库，它通过订阅和发布坐标变换消息来维护系统的坐标系。
   * 目前 ROS 已经不推荐使用tf库了，它们在推广 tf2 的使用，但其基本思想都是一样的，而且同一个系统中还是可以运行两种库的。
   */
  constexpr double kTfBufferCacheTimeInSeconds = 10.;  // kTfBufferCacheTimeInSeconds 描述了缓存的时间长度
  // tf_buffer 缓存了 ROS 系统中发布的坐标变换(tf)消息，::ros::Duration 表示时间间隔
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);  // 使用 tf2 定义了 ROS 的坐标系统监听器 tf
  /**
   * （1）NodeOptions 在 "/cartographer_ros/cartographer_ros/cartographer_ros/node_options.h" 中定义；
   *     该 struct 中包含了对一些基本参数的设置，比如接收 tf 的 timeout 时间设置、子图发布周期设置等。
   * （2）TrajectoryOptions 在 "/cartographer_ros/cartographer_ros/cartographer_ros/trajectory_options.h" 中定义。
   */
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  /**
   * 调用定义在 node_options.cc 中的函数 LoadOptions 加载配置文件。该函数的两个参数分别是配置文件目录和名称的运行参数。
   * std::tie 和 std::tuple 配合使用，用于解包（unpack），这里表示拆分 node_options 和 trajectory_options。
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

  /**
   * （1）构建建图器 map_builder。
   *     map_builder 为一个 std::unique_ptr 指针，指向的对象类型为 cartographer::mapping::MapBuilder，
   *     而对象的构造函数参数为 node_options.map_builder_options，这是一个 ProtocolBuffer 消息类型。
   *     cartographer::common::make_unique 定义在 common 文件夹下的 "make_unique.h" 文件中。
   * （2）node_options.map_builder_options 的成员判断是否使用局部 SLAM 的配置文件以及全局 SLAM 的配置文件。
   *     例如局部 SLAM 由 use_trajectory_builder_2d 和 use_trajectory_builder_3d 确定，全局 SLAM 由 pose_graph_options 确定。
   * （3）auto 关键字：auto 可以在声明变量的时候根据变量初始值的类型自动为此变量选择匹配的类型，类似的关键字还有 decltype。
   *     举个例子：int a = 10;auto au_a = a;  //自动类型推断，au_a为int类型； cout << typeid(au_a).name() << endl;
   */
  auto map_builder =
      cartographer::common::make_unique<cartographer::mapping::MapBuilder>(
          node_options.map_builder_options);
  /**
   * 构建 ROS 封装对象 node。
   * Node 在 "/cartographer_ros/cartographer_ros/cartographer/node.h" 中定义；
   * 在该构造函数中订阅了很多传感器的 topic，收集传感器数据。
   */
  Node node(node_options, std::move(map_builder), &tf_buffer);

  // 如果在运行 cartographer_node 的时候通过命令行参数 load_state_filename 指定了保存有 SLAM 状态的文件，
  // 那么就加载该状态文件继续运行。
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  // 如果在运行 cartographer_node 的时候指定了命令行参数 start_trajectory_with_default_topics 为 true，
  // 则使用默认的主题直接开始第一条轨迹的跟踪
  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  // 开启 ROS 的逻辑循环，node 对象将通过消息回调多线程等机制完成整个定位建图过程。
  // ros::spin() 将会进入循环，一直调用回调函数。
  ::ros::spin();

  // 如果在 ROS 的逻辑循环过程中，触发了退出机制，Run 函数就会接着执行如下的语句，完成最终的路径和地图的优化。
  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  // 最后如果运行参数要求保存系统状态，则将当前的系统状态存到参数 save_state_filename 所指定的文件中。
  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // 初始化谷歌的日志系统 glog
  google::InitGoogleLogging(argv[0]);
  // 通过 gflags 解析运行 cartographer_node 时的运行参数，一般都放在 main 函数中开始位置
  google::ParseCommandLineFlags(&argc, &argv, true);

  // 检查是否在运行参数中指定了配置文件 configuration_basename 和目录 configuration_directory，若没有程序会报错退出。
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
  cartographer_ros::Run();  // 调用函数 Run() 开始进行定位建图
  ::ros::shutdown();        // 如果程序正常退出则关闭 ROS 系统
}
