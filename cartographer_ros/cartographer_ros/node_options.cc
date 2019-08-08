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

#include "cartographer_ros/node_options.h"

#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder.h"
#include "glog/logging.h"

namespace cartographer_ros {

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  NodeOptions options;
  // 获取map_builder的构建地图的配置选项，具体的配置文件为
  // "/home/aicrobo/catkin_cartographer/src/cartographer/configuration_files/map_builder.lua"
  options.map_builder_options =
      ::cartographer::mapping::CreateMapBuilderOptions(
          lua_parameter_dictionary->GetDictionary("map_builder").get());
  options.map_frame = lua_parameter_dictionary->GetString("map_frame");
  options.lookup_transform_timeout_sec =
      lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  options.submap_publish_period_sec =
      lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
  options.pose_publish_period_sec =
      lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
  options.trajectory_publish_period_sec =
      lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
  return options;
}

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    // configuration_directory表示配置文件路径名字，configuration_basename表示配置文件名字。
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
  /*
   * （1）file_resolver为一个std::unique_ptr指针，指向的对象类型为cartographer::common::ConfigurationFileResolver，
   *     这个类主要用于解析配置文件。
   * （2）make_unique是smart pointer，创建并返回unique_ptr至指定类型的对象，这里指向的对象类型为
   *     cartographer::common::ConfigurationFileResolver，而对象的构造函数参数为数组
   *     std::vector<std::string>{configuration_directory}，这个类主要用于解析配置文件。
   */
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<std::string>{configuration_directory});
  /*
   * （1）code包含配置文件configuration_basename（例如hokuyo_2d.lua）中的全部内容
   * （2）函数GetFileContentOrDie获取配置文件configuration_basename中的全部内容，存放在code中
   */
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
//   std::cout << "code: " << std::endl << code << std::endl << "end of code" << std::endl;
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
}

}  // namespace cartographer_ros
