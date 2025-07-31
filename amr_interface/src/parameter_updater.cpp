#include "amr_interface/parameter_updater.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>

namespace AMR {

ParameterUpdater::ParameterUpdater() 
  : Node("parameter_updater"), yaml_loaded_(false)
{
  // Declare YAML file parameter.
  this->declare_parameter<std::string>("yaml_file", "");

  if (!this->get_parameter("yaml_file", yaml_file_) || yaml_file_.empty()) {
    // 작업 디렉토리 기준 상대경로 (설치 없이 사용 가능)
    yaml_file_ = "src/redbot_ros2_project/parameter_file/redbot.yaml";
  }

  if (loadYamlFile(yaml_file_)) {
    RCLCPP_INFO(this->get_logger(), "Loaded YAML file successfully: %s", yaml_file_.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", yaml_file_.c_str());
  }
}

bool ParameterUpdater::updateParametersDirectly(const std::string &mode)
{
  if (!yaml_loaded_) {
    if (!loadYamlFile(yaml_file_)) {
      return false;
    }
  }
  return loadParams(mode);
}

void ParameterUpdater::printUpdateResult(const std::string &mode, bool success)
{
  if (success) {
    RCLCPP_INFO(this->get_logger(), "################## Parameters updated successfully for mode: %s #####################", mode.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to update parameters for mode: %s", mode.c_str());
  }
}

bool ParameterUpdater::loadYamlFile(const std::string &file)
{
  try {
    auto start_time = std::chrono::steady_clock::now();
    cached_yaml_ = YAML::LoadFile(file);
    auto end_time = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(this->get_logger(), "YAML file loaded in %.2f ms", elapsed);
    yaml_loaded_ = true;
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "YAML file load error: %s", e.what());
    yaml_loaded_ = false;
    return false;
  }
}

bool ParameterUpdater::loadParams(const std::string &mode)
{
  if (!cached_yaml_[mode]) {
    RCLCPP_ERROR(this->get_logger(), "Mode %s not found in YAML file.", mode.c_str());
    return false;
  }

  YAML::Node section = cached_yaml_[mode];

  // Update controller_server parameters.
  if (section["controller_server"]) {
    YAML::Node ctrlNode = section["controller_server"]["ros__parameters"];
    if (!updateParameterGroup(ctrlNode, "/controller_server/set_parameters", "stopped_goal_checker"))
      return false;
    if (!updateParameterGroup(ctrlNode, "/controller_server/set_parameters", "FollowPath"))
      return false;
  }

  // Update velocity_smoother parameters.
  if (section["velocity_smoother"]) {
    YAML::Node velNode = section["velocity_smoother"]["ros__parameters"];
    if (!updateVectorParameter(velNode, "/velocity_smoother/set_parameters", "max_velocity"))
      return false;
    if (!updateVectorParameter(velNode, "/velocity_smoother/set_parameters", "min_velocity"))
      return false;
    if (!updateVectorParameter(velNode, "/velocity_smoother/set_parameters", "max_accel"))
      return false;
    if (!updateVectorParameter(velNode, "/velocity_smoother/set_parameters", "max_decel"))
      return false;
  }

  // Update local_costmap parameters.
  if (section["local_costmap"]) {
    YAML::Node localNode = section["local_costmap"]["local_costmap"]["ros__parameters"];
    if (!updateBooleanParameters(localNode, "/local_costmap/local_costmap/set_parameters"))
      return false;
  }

  // Update global_costmap parameters.
  if (section["global_costmap"]) {
    YAML::Node globalNode = section["global_costmap"]["global_costmap"]["ros__parameters"];
    if (!updateBooleanParameters(globalNode, "/global_costmap/global_costmap/set_parameters"))
      return false;
  }

  return true;
}

bool ParameterUpdater::updateParameterGroup(const YAML::Node &node, const std::string &service, const std::string &group)
{
  if (node[group]) {
    YAML::Node paramsNode = node[group];
    std::vector<rcl_interfaces::msg::Parameter> params;
    for (auto it = paramsNode.begin(); it != paramsNode.end(); ++it) {
      std::string param_name = group + "." + it->first.as<std::string>();
      // 먼저 double로 변환 시도
      try {
        double value = it->second.as<double>();
        params.push_back(createParameter(param_name, value));
        RCLCPP_INFO(this->get_logger(), "Preparing parameter: %s = %f", param_name.c_str(), value);
      } catch (const std::exception &e) {
        // double 변환 실패 시 string으로 처리
        try {
          std::string value = it->second.as<std::string>();
          params.push_back(createParameter(param_name, value));
          RCLCPP_INFO(this->get_logger(), "Preparing parameter: %s = %s", param_name.c_str(), value.c_str());
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(), "Failed to parse parameter: %s", param_name.c_str());
          continue;
        }
      }
    }
    updateParams(params, service);
  }
  return true;
}

bool ParameterUpdater::updateVectorParameter(const YAML::Node &node, const std::string &service, const std::string &key)
{
  if (node[key]) {
    YAML::Node arrayNode = node[key];
    if (arrayNode.IsSequence() && arrayNode.size() == 3) {
      std::vector<rcl_interfaces::msg::Parameter> params;
      params.push_back(createParameter(key + ".x", arrayNode[0].as<double>()));
      params.push_back(createParameter(key + ".y", arrayNode[1].as<double>()));
      params.push_back(createParameter(key + ".z", arrayNode[2].as<double>()));
      RCLCPP_INFO(this->get_logger(), "Preparing vector parameter: %s = [%f, %f, %f]",
                  key.c_str(), arrayNode[0].as<double>(), arrayNode[1].as<double>(), arrayNode[2].as<double>());
      updateParams(params, service);
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid sequence for %s", key.c_str());
      return false;
    }
  }
  return true;
}

bool ParameterUpdater::updateBooleanParameters(const YAML::Node &node, const std::string &service)
{
  if (!node)
    return true;
  std::vector<rcl_interfaces::msg::Parameter> params;
  for (auto it = node.begin(); it != node.end(); ++it) {
    if (it->second["enabled"]) {
      bool value = it->second["enabled"].as<bool>();
      std::string param_name = it->first.as<std::string>() + ".enabled";
      params.push_back(createParameter(param_name, value));
      RCLCPP_INFO(this->get_logger(), "Preparing boolean parameter: %s = %s", 
                  param_name.c_str(), value ? "true" : "false");
    }
  }
  updateParams(params, service);
  return true;
}

rcl_interfaces::msg::Parameter ParameterUpdater::createParameter(const std::string &name, double value)
{
  rcl_interfaces::msg::Parameter param;
  param.name = name;
  param.value.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  param.value.double_value = value;
  return param;
}

rcl_interfaces::msg::Parameter ParameterUpdater::createParameter(const std::string &name, bool value)
{
  rcl_interfaces::msg::Parameter param;
  param.name = name;
  param.value.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param.value.bool_value = value;
  return param;
}

rcl_interfaces::msg::Parameter ParameterUpdater::createParameter(const std::string &name, const std::string &value)
{
  rcl_interfaces::msg::Parameter param;
  param.name = name;
  param.value.type = rclcpp::ParameterType::PARAMETER_STRING;
  param.value.string_value = value;
  return param;
}

rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr 
ParameterUpdater::getServiceClient(const std::string &service)
{
  if (service_clients_.find(service) == service_clients_.end()) {
    auto client = this->create_client<rcl_interfaces::srv::SetParameters>(service);
    service_clients_[service] = client;
    return client;
  }
  return service_clients_[service];
}

void ParameterUpdater::updateParams(const std::vector<rcl_interfaces::msg::Parameter> &params, const std::string &service)
{
  auto client = getServiceClient(service);
  if (!client->wait_for_service(std::chrono::milliseconds(1000))) {
    RCLCPP_ERROR(this->get_logger(), "Service %s unavailable.", service.c_str());
    return;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters_Request>();
  request->parameters = params;

  auto callback = [this, service](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
    try {
      auto response = future.get();
      bool all_successful = true;
      for (const auto &result : response->results) {
        if (!result.successful) {
          all_successful = false;
          RCLCPP_WARN(this->get_logger(), "A parameter update failed for service %s: %s", service.c_str(), result.reason.c_str());
        }
      }
      if (all_successful) {
        RCLCPP_INFO(this->get_logger(), "Asynchronously updated parameters at %s.", service.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "One or more parameters failed to update for service %s.", service.c_str());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception on async parameter update for service %s: %s", service.c_str(), e.what());
    }
  };

  client->async_send_request(request, callback);
}

} // namespace AMR
