#include "amr_interface/node_manager.hpp"
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/wait.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstdio>
#include <string>

namespace AMR
{

// 생성자: 프로세스 ID 초기화
NodeManager::NodeManager() : nav_pid_(-1), slam_pid_(-1), auto_slam_pid_(-1)
{
}

NodeManager::~NodeManager()
{
    // 필요시 남아있는 프로세스를 종료하도록 할 수 있음.
}

// launchProcess: 새로운 프로세스 그룹(세션)으로 명령어를 실행하여 해당 프로세스의 PID를 반환
pid_t NodeManager::launchProcess(const std::string &command)
{
    pid_t pid = fork();
    if (pid == 0) // 자식 프로세스
    {
        // 새로운 세션을 시작하여 프로세스 그룹을 만듦
        setsid();
        // /bin/sh 셸을 통해 명령어 실행
        execl("/bin/sh", "sh", "-c", command.c_str(), (char*)NULL);
        // execl 실패 시
        std::cerr << "Failed to execute command: " << command << std::endl;
        exit(EXIT_FAILURE);
    }
    else if (pid > 0)
    {
        // 부모 프로세스: 자식의 pid 반환
        return pid;
    }
    else
    {
        // fork 실패
        std::cerr << "fork() failed for command: " << command << std::endl;
        return -1;
    }
}

// stopProcessGroup: 해당 PID가 속한 프로세스 그룹 전체에 종료 신호를 보냄
bool NodeManager::stopProcessGroup(pid_t pid)
{
    if (pid <= 0)
    {
        return true;
    }
    // 프로세스 그룹 전체에 SIGTERM 신호 전송 (-pid로 전달)
    if (kill(-pid, SIGTERM) != 0)
    {
        std::cerr << "Failed to send SIGTERM to process group with pid " << pid << std::endl;
    }
    // graceful shutdown을 위한 대기
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 아직 살아있는 프로세스가 있으면 SIGKILL 전송
    if (kill(-pid, 0) == 0)
    {
        if (kill(-pid, SIGKILL) != 0)
        {
            std::cerr << "Failed to send SIGKILL to process group with pid " << pid << std::endl;
            return false;
        }
    }
    return true;
}

void NodeManager::runNavigation(std::string map_file)
{
    #if SIMULATION
    std::string command = "ros2 launch amr_navigation navigation_robot_simulator.launch.py map:=" + map_file + ".yaml use_sim_time:=True";
    #else 
    std::string command = "ros2 launch amr_navigation navigation_redbot.launch.py map:=" + map_file + ".yaml";
    #endif
    // 새로운 프로세스 그룹으로 launch 명령어 실행 후 PID 저장
    nav_pid_ = launchProcess(command);
}

void NodeManager::runSLAM()
{
    #if SIMULATION
    std::string command = "ros2 launch amr_cartographer amr_cartographer.launch.py use_sim_time:=True";
    #else
    std::string command = "ros2 launch amr_cartographer amr_cartographer.launch.py";
    #endif
    slam_pid_ = launchProcess(command);
}

void NodeManager::runAutoSLAM()
{
    #if SIMULATION
    std::string command = "ros2 launch airbot_explore explore_simulation.launch.py";
    #else
    std::string command = "ros2 launch airbot_explore explore_all.launch.py";
    #endif
    auto_slam_pid_ = launchProcess(command);
}

bool NodeManager::pauseNavigation()
{
    auto temp_node = rclcpp::Node::make_shared("temp_lifecycle_client");
    auto lifecycle_client = temp_node->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");

    RCLCPP_INFO(temp_node->get_logger(), "Waiting for navigation lifecycle manager service...");
    if (!lifecycle_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(temp_node->get_logger(), "Navigation lifecycle manager service not available.");
        return false;
    }

    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = 1;   // pause

    auto future = lifecycle_client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        RCLCPP_ERROR(temp_node->get_logger(), "Lifecycle manager service call timed out.");
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(temp_node->get_logger(), "Failed to pause navigation lifecycle nodes.");
        return false;
    }
    RCLCPP_INFO(temp_node->get_logger(), "Navigation lifecycle nodes paused successfully.");
    return true;
}

bool NodeManager::resumeNavigation()
{
    auto temp_node = rclcpp::Node::make_shared("temp_lifecycle_client");
    auto lifecycle_client = temp_node->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");

    RCLCPP_INFO(temp_node->get_logger(), "Waiting for navigation lifecycle manager service...");
    if (!lifecycle_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(temp_node->get_logger(), "Navigation lifecycle manager service not available.");
        return false;
    }

    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request->command = 2;   // resume

    auto future = lifecycle_client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        RCLCPP_ERROR(temp_node->get_logger(), "Lifecycle manager service call timed out.");
        return false;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(temp_node->get_logger(), "Failed to resume navigation lifecycle nodes.");
        return false;
    }
    RCLCPP_INFO(temp_node->get_logger(), "Navigation lifecycle nodes resumed successfully.");
    return true;
}

bool NodeManager::updateMapServer(const std::string & new_map_file)
{
  // 임시 노드를 생성하여 서비스 클라이언트를 생성합니다.
  auto temp_node = rclcpp::Node::make_shared("temp_map_server_client");
  auto client = temp_node->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

  RCLCPP_INFO(temp_node->get_logger(), "Waiting for map_server load_map service...");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(temp_node->get_logger(), "Map server load_map service not available.");
    return false;
  }

  // LoadMap 요청을 생성하고, 새 맵 파일 경로를 설정합니다.
  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = new_map_file;  // 새 맵 파일 경로

  // 서비스 호출 및 응답 대기
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_WARN(temp_node->get_logger(), "Map server load_map request timed out.");
    return false;
  }

  try {
    auto response = future.get();
    if (response->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
      RCLCPP_ERROR(temp_node->get_logger(), "Failed to load map: %s", new_map_file.c_str());
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(temp_node->get_logger(), "Exception during map load: %s", e.what());
    return false;
  }

  RCLCPP_INFO(temp_node->get_logger(), "Map server loaded new map: %s", new_map_file.c_str());
  return true;
}


bool NodeManager::stopNavigation()
{
    bool success = stopProcessGroup(nav_pid_);
    nav_pid_ = -1;
    std::this_thread::sleep_for(std::chrono::seconds(15)); 
    return success;
}



bool NodeManager::stopSLAM()
{
    bool success = stopProcessGroup(slam_pid_);
    slam_pid_ = -1;
    std::this_thread::sleep_for(std::chrono::seconds(15)); 
    return success;
}

bool NodeManager::stopAutoSLAM()
{
    bool success = stopProcessGroup(auto_slam_pid_);
    auto_slam_pid_ = -1;
    std::this_thread::sleep_for(std::chrono::seconds(15)); 
    return success;
}

void NodeManager::saveMap(std::string map_name)
{
    std::string command = "ros2 run nav2_map_server map_saver_cli -f " + map_name;
    std::system(command.c_str());
}

} // namespace AMR
