#include "amr_interface/execution.hpp"

using namespace std::chrono_literals;

namespace AMR {

//----------------------------
// StateExecutor Implementation
//----------------------------

// Store the latest navigation and docking goal handles for cancellation
rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav_goal_handle_ = nullptr;
rclcpp_action::ClientGoalHandle<san_msgs::action::MarkerDocking>::SharedPtr docking_goal_handle_ = nullptr;

rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_goal_handle_ = nullptr;
rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_goal_handle_ = nullptr;

/**
 * @brief Handles the INIT state. Checks sensors, loads parameters, and initializes the map if available.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::onInit(Interface* interface)
{
    RCLCPP_INFO(interface->get_logger(), "Executing INIT: Checking sensors and loading parameters...");
    
    // if (!checkOdometry(interface)) {
    //     return;
    // }

    // Manual Mode Triggered
    if(interface->behavior_manual_mode_triggered_){
        interface->setState(RobotState::MANUAL);
    }

    handleResetButton(interface);
    handleMapInitialization(interface);
}

bool StateExecutor::checkOdometry(Interface* interface)
{
    if (!interface->odometry_subscribed_) {
        return false;
    }
    return true;
}

void StateExecutor::handleResetButton(Interface* interface)
{
    if (interface->reset_button_triggered_) {
        interface->reset_button_triggered_ = false;
    }
}

void StateExecutor::handleMapInitialization(Interface* interface)
{
    YAML::Node config = YAML::LoadFile(interface->webserver_config_file_);
    
    if (!config["currentMapNode"] || !config["currentMapNode"]["currentMap"]) {
        RCLCPP_INFO(interface->get_logger(), "No current map configuration found");
        return;
    }

    if (config["currentMapNode"]["currentMap"].IsNull()) {
        RCLCPP_INFO(interface->get_logger(), "Current map configuration is null");
        return;
    }

    current_map_file = config["currentMapNode"]["currentMap"].as<std::string>("");
    if (current_map_file.empty()) {
        RCLCPP_INFO(interface->get_logger(), "Current map filename is empty");
        return;
    }

    RCLCPP_INFO(interface->get_logger(), "Initializing map: %s", current_map_file.c_str());
    current_map_checker_ = true;

    if (shouldStartNavigation(interface)) {
        startNavigation(interface);
    }

    if (current_map_checker_) {
        interface->setState(RobotState::IDLE);
    }
}

bool StateExecutor::shouldStartNavigation(Interface* interface)
{
    return interface->map_manager_->checkMap(current_map_file) && 
           !interface->running_navigation_;
}

void StateExecutor::startNavigation(Interface* interface)
{
    std::string map_file = interface->map_manager_->getDir() + "/" + current_map_file;
    RCLCPP_INFO(interface->get_logger(), "Starting navigation with map: %s", map_file.c_str());
    
    std::thread command_thread(&NodeManager::runNavigation, interface->node_manager_, map_file);
    command_thread.detach();
    std::this_thread::sleep_for(10000ms);    // 10초 대기
    interface->running_navigation_ = true;

    // callReinitializeGlobalLocalization(interface);
}

void StateExecutor::callReinitializeGlobalLocalization(Interface* interface) {
    auto client = interface->create_client<std_srvs::srv::Empty>("/reinitialize_global_localization");
    for (int i = 0; i < 5; ++i) {
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(interface->get_logger(), "Service not available, waiting again...");
            continue;
        }
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = client->async_send_request(request);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    RCLCPP_INFO(interface->get_logger(), "Called /reinitialize_global_localization 5 times.");
}

/**
 * @brief Handles the IDLE state. Waits for task commands and processes map or mode change requests.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::onIdle(Interface* interface)
{
    updateParameters(interface, "NORMAL");
    RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, 
                         "Executing IDLE: Waiting for task command...");
    
    
    if (handleMapRequests(interface)) {
        return;
    }

    if(interface->task_move_completed_){
        interface->task_move_completed_ = false;
    }

    if (interface->behavior_charging_redocking_triggered_ || 
        interface->behavior_manipulation_redocking_triggered_) {
        interface->setState(RobotState::UNDOCKING);
    }
    else if (interface->task_home_started_ || 
             interface->task_move_started_ || 
             interface->task_load_started_ || 
             interface->task_unload_started_ ||
             interface->task_one_move_started_) {
        interface->setState(RobotState::AUTO);
    }
    else if (interface->behavior_manual_mode_triggered_) {
        interface->setState(RobotState::MANUAL);
    }

}

bool StateExecutor::handleMapRequests(Interface* interface)
{
    if (handleSaveOrCancelMap(interface)) {
        return true;
    }

    if (handleMapChange(interface)) {
        return true;
    }

    if (interface->build_map_requested_) {
        interface->setState(RobotState::SLAM);
        return true;
    }

    return false;
}

bool StateExecutor::handleSaveOrCancelMap(Interface* interface)
{
    if (!interface->save_map_requested_ && !interface->cancel_map_requested_) {
        return false;
    }

    if (interface->save_map_requested_) {
        interface->save_map_requested_ = false;
        interface->save_map_completed_ = true;
    }

    if (interface->cancel_map_requested_) {
        interface->cancel_map_requested_ = false;
        interface->cancel_map_completed_ = true;
    }

    std::thread command_thread(&NodeManager::resumeNavigation, interface->node_manager_);
    command_thread.detach();
    interface->running_navigation_ = true;
    
    return true;
}

bool StateExecutor::handleMapChange(Interface* interface)
{
    if (!interface->change_current_map_requested_) {
        return false;
    }

    YAML::Node config = YAML::LoadFile(interface->webserver_config_file_);
    if (!config["currentMapNode"] || !config["currentMapNode"]["currentMap"]) {
        RCLCPP_INFO(interface->get_logger(), "No current map configuration found");
        return true;
    }

    if (config["currentMapNode"]["currentMap"].IsNull()) {
        RCLCPP_INFO(interface->get_logger(), "Current map configuration is null");
        return true;
    }

    current_map_file = config["currentMapNode"]["currentMap"].as<std::string>("");
    if (current_map_file.empty()) {
        RCLCPP_INFO(interface->get_logger(), "Current map filename is empty");
        return true;
    }

    if (interface->map_manager_->checkMap(current_map_file)) {
        std::string new_map_file = interface->map_manager_->getDir() + "/" + current_map_file + ".yaml";
        RCLCPP_INFO(interface->get_logger(), "Loading new map: %s", new_map_file.c_str());
        
        std::thread command_thread(&NodeManager::updateMapServer, interface->node_manager_, new_map_file);
        command_thread.detach();
        std::this_thread::sleep_for(5000ms);
        interface->running_navigation_ = true;
    }

    interface->change_current_map_requested_ = false;
    return true;
}

/**
 * @brief Handles the SLAM state. Starts, cancels, or saves SLAM (map building) operations.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::onSLAM(Interface* interface)
{
    RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 1000, "Executing SLAM..");

    // Set SLAM behavior started
    if (!interface->behavior_slam_started_) {
        interface->behavior_slam_started_ = true;
        interface->behavior_slam_completed_ = false;
    }

    if (interface->build_map_requested_) {
        handleBuildMap(interface);
    }

    if (interface->cancel_map_requested_ && interface->running_slam_) {
        handleCancelSLAM(interface);
    }

    if (interface->save_map_requested_ && interface->running_slam_) {
        handleSaveMap(interface);
    }
}

void StateExecutor::handleBuildMap(Interface* interface)
{
    if (interface->running_navigation_) {
        RCLCPP_INFO(interface->get_logger(), "----------PAUSE NAVIGATION-----------");
        std::thread command_thread(&NodeManager::pauseNavigation, interface->node_manager_);
        command_thread.detach();
        interface->running_navigation_ = false;
    }

    RCLCPP_INFO(interface->get_logger(), "run SLAM");
    std::thread command_thread(&NodeManager::runSLAM, interface->node_manager_);
    command_thread.detach();
    interface->running_slam_ = true;
    interface->build_map_requested_ = false;
    interface->build_map_completed_ = true;
}

void StateExecutor::handleCancelSLAM(Interface* interface)
{
    RCLCPP_INFO(interface->get_logger(), "stop SLAM");
    while(!interface->node_manager_->stopSLAM() && rclcpp::ok());
    interface->running_slam_ = false;
    interface->cancel_map_completed_ = true;  // Set completion flag
    interface->behavior_slam_started_ = false;  // Reset started flag
    interface->behavior_slam_completed_ = true;  // Set completed flag
    interface->setState(RobotState::IDLE);
}

void StateExecutor::handleSaveMap(Interface* interface)
{
    // std::string map_path = interface->map_manager_->getDir() + "/" + interface->save_map_requested_name_;
    // interface->node_manager_->saveMap(map_path);
    // rclcpp::sleep_for(1s);
    // interface->node_manager_->saveMap(map_path+"_copy");
    // rclcpp::sleep_for(1s);
    // interface->map_manager_->getMapList();

    RCLCPP_INFO(interface->get_logger(), "stop SLAM");
    while(!interface->node_manager_->stopSLAM() && rclcpp::ok());
    interface->running_slam_ = false;
    interface->save_map_completed_ = true;  // Set completion flag
    interface->behavior_slam_started_ = false;  // Reset started flag
    interface->behavior_slam_completed_ = true;  // Set completed flag
    interface->setState(RobotState::IDLE);
}

/**
 * @brief Handles the AUTO state. Sends navigation goals and manages automatic mission execution.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::onAuto(Interface* interface)
{
    RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000,
                        "Executing AUTO: Preparing for task execution and sending goal...");    
    
    interface->behavior_auto_canceled_ = false;

    // Manual Mode Triggered
    if(interface->behavior_manual_mode_triggered_){
        interface->setState(RobotState::MANUAL);
    }

    /////////////REAL NODE BASED ////////////
    if (!interface->navigation_goal_sent_) {

        updateParameters(interface, "AUTO");

        if(interface->task_move_started_ ||
            interface->task_home_started_ ||
            interface->task_load_started_ ||
            interface->task_unload_started_){

            // sendNavigateThroughPoses(interface, interface->current_waypoints_);
            sendWaypointGoalAsync(interface, interface->current_waypoints_);
        }
        else if(interface->task_one_move_started_){

            sendNavigationGoalAsync(interface, interface->goal_position_);
            // sendWaypointGoalAsync(interface, interface->goal_position_);

        }
        // sendWaypointGoalAsync(interface, interface->current_waypoints_);
        interface->navigation_goal_sent_ = true;

    } else {
        RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "Navigation goal already sent. Waiting for result...");
        
        if(interface->distance_in_threshold_ &&
        (interface->task_load_started_ || interface->task_unload_started_
        || interface->task_home_started_)){

            updateParameters(interface, "DOCK");
            interface->distance_in_threshold_ = false;

        }
    
    }
}

/**
 * @brief Handles the MANUAL state. Waits for manual control commands and returns to previous state if needed.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::onManual(Interface* interface)
{
    RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "Executing MANUAL: Awaiting manual control commands...");

    if(interface->behavior_manual_mode_triggered_ == false){

        // Return To Previous State
        resumeRobotstate(interface);

    }

}

/**
 * @brief Handles the DOCKING state. Sends docking goals and manages docking retries.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::onDocking(Interface* interface)
{
    // Docking State
    RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "Executing DOCKING: Attempting to dock...");

    // Set DOCKING behavior started
    if (!interface->behavior_docking_started_) {
        interface->behavior_docking_started_ = true;
        interface->behavior_docking_canceled_ = false;
        interface->behavior_docking_completed_ = false;
    }

    // Manual Mode Triggered
    if(interface->behavior_manual_mode_triggered_){
        interface->setState(RobotState::MANUAL);
    }

    if(interface->behavior_manipulation_redocking_triggered_
        || interface->behavior_charging_redocking_triggered_) {

        if(interface->behavior_manipulation_redocking_triggered_)
        {
            // Send Manipulation ReDocking Action
            if (!interface->docking_goal_sent_) {
                
                // 카메라 앞만 보기 X -> Loading/Unloading도 무조건 뒤로 도킹
                // sendDockingGoalAsync(interface, "front", interface->marker_gap_, interface->marker_id_); // marker_id = 0
                sendDockingGoalAsync(interface, "rear", interface->working_marker_gap_, interface->marker_id_); // marker_id = 0
                interface->docking_goal_sent_ = true;

            } else {

                RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "Manipulation ReDocking goal already sent. Waiting for result...");
            
            }
        }

        else if(interface->behavior_charging_redocking_triggered_)
        {
            // Send Charging ReDocking Action
            if (!interface->docking_goal_sent_) {

                // 카메라 뒤만 보기
                sendDockingGoalAsync(interface, "rear", interface->charging_marker_gap_, interface->marker_id_); // marker_id = 0
                interface->docking_goal_sent_ = true;
    
            } else {
    
                RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "Charging ReDocking goal already sent. Waiting for result...");
            
            }
        }
    }
    
    // Todolist: Charging Requested -> 뒤로 도킹 + Home Aruco Marker Detection (maybe Marker_id = 0) 
    if(interface->task_home_started_) {

        // Send Docking Action
        if (!interface->docking_goal_sent_) {

            // 카메라 뒤만 보기
            sendDockingGoalAsync(interface, "rear", interface->charging_marker_gap_, interface->marker_id_); // marker_id = 0
            interface->docking_goal_sent_ = true;

        } else {

            RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "Docking goal already sent. Waiting for result...");
        
        }

    }

    // Todolist: Working Requested -> 앞으로 도킹 + Each Work Station Aruco Marker Detection (maybe Marker_id = 1 ~)
    if(interface->task_load_started_ || interface->task_unload_started_) {

        // Send Docking Action
        if (!interface->docking_goal_sent_) {
            
            // 카메라 앞만 보기 X -> Loading/Unloading도 무조건 뒤로 도킹
            sendDockingGoalAsync(interface, "rear", interface->working_marker_gap_, interface->marker_id_); // marker_id = 0
            interface->docking_goal_sent_ = true;

        } else {

            RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "Docking goal already sent. Waiting for result...");
        
        }

    }
}

/**
 * @brief Handles the UNDOCKING state. Sends undocking goals and manages state transitions after undocking.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::onUndocking(Interface* interface)
{
    RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "Executing UNDOCKING: Attempting to undock...");
   
    // Set UNDOCKING behavior started
    if (!interface->behavior_undocking_started_) {
        interface->behavior_undocking_started_ = true;
        interface->behavior_undocking_completed_ = false;
    }
   
    // Manual Mode Triggered
    if(interface->behavior_manual_mode_triggered_){
        interface->setState(RobotState::MANUAL);
    }
    
    // (Redocking State) Execute when Redocking Triggered
    if(interface->behavior_manipulation_redocking_triggered_
        || interface->behavior_charging_redocking_triggered_) {

        // Send Only Last Destination of Goal
        if (!interface->navigation_goal_sent_) {

            if(interface->behavior_manipulation_redocking_triggered_)
            {
                updateParameters(interface, "CHARGE_REDOCK");
            }
            if(interface->behavior_charging_redocking_triggered_)
            {
                updateParameters(interface, "CHARGE_REDOCK");
            }

             // Start undocking delay if not already started
            if (!interface->undocking_delay_started_) {
                interface->undocking_delay_started_ = true;
                interface->undocking_delay_counter_ = 0;
                RCLCPP_INFO(interface->get_logger(), "Starting undocking delay (3 seconds)...");
            }
            
            // Wait for 2 seconds (20 * 100ms = 2000ms = 2s)
            if (interface->undocking_delay_counter_ < 20) {
                interface->undocking_delay_counter_++;
                RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 1000, 
                    "Undocking delay: %d/20", interface->undocking_delay_counter_);
                return; // Wait more
            }
            
            // 3 seconds passed, proceed with navigation
            interface->undocking_delay_started_ = false;
            interface->undocking_delay_counter_ = 0;

            interface->set_parameter(rclcpp::Parameter("scan_mode", "all"));

            geometry_msgs::msg::PoseStamped last_goal_pose_;
            last_goal_pose_.header.frame_id = "map";  
            last_goal_pose_.header.stamp = interface->now(); 
            last_goal_pose_.pose = interface->last_waypoints_.poses.back();

            sendNavigationGoalAsync(interface, last_goal_pose_);
            interface->navigation_goal_sent_ = true;

            
        } else {
            RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "Redocking goal already sent. Waiting for result...");
            
            if(interface->distance_in_threshold_){

                updateParameters(interface, "DOCK");
                interface->distance_in_threshold_ = false;

            }
        }
    }

    // Todolist: 충전 완료시 전진으로 빠져야 함
    if((interface->behavior_charge_completed_
        || interface->task_unload_started_
        || interface->task_load_started_
        || interface->task_move_started_
        || interface->build_map_requested_)
        && !interface->navigation_goal_sent_ ){
        
        // 충전 스테이션 Home Position 여부 확인
        if(!interface->battery_data_.charging_jack_state &&
           !interface->motor_battery_charging_triggered_){

            updateParameters(interface, "DOCK");
            
            // Start undocking delay if not already started
            if (!interface->undocking_delay_started_) {
                interface->undocking_delay_started_ = true;
                interface->undocking_delay_counter_ = 0;
                RCLCPP_INFO(interface->get_logger(), "Starting undocking delay (3 seconds)...");
            }
            
            // Wait for 2 seconds (20 * 100ms = 2000ms = 2s)
            if (interface->undocking_delay_counter_ < 20) {
                interface->undocking_delay_counter_++;
                RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 1000, 
                    "Undocking delay: %d/20", interface->undocking_delay_counter_);
                return; // Wait more
            }
            
            // 3 seconds passed, proceed with navigation
            interface->undocking_delay_started_ = false;
            interface->undocking_delay_counter_ = 0;
            
            // Set scan_mode "all"
            interface->set_parameter(rclcpp::Parameter("scan_mode", "all"));

            geometry_msgs::msg::PoseStamped last_goal_pose_;
            last_goal_pose_.header.frame_id = "map";  
            last_goal_pose_.header.stamp = interface->now(); 
            last_goal_pose_.pose = interface->last_waypoints_.poses.back();

            sendNavigationGoalAsync(interface, last_goal_pose_);
            interface->navigation_goal_sent_ = true;
            RCLCPP_INFO(interface->get_logger(), "Undocking delay completed. Sending navigation goal.");
        }
        
    }
    else{

        RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "UNDOCKING goal already sent. Waiting for result...");
    
    }


    // // Todolist: LOAD / UNLOAD 완료시 후진(x) -> 전진으로 빠져야 함
    // if((interface->task_load_started_ || interface->task_unload_started_) 
    //     && !interface->navigation_goal_sent_){

    //     updateParameters(interface, "DOCK");
    //     // std::this_thread::sleep_for(5s);

    //     // Set scan_mode "all"
    //     interface->set_parameter(rclcpp::Parameter("scan_mode", "all"));

    //     geometry_msgs::msg::PoseStamped last_goal_pose_;
    //     last_goal_pose_.header.frame_id = "map";  
    //     last_goal_pose_.header.stamp = interface->now(); 
    //     last_goal_pose_.pose = interface->last_waypoints_.poses.back();

    //     sendNavigationGoalAsync(interface, last_goal_pose_);
    //     interface->navigation_goal_sent_ = true;

    // }
    // else if((interface->task_load_started_ || interface->task_unload_started_) 
    //         && interface->navigation_goal_sent_){
    //     RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "UNDOCKING goal already sent. Waiting for result...");
    // }

}

/**
 * @brief Handles the CHARGING state. Manages charging process and transitions to undocking when charging is complete.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::onCharging(Interface* interface)
{
    // Todolist: 충전 관련 인터페이스 필요!!
    RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "Executing CHARGING: Starting charging process...");
    
    // Set CHARGING behavior started
    if (!interface->behavior_charge_started_) {
        interface->behavior_charge_started_ = true;
        interface->behavior_charge_completed_ = false;
    }

    if(interface->task_home_started_ && 
        interface->battery_data_.charging_state){

        interface->task_home_started_ = false;
        interface->task_home_completed_ = true;
    }
    
    // interface->task_home_completed_ = false;

    // 라이다 앞만 보기
    interface->set_parameter(rclcpp::Parameter("scan_mode", "front"));

    // [Execution] batteryCheck 도중 충전 실패로 redocking이 triggered 되었을 떄
    if(interface->behavior_charging_redocking_triggered_){
        interface->setState(RobotState::EMERGENCY);
    }

    // Battery_Charged_Threshold 충족 & setTask or BuildMap 명령 수신
    if((interface->task_unload_started_
        || interface->task_load_started_
        || interface->task_move_started_
        || interface->build_map_requested_)
        || interface->behavior_charge_completed_){

        // Set CHARGING behavior completed
        interface->behavior_charge_started_ = false;
        interface->battery_data_.is_charging = false;
        // interface->behavior_charge_completed_ = true;
        
        interface->setState(RobotState::UNDOCKING);

    }

}

/**
 * @brief Handles the MANIPULATION state. Manages work tasks and transitions after manipulation is complete.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::onManipulating(Interface* interface)
{
    RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 5000, "Executing MANIPULATION: Performing work task...");

    if(interface->behavior_manipulation_redocking_triggered_){
        interface->setState(RobotState::EMERGENCY);
    }

    // Lidar 뒤만 보기 X -> 뒤로 도킹하기 때문에 Lidar는 앞만 봄
    interface->set_parameter(rclcpp::Parameter("scan_mode", "front"));
  
    // Todolist: 작업 끝난 신호 올 때 -> IDLE && interface->task_load_started_ = false;
    // or 작업이 끝남 & 무언가를 들고 있을 때 -> IDLE && interface->task_load_started_ = true;

    /************************** REAL *************************/
    // Mainpulator로부터 EMERGENCY_STOP이 들어왔을 때
    if(interface->behavior_manipulator_emergency_triggered_)
    {
        interface->setState(RobotState::EMERGENCY);
    }

    // Manipulator로부터 성공 신호 수신시
    if(interface->behavior_manipulator_completed_)
    {
        // Initialize
        interface->behavior_manipulator_completed_ = false;
        
        // LOAD 일 때
        // if(interface->task_load_started_)
        // {
        //     interface->task_load_started_ = false;
        //     interface->task_load_completed_ = true;
        // }
        
        // if(interface->task_unload_started_)
        // {
        //     interface->task_unload_started_ = false;
        //     interface->task_unload_completed_ = true;
        // }

        interface->setState(RobotState::UNDOCKING);
    }
    
    /*********************************************************/
    /************************** TEST *************************/
    // manipulation_count++;
    // if(manipulation_count > 200) {
    //     manipulation_count = 0;
    //     interface->behavior_manipulator_completed_ = true;
    
    //     if(interface->task_load_started_)
    //     {
    //         interface->task_load_started_ = false;
    //         interface->load_completed_ = true;
    //     }
        
    //     if(interface->task_unload_started_)
    //     {
    //         interface->task_unload_started_= false;
    //         interface->task_unload_completed_ = true;
    //     }

    //     interface->setState(RobotState::UNDOCKING);
    // }
    /*********************************************************/

}

/**
 * @brief Handles the EMERGENCY state. Stops all operations and transitions to STOP state.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::onEmergency(Interface* interface)
{
    RCLCPP_ERROR_THROTTLE(interface->get_logger(), *interface->get_clock(), 1000, "Executing EMERGENCY: Stopping all operations immediately!");
    
    if(!interface->emergency_quick_stop_triggered_ &&
       !interface->motor_emergency_triggered_)
    {

        if(interface->behavior_manipulation_redocking_triggered_
        || interface->behavior_charging_redocking_triggered_ ){

            interface->setState(RobotState::IDLE);

        }
        else{

            resumeRobotstate(interface);

        }
    
    }
    
    // interface->setState(RobotState::STOP);
}

/**
 * @brief Handles the STOP state. Halts the robot and manages reset or redocking triggers.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::onStop(Interface* interface)
{
    RCLCPP_WARN_THROTTLE(interface->get_logger(), *interface->get_clock(), 1000, "Executing STOP: Robot is halted.");

    // STOP -> Resume
    if (interface->resume_requested_) {
        interface->resume_requested_ = false;
        resumeRobotstate(interface);
    }
}

/**
 * @brief Resumes the robot state based on the previous state.
 * @param interface Pointer to the main robot interface.
 */
void StateExecutor::resumeRobotstate(Interface* interface)
{
    if(interface->getPreviousState() == RobotState::INIT) {
        interface->setState(RobotState::INIT);
    }
    else if(interface->getPreviousState() == RobotState::IDLE) {
        interface->setState(RobotState::IDLE);
    }
    else if(interface->getPreviousState() == RobotState::AUTO) {
        interface->setState(RobotState::AUTO);
    }
    else if(interface->getPreviousState() == RobotState::MANUAL) {
        interface->setState(RobotState::MANUAL);
    }
    else if(interface->getPreviousState() == RobotState::DOCKING) {
        interface->setState(RobotState::DOCKING);
    }
    else if(interface->getPreviousState() == RobotState::UNDOCKING) {
        interface->setState(RobotState::UNDOCKING);
    }
    else if(interface->getPreviousState() == RobotState::CHARGING) {
        interface->setState(RobotState::CHARGING);
    }
    else if(interface->getPreviousState() == RobotState::MANIPULATION) {
        interface->setState(RobotState::MANIPULATION);
    }
    else if(interface->getPreviousState() == RobotState::SLAM) {
        interface->setState(RobotState::SLAM);
    }
}
/***************************************STATE END*****************************************/

// Action Goal Send Function (Necessary)
/**
 * @brief Sends an asynchronous action goal to a ROS2 action server.
 * @tparam ActionT The action type.
 * @param interface Pointer to the main robot interface.
 * @param action_name Name of the action server.
 * @param fill_goal Lambda to fill the goal message.
 * @param result_callback Lambda to handle the result.
 */
template<typename ActionT>
void StateExecutor::sendGoalAsync(
    Interface* interface,
    const std::string & action_name,
    std::function<void(typename ActionT::Goal&)> fill_goal,
    std::function<void(const typename rclcpp_action::Client<ActionT>::WrappedResult &)> result_callback)
{
    using ClientT = rclcpp_action::Client<ActionT>;
    RCLCPP_INFO(interface->get_logger(), "Creating action client for '%s'", action_name.c_str());
    auto action_client = rclcpp_action::create_client<ActionT>(interface, action_name);

    RCLCPP_INFO(interface->get_logger(), "Waiting for action server for '%s'...", action_name.c_str());
    if (!action_client->wait_for_action_server(10s)) {
        RCLCPP_ERROR(interface->get_logger(), "Action server '%s' not available!", action_name.c_str());
        interface->setState(RobotState::EMERGENCY);
        return;
    }

    // Goal Msg Generation & Fill Goal
    typename ActionT::Goal goal_msg;
    fill_goal(goal_msg);
    RCLCPP_INFO(interface->get_logger(), "Sending goal for '%s'", action_name.c_str());

    typename ClientT::SendGoalOptions options;
    options.goal_response_callback =
      [interface, action_client](auto goal_handle) -> void {
          if (!goal_handle) {
              RCLCPP_ERROR(interface->get_logger(), "Goal was rejected by the server");
          } else {
              RCLCPP_INFO(interface->get_logger(), "Goal accepted by the server");
          }
      };

    options.feedback_callback =
      [interface, action_client](auto, const auto & feedback) -> void {
        (void)feedback;
        //   RCLCPP_INFO(interface->get_logger(), "Feedback received");
      };

    options.result_callback =
      [interface, result_callback, action_client](const typename ClientT::WrappedResult & result) -> void {
          result_callback(result);
      };

    action_client->async_send_goal(goal_msg, options);
}

//----------------------------
// Navigation Action Helper Function
// (Private member function of StateExecutor)
//----------------------------
/**
 * @brief Sends a navigation goal to the NavigateToPose action server and handles retries and state transitions.
 * @param interface Pointer to the main robot interface.
 * @param goal_position The target pose for navigation.
 */
void StateExecutor::sendNavigationGoalAsync(Interface* interface, geometry_msgs::msg::PoseStamped & goal_position)
{
    static int retry_count = 0;
    const int MAX_RETRIES = 3;
    // Assign to member variable for later cancellation
    nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(interface, "navigate_to_pose");
    auto action_client = nav_action_client_;

    if (!action_client->wait_for_action_server(10s)) {
        RCLCPP_ERROR(interface->get_logger(), "Action server 'navigate_to_pose' not available!");
        interface->setState(RobotState::EMERGENCY);
        return;
    }

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = interface->now();
    goal.pose = goal_position;

    RCLCPP_INFO(interface->get_logger(),
                "Navigation goal prepared: position=(%.2f, %.2f, %.2f)",
                goal.pose.pose.position.x,
                goal.pose.pose.position.y,
                goal.pose.pose.position.z);
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;

    options.result_callback = [this, interface](const auto & result) {
        // ... existing result handling ...
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {

            RCLCPP_INFO(interface->get_logger(), "Navigation succeeded");
            interface->navigation_goal_sent_ = false;

            if (interface->getState() == RobotState::UNDOCKING){

                interface->behavior_undocking_started_ = false;
                interface->behavior_undocking_completed_ = true;

                if(interface->behavior_manipulation_redocking_triggered_ || interface->behavior_charging_redocking_triggered_)
                {

                    updateParameters(interface, "DOCK");
                    interface->setState(RobotState::DOCKING);
                    // interface->setState(RobotState::IDLE);

                }
                else {


                    if(interface->getPreviousState() != RobotState::CHARGING){
                        if(interface->task_load_started_){ 
                            interface->task_load_started_ = false; 
                            interface->task_load_completed_ = true;
                        }
                        if(interface->task_unload_started_){ 
                            interface->task_unload_started_ = false; 
                            interface->task_unload_completed_ = true;
                        }
                    }

                    interface->setState(RobotState::IDLE);

                }
            }

            else if(interface->getState() == RobotState::AUTO){

                interface->task_one_move_started_ = false;
                interface->task_one_move_completed_ = true;
                interface->behavior_auto_started_ = false;
                interface->behavior_auto_completed_ = true;

                interface->setState(RobotState::IDLE);

            }

        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {

            RCLCPP_WARN(interface->get_logger(), "Navigation aborted! Retrying...");

            if (retry_count < MAX_RETRIES) {

                retry_count++;
                RCLCPP_WARN(interface->get_logger(), "Retrying navigation... Attempt %d/%d", retry_count, MAX_RETRIES);
                sendNavigationGoalAsync(interface, interface->goal_position_);

            } 
            else {

                RCLCPP_ERROR(interface->get_logger(), "Navigation failed after %d attempts. Switching to EMERGENCY mode.", MAX_RETRIES);
                interface->setState(RobotState::EMERGENCY);
                retry_count = 0;

            }
        } 
        else {

            RCLCPP_ERROR(interface->get_logger(), "Navigation failed with code: %d",
                         static_cast<int>(result.code));

            interface->setState(RobotState::EMERGENCY);

        }
    };

    auto future_handle = action_client->async_send_goal(goal, options);
    std::thread([this, future_handle = std::move(future_handle)]() mutable {
        nav_goal_handle_ = future_handle.get();
    }).detach();
}

void StateExecutor::sendWaypointGoalAsync(Interface* interface, const geometry_msgs::msg::PoseArray & waypoint_list)
{
    static size_t current_waypoint_index = 0;
    
    // Create action client for FollowWaypoints
    follow_waypoints_action_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(interface, "follow_waypoints");
    auto action_client = follow_waypoints_action_client_;
    
    if (!action_client->wait_for_action_server(10s)) {
        RCLCPP_ERROR(interface->get_logger(), "Action server 'follow_waypoints' not available!");
        interface->setState(RobotState::EMERGENCY);
        return;
    }
    
    nav2_msgs::action::FollowWaypoints::Goal goal;
    goal.poses.clear();

    for (size_t i = current_waypoint_index; i < waypoint_list.poses.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = interface->now();
        pose_stamped.pose = waypoint_list.poses[i];
        goal.poses.push_back(pose_stamped);
    }

    RCLCPP_INFO(interface->get_logger(), "FollowWaypoints goal prepared with %zu poses", goal.poses.size());
    
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions options;
    
    options.goal_response_callback = [this, interface](auto goal_handle) -> void {
        if (!goal_handle) {
            RCLCPP_ERROR(interface->get_logger(), "FollowWaypoints goal was rejected by server");
            interface->setState(RobotState::EMERGENCY);
        } else {
            RCLCPP_INFO(interface->get_logger(), "FollowWaypoints goal accepted by server");
        }
    };
    
    options.feedback_callback = [this, interface](auto, const auto & feedback) -> void {
        RCLCPP_DEBUG(interface->get_logger(), "FollowWaypoints feedback: current_waypoint = %d", feedback->current_waypoint);
    };
    
    options.result_callback = [this, interface, waypoint_list](const auto & result) mutable {
        
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(interface->get_logger(), "Navigation succeeded");

                // Save Last Waypoints
                interface->last_waypoints_.poses.clear();
                interface->last_waypoints_.header = waypoint_list.header;
                interface->last_waypoints_.poses.push_back(waypoint_list.poses.back());
                RCLCPP_DEBUG(interface->get_logger(), "last_waypoints_ updated with only the last pose");
            

                // State IDLE
                interface->navigation_goal_sent_ = false;

                // Set AUTO behavior completed
                interface->behavior_auto_started_ = false;
                interface->behavior_auto_completed_ = true;

                if (interface->task_load_started_ || 
                    interface->task_unload_started_ || 
                    interface->task_home_started_) {  

                    interface->setState(RobotState::DOCKING);

                } else if(interface->task_move_started_){

                    interface->task_move_started_ = false;
                    interface->task_move_completed_ = true;
                    interface->setState(RobotState::IDLE);

                }
            }
    };
    auto future_handle = action_client->async_send_goal(goal, options);
    std::thread([this, future_handle = std::move(future_handle)]() mutable {
        follow_waypoints_goal_handle_ = future_handle.get();
    }).detach();
}

/**
 * @brief Sends a sequence of waypoints to the NavigateThroughPoses action server.
 * @param interface Pointer to the main robot interface.
 * @param waypoint_list List of waypoints to navigate through.
 */
void StateExecutor::sendNavigateThroughPoses(Interface* interface, const geometry_msgs::msg::PoseArray & waypoint_list)
{
    static size_t current_waypoint_index = 0;
    nav_through_poses_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(interface, "navigate_through_poses");
    auto action_client = nav_through_poses_action_client_;
    if (!action_client->wait_for_action_server(10s)) {
        RCLCPP_ERROR(interface->get_logger(), "Action server 'navigate_through_poses' not available!");
        interface->setState(RobotState::EMERGENCY);
        return;
    }
    
    nav2_msgs::action::NavigateThroughPoses::Goal goal;
    
    goal.poses.clear();
    
    for (size_t i = current_waypoint_index; i < waypoint_list.poses.size(); ++i) {
    
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = interface->now();
        pose_stamped.pose = waypoint_list.poses[i];
        goal.poses.push_back(pose_stamped);
    
    }

    RCLCPP_INFO(interface->get_logger(), "NavigateThroughPoses goal prepared with %zu remaining poses", goal.poses.size());
    
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions options;
    
    options.result_callback = [this, interface, waypoint_list](const auto & result) mutable {
        
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(interface->get_logger(), "Navigation succeeded");

                // Save Last Waypoints
                interface->last_waypoints_.poses.clear();
                interface->last_waypoints_.header = waypoint_list.header;
                interface->last_waypoints_.poses.push_back(waypoint_list.poses.back());
                RCLCPP_DEBUG(interface->get_logger(), "last_waypoints_ updated with only the last pose");
            

                // State IDLE
                interface->navigation_goal_sent_ = false;

                // Set AUTO behavior completed
                interface->behavior_auto_started_ = false;
                interface->behavior_auto_completed_ = true;

                if (interface->task_load_started_ || 
                    interface->task_unload_started_ || 
                    interface->task_home_started_) {  

                    interface->setState(RobotState::DOCKING);

                } else if(interface->task_move_started_){

                    interface->task_move_started_ = false;
                    interface->task_move_completed_ = true;
                    interface->setState(RobotState::IDLE);

                }
            }
    };
    auto future_handle = action_client->async_send_goal(goal, options);
    std::thread([this, future_handle = std::move(future_handle)]() mutable {
        nav_through_poses_goal_handle_ = future_handle.get();
    }).detach();
}


/**
 * @brief Sends a docking goal to the docking action server.
 * @param interface Pointer to the main robot interface.
 * @param camera_mode Camera mode for docking (e.g., "front" or "rear").
 * @param marker_gap Gap distance to the marker.
 * @param marker_id ID of the marker to dock to.
 */
void StateExecutor::sendDockingGoalAsync(Interface* interface, const std::string & camera_mode, double & marker_gap, int & marker_id)
{
    (void)marker_id; // Suppress unused parameter warning
    docking_action_client_ = rclcpp_action::create_client<san_msgs::action::MarkerDocking>(interface, "aruco_marker_docking");
    auto action_client = docking_action_client_;
    if (!action_client->wait_for_action_server(10s)) {
        RCLCPP_ERROR(interface->get_logger(), "Action server 'aruco_marker_docking' not available!");
        interface->setState(RobotState::EMERGENCY);
        return;
    }
    san_msgs::action::MarkerDocking::Goal goal;
    goal.marker_gap = marker_gap;
    goal.camera_mode = camera_mode;
    RCLCPP_INFO(interface->get_logger(), "Sending docking goal: marker_gap=%.2f", goal.marker_gap);
    rclcpp_action::Client<san_msgs::action::MarkerDocking>::SendGoalOptions options;
    options.result_callback = [this, interface](const auto & result) {
        interface->docking_goal_sent_ = false;
        
        // Docking Success
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(interface->get_logger(), "Docking succeeded");
            
            // Set DOCKING behavior completed
            interface->behavior_docking_started_ = false;
            interface->behavior_docking_completed_ = true;
            
            // State Transition From DOCKING

            if (interface->task_load_started_ || 
                interface->task_unload_started_ ||
                interface->behavior_manipulation_redocking_triggered_ ) {
                
                if(interface->behavior_manipulation_redocking_triggered_){

                    interface->behavior_manipulation_redocking_triggered_ = false;

                }

                interface->setState(RobotState::MANIPULATION);

            } 
            else if (interface->behavior_charging_redocking_triggered_) {
                interface->behavior_charging_redocking_triggered_ = false;
                interface->setState(RobotState::CHARGING);
            }
            else if (interface->task_home_started_) {

                // interface->task_home_started_ = false;
                                
                interface->setState(RobotState::CHARGING);
            } 
            else {
                interface->setState(RobotState::IDLE);
            }

        } 
        
        // Docking Fail
        else {
            RCLCPP_ERROR(interface->get_logger(), "Docking failed with code: %d",
                         static_cast<int>(result.code));
            
            // Enter to Redocking Mode 1
            if(interface->task_load_started_ || interface->task_unload_started_)
            {
                interface->behavior_manipulation_redocking_triggered_ = true;
            }
            if(interface->task_home_started_)
            {
                interface->behavior_charging_redocking_triggered_ = true;
            }

            interface->setState(RobotState::EMERGENCY);
        }
    };
    auto future_handle = action_client->async_send_goal(goal, options);
    std::thread([this, future_handle = std::move(future_handle)]() mutable {
        docking_goal_handle_ = future_handle.get();
    }).detach();
}

/**
 * @brief Updates robot parameters for a given mode using the parameter updater.
 * @param interface Pointer to the main robot interface.
 * @param mode The mode string (e.g., "AUTO", "DOCK").
 */
void StateExecutor::updateParameters(Interface* interface, const std::string & mode)
{
    // 이전 모드와 동일하면 업데이트 스킵
    if (interface->parameter_mode_ == mode) {
        RCLCPP_INFO_THROTTLE(interface->get_logger(), *interface->get_clock(), 10000, "Skipping parameter update: Mode '%s' is already set.", mode.c_str());
        // RCLCPP_INFO(interface->get_logger(), "Skipping parameter update: Mode '%s' is already set.", mode.c_str());
        return;
    }

    // 업데이트 수행
    bool success = interface->parameter_updater_->updateParametersDirectly(mode);

    // 결과 출력 및 현재 모드 저장
    interface->parameter_updater_->printUpdateResult(mode, success);
    
    if (success) {
        interface->parameter_mode_ = mode;  // 새로운 모드 저장
    }
}


// Implement cancelAllGoals to cancel both navigation and docking goals
void StateExecutor::cancelAllGoals(Interface* interface) {
    // Cancel navigation goal if active
    if (nav_goal_handle_ && nav_action_client_ && is_goal_active(nav_goal_handle_)) {

        nav_action_client_->async_cancel_goal(nav_goal_handle_);

        interface->navigation_goal_sent_ = false;

        interface->behavior_auto_canceled_ = true;
        interface->behavior_auto_started_ = false;
        interface->behavior_auto_completed_ = false;

        // nav_goal_handle_ = nullptr;
        RCLCPP_INFO(interface->get_logger(), "[Webserver Goal Canceled] Navigation goal canceled");
    }
    // Cancel FollowWaypoints goal if active
    if (follow_waypoints_goal_handle_ && follow_waypoints_action_client_ && is_goal_active(follow_waypoints_goal_handle_)) {
        
        follow_waypoints_action_client_->async_cancel_goal(follow_waypoints_goal_handle_);

        interface->navigation_goal_sent_ = false;

        interface->behavior_auto_canceled_ = true;
        interface->behavior_auto_started_ = false;
        interface->behavior_auto_completed_ = false;
    
        // follow_waypoints_goal_handle_ = nullptr;
        RCLCPP_INFO(interface->get_logger(), "[Webserver Goal Canceled] FollowWaypoints goal canceled");
    
    }
    // Cancel NavigateThroughPoses goal if active
    if (nav_through_poses_goal_handle_ && nav_through_poses_action_client_ && is_goal_active(nav_through_poses_goal_handle_)) {
        
        nav_through_poses_action_client_->async_cancel_goal(nav_through_poses_goal_handle_);

        interface->navigation_goal_sent_ = false;

        interface->behavior_auto_canceled_ = true;
        interface->behavior_auto_started_ = false;
        interface->behavior_auto_completed_ = false;
    
        // nav_through_poses_goal_handle_ = nullptr;
        RCLCPP_INFO(interface->get_logger(), "[Webserver Goal Canceled] NavigateThroughPoses goal canceled");
    
    }
    // Cancel docking goal if active
    if (docking_goal_handle_ && docking_action_client_ && is_goal_active(docking_goal_handle_)) {
    
        docking_action_client_->async_cancel_goal(docking_goal_handle_);
    
        interface->behavior_docking_canceled_ = true;
        interface->behavior_docking_started_ = false;
        interface->behavior_docking_completed_ = false;

        // docking_goal_handle_ = nullptr;
    
        RCLCPP_INFO(interface->get_logger(), "[Webserver Goal Canceled] Docking goal canceled");
    }

    // interface->task_canceled_ = false;
}

/****************************************************************************************** */


//----------------------------
// ExecutionNode Implementation
//----------------------------

ExecutionNode::ExecutionNode(std::shared_ptr<Interface> interface)
    : Node("execution_node"), interface(interface)
{
    this->declare_parameter<double>("charging_timout", 60.0);
    charging_timeout_ = this->get_parameter("charging_timout").as_double();

    this->declare_parameter<double>("charging_relay_timeout", 60.0);
    charging_relay_timeout_ = this->get_parameter("charging_relay_timeout").as_double();

    this->declare_parameter<double>("goal_distance_threshold", 1.0);
    goal_distance_threshold_ = this->get_parameter("goal_distance_threshold").as_double();

    this->declare_parameter<std::string>("filtered_scan_topic", "filtered_scan");
    filtered_scan_topic_ = this->get_parameter("filtered_scan_topic").as_string();

    this->declare_parameter<double>("obstacle_distance_threshold", 1.8);
    obstacle_distance_threshold_ = this->get_parameter("obstacle_distance_threshold").as_double();

    this->declare_parameter<int>("min_cluster_size", 3);
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();

    this->declare_parameter<double>("dynamic_threshold", 0.2);
    dynamic_threshold_ = this->get_parameter("dynamic_threshold").as_double();

    this->declare_parameter<int>("uss_threshold", 3);
    uss_threshold_ = this->get_parameter("uss_threshold").as_int();

    check_timer_ = this->create_wall_timer(
        100ms, std::bind(&ExecutionNode::loopChecker, this)    // 10 Hz Loop
    );
    RCLCPP_INFO(this->get_logger(), "ExecutionNode started.");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    previous_soc_ = -1.0;
    // update_client_ = this->create_client<san_msgs::srv::UpdateParams>("update_params");

    parameter_updater_ = std::make_shared<ParameterUpdater>();

    // manipulation 모드일 때 obstacle_detected  현재 사용x 주석 해제 요망
    // scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     "scan", rclcpp::SensorDataQoS(),
    //     std::bind(&ExecutionNode::obstacleChecker, this, std::placeholders::_1));
    
    filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        filtered_scan_topic_, rclcpp::SensorDataQoS());

    obstacle_detected_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "obstacle_detected", 10);

    slam_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "slam_pose", 10);

}

void ExecutionNode::loopChecker()
{
     // (if LOAD or UNLOAD or HOME Mission & AUTO state) Robot pose checker
    if (interface->getState() == RobotState::AUTO ||
        interface->getState() == RobotState::MANUAL ||
        interface->getState() == RobotState::SLAM) {
        if(!interface->task_move_started_){
            updateCurrentPose();
        }  
    }  

    if (interface->getState() == RobotState::AUTO
        || interface->getState() == RobotState::EMERGENCY
        || interface->getState() == RobotState::STOP) {

        // 초음파 센서 연결 시, 주석해제!!
        // ussChecker();

    }

    // Docking Timeout Checking
    if (interface->getState() == RobotState::DOCKING) {

        if (!docking_in_progress_) {
            docking_in_progress_ = true;
            docking_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Docking started. Monitoring timeout...");
        } else {
            auto elapsed = this->now() - docking_start_time_;
            if (elapsed.seconds() > 1800.0) { // 3 min
                RCLCPP_WARN(this->get_logger(), "Docking timeout (%.2f s)! Initiating redocking sequence.", elapsed.seconds());
                if(interface->task_load_started_ || interface->task_unload_started_)
                {
                    interface->behavior_manipulation_redocking_triggered_ = true;
                }
                if(interface->task_home_started_)
                {
                    interface->behavior_charging_redocking_triggered_ = true;
                }
                docking_in_progress_ = false;
            }
        }
    } 
    else {
        docking_in_progress_ = false;
    }

    // ToDolists: 실환경에서 주석해제
    if(interface->getState() == RobotState::MANIPULATION) {

        std_msgs::msg::Bool msg;
        msg.data = interface->obstacle_detected_;   
        if(msg.data){
            RCLCPP_INFO(this->get_logger(), "MANIPULATION: obstacle detected: True");
        }
        obstacle_detected_publisher_->publish(msg);

    }

    // ToDoLists: 배터리 관련 행동
    if (interface->getState() == RobotState::CHARGING) {
        if(interface->task_unload_started_
            || interface->task_load_started_
            || interface->task_move_started_
            || interface->build_map_requested_)
        {

            interface->battery_data_.is_charging = false;
            return;
        
        }

        if(interface->battery_data_.is_charging)
        {
            if(interface->battery_data_.isCharged()){
                //  interface->task_home_completed_ = true;
                interface->behavior_charge_completed_ = true;
                interface->battery_data_.is_charging = false;
                RCLCPP_INFO(interface->get_logger(), "Behavior Charge Completed!!");
                return;
            }
        }

        if(batteryChecker())
        {
            interface->battery_data_.is_charging = true;
        }
        else{
            interface->battery_data_.is_charging = false;
        }
        
        
    }
}

void ExecutionNode::updateCurrentPose()
{
  // Attempt to lookup the transform from "map" to "base_link".
  geometry_msgs::msg::TransformStamped map_to_base;
  try {
    map_to_base = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    // Convert the transform into a PoseStamped message.
    current_pose_.header = map_to_base.header;
    current_pose_.pose.position.x = map_to_base.transform.translation.x;
    current_pose_.pose.position.y = map_to_base.transform.translation.y;
    current_pose_.pose.position.z = map_to_base.transform.translation.z;
    current_pose_.pose.orientation = map_to_base.transform.rotation;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform from map to base_link: %s", ex.what());
    return;
  }

  // After updating the current pose, check if we should update parameters.
  currentPoseChecker();
}

void ExecutionNode::currentPoseChecker()
{
    // Goal pose 업데이트
    if (!interface->current_waypoints_.poses.empty()) {
        goal_pose_.pose = interface->current_waypoints_.poses.back();
    } else {
        // RCLCPP_WARN(this->get_logger(), "Waypoint list is empty. Using default goal position.");
        goal_pose_ = interface->goal_position_;  
    }
    
    // 현재 위치와 목표 위치 사이의 유클리드 거리 계산
    double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
    double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    if(interface->getState() == RobotState::AUTO) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Distance to goal: %.2f m", distance);
    }

    // 목표 영역 내에 있고, 아직 업데이트하지 않은 경우에만 서비스 호출
    if (distance <= goal_distance_threshold_ && interface->parameter_mode_ != "DOCK" ) {
        // RCLCPP_INFO(this->get_logger(), "Within threshold distance. Updating parameters using ParameterUpdater...");

        interface->distance_in_threshold_ = true;

    }else{
        interface->distance_in_threshold_ = false;
    }

}

bool ExecutionNode::batteryChecker()
{
    // soc와 soc_top 중 더 낮은 값을 기준으로 사용
    double current_soc = std::min(interface->battery_data_.soc, interface->battery_data_.soc_top);
    static bool inactive_timer_started = false;

    static rclcpp::Time battery_relay_inactive_start_time;
    rclcpp::Time now_time = this->now();

    // battery_charge_relay_ 활성화 시
    if (interface->battery_data_.charge_relay) {
        RCLCPP_INFO(this->get_logger(), "Debug:[Activate] Battery Charge Relay. SOC: %.2f, SOC_TOP: %.2f", 
                    interface->battery_data_.soc, interface->battery_data_.soc_top);

        inactive_timer_started = false;

        // 이전 SOC 값이 초기값이면 현재 값과 시각을 초기화
        if (previous_soc_ < 0) {
            previous_soc_ = current_soc;
            previous_soc_check_time_ = now_time;
        }
        // 이전 SOC와 현재 SOC를 비교, charging_timout 확인
        else if ((now_time - previous_soc_check_time_).seconds() >= charging_timeout_) {

            if (interface->battery_data_.current > -0.01) 
            {

                RCLCPP_INFO(this->get_logger(), "Battery Current is positive. Charging in progress. SOC: %.2f, SOC_TOP: %.2f", 
                            interface->battery_data_.soc, interface->battery_data_.soc_top);
                previous_soc_ = current_soc;
                previous_soc_check_time_ = now_time;
                return true;

            }
            
            else 
            {
            
                RCLCPP_WARN(this->get_logger(), "Battery SOC is not increasing after %f seconds of charging. SOC: %.2f, SOC_TOP: %.2f", 
                            charging_timeout_, interface->battery_data_.soc, interface->battery_data_.soc_top);
            
            }
            // 현재 SOC와 시각을 업데이트하여 다음 charging_timeout 간격으로 검사
            previous_soc_ = current_soc;
            previous_soc_check_time_ = now_time;
        }
    } 
    // battery_charge_relay_ 비활성화된 상태 -> 충전 X
    else {
        RCLCPP_WARN(this->get_logger(), "[Deactivate] Battery Charge Relay.");

        // 비활성 상태가 시작되었는지 확인하고, 아니라면 시작 시각 기록
        if (!inactive_timer_started) {
            battery_relay_inactive_start_time = now_time;
            inactive_timer_started = true;
        }
        else {
            // 비활성 상태가 지속된 시간 계산
            double elapsed = (now_time - battery_relay_inactive_start_time).seconds();

            if (elapsed >= charging_relay_timeout_) {    //  60s
                RCLCPP_ERROR(this->get_logger(), "Battery Charge Relay has been inactive for %.2f seconds. Triggering battery error procedure.", elapsed);
                
                interface->behavior_charging_redocking_triggered_ = true;
                interface->task_home_started_ = true;
                inactive_timer_started = false;

            }
        }
    }

    return false;
}

// Dynamic Obstacle Detection
void ExecutionNode::obstacleChecker(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 1. 미디언 필터 적용 (노이즈 제거)
    std::vector<float> filtered_ranges = msg->ranges;
    const int window_size = 3;
    for (size_t i = 1; i + 1 < filtered_ranges.size(); ++i) {
        std::vector<float> window;
        for (int j = -1; j <= 1; ++j) {
            if (std::isfinite(msg->ranges[i + j]))
                window.push_back(msg->ranges[i + j]);
        }
        if (!window.empty()) {
            std::nth_element(window.begin(), window.begin() + window.size()/2, window.end());
            filtered_ranges[i] = window[window.size()/2];
        }
    }

    // 2. 클러스터링 기반 장애물 감지
    int cluster_count = 0;
    int max_cluster = 0;
    float prev_range = std::numeric_limits<float>::quiet_NaN();
    for (size_t i = 0; i < filtered_ranges.size(); ++i) {
        float r = filtered_ranges[i];
        if (std::isfinite(r) && r < obstacle_distance_threshold_) {
            if (std::isfinite(prev_range) && std::fabs(r - prev_range) < 0.2) { // 20cm 이내면 같은 클러스터
                cluster_count++;
            } else {
                max_cluster = std::max(max_cluster, cluster_count);
                cluster_count = 1;
            }
        } else {
            max_cluster = std::max(max_cluster, cluster_count);
            cluster_count = 0;
        }
        prev_range = r;
    }
    max_cluster = std::max(max_cluster, cluster_count);

    // 3. 동적 장애물 감지 (이전 프레임과 비교)
    bool dynamic_detected = false;
    if (prev_ranges_.size() == filtered_ranges.size()) {
        int dynamic_cluster = 0;
        for (size_t i = 0; i < filtered_ranges.size(); ++i) {
            float curr = filtered_ranges[i];
            float prev = prev_ranges_[i];
            if (std::isfinite(curr) && std::isfinite(prev)) {
                float diff = std::fabs(curr - prev);
                if (diff > dynamic_threshold_ && curr < obstacle_distance_threshold_) {
                    dynamic_cluster++;
                    if (dynamic_cluster >= min_cluster_size_) {
                        dynamic_detected = true;
                        break;
                    }
                } else {
                    dynamic_cluster = 0;
                }
            } else {
                dynamic_cluster = 0;
            }
        }
    }
    prev_ranges_ = filtered_ranges;

    // 4. 상태 반영 (클러스터 크기와 동적 감지 모두 고려)
    if (interface->getState() == RobotState::MANIPULATION) {
        interface->obstacle_detected_ = (max_cluster >= min_cluster_size_ || dynamic_detected);
    } else {
        interface->obstacle_detected_ = false;
    }
}




// STATE:AUTO 일 때만 실행됨
// 10cm 이하의 값이 나오면 EMERGENCY 모드로 전환됨
// 전진시는 0,1번 센서만 / 회전시에는 2,3,6,7 센서만 확인함
void ExecutionNode::ussChecker() 
{
    const double epsilon = 1e-3; // 속도가 0과 구분할 임계값
    double linearSpeed = interface->cmd_vel->linear.x;
    double angularSpeed = interface->cmd_vel->angular.z;
    
    int16_t uss_sensor[8];
    std::memcpy(uss_sensor, interface->uss_data.sensor, sizeof(uss_sensor));  

    bool emergency = false;

    // 1. 전진만 하는 경우: 전진 속도 > 0, 회전 속도 거의 0
    if (linearSpeed > epsilon && std::fabs(angularSpeed) < epsilon) {
        // 전방 센서: 인덱스 0, 1
        if (uss_sensor[0] <= uss_threshold_ || uss_sensor[1] <= uss_threshold_) {
            RCLCPP_ERROR(this->get_logger(),
                         "EMERGENCY: Obstacle detected in front! Sensor 0: %d, Sensor 1: %d",
                         uss_sensor[0], uss_sensor[1]);
            emergency = true;
        }
    }
    // 2. 전진하며 우회전하는 경우: 전진 속도 > 0, 회전 속도 음수
    else if (linearSpeed > epsilon && angularSpeed < -epsilon) {
        // 전면 및 우측 센서: 0, 1, 2, 3
        if (uss_sensor[0] <= uss_threshold_ || uss_sensor[1] <= uss_threshold_ ||
            uss_sensor[2] <= uss_threshold_ || uss_sensor[3] <= uss_threshold_) {
            RCLCPP_ERROR(this->get_logger(),
                         "EMERGENCY: Obstacle detected during forward right turn! Front: 0: %d, 1: %d, Right: 2: %d, 3: %d",
                         uss_sensor[0], uss_sensor[1], uss_sensor[2], uss_sensor[3]);
            emergency = true;
        }
    }
    // 3. 전진하며 좌회전하는 경우: 전진 속도 > 0, 회전 속도 양수
    else if (linearSpeed > epsilon && angularSpeed > epsilon) {
        // 전면 및 좌측 센서: 0, 1, 6, 7
        if (uss_sensor[0] <= uss_threshold_ || uss_sensor[1] <= uss_threshold_ ||
            uss_sensor[6] <= uss_threshold_ || uss_sensor[7] <= uss_threshold_) {
            RCLCPP_ERROR(this->get_logger(),
                         "EMERGENCY: Obstacle detected during forward left turn! Front: 0: %d, 1: %d, Left: 6: %d, 7: %d",
                         uss_sensor[0], uss_sensor[1], uss_sensor[6], uss_sensor[7]);
            emergency = true;
        }
    }
    // 4. 후진만 하는 경우: 후진 속도 < 0, 회전 속도 거의 0
    else if (linearSpeed < -epsilon && std::fabs(angularSpeed) < epsilon) {
        // 후면 센서: 4, 5
        if (uss_sensor[4] <= uss_threshold_ || uss_sensor[5] <= uss_threshold_) {
            RCLCPP_ERROR(this->get_logger(),
                         "EMERGENCY: Obstacle detected in rear! Rear: 4: %d, 5: %d",
                         uss_sensor[4], uss_sensor[5]);
            emergency = true;
        }
    }
    // 5. 후진하며 우회전하는 경우: 후진 속도 < 0, 회전 속도 음수
    else if (linearSpeed < -epsilon && angularSpeed < -epsilon) {
        // 후면 및 우측 센서: 후면(4, 5)와 우측(2, 3)
        if (uss_sensor[4] <= uss_threshold_ || uss_sensor[5] <= uss_threshold_ ||
            uss_sensor[2] <= uss_threshold_ || uss_sensor[3] <= uss_threshold_) {
            RCLCPP_ERROR(this->get_logger(),
                         "EMERGENCY: Obstacle detected during reverse right turn! Rear: 4: %d, 5: %d, Right: 2: %d, 3: %d",
                         uss_sensor[4], uss_sensor[5], uss_sensor[2], uss_sensor[3]);
            emergency = true;
        }
    }
    // 6. 후진하며 좌회전하는 경우: 후진 속도 < 0, 회전 속도 양수
    else if (linearSpeed < -epsilon && angularSpeed > epsilon) {
        // 후면 및 좌측 센서: 후면(4, 5)와 좌측(6, 7)
        if (uss_sensor[4] <= uss_threshold_ || uss_sensor[5] <= uss_threshold_ ||
            uss_sensor[6] <= uss_threshold_ || uss_sensor[7] <= uss_threshold_) {
            RCLCPP_ERROR(this->get_logger(),
                         "EMERGENCY: Obstacle detected during reverse left turn! Rear: 4: %d, 5: %d, Left: 6: %d, 7: %d",
                         uss_sensor[4], uss_sensor[5], uss_sensor[6], uss_sensor[7]);
            emergency = true;
        }
    }
    // 7. 제자리 회전하는 경우: 전진/후진 속도 거의 0, 회전 속도 ≠ 0
    else if (std::fabs(linearSpeed) < epsilon && std::fabs(angularSpeed) > epsilon) {
        // 양쪽 측면 센서: 2, 3, 6, 7
        if (uss_sensor[2] <= uss_threshold_ || uss_sensor[3] <= uss_threshold_ ||
            uss_sensor[6] <= uss_threshold_ || uss_sensor[7] <= uss_threshold_) {
            RCLCPP_ERROR(this->get_logger(),
                         "EMERGENCY: Obstacle detected during in-place rotation! Side Sensors: 2: %d, 3: %d, 6: %d, 7: %d",
                         uss_sensor[2], uss_sensor[3], uss_sensor[6], uss_sensor[7]);
            emergency = true;
        }
    }
    // 그 외의 경우 추가 처리가 필요하면 여기에 작성

    if (emergency) {
        interface->uss_emergency_triggered_ = true;
    }
    else {
        interface->uss_emergency_triggered_ = false;  
    }
}



} // end namespace AMR

