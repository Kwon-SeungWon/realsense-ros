#include "amr_interface/interface.hpp"

using namespace std::chrono_literals;
using BehaviorMsg = san_msgs::msg::Behavior;
using TaskMsg = san_msgs::msg::Task;

namespace AMR {

//---------------------------------------------------------------------
// Interface Class Implementation
//---------------------------------------------------------------------

// Constructor: Initializes the node and sets up publishers, subscribers, service, and timers.
Interface::Interface()
: Node("interface_node"),
  current_state_(RobotState::INIT),
//   behavior_auto_(false),
//   behavior_manual_requested_(false),
  docking_requested_(false),
  emergency_triggered_(false),
  resume_requested_(false) // STOP 상태 resume 플래그 추가
{
    RCLCPP_INFO(this->get_logger(), "Interface Node started.");

    last_waypoints_ = geometry_msgs::msg::PoseArray();
    prev_last_waypoints_ = geometry_msgs::msg::PoseArray();
    geometry_msgs::msg::Pose zero_pose;
    zero_pose.position.x = 0.0;
    zero_pose.position.y = 0.0;
    zero_pose.position.z = 0.0;
    zero_pose.orientation.x = 0.0;
    zero_pose.orientation.y = 0.0;
    zero_pose.orientation.z = 0.0;
    zero_pose.orientation.w = 1.0; // 단위 쿼터니언
    last_waypoints_.poses.push_back(zero_pose);
    prev_last_waypoints_.poses.push_back(zero_pose);
    current_waypoints_ = geometry_msgs::msg::PoseArray();

    // Methods to initialize parameters, publishers, subscribers, and service servers
    setParam();
    setCallbackGroups();
    setSubscriber();
    setPublisher();
    setActionServer();
    setClient();

    // Initialize the periodic timer for state updates
    initStateTimer();
    last_odom_msg_time_ = this->get_clock()->now();
    last_manual_msg_time_ = this->get_clock()->now();

    // Node Manager & Map Manager
    node_manager_ = std::make_shared<NodeManager>();

    RCLCPP_INFO(this->get_logger(), "pakage path : %s",webserver_map_path_.c_str());
    map_manager_ = std::make_shared<MapManager>(webserver_map_path_);

    parameter_updater_ = std::make_shared<ParameterUpdater>();

    cmd_vel = std::make_shared<geometry_msgs::msg::Twist>();

    std::set<std::shared_ptr<GoalHandleTaskAction>> active_goal_handles_; // Track all active TaskAction goal handles

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    node_graph_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("node_graph", 1);
    parseNodeGraphFromFile(full_node_file_path_);
    node_graph_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&Interface::publishNodeGraph, this)
    );
}

Interface::~Interface()
{
    node_manager_->stopNavigation();
    node_manager_->stopSLAM();
    rclcpp::shutdown();
}

// setParam: Declare & Get from .yaml
void Interface::setParam()
{
    // Webserver Config Path
    this->declare_parameter<std::string>("webserver_config_file", ament_index_cpp::get_package_share_directory("webserver_interface_ros2")+"/config/robot_param.yaml");
    this->get_parameter("webserver_config_file", webserver_config_file_);

    // Webserver Map files Path (기본값은 빈 문자열)
    this->declare_parameter<std::string>("webserver_map_path", "");
    this->get_parameter("webserver_map_path", webserver_map_path_);

    // data_dir.yaml에서 map_data 경로 읽기 (webserver_map_path_에 할당)
    try {
        YAML::Node config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("webserver_interface_ros2")+"/config/data_dir.yaml");
        if (config["map_data"]) {
            webserver_map_path_ = config["map_data"].as<std::string>();
            RCLCPP_INFO(this->get_logger(), "webserver_map_path_ set from data_dir.yaml: %s", webserver_map_path_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "map_data not found in data_dir.yaml");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load data_dir.yaml: %s", e.what());
    }

    // Webserver Map files Path (기본값은 빈 문자열)
    this->declare_parameter<std::string>("webserver_node_path", "");
    this->get_parameter("webserver_node_path", webserver_node_path_);

    // data_dir.yaml에서 node_data 경로 읽기 (webserver_map_path_에 할당)
    try {
        YAML::Node config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("webserver_interface_ros2")+"/config/data_dir.yaml");
        if (config["node_data"]) {
            webserver_node_path_ = config["node_data"].as<std::string>();
            RCLCPP_INFO(this->get_logger(), "webserver_node_path_ set from data_dir.yaml: %s", webserver_node_path_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "node_data not found in data_dir.yaml");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load data_dir.yaml: %s", e.what());
    }

    // Node file path 
    getFullNodeFilePath();

    this->declare_parameter("charging_marker_gap", 0.25);
    this->get_parameter("charging_marker_gap", charging_marker_gap_);

    this->declare_parameter("working_marker_gap", 0.25);
    this->get_parameter("working_marker_gap", working_marker_gap_);

    this->declare_parameter("marker_id", 1);
    this->get_parameter("marker_id", marker_id_);

    // ODOM timout
    this->declare_parameter<double>("odom_timeout", 1.0);
    this->get_parameter("odom_timeout", odom_timeout_);

    // Battery Low Threshold
    this->declare_parameter<double>("battery_low_threshold", 15.0);
    double battery_low_threshold;
    this->get_parameter("battery_low_threshold", battery_low_threshold);

    // Battery Charged Threshold
    this->declare_parameter<double>("battery_charged_threshold", 80.0);
    double battery_charged_threshold;
    this->get_parameter("battery_charged_threshold", battery_charged_threshold);
    
    // Set battery thresholds
    battery_data_.setThresholds(battery_low_threshold, battery_charged_threshold);

    // State Timer Loop Period
    this->declare_parameter<int>("state_timer_period_ms_int", 100);
    this->get_parameter("state_timer_period_ms_int", state_timer_period_ms_int_);

    this->declare_parameter<std::string>("scan_mode", "all");

    this->declare_parameter<double>("waypoint_reach_threshold", 0.3);
    this->declare_parameter<double>("waypoint_missed_distance", 100.0);

    this->get_parameter("waypoint_reach_threshold", waypoint_reach_threshold_);
    this->get_parameter("waypoint_missed_distance", waypoint_missed_distance_);
}

void Interface::setCallbackGroups()
{
    // -------- Callback Groups --------
    emergency_group_   = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sensor_group_      = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    core_group_        = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    web_group_         = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    manipulator_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // -------- Subscription Options --------
    emergency_opts_.callback_group = emergency_group_;
    sensor_opts_.callback_group = sensor_group_;
    core_opts_.callback_group = core_group_;
    web_opts_.callback_group = web_group_;
    manipulator_opts_.callback_group = manipulator_group_;
}

// setSubscriber: Create subscriptions for command, battery, and emergency button topics.
void Interface::setSubscriber()
{
    // TEST Subscribe to the "command" topic (std_msgs::msg::String)
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "command", 10,
        std::bind(&Interface::commandMsgCallback, this, std::placeholders::_1)
    );

    /********************** BECKHOFF SUBSCRIBER ************************/

    // Subscribe to the "emergency_button" topic (std_msgs::msg::Bool)
    emergency_button_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "emergency_button", 10,
        std::bind(&Interface::emergencyButtonCallback, this, std::placeholders::_1),
        emergency_opts_
    );
    
    reset_button_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "reset_button", 10,
        std::bind(&Interface::resetButtonCallback, this, std::placeholders::_1),
        emergency_opts_
    );
    
    sto_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "sto_status", 10,
        std::bind(&Interface::stoStatusCallback, this, std::placeholders::_1),
        emergency_opts_
    );

    // Subscribe to the "Bumper Status" topic, 8 Bumpers (std_msgs::msg::UInt8)
    bumper_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "bumper_status", 10,
        std::bind(&Interface::bumperStatusCallback, this, std::placeholders::_1),
        sensor_opts_
    );
    
    ultrasonic_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "ultrasonic_status", 10,
        std::bind(&Interface::ultrasonicStatusCallback, this, std::placeholders::_1),
        sensor_opts_
    );
    
    battery_sub_ = this->create_subscription<san_msgs::msg::BatteryStatus>(
        "battery_status", 10,
        std::bind(&Interface::batteryStatusCallback, this, std::placeholders::_1),
        sensor_opts_
    );

    motor_status_sub_ = this->create_subscription<san_msgs::msg::MotorStatus>(
        "motor_status", 10,
        std::bind(&Interface::motorStatusCallback, this, std::placeholders::_1),
        sensor_opts_
    );

    /************************** CORE SUBSCRIBER *************************/
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&Interface::odomCallback, this, std::placeholders::_1),
        core_opts_
    );
    
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&Interface::cmdVelCallback, this, std::placeholders::_1),
        core_opts_
    );

    /********************** WEBSERVER SUBSCRIBER ************************/
    goal_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        std::bind(&Interface::goalPoseCallback, this, std::placeholders::_1),
        web_opts_
    );
    
    waypoint_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "waypoint_list", 10,
        std::bind(&Interface::waypointListCallback, this, std::placeholders::_1),
        web_opts_
    );
    
    change_map_receive_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "set_current_map", 10,
        std::bind(&Interface::changeMapCallback, this, std::placeholders::_1),
        web_opts_
    );

    manual_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "manual_vel", 10,
        std::bind(&Interface::manualVelCallback, this, std::placeholders::_1),
        web_opts_
    );

    joy_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "joy_vel", 10,
        std::bind(&Interface::joyVelCallback, this, std::placeholders::_1),
        web_opts_
    );

    emergency_quick_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "emergency_quick_stop", 10,
        std::bind(&Interface::emergencyQuickStopCallback, this, std::placeholders::_1),
        web_opts_
    );

    /********************** MANIPULATOR SUBSCRIBER ************************/
    manipulator_state_machine_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "manipulator_state_machine", 10,
        std::bind(&Interface::manipulatorStateMachineCallback, this, std::placeholders::_1),
        manipulator_opts_
    );
}

// setPublisher: Create a publisher for the "state_machine" topic.
void Interface::setPublisher()
{
    state_pub_ = this->create_publisher<std_msgs::msg::String>(
        "state_machine", 10
    );

    behavior_pub_ = this->create_publisher<san_msgs::msg::Behavior>(
        "current_behavior", 10
    );

    task_pub_ = this->create_publisher<san_msgs::msg::Task>(
        "current_task", 10
    );


    /********************** HAMR ************************/
    driving_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "driving_status", 10
    );

    driving_mode_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "driving_mode", 10
    );

    order_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
        "order_status", 10
    );

    path_status_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "path_status", 10
    );

    path_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>(
        "node_path", 10
    );

    path_status_manager_ = std::make_unique<PathStatusManager>(path_pub_, path_status_pub_, waypoint_reach_threshold_, waypoint_missed_distance_);

    error_code_pub_ = this->create_publisher<std_msgs::msg::Int16>(
        "error_code", 10
    );

    error_code_desc_pub_ = this->create_publisher<std_msgs::msg::String>(
        "error_code_desc", 10
    );
}

// setService: Create a service server for task command requests.
void Interface::setActionServer()
{
    using namespace std::placeholders;

    this->task_action_server_ = rclcpp_action::create_server<TaskAction>(
        this,
        "task_command_action",
        std::bind(&Interface::handle_goal, this, _1, _2),
        std::bind(&Interface::handle_cancel, this, _1),
        std::bind(&Interface::handle_accepted, this, _1));
    
    // From WebServer BuildMap Callback Server
    build_map_service_ = this->create_service<san_msgs::srv::BuildMap>(
        "build_map",
        std::bind(&Interface::buildMapCallback, this,
                    std::placeholders::_1, std::placeholders::_2)
    );

    pause_task_service_ = this->create_service<san_msgs::srv::PauseTask>(
        "pause_task",
        std::bind(&Interface::pauseTaskCallback, this,
                    std::placeholders::_1, std::placeholders::_2)
    );

    manipulator_mission_service_ = this->create_service<std_srvs::srv::Trigger>(
        "manipulator_mission_complete",
        std::bind(&Interface::manipulatorMissionService, this, std::placeholders::_1, std::placeholders::_2)
    );
}

// setClient: Create a service Client for Undocking Signal to Beckhoff.
void Interface::setClient()
{
    // To Beckhoff Send Undocking Signal
    undocking_client_ = this->create_client<std_srvs::srv::SetBool>(
        "undocking_signal"
    );

    // Get waypoints from waypoint_node_manager
    get_waypoints_client_ = this->create_client<san_msgs::srv::GetWaypoints>(
        "get_waypoints"
    );
}

// initStateTimer: Initialize a timer that calls stateLoopCallback at a fixed interval.
void Interface::initStateTimer()
{
    auto state_timer_period_ms_ = std::chrono::milliseconds(state_timer_period_ms_int_);

    state_machine_timer_ = this->create_wall_timer(
        state_timer_period_ms_, // 100ms:10hz
        std::bind(&Interface::stateLoopCallback, this)
    );
}

// stateLoopCallback: Main state update loop executed periodically.
// This function handles emergency and battery checks before invoking state-specific executor functions.
void Interface::stateLoopCallback()
{
    // Get Current State
    RobotState current = getState();

    // Odometry Check (Default 1.0 seconds) 
    rclcpp::Time check_time = this->get_clock()->now();
    double odom_timeout = (check_time - last_odom_msg_time_).seconds();
    static int odom_timeout_cnt = 0;

    // From Webserver UI Emergency Quick Stop Triggered
    if(emergency_quick_stop_triggered_)
    {
        RCLCPP_ERROR(this->get_logger(), "[Webserver UI] Emergency Quick Stop Triggered!. [State: EMERGENCY]");
        if(current != RobotState::EMERGENCY) setState(RobotState::EMERGENCY);
    }
    
    // From Beckhoff Motor Emergency Triggered
    if(motor_emergency_triggered_)
    {
        RCLCPP_ERROR(this->get_logger(), "[Beckhoff] Motor Emergency Triggered!. [State: EMERGENCY]");
        if(current != RobotState::EMERGENCY) setState(RobotState::EMERGENCY);
    }

    // Immediately transition to STOP if the (emergency button/ bumper collision / sto)  was triggered.
    if (emergency_button_triggered_
        || bumper_collision_triggered_
        || sto_triggered_
        || uss_emergency_triggered_) {

        RCLCPP_ERROR(this->get_logger(), "Emergency Triggered!. [State: EMERGENCY]");
        setState(RobotState::EMERGENCY);

    }

    // If battery level is below threshold and the current state is one where charging is allowed,
    // trigger a charge behavior.
    if (battery_data_.isLow()
        && !task_home_started_) {

        // RCLCPP_ERROR(this->get_logger(), "Battery is LOW!!. Have to Charge!!!");

        // To do list: 배터리 부족시 충전을 어느 스테이트에서 할 건지. (IDLE)
        if (current == RobotState::IDLE 
            /*|| current == RobotState::MANIPULATION*/ ){
            
            RCLCPP_WARN(this->get_logger(), "Battery low! Initiating charge behavior.");

            task_move_started_ = false;
            task_load_started_ = false;
            task_unload_started_ = false;
            
            task_home_started_ = true;
    
            setState(RobotState::AUTO); // onAuto should eventually lead to docking and then charging.
                                        // Go Home (Charging Station) triggered
     
        }
    }

    // Battery is Low & Charging is started -> Block ALL task
    if (battery_data_.isLow() 
        && current == RobotState::CHARGING) {
        task_move_started_ = false;
        task_load_started_ = false;
        task_unload_started_ = false;
        task_home_started_ = false;

        RCLCPP_WARN(this->get_logger(), "Battery is Low & Charging is started -> Block ALL task");
    }

    // Manual Mode Check
    if ((check_time - last_manual_msg_time_).seconds() > 1.0) {
        if (behavior_manual_mode_triggered_) {
          RCLCPP_INFO(this->get_logger(), "No Manual Velocity Sub");
        }
        behavior_manual_mode_triggered_ = false;
    }
    
    // Call the appropriate state executor function based on the current state.
    if (executor_) {
        switch (current_state_) {
            case RobotState::INIT:
                executor_->onInit(this);
                break;
            case RobotState::IDLE:
                executor_->onIdle(this);
                break;
            case RobotState::AUTO:
                executor_->onAuto(this);
                break;
            case RobotState::MANUAL:
                executor_->onManual(this);
                break;
            case RobotState::DOCKING:
                executor_->onDocking(this);
                break;
            case RobotState::UNDOCKING:
                executor_->onUndocking(this);
                break;
            case RobotState::CHARGING:
                executor_->onCharging(this);
                break;
            case RobotState::MANIPULATION:
                executor_->onManipulating(this);
                break;
            case RobotState::EMERGENCY:
                executor_->onEmergency(this);
                break;
            case RobotState::STOP:
                executor_->onStop(this);
                break;
            case RobotState::SLAM:
                executor_->onSLAM(this);
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown state!");
                break;
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Executor not set!");
    }

    publishPathStatus();

    publishTaskBehaviorStatus();

    publishDrivingStatusAndMode();
}

void Interface::publishPathStatus()
{
    // tf2 map->base_footprint
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer_->lookupTransform(
            "map", "base_footprint", tf2::TimePointZero);
        geometry_msgs::msg::Pose robot_pose;
        robot_pose.position.x = transformStamped.transform.translation.x;
        robot_pose.position.y = transformStamped.transform.translation.y;
        robot_pose.position.z = transformStamped.transform.translation.z;
        robot_pose.orientation = transformStamped.transform.rotation;

        path_status_manager_->updateStatus(robot_pose);
        
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "Could not transform map to base_footprint: %s", ex.what());
    }   
}

void Interface::publishTaskBehaviorStatus()
{
    // Publish the current state to the "state_machine" topic.
    auto state_to_string_msg = robotStateToString(current_state_);
    std_msgs::msg::String state_msg;
    state_msg.data = state_to_string_msg; 
    state_pub_->publish(state_msg);

    TaskMsg task_msg;
    task_msg.task_load_started = task_load_started_;
    task_msg.task_unload_started = task_unload_started_;
    task_msg.task_move_started = task_move_started_;
    task_msg.task_home_started = task_home_started_;
    task_msg.task_load_completed = task_load_completed_;
    task_msg.task_unload_completed = task_unload_completed_;
    task_msg.task_move_completed = task_move_completed_;
    task_msg.task_home_completed = task_home_completed_;
    task_pub_->publish(task_msg);

    // order_status: 3(canceled) > 2(completed) > 1(working) > 0(idle)
    uint8_t order_status = 0;
    if (task_cancel_completed_) {
        order_status = 3;
        task_cancel_completed_ = false;
    } else if (task_move_completed_ || task_load_completed_ || task_unload_completed_ || task_home_completed_) {
        order_status = 2;
    } else if (task_move_started_ || task_load_started_ || task_unload_started_ || task_home_started_) {
        order_status = 1;
    } else {
        order_status = 0;
    }
    std_msgs::msg::UInt8 order_status_msg;
    order_status_msg.data = order_status;
    order_status_pub_->publish(order_status_msg);

    BehaviorMsg behavior_msg;
    behavior_msg.behavior_auto_started = behavior_auto_started_;
    behavior_msg.behavior_slam_started = behavior_slam_started_;
    behavior_msg.behavior_charge_started = behavior_charge_started_;
    behavior_msg.behavior_docking_started = behavior_docking_started_;
    behavior_msg.behavior_undocking_started = behavior_undocking_started_;
    behavior_msg.behavior_auto_completed = behavior_auto_completed_;
    behavior_msg.behavior_slam_completed = behavior_slam_completed_;
    behavior_msg.behavior_charge_completed = behavior_charge_completed_;
    behavior_msg.behavior_docking_completed = behavior_docking_completed_;
    behavior_msg.behavior_undocking_completed = behavior_undocking_completed_;
    behavior_msg.behavior_manipulation_redocking_triggered = behavior_manipulation_redocking_triggered_;
    behavior_msg.behavior_charging_redocking_triggered = behavior_charging_redocking_triggered_;
    behavior_msg.behavior_manipulator_completed = behavior_manipulator_completed_;
    behavior_msg.behavior_manipulator_emergency_triggered = behavior_manipulator_emergency_triggered_;
    behavior_msg.behavior_manual_mode_triggered = behavior_manual_mode_triggered_;
    behavior_pub_->publish(behavior_msg);

    if (current_goal_handle_) {
        auto feedback = std::make_shared<TaskAction::Feedback>();
        feedback->current_behavior = behavior_msg;
        current_goal_handle_->publish_feedback(feedback);
    }
}

void Interface::publishDrivingStatusAndMode()
{
    // driving_status
    std_msgs::msg::UInt8 driving_status_msg;
    if (current_state_ == RobotState::AUTO) {
        driving_status_msg.data = 2; // driving
    } else if (current_state_ == RobotState::STOP || current_state_ == RobotState::EMERGENCY) {
        driving_status_msg.data = 1; // stop
    } else {
        driving_status_msg.data = 0; // idle
    }
    driving_status_pub_->publish(driving_status_msg);

    // driving_mode
    std_msgs::msg::UInt8 driving_mode_msg;
    if (current_state_ == RobotState::AUTO) {
        driving_mode_msg.data = 1; // auto
    } else if (current_state_ == RobotState::MANUAL) {
        driving_mode_msg.data = 2; // manual
    } else {
        driving_mode_msg.data = 0; // idle
    }
    driving_mode_pub_->publish(driving_mode_msg);
}

// setExecutor: Assign an executor object that implements state-specific behavior.
void Interface::setExecutor(std::shared_ptr<IStateExecutor> executor)
{
    executor_ = executor;
}

// setState: Update the current state and log the state change.
void Interface::setState(RobotState new_state)
{
    if(current_state_ != new_state) {
         
        previous_state_ = current_state_;
      
        RCLCPP_INFO(this->get_logger(), "State changed: %d -> %d",
                    static_cast<int>(current_state_), static_cast<int>(new_state));
        current_state_ = new_state;

        // If the state changes to IDLE, it might mean a task is completed.
        if (new_state == RobotState::IDLE && current_goal_handle_) {
             if (current_goal_handle_->is_active()) {
                auto result = std::make_shared<TaskAction::Result>();
                result->success = true;
                result->message = "Task completed and robot is now IDLE.";
                current_goal_handle_->succeed(result);
                current_goal_handle_.reset();
             }
        }
    }
}

// getState: Return the current robot state.
RobotState Interface::getState() const
{
    return current_state_;
}

RobotState Interface::getPreviousState() const
{
    return previous_state_;
}

/************************* Webserver CALLBACK **************************/

void Interface::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if(current_state_ == RobotState::IDLE &&
        !task_move_started_) {

        goal_position_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received new goal from RViz: (%.2f, %.2f)",
                    goal_position_.pose.position.x, goal_position_.pose.position.y);

        task_one_move_started_ = true;

        RCLCPP_INFO(this->get_logger(), "current_waypoints_ updated with the new single goal.");        

    }
    else {
        RCLCPP_WARN(this->get_logger(), "Goal Pose Callback: Not in IDLE State");
    }

}

// waypointListCallback: Wrapper to extract the Waypoints From Webserver
void Interface::waypointListCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (!msg || msg->poses.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Received waypoint list is empty or null, skipping backup");
        return;
    }
    current_waypoints_ = *msg;
    path_status_manager_->setWaypoints(*msg);
    RCLCPP_INFO(this->get_logger(), "Waypoint list received with %zu poses", msg->poses.size());
}

void Interface::changeMapCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data)
    {
        change_current_map_requested_ = true;
    }
    else
    {
        change_current_map_requested_ = false;
    }
}

void Interface::manualVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs)
{
    behavior_manual_mode_triggered_ = true;
    last_manual_msg_time_ = this->get_clock()->now();
}

void Interface::joyVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs)
{
    behavior_manual_mode_triggered_ = true;
    last_manual_msg_time_ = this->get_clock()->now();
}

void Interface::emergencyQuickStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data)
    {
        emergency_quick_stop_triggered_ = true;
    }
    else
    {
        emergency_quick_stop_triggered_ = false;
    }
}

/***************************** Core CALLBACK ******************************/

void Interface::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msgs)
{
    odometry_subscribed_ = true;
    last_odom_msg_time_ = this->get_clock()->now();
    last_robot_pose_ = msgs->pose.pose;
}

void Interface::cmdVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs)
{
    *cmd_vel = *msgs;
}

/********************************** Beckhoff CALLBACK **********************************/

// commandMsgCallback: Wrapper to extract the command string and call commandCallback.
void Interface::commandMsgCallback(const std_msgs::msg::String::SharedPtr msg)
{
    commandCallback(msg->data);
}

// emergencyButtonCallback: Callback for the emergency button topic. Sets the flag if pressed.
void Interface::emergencyButtonCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        emergency_button_triggered_ = true;
        RCLCPP_WARN(this->get_logger(), "Emergency button pressed!");
    }
    else{
        emergency_button_triggered_ = false;
    }
}

// resetButtonCallback: Callback for the emergency button topic. Sets the flag if pressed.
void Interface::resetButtonCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        reset_button_triggered_ = true;
        RCLCPP_WARN(this->get_logger(), "Reset button pressed!");
    }
}

void Interface::stoStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{

    if (msg->data) {
        sto_triggered_ = true;
        RCLCPP_WARN(this->get_logger(), "Reset button pressed!");
    }
    else{
        sto_triggered_ = false;
    }   
    
}

// batteryMsgCallback: Wrapper to extract battery level data and call batteryCallback.
/*
   * float32 voltage       # Voltage (V)        
   * float32 current       # Current (A)        
   * float32 soc           # State of Charge %
   * float32 soh           # State of Health %
   * bool    charge_relay  # Charge Relay       
*/
void Interface::batteryStatusCallback(const san_msgs::msg::BatteryStatus::SharedPtr msg)
{
    // Update battery data from message
    battery_data_.updateFromMessage(msg);

    // When Battery is low
    if (battery_data_.isLow()) {
        RCLCPP_INFO(this->get_logger(), "Battery is low! SOC: %.2f, SOC_TOP: %.2f, Threshold: %.2f", 
                    battery_data_.soc, battery_data_.soc_top, battery_data_.low_threshold);
    }
}

void Interface::bumperStatusCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    // 8 Bumpers Status Parsing
    uint8_t bumper_mask = msg->data;

    // [LSB] Check each bit: bit i == 1 means collision detected on bumper i.
    for (int i = 0; i < 8; i++) {
        bool collision = ((bumper_mask >> i) & 0x01) != 0;
        RCLCPP_INFO(this->get_logger(), "Bumper %d collision: %s", i+1, collision ? "true" : "false");
        if(collision){

            bumper_collision_triggered_ = true;
        
        }
        else{
            bumper_collision_triggered_ = false;
        }
    }
    
}

void Interface::ultrasonicStatusCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    // 8 USS Status Parsing
    if (msg->data.size() < 8) {
        RCLCPP_WARN(this->get_logger(), "Received ultrasonic data size is less than 8: %zu", msg->data.size());
        return;
    }

    for (int i = 0; i < 8; i++) {
        uss_data.sensor[i] = msg->data[i];
    }
    
}

void Interface::motorStatusCallback(const san_msgs::msg::MotorStatus::SharedPtr msg)
{
    motor_status_ = msg->state;

    if(motor_status_ == STATE_MOTOR_SERVO_OFF  ||
       motor_status_ == STATE_MOTOR_QUICK_STOP ||
       motor_status_ == STATE_MOTOR_STO)
    {
        motor_emergency_triggered_ = true;
    }
    else if(motor_status_ == STATE_MOTOR_SERVO_ON){
        motor_emergency_triggered_ = false;
    }
    
    if(motor_status_ == STATE_BATTERY_CHARGING){
        motor_battery_charging_triggered_ = true;
    }
    else{
        motor_battery_charging_triggered_ = false;
    }

}

/************************* Action Server CALLBACK **************************/
rclcpp_action::GoalResponse Interface::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TaskAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with mission: %s", goal->mission.c_str());
    (void)uuid;

    // Accept goals when robot is in IDLE, CHARGING, or when no current goal is active
    if (current_state_ == RobotState::IDLE || 
        current_state_ == RobotState::CHARGING ||
        !current_goal_handle_ ||
        !current_goal_handle_->is_active()) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    RCLCPP_WARN(this->get_logger(), "Goal rejected: Robot is busy with current task");
    return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse Interface::handle_cancel(
    const std::shared_ptr<GoalHandleTaskAction> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    
    task_cancel_started_ = true;
    
    // Cancel navigation/docking goals via executor first
    if (executor_) {
        executor_->cancelAllGoals(this);
    }
    
    // Reset only the current task flags (not all flags)
    if (task_load_started_) {
        task_load_started_ = false;
        task_load_completed_ = false;
    }
    if (task_unload_started_) {
        task_unload_started_ = false;
        task_unload_completed_ = false;
    }
    if (task_move_started_) {
        task_move_started_ = false;
        task_move_completed_ = false;
    }
    if (task_home_started_) {
        task_home_started_ = false;
        task_home_completed_ = false;
    }
    
    // Reset navigation flags
    navigation_goal_sent_ = false;
    docking_goal_sent_ = false;
    
    // Remove the canceled goal handle from active handles
    active_goal_handles_.erase(goal_handle);
    
    // If this was the current goal handle, reset it
    if (current_goal_handle_ == goal_handle) {
        current_goal_handle_.reset();
    }
    
    // Set robot state to IDLE to allow new goals
    if(current_state_ == RobotState::CHARGING){
        RCLCPP_INFO(this->get_logger(), "Goal canceled. Robot is in CHARGING state. Undocking...");
        setState(RobotState::UNDOCKING);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Goal canceled. Robot is in IDLE state.");
        setState(RobotState::IDLE);
    }
    
    RCLCPP_INFO(this->get_logger(), "Goal canceled. Robot ready for new goals.");
    
    task_cancel_started_ = false;
    task_cancel_completed_ = true;

    return rclcpp_action::CancelResponse::ACCEPT;
}

void Interface::handle_accepted(const std::shared_ptr<GoalHandleTaskAction> goal_handle)
{
    // Store the new goal handle in the set of active handles
    active_goal_handles_.insert(goal_handle);
    current_goal_handle_ = goal_handle;
    std::thread{std::bind(&Interface::execute_task, this, std::placeholders::_1), goal_handle}.detach();
}

// execute_task: Execute the task based on the mission type.
void Interface::execute_task(const std::shared_ptr<GoalHandleTaskAction> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "Executing TaskCommand: mission=%s", goal->mission.c_str());

    if(current_state_ == RobotState::IDLE
        || current_state_ == RobotState::CHARGING
        || !current_goal_handle_
        || !current_goal_handle_->is_active())  {

        if (goal->waypoint_lists.poses.empty()) {
            RCLCPP_INFO(this->get_logger(), "Waypoint list is empty, calling get_waypoints service");
            auto request = std::make_shared<san_msgs::srv::GetWaypoints::Request>();
            request->start_node = goal->start_node;
            request->goal_node = goal->goal_node;
            request->node_file = full_node_file_path_;
            if (!get_waypoints_client_->wait_for_service(std::chrono::seconds(3))) {
                RCLCPP_ERROR(this->get_logger(), "Get waypoints service not available");
                auto result = std::make_shared<TaskAction::Result>();
                result->success = false;
                result->message = "Get waypoints service not available";
                goal_handle->abort(result);
                current_goal_handle_.reset();
                active_goal_handles_.erase(goal_handle);
                return;
            }
            get_waypoints_client_->async_send_request(
                request,
                [this, goal_handle, goal](rclcpp::Client<san_msgs::srv::GetWaypoints>::SharedFuture future_result) {
                    auto result = future_result.get();
                    if (result->success) {
                        RCLCPP_INFO(this->get_logger(), "Successfully received waypoints from service");
                        current_waypoints_ = result->waypoints;
                        if (result->waypoints.poses.empty()) {
                            RCLCPP_ERROR(this->get_logger(), "Received empty waypoints from service!");
                            return;
                        }
                        if (result->node_path.empty()) {
                            RCLCPP_ERROR(this->get_logger(), "Received empty node_path from service!");
                            return;
                        }
                        bool valid_node_path = true;
                        for (auto idx : result->node_path) {
                            if (idx > 255) {
                                RCLCPP_ERROR(this->get_logger(), "node_path contains value out of uint8 range: %d", idx);
                                valid_node_path = false;
                                break;
                            }
                        }
                        if (!valid_node_path) {
                            RCLCPP_ERROR(this->get_logger(), "Invalid node_path received, aborting.");
                            return;
                        }
                        path_status_manager_->setWaypoints(result->waypoints);
                        path_status_manager_->setNodePath(result->node_path);
                        // 미션별 process_mission 실행 (별도 스레드)
                        std::thread([this, goal_handle, mission=goal->mission]() {
                            process_mission(goal_handle, mission);
                        }).detach();
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to get waypoints: %s", result->message.c_str());
                        auto action_result = std::make_shared<TaskAction::Result>();
                        action_result->success = false;
                        action_result->message = "Failed to get waypoints: " + result->message;
                        goal_handle->abort(action_result);
                        current_goal_handle_.reset();
                        active_goal_handles_.erase(goal_handle);
                    }
                }
            );
        } else {
            current_waypoints_ = goal->waypoint_lists;
            process_mission(goal_handle, goal->mission);
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Cannot execute task in current state: %s", robotStateToString(current_state_).c_str());
        auto result = std::make_shared<TaskAction::Result>();
        result->success = false;
        result->message = "Cannot execute task in current state";
        goal_handle->abort(result);
        current_goal_handle_.reset();
        active_goal_handles_.erase(goal_handle);
    }
}

void Interface::process_mission(const std::shared_ptr<GoalHandleTaskAction> goal_handle, const std::string& mission)
{
    if (mission == "load" || mission == "LOAD") {
        task_load_started_ = true;
        while (goal_handle->is_active()) {
            if (task_load_completed_) {
                auto result = std::make_shared<TaskAction::Result>();
                result->success = true;
                result->message = "Load task completed";
                goal_handle->succeed(result);
                break;
            }
            if (!goal_handle->is_active()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else if (mission == "unload" || mission == "UNLOAD") {
        task_unload_started_ = true;
        while (goal_handle->is_active()) {
            if (task_unload_completed_) {
                auto result = std::make_shared<TaskAction::Result>();
                result->success = true;
                result->message = "Unload task completed";
                goal_handle->succeed(result);
                break;
            }
            if (!goal_handle->is_active()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else if (mission == "move" || mission == "MOVE") {
        task_move_started_ = true;
        while (goal_handle->is_active()) {
            if (task_move_completed_) {
                auto result = std::make_shared<TaskAction::Result>();
                result->success = true;
                result->message = "Move task completed";
                goal_handle->succeed(result);
                break;
            }
            if (!goal_handle->is_active()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else if (mission == "home" || mission == "HOME") {
        task_home_started_ = true;
        while (goal_handle->is_active()) {
            if (task_home_completed_) {
                auto result = std::make_shared<TaskAction::Result>();
                result->success = true;
                result->message = "Home task completed";
                goal_handle->succeed(result);
                break;
            }
            if (!goal_handle->is_active()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown mission requested!");
        auto result = std::make_shared<TaskAction::Result>();
        result->success = false;
        result->message = "Unknown mission: " + mission;
        goal_handle->abort(result);
    }
    active_goal_handles_.erase(goal_handle);
}

/********************************** Webserver Service BuildMap CALLBACK **********************************/
void Interface::buildMapCallback(
    const std::shared_ptr<san_msgs::srv::BuildMap::Request> request,
    std::shared_ptr<san_msgs::srv::BuildMap::Response> response)
{
    // Receive task when only IDLE or CHARGING
    if(current_state_ == RobotState::IDLE
        || current_state_ == RobotState::CHARGING)  {

        if(request->command == 1){ // Build Map Start Request
            build_map_requested_ = true;
            
            // Start separate thread to wait for completion
            std::thread([this, response, request]() {
                // Wait for build map completion
                while (!build_map_completed_ && rclcpp::ok()) {
                    std::this_thread::sleep_for(100ms);
                }
                
                if (build_map_completed_) {
                    build_map_completed_ = false;  // Reset flag
                    response->success = true;
                    response->message = std::to_string(request->command);
                } else {
                    response->success = false;
                    response->message = std::to_string(request->command);
                }
            }).detach();
        }
        else {
            response->success = false;
            response->message = std::to_string(request->command);
        }
    }

    // Receive task when only MANUAL or SLAM
    if(current_state_ == RobotState::MANUAL
        || current_state_ == RobotState::SLAM) {
    
        if(request->command == 2){ // Cancel Building Map Request
            cancel_map_requested_ = true;
            
            // Start separate thread to wait for completion
            std::thread([this, response, request]() {
                // Wait for cancel map completion
                while (!cancel_map_completed_ && rclcpp::ok()) {
                    std::this_thread::sleep_for(100ms);
                }
                
                if (cancel_map_completed_) {
                    cancel_map_completed_ = false;  // Reset flag
                    response->success = true;
                    response->message = std::to_string(request->command);

                } else {
                    response->success = false;
                    response->message = std::to_string(request->command);
                }
            }).detach();
        }
        else if(request->command == 3){ // Save Map Request
            save_map_requested_ = true;
            save_map_requested_name_ = request->map_name;
            
            // Start separate thread to wait for completion
            std::thread([this, response, request]() {
                // Wait for save map completion
                while (!save_map_completed_ && rclcpp::ok()) {
                    std::this_thread::sleep_for(100ms);
                }
                
                if (save_map_completed_) {
                    save_map_completed_ = false;  // Reset flag
                    response->success = true;
                    response->message = std::to_string(request->command);
                } else {
                    response->success = false;
                    response->message = std::to_string(request->command);
                }
            }).detach();
        }
        else {
            response->success = false;
            response->message = std::to_string(request->command);
        }
    }
}

void Interface::pauseTaskCallback(
    const std::shared_ptr<san_msgs::srv::PauseTask::Request> request,
    std::shared_ptr<san_msgs::srv::PauseTask::Response> response)
{
RCLCPP_INFO(this->get_logger(), "Pause Task Request Received!");

// Only for Auto and Docking State
if(current_state_ == RobotState::AUTO
    || current_state_ == RobotState::DOCKING){

    // Pause Task
    if(request->pause_task){
        
        RCLCPP_INFO(this->get_logger(), "Pause Task Request Received!");
        setState(RobotState::STOP);

        response->success = true;
    
    }
}

if(current_state_ == RobotState::STOP){
    // Resume Task
    if(!request->pause_task){
        
        RCLCPP_INFO(this->get_logger(), "Resume Task Request Received!");
        resume_requested_ = true; // resume 요청 시 플래그 세팅
        response->success = true;
    
    }
        
}

}

/********************************** Manipulator CALLBACK **********************************/
void Interface::manipulatorStateMachineCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if(msg->data == "EMERCENCY_STOP"){
        RCLCPP_WARN(this->get_logger(), "Manipulator EMERGENCY STOP!");
        behavior_manipulator_emergency_triggered_ = true;
    }
    else{
        behavior_manipulator_emergency_triggered_ = false;
    }

}

void Interface::manipulatorMissionService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    behavior_manipulator_completed_ = true;
    RCLCPP_WARN(this->get_logger(), "Manipulator Mission Triggered by Service!");
    response->success = true;
    response->message = "Manipulator mission completed!";
}

void Interface::call_undocking_service(bool signal, std::function<void(bool)> callback)
{
    if (!undocking_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "Undocking service not available");
        callback(false);
        return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = signal;

    undocking_client_->async_send_request(
        request,
        [this, callback](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future_result) {
            auto result = future_result.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "Undocking service call succeeded");
                callback(true);
            } else {
                RCLCPP_WARN(this->get_logger(), "Undocking service call failed");
                callback(false);
            }
        }
    );
}


/////////////////////////////// TEST ////////////////////////////////////
// commandCallback: Process incoming command strings and set appropriate state flags.
void Interface::commandCallback(const std::string & cmd)
{
    RCLCPP_INFO(this->get_logger(), "Received command: %s", cmd.c_str());
    if (cmd == "auto") {
        task_move_started_ = true;
        setState(RobotState::AUTO);
    }else if(cmd == "idle") {
        setState(RobotState::IDLE);
    } else if (cmd == "manual") {
        behavior_manual_mode_triggered_ = true;
        setState(RobotState::MANUAL);
    } else if (cmd == "dock") {
        docking_requested_ = true;
        setState(RobotState::DOCKING);
    } else if (cmd == "emergency_true") {
            emergency_button_triggered_ = true;
    } else if (cmd == "emergency_false") {
        emergency_button_triggered_ = false;
    } else if (cmd == "stop") {
        setState(RobotState::STOP);
    } else if (cmd == "mani") {
        setState(RobotState::MANIPULATION);
    }

}

/////////////////////////////////////////////////////////////////////////

// Node file full path
void Interface::getFullNodeFilePath() {
    std::string current_node_file;
    try {
        YAML::Node config = YAML::LoadFile(webserver_config_file_);
        if (config["currentMapNode"] && config["currentMapNode"]["currentNode"]) {
            current_node_file = config["currentMapNode"]["currentNode"].as<std::string>("");
            if (!webserver_node_path_.empty() && !current_node_file.empty()) {
                full_node_file_path_ = webserver_node_path_ + "/" + current_node_file + ".yaml";
                RCLCPP_INFO(this->get_logger(), "Full node file path: %s", full_node_file_path_.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "webserver_node_path_ or current_node_file is empty");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "currentNode not found in robot_param.yaml");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load robot_param.yaml: %s", e.what());
    }
}

void Interface::parseNodeGraphFromFile(const std::string& file_path) {
    node_positions_.clear();
    node_edges_.clear();
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        if (!config["node"]) return;
        const auto& nodes = config["node"];
        std::map<int, geometry_msgs::msg::Point> idx2pos;
        for (const auto& n : nodes) {
            int idx = n["index"].as<int>();
            geometry_msgs::msg::Point pt;
            pt.x = n["position"]["x"].as<double>();
            pt.y = n["position"]["y"].as<double>();
            pt.z = 0.0;
            idx2pos[idx] = pt;
            node_positions_.push_back(pt);
            // edges
            if (n["connection"]) {
                for (const auto& conn : n["connection"]) {
                    int to = conn.as<int>();
                    node_edges_.emplace_back(idx, to);
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse node file: %s", e.what());
    }
}

void Interface::publishNodeGraph() {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = "map";
    node_marker.header.stamp = this->now();
    node_marker.ns = "nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.scale.x = 0.2;
    node_marker.scale.y = 0.2;
    node_marker.scale.z = 0.2;
    node_marker.color.r = 0.0f;
    node_marker.color.g = 0.0f;
    node_marker.color.b = 0.0f;
    node_marker.color.a = 1.0f;
    node_marker.points = node_positions_;
    arr.markers.push_back(node_marker);
    int text_id = 1000;
    for (size_t i = 0; i < node_positions_.size(); ++i) {
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = this->now();
        text_marker.ns = "node_labels";
        text_marker.id = text_id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.scale.z = 0.25;
        text_marker.color.r = 0.0f;
        text_marker.color.g = 0.0f;
        text_marker.color.b = 0.0f;
        text_marker.color.a = 1.0f;
        text_marker.pose.position = node_positions_[i];
        text_marker.pose.position.x -= 0.15; 
        text_marker.pose.position.y += 0.15; 
        text_marker.pose.position.z += 0.25;
        text_marker.text = std::to_string(i+1); 
        arr.markers.push_back(text_marker);
    }
    int edge_id = 1;
    for (const auto& e : node_edges_) {
        visualization_msgs::msg::Marker edge_marker;
        edge_marker.header.frame_id = "map";
        edge_marker.header.stamp = this->now();
        edge_marker.ns = "edges";
        edge_marker.id = edge_id++;
        edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::msg::Marker::ADD;
        edge_marker.scale.x = 0.05;
        edge_marker.color.r = 0.0f;
        edge_marker.color.g = 0.0f;
        edge_marker.color.b = 0.0f;
        edge_marker.color.a = 1.0f;
        geometry_msgs::msg::Point p1 = node_positions_[e.first-1];
        geometry_msgs::msg::Point p2 = node_positions_[e.second-1];
        edge_marker.points.push_back(p1);
        edge_marker.points.push_back(p2);
        arr.markers.push_back(edge_marker);
    }
    node_graph_pub_->publish(arr);
}

}  // namespace amr


