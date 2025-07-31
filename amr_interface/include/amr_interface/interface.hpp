#ifndef __INTERFACE_HPP__
#define __INTERFACE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <set>

#include <vector>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Message type headers
#include "san_msgs/msg/state_machine.hpp"
#include "san_msgs/msg/emergency.hpp"
#include "san_msgs/msg/battery_status.hpp"
#include "san_msgs/msg/behavior.hpp"
#include "san_msgs/msg/task.hpp"
#include "san_msgs/msg/motor_status.hpp"
#include "san_msgs/srv/pause_task.hpp"


#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "san_msgs/srv/get_waypoints.hpp"

// Custom service message (for task command requests)
#include "san_msgs/action/task_command_action.hpp"
#include "san_msgs/srv/build_map.hpp"

#include "amr_interface/map_manager.hpp"
#include "amr_interface/node_manager.hpp"
#include "amr_interface/parameter_updater.hpp"

#include <rclcpp_action/rclcpp_action.hpp>

#include "amr_interface/path_status_manager.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>

namespace AMR {

/**
 * @brief Enumeration of robot states.
 */
enum class RobotState : uint16_t {
    INIT            = 0x00, ///< Initialization state
    IDLE            = 0x10, ///< Idle state
    AUTO            = 0x20, ///< Autonomous operation state
    MANUAL          = 0x30, ///< Manual control state
    DOCKING         = 0x40, ///< Docking procedure state
    UNDOCKING       = 0x50, ///<Undocking procedure state
    CHARGING        = 0x60, ///< Charging state
    MANIPULATION    = 0x70, ///< Work execution state
    EMERGENCY       = 0x80, ///< Emergency state
    STOP            = 0x90, ///< Stop state
    SLAM            = 0x100  ///< SLAM state
};

struct UltrasonicData 
{    
    int16_t sensor[8];
};

struct BatteryData 
{    
    // Top battery data
    double voltage_top{0.0};
    double soc_top{0.0};
    double soh_top{0.0};
    
    // Combined battery data
    double voltage{0.0};
    double current{0.0};
    double soc{0.0};
    double soh{0.0};
    
    // Battery status
    bool charge_relay{false};
    bool charging_jack_state{false};
    bool charging_state{false};
    bool is_charging{false};
    bool is_full{false};
    
    // Battery thresholds
    double low_threshold{15.0};
    double charged_threshold{80.0};
    
    // Battery status flags
    bool low_triggered{false};
    
    // Methods for battery management
    void updateFromMessage(const san_msgs::msg::BatteryStatus::SharedPtr& msg) {
        // Top battery data
        voltage_top = msg->voltage_top;
        soc_top = msg->soc_top;
        soh_top = msg->soh_top;
        
        // Combined battery data
        voltage = msg->voltage;
        current = msg->current;
        soc = msg->soc;
        soh = msg->soh;
        
        // Battery status
        charge_relay = msg->charge_relay;
        charging_jack_state = msg->charging_jack_state;
        charging_state = msg->charging_state;
        
        // Update status flags
        is_charging = charging_state;
        is_full = soc >= charged_threshold;
        low_triggered = isLow();
    }
    bool isLow() const { 
        // soc와 soc_top 모두 고려하여 낮은지 판단
        return (soc < low_threshold) || (soc_top < low_threshold); 
    }
    bool isCharged() const { 
        // soc와 soc_top 모두 고려하여 충전되었는지 판단
        return (soc >= charged_threshold) && (soc_top >= charged_threshold); 
    }
    bool needsCharging() const { 
        // soc나 soc_top 중 하나라도 낮고 충전 중이 아니면 충전 필요
        return isLow() && !is_charging; 
    }
    void setThresholds(double low, double charged) { 
        low_threshold = low; 
        charged_threshold = charged; 
    }
};
enum MotorState {
    STATE_MOTOR_SERVO_OFF      = 0,
    STATE_MOTOR_SERVO_ON       = 1,
    STATE_MOTOR_QUICK_STOP     = 2,
    STATE_MOTOR_BRAKE_DISABLE  = 3,
    STATE_MOTOR_STO            = 4,
    STATE_BATTERY_CHARGING      = 6,
    STATE_MOTOR_SW_STOP         = 8
};

/**
 * @brief Interface for executing actions associated with each robot state.
 *
 * Implementations of this interface must define the behavior for each state.
 */
class IStateExecutor {
public:
    virtual ~IStateExecutor() = default;
    
    virtual void onInit(class Interface* interface) = 0;
    virtual void onIdle(class Interface* interface) = 0;
    virtual void onAuto(class Interface* interface) = 0;
    virtual void onManual(class Interface* interface) = 0;
    virtual void onDocking(class Interface* interface) = 0;
    virtual void onUndocking(class Interface* interface) = 0;
    virtual void onCharging(class Interface* interface) = 0;
    virtual void onManipulating(class Interface* interface) = 0;
    virtual void onEmergency(class Interface* interface) = 0;
    virtual void onStop(class Interface* interface) = 0;
    virtual void onSLAM(class Interface* interface) = 0;
    virtual void cancelAllGoals(class Interface* interface) = 0;
    virtual void resumeRobotstate(class Interface* interface) = 0;
};

/**
 * @brief The Interface class manages the robot's state machine.
 *
 * This node handles state transitions, incoming commands (via topics and services),
 * and publishes the current state. It also subscribes to battery and emergency-button topics.
 */
class Interface : public rclcpp::Node {
public:
    using TaskAction = san_msgs::action::TaskCommandAction;
    using GoalHandleTaskAction = rclcpp_action::ServerGoalHandle<TaskAction>;

    Interface();
    ~Interface() override;

    std::string robotStateToString(RobotState state) {
        switch (state) {
            case RobotState::INIT:         return "INIT";
            case RobotState::IDLE:         return "IDLE";
            case RobotState::AUTO:         return "AUTO";
            case RobotState::MANUAL:       return "MANUAL";
            case RobotState::DOCKING:      return "DOCKING";
            case RobotState::UNDOCKING:    return "UNDOCKING";
            case RobotState::CHARGING:     return "CHARGING";
            case RobotState::MANIPULATION: return "MANIPULATION";
            case RobotState::EMERGENCY:    return "EMERGENCY";
            case RobotState::STOP:         return "STOP";
            case RobotState::SLAM:         return "SLAM";
            default:                       return "UNKNOWN";
        }
    }

    // State management functions
    void setState(RobotState new_state);
    RobotState getState() const;
    RobotState getPreviousState() const;

    // Command handling functions
    void commandCallback(const std::string & cmd);

    // Webserver
    void emergencyQuickStopCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // Beckhoff
    void emergencyButtonCallback(const std_msgs::msg::Bool::SharedPtr msg); // Callback for emergency button press
    void resetButtonCallback(const std_msgs::msg::Bool::SharedPtr msg); // Callback for reset button press

    // Core
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msgs);
    void cmdVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);
    
    void manualVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);
    void joyVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);

    // Set the executor that implements state-specific behaviors
    void setExecutor(std::shared_ptr<IStateExecutor> executor);

    // Methods to initialize parameters, publishers, subscribers, and service servers
    void setParam();
    void setCallbackGroups();
    void setSubscriber();
    void setPublisher();
    void setActionServer();
    void setClient();

    void getFullNodeFilePath();

    std::shared_ptr<ParameterUpdater> parameter_updater_;

    std::shared_ptr<NodeManager> node_manager_;
    std::shared_ptr<MapManager> map_manager_;
    std::string webserver_config_file_;
    std::string webserver_map_path_;
    std::string webserver_node_path_;
    std::string full_node_file_path_;
    std::shared_ptr<geometry_msgs::msg::Twist> cmd_vel;
    std::shared_ptr<geometry_msgs::msg::Twist> manual_vel;
    std::shared_ptr<geometry_msgs::msg::Twist> joy_vel;

    // Service callback to process task command requests
    // void taskCommandCallback(
    //     const std::shared_ptr<san_msgs::srv::TaskCommand::Request> request,
    //     std::shared_ptr<san_msgs::srv::TaskCommand::Response> response);

    // Build Map
    void buildMapCallback(
        const std::shared_ptr<san_msgs::srv::BuildMap::Request> request,
        std::shared_ptr<san_msgs::srv::BuildMap::Response> response);
    
    // Pause Task
    void pauseTaskCallback(
        const std::shared_ptr<san_msgs::srv::PauseTask::Request> request,
        std::shared_ptr<san_msgs::srv::PauseTask::Response> response);

    // Change the Map name (추후 커스텀 서비스 변경)
    // void mapNameChangeCallback(const std::shared_ptr<san_msgs::srv::MapName::Request> request,
    //     std::shared_ptr<san_msgs::srv::MapName::Response> response);

    // Request Undocking to Beckhoff
    void call_undocking_service(bool signal, std::function<void(bool)> callback);

    // Timer functions
    void initStateTimer();
    void stateLoopCallback();

    // Node Graph
    void publishNodeGraph();
    void parseNodeGraphFromFile(const std::string& file_path);

    // Public flags and variables for task type
    bool task_load_started_{false};
    bool task_unload_started_{false};
    bool task_move_started_{false};
    bool task_home_started_{false};
    bool task_cancel_started_{false};
    
    bool task_load_completed_{false};
    bool task_unload_completed_{false};
    bool task_move_completed_{false};
    bool task_home_completed_{false};
    bool task_cancel_completed_{false};

    bool task_one_move_started_{false};
    bool task_one_move_completed_{false};

    // Public flags and variables for behavior type
    bool behavior_auto_started_{false};
    bool behavior_slam_started_{false};
    bool behavior_charge_started_{false};
    bool behavior_docking_started_{false};
    bool behavior_undocking_started_{false};

    bool behavior_auto_canceled_{false};
    bool behavior_docking_canceled_{false};

    bool behavior_auto_completed_{false};
    bool behavior_slam_completed_{false};
    bool behavior_charge_completed_{false};
    bool behavior_docking_completed_{false};
    bool behavior_undocking_completed_{false};
    
    // Redocking Behavior Triggered
    bool behavior_manipulation_redocking_triggered_{false};
    bool behavior_charging_redocking_triggered_{false};

    // Manipulation Behaviors Triggered
    bool behavior_manipulator_completed_{false};
    bool behavior_manipulator_emergency_triggered_{false};

    // Manual Mode Trigger
    bool behavior_manual_mode_triggered_{false};

    bool build_map_requested_{false};      ///< Flag indicating if a "Build Map Start Request" task is requested
    bool build_map_completed_{false};      ///< Flag indicating if a "Build Map Completed" task is requested
    
    bool cancel_map_requested_{false};      ///< Flag indicating if a "Cancel Building Map Request" task is requested
    bool cancel_map_completed_{false};      ///< Flag indicating if a "Cancel Building Map Completed" task is requested
    
    bool save_map_requested_{false};    ///< Flag indicating if a "Save Map Request" task is requested
    bool save_map_completed_{false};    ///< Flag indicating if a "Save Map Completed" task is requested

    bool running_navigation_{false};    ///< Flag indicating if a "HOME" task is requested
    bool running_slam_{false};    ///< Flag indicating if a "HOME" task is requested

    bool change_current_map_requested_{false};    ///< Flag indicating if a "HOME" task is requested
    
    bool odometry_subscribed_{false};

    bool emergency_quick_stop_triggered_{false};
        
    std::string parameter_mode_{"UNKNOWN"};

    geometry_msgs::msg::PoseStamped goal_position_; ///< Target goal position (used by navigation)
    geometry_msgs::msg::PoseArray current_waypoints_;
    geometry_msgs::msg::PoseArray last_waypoints_;
    geometry_msgs::msg::PoseArray prev_last_waypoints_; // 이전의 이전 웨이포인트
    std::string save_map_requested_name_;

    // Docking arameter
    double charging_marker_gap_;
    double working_marker_gap_;
    int marker_id_;

    int state_timer_period_ms_int_;

    // Emergency Triggered
    BatteryData battery_data_;

    bool emergency_button_triggered_{false};  ///< Flag set when the emergency button is pressed
    bool sto_triggered_{false};
    bool bumper_collision_triggered_{false};
    bool uss_emergency_triggered_{false};
    bool motor_emergency_triggered_{false};
    bool motor_battery_charging_triggered_{false};

    bool reset_button_triggered_{false};

    bool navigation_goal_sent_ = false;
    bool docking_goal_sent_ = false;
    
    // Undocking delay counter (for 3 second delay)
    int undocking_delay_counter_ = 0;
    bool undocking_delay_started_ = false;

    bool distance_in_threshold_{false};

    // USS Data
    UltrasonicData uss_data;

    // Manipulator Trigger
    bool obstacle_detected_{false};

    uint8_t motor_status_;    

    double waypoint_reach_threshold_;
    double waypoint_missed_distance_;
    // STOP -> Resume Flag
    bool resume_requested_{false};

protected:
    // ROS2 subscriptions, publishers, and service servers
    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr emergency_group_;
    rclcpp::CallbackGroup::SharedPtr sensor_group_;
    rclcpp::CallbackGroup::SharedPtr core_group_;
    rclcpp::CallbackGroup::SharedPtr web_group_;
    rclcpp::CallbackGroup::SharedPtr manipulator_group_;

    // Subscription Options
    rclcpp::SubscriptionOptions emergency_opts_;
    rclcpp::SubscriptionOptions sensor_opts_;
    rclcpp::SubscriptionOptions core_opts_;
    rclcpp::SubscriptionOptions web_opts_;
    rclcpp::SubscriptionOptions manipulator_opts_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr change_map_receive_subscriber_;
    
    // Core
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_vel_sub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

    // Manipulator
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr manipulator_state_machine_subscriber_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manipulator_mission_service_;


    rclcpp::Subscription<san_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr bumper_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr ultrasonic_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_button_sub_; // For emergency button
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_button_sub_; // For emergency button
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sto_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<san_msgs::msg::Behavior>::SharedPtr behavior_pub_;
    rclcpp::Publisher<san_msgs::msg::Task>::SharedPtr task_pub_;

    /********************** HAMR ************************/
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr driving_status_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr driving_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr order_status_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr path_status_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr error_code_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_code_desc_pub_;

    rclcpp::Service<san_msgs::srv::BuildMap>::SharedPtr build_map_service_; // Build Map

    rclcpp::Service<san_msgs::srv::PauseTask>::SharedPtr pause_task_service_; // Pause Task

    rclcpp::Client<san_msgs::srv::GetWaypoints>::SharedPtr get_waypoints_client_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr undocking_client_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_quick_stop_sub_;

    rclcpp::Subscription<san_msgs::msg::MotorStatus>::SharedPtr motor_status_sub_;

private:

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::TimerBase::SharedPtr state_machine_timer_;
    RobotState current_state_;
    RobotState previous_state_;


    double odom_timeout_;
    rclcpp::Time last_odom_msg_time_;
    rclcpp::Time last_manual_msg_time_;


    // Internal command flags
    bool docking_requested_;
    bool emergency_triggered_;
    
    // Pointer to the state executor (implements state-specific behavior)
    std::shared_ptr<IStateExecutor> executor_;

    // Internal callback wrapper functions
    void commandMsgCallback(const std_msgs::msg::String::SharedPtr msg);
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void waypointListCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void changeMapCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void batteryStatusCallback(const san_msgs::msg::BatteryStatus::SharedPtr msg);
    void bumperStatusCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void ultrasonicStatusCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
    void stoStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void motorStatusCallback(const san_msgs::msg::MotorStatus::SharedPtr msg);


    // Manipulator
    void manipulatorStateMachineCallback(const std_msgs::msg::String::SharedPtr msg);
    void manipulatorMissionService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Action Server
    rclcpp_action::Server<TaskAction>::SharedPtr task_action_server_;
    
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const TaskAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleTaskAction> goal_handle);

    void handle_accepted(
        const std::shared_ptr<GoalHandleTaskAction> goal_handle);
    
    void execute_task(
        const std::shared_ptr<GoalHandleTaskAction> goal_handle);

    std::shared_ptr<GoalHandleTaskAction> current_goal_handle_;

    // Track all active TaskAction goal handles for cancellation
    std::set<std::shared_ptr<GoalHandleTaskAction>> active_goal_handles_;
   
    void process_mission(const std::shared_ptr<GoalHandleTaskAction> goal_handle, const std::string& mission);

    void publishPathStatus();
    void publishDrivingStatusAndMode();
    void publishTaskBehaviorStatus();
   
    std::unique_ptr<PathStatusManager> path_status_manager_;
    geometry_msgs::msg::Pose last_robot_pose_;

    // Node graph visualization
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr node_graph_pub_;
    rclcpp::TimerBase::SharedPtr node_graph_timer_;
    std::vector<geometry_msgs::msg::Point> node_positions_;
    std::vector<std::pair<int, int>> node_edges_;

};  

} // namespace AMR

#endif // __INTERFACE_HPP__
