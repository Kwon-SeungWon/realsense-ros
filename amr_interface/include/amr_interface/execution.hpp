#ifndef __EXECUTION_HPP__
#define __EXECUTION_HPP__

#include <rclcpp/rclcpp.hpp>

#include "amr_interface/interface.hpp"  // Contains definition of Interface and IStateExecutor
#include "amr_interface/parameter_updater.hpp" 

#include <future>
#include <chrono>
#include <thread>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp" 
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

#include "san_msgs/action/marker_docking.hpp"
// #include "san_msgs/srv/update_params.hpp" 


namespace AMR {

/**
 * @brief StateExecutor implements the IStateExecutor interface, providing concrete
 * implementations for robot state actions. These methods are invoked by the Interface node
 * to perform specific behaviors corresponding to the current robot state.
 */
class StateExecutor : public IStateExecutor {
public:
    StateExecutor() = default;
    virtual ~StateExecutor() = default;

    // IStateExecutor interface methods:
    /**
     * @brief Initializes the robot systems (e.g., sensors, parameters) and transitions to IDLE.
     */
    void onInit(Interface* interface) override;

    /**
     * @brief Waits for a task command in the IDLE state.
     */
    void onIdle(Interface* interface) override;

    /**
     * @brief Prepares for task execution by sending a navigation goal to the action server,
     * then transitions to the DOCKING state.
     */
    void onAuto(Interface* interface) override;

    /**
     * @brief Processes manual control commands.
     */
    void onManual(Interface* interface) override;

    /**
     * @brief Attempts to dock the robot. Based on the current work/charge request flag,
     * transitions to either MANIPULATION or CHARGING.
     */
    void onDocking(Interface* interface) override;

    /**
     * @brief Attempts to undock the robot. Based on the current load/unload/charge completed flag,
     * transitions to IDLE
     */
    void onUndocking(Interface* interface) override;

    /**
     * @brief Executes the charging procedure and returns to IDLE when complete.
     */
    void onCharging(Interface* interface) override;

    /**
     * @brief Executes the manipulating task and transitions back to IDLE.
     */
    void onManipulating(Interface* interface) override;

    /**
     * @brief Handles emergency situations by stopping all operations and transitioning to EMERGENCY.
     */
    void onEmergency(Interface* interface) override;

    /**
     * @brief Keeps the robot in a halted state.
     */
    void onStop(Interface* interface) override;

    /**
     * @brief Executes the robot in a SLAM state.
     */
    void onSLAM(Interface* interface) override;

    // Cancel all active navigation/docking action goals (for use by Interface on cancel)
    void cancelAllGoals(Interface* interface) override;

    // Resume RobotState
    void resumeRobotstate(Interface* interface) override;
private:
    // Initialization helper functions
    bool checkOdometry(Interface* interface);
    void handleResetButton(Interface* interface);
    void handleMapInitialization(Interface* interface);
    bool shouldStartNavigation(Interface* interface);
    void startNavigation(Interface* interface);
    void callReinitializeGlobalLocalization(Interface* interface);

    // Idle state helper functions
    bool handleMapRequests(Interface* interface);
    bool handleSaveOrCancelMap(Interface* interface);
    bool handleMapChange(Interface* interface);

    // SLAM-related helper functions
    void handleBuildMap(Interface* interface);
    void handleCancelSLAM(Interface* interface);
    void handleSaveMap(Interface* interface);
    // void resumeRobotstate(Interface* interface);

    /**
     * @brief Helper function to send a navigation goal to the Nav2 action server.
     * This function creates an action client for the NavigateToPose action, sends a goal
     * using the Interface's stored target pose, and sets up the necessary callbacks.
     *
     * @param interface Pointer to the Interface node that stores the target goal.
     * @param action_name The name of the navigation action server topic.
     */
    template<typename ActionT>
    void sendGoalAsync(
        Interface* interface,
        const std::string & action_name,
        std::function<void(typename ActionT::Goal&)> fill_goal,
        std::function<void(const typename rclcpp_action::Client<ActionT>::WrappedResult &)> result_callback);
    
    // Helper: Check if a goal handle is active (template for each action type)
    template<typename ActionT>
    static bool is_goal_active(const std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>> &handle) {
        if (!handle) return false;
        auto status = handle->get_status();
        return status == rclcpp_action::GoalStatus::STATUS_EXECUTING || status == rclcpp_action::GoalStatus::STATUS_ACCEPTED;
    }
    
    // navigation, docking wrapper functions
    void sendNavigationGoalAsync(Interface* interface, geometry_msgs::msg::PoseStamped & goal_position);
    void sendWaypointGoalAsync(Interface* interface, const geometry_msgs::msg::PoseArray & waypoint_list);
    void sendNavigateThroughPoses(Interface* interface, const geometry_msgs::msg::PoseArray & waypoint_list);
    void sendDockingGoalAsync(Interface* interface, const std::string & camera_mode, double & marker_gap, int & marker_id); // 추후 marker_id 추가
    void updateParameters(Interface* interface, const std::string & mode);

    std::string current_map_file;
    int manipulation_count;

    bool current_map_checker_{false};

    void ussChecker();

    // Store the latest goal handles for cancellation
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_action_client_;
    rclcpp_action::Client<san_msgs::action::MarkerDocking>::SharedPtr docking_action_client_;
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_action_client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav_goal_handle_ = nullptr;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_goal_handle_ = nullptr;
    rclcpp_action::ClientGoalHandle<san_msgs::action::MarkerDocking>::SharedPtr docking_goal_handle_ = nullptr;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_goal_handle_ = nullptr;

};

/**
 * @brief ExecutionNode is a separate ROS2 node that monitors the docking process.
 * It uses a timer to periodically check if the docking state has timed out and,
 * if so, initiates a recovery sequence (e.g., emergency stop and reattempt docking).
 */
class ExecutionNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Execution Node.
     *
     * @param state_machine A shared pointer to the Interface node that manages the state machine.
     */
    ExecutionNode(std::shared_ptr<Interface> interface);
    virtual ~ExecutionNode() = default;

private:
    std::shared_ptr<Interface> interface;         ///< Pointer to the Interface node (state machine)
    rclcpp::TimerBase::SharedPtr check_timer_;         ///< Timer for monitoring docking timeout
    rclcpp::Time docking_start_time_;                    ///< Timestamp when docking started
    rclcpp::Time previous_soc_check_time_;   
    
    bool docking_in_progress_ = false;                   ///< Flag indicating if docking is in progress

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_publisher_;

    // rclcpp::Client<san_msgs::srv::UpdateParams>::SharedPtr update_client_;                ///< Client for the update_params service.

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_publisher_;


    // Parameters
    double goal_distance_threshold_;  ///< Threshold distance (in meters) to trigger the parameter update.
    bool parameters_updated_;        ///< Flag to ensure the service is called only once per goal.
    double charging_timeout_;
    double charging_relay_timeout_;
    std::string filtered_scan_topic_;
    double obstacle_distance_threshold_;  // 로봇팔 회전반경
    int min_cluster_size_; // 3개의 연속된 점
    int uss_threshold_;
    double dynamic_threshold_;

    std::vector<float> prev_ranges_;
    bool tracking_active_{false};

    // Latest poses
    geometry_msgs::msg::PoseStamped current_pose_;  ///< Current robot pose (obtained via TF2).
    geometry_msgs::msg::PoseStamped goal_pose_;     ///< Latest goal pose (from Nav2).

    // TF2 objects for looking up the robot's current pose.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<ParameterUpdater> parameter_updater_;

    // Battery_SoC
    double previous_soc_;

    /**
     * @brief Checks if the docking process has exceeded the timeout threshold.
     * If the timeout is exceeded, initiates a recovery sequence by sending emergency,
     * stop, auto, and dock commands.
     */
    void loopChecker();

    /**
     * @brief Timer callback that uses TF2 to update the current robot pose.
     *
     * Looks up the transform from "map" to "base_link" and stores it as the current pose, then checks
     * if the distance to the goal is within the threshold.
     */
    void updateCurrentPose();

    /**
     * @brief Checks the distance between the current pose (obtained via TF2) and the goal pose.
     *
     * If the robot is within the threshold distance and the service has not yet been called for this goal,
     * it triggers the update service.
     */
    void currentPoseChecker();

    /**
     * @brief Calls the "update_params" service to update parameters.
     *
     * Waits for the service to be available, sends a request, and logs the result.
     */
    void updateParameters(const std::string & mode);

    bool batteryChecker();

    void obstacleChecker(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void ussChecker();
};

} // end namespace amr

#endif // __EXECUTION_HPP__