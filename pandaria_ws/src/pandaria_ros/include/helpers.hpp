#pragma once

#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "controller_manager_msgs/SwitchController.h"
#include "moveit/planning_interface/planning_response.h"
#include "ros/service_client.h"

namespace pandaria_ros
{

namespace helpers
{

/**
 * @brief Switch low-level controllers
 *
 * @return true Success
 * @return false Failed
 */
inline bool switchControllers(std::vector<std::string> &start_controllers, std::vector<std::string> &stop_controllers,
                              const std::shared_ptr<ros::ServiceClient> controller_manager_client)
{
    controller_manager_msgs::SwitchControllerRequest msg;
    msg.start_controllers = start_controllers;
    msg.stop_controllers  = stop_controllers;
    msg.strictness        = controller_manager_msgs::SwitchController::Request::STRICT;
    msg.timeout           = 1;

    controller_manager_msgs::SwitchControllerResponse res;
    if (controller_manager_client->call(msg, res) && res.ok)
        return true;
    else
    {
        ROS_ERROR("Failed to switch controllers.");
        return false;
    }
}

/**
 * @brief Switch low-level controllers
 *
 * @return true Success
 * @return false Failed
 */
inline bool switchControllers(const std::string start_controller, const std::string stop_controller,
                              const std::shared_ptr<ros::ServiceClient> controller_manager_client)
{
    std::vector<std::string> start_controllers({start_controller});
    std::vector<std::string> stop_controllers({stop_controller});
    return switchControllers(start_controllers, stop_controllers, controller_manager_client);
}

inline control_msgs::FollowJointTrajectoryGoal motionPlan2JointTrajectoryGoal(const planning_interface::MotionPlanResponse &response)
{
    moveit_msgs::RobotTrajectory moveitTraj_tmp;
    response.trajectory_->getRobotTrajectoryMsg(moveitTraj_tmp);

    control_msgs::FollowJointTrajectoryGoal goalTraj;
    goalTraj.trajectory = moveitTraj_tmp.joint_trajectory;
    return goalTraj;
}

}  // namespace helpers

}  // namespace pandaria_ros