#include "gripper.h"

namespace pandaria_ros
{

/**
 * @brief Wrapper class for controlling the franka hand
 *
 */
Gripper::Gripper()
    : grasp_client_("franka_gripper/grasp", false), stop_client_("franka_gripper/stop", false), move_client_("franka_gripper/move", false),
      home_client_("franka_gripper/homing", false)
{
    ros::Rate rate(0.1);
    while (!grasp_client_.isServerConnected() || !stop_client_.isServerConnected() || !move_client_.isServerConnected() ||
           !home_client_.isServerConnected())
    {
        ROS_INFO_DELAYED_THROTTLE(1, "Waiting for gripper services to be advertised...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Gripper connected");
}

/**
 * @brief Home the gripper (i.e. calibrate fingers)
 *
 * @return successful?
 */
void Gripper::home()
{
    ROS_INFO("Homing gripper");
    home_client_.sendGoal(franka_gripper::HomingGoal());
}

/**
 * @brief Wait until previously called home-action finishes
 *
 * @return successful?
 */
bool Gripper::waitForHomingToFinish()
{
    if (home_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
        (home_client_.waitForResult(ros::Duration(10.0)) && home_client_.getResult()->success))
        return true;
    else
    {
        ROS_ERROR_STREAM("Homing gripper was not successful: " << home_client_.getState().getText());
        return false;
    }
}

/**
 * @brief Home the gripper (i.e. calibrate fingers) and wait for finish
 *
 * @return successful?
 */
bool Gripper::homeAndWait()
{
    this->home();
    return this->waitForHomingToFinish();
}

/**
 * @brief Move gripper fingers without applying force
 * Use \ref Gripper::grasp to grasp an object
 *
 * @return successful?
 */
void Gripper::move(const double width, const double move_speed)
{
    ROS_INFO_STREAM("Moving fingers to width " << width << "m");
    franka_gripper::MoveGoal goal;
    goal.width = width;
    goal.speed = move_speed;
    move_client_.sendGoal(goal);
}

/**
 * @brief Wait until previously called move-action finishes
 *
 * @return successful?
 */
bool Gripper::waitForMovingToFinish()
{

    if (move_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
        (move_client_.waitForResult(ros::Duration(10)) && move_client_.getResult()->success))
    {
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("Moving fingers was not successful");
        return false;
    }
}

/**
 * @brief Move gripper fingers without applying force and wait for finish
 * Use \ref Gripper::grasp to grasp an object
 *
 * @return successful?
 */
bool Gripper::moveAndWait(const double width, const double move_speed)
{
    this->move(width, move_speed);
    return this->waitForMovingToFinish();
}

/**
 * @brief Grasp an object
 *
 * @param width expected width of object (distance between fingers)
 * @param grasp_force desired grasping force
 * @param move_speed speed of fingers
 * @param max_delta_width_inner final grasp distance must be larger than \arg width - \arg max_delta_width_inner
 * @param max_delta_width_outer final grasp distance must be smaller than \arg width + \arg max_delta_width_outer
 * @return successful?
 */
bool Gripper::grasp(const double width, const double grasp_force, const double move_speed, const double max_delta_width_inner,
                    const double max_delta_width_outer)
{
    ROS_INFO_STREAM("Grasping with width " << width << "m");
    franka_gripper::GraspGoal goal;
    goal.width         = width;
    goal.speed         = move_speed;
    goal.force         = grasp_force;
    goal.epsilon.inner = max_delta_width_inner;
    goal.epsilon.outer = max_delta_width_outer;
    grasp_client_.sendGoal(goal);
    if (grasp_client_.waitForResult(ros::Duration(10)) && grasp_client_.getResult()->success)
    {
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("Grasping with width " << width << "m was not successful");
        return false;
    }
}

/**
 * @brief Stop current gripper action. Can be used to stop applying forces after grasping.
 *
 * @return successful?
 */
bool Gripper::stop()
{
    ROS_INFO_STREAM("Halting gripper action");
    franka_gripper::StopGoal goal;
    stop_client_.sendGoal(goal);
    if (stop_client_.waitForResult(ros::Duration(2)) && stop_client_.getResult()->success)
    {
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("Failed to stop gripper action");
        return false;
    }
}

}  // namespace pandaria_ros