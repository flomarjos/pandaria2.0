#include "task_start_pose.h"
#include "helpers.hpp"
#include "yaml_parser.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <tf2_eigen/tf2_eigen.h>

namespace pandaria_ros
{

TaskStartPose::TaskStartPose(ros::NodeHandle &node, const XmlRpc::XmlRpcValue &parameter)
{
    // Read button pose from parameters
    ROS_INFO("Start Pose Task initialized");
    start_pose_.setIdentity();
    pose_above_center_.setIdentity();

    Eigen::VectorXd tmp;
    if (yaml_parser::yamlReadMember(parameter, "translation_inital", &tmp) && tmp.rows() == 3)
        start_pose_.translation() = tmp;
    else
        ROS_ERROR("Failed to parse start pose translation");

    if (yaml_parser::yamlReadMember(parameter, "orientation", &tmp) && tmp.rows() == 4)
    {
        start_pose_.rotate(Eigen::Quaterniond(tmp.coeff(0), tmp.coeff(1), tmp.coeff(2), tmp.coeff(3)));
    }
    else
        ROS_ERROR("Failed to parse start pose orientation");

    if (yaml_parser::yamlReadMember(parameter, "translation_above_center", &tmp) && tmp.rows() == 3)
        pose_above_center_.translation() = tmp;
    else
        ROS_ERROR("Failed to parse translation above center point");

    if (yaml_parser::yamlReadMember(parameter, "orientation_above_center", &tmp) && tmp.rows() == 4)
    {
        pose_above_center_.rotate(Eigen::Quaterniond(tmp.coeff(0), tmp.coeff(1), tmp.coeff(2), tmp.coeff(3)));
    }
    else
        ROS_ERROR("Failed to parse start pose orientation");

    start_filter_center_  = node.advertise<std_msgs::Int64>("/startFilterCenter", 1);
    start_filter_refined_ = node.advertise<std_msgs::Int64>("/startFilterRefined", 1);
}

/**
 * @brief Execute task
 *
 * @param tf_buffer
 * @return true on success
 * @return false on failure
 */
bool TaskStartPose::execute(tf2_ros::Buffer &tf_buffer, Planner &planner, const std::shared_ptr<Gripper> &gripper,
                            const std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> follow_joint_traj_client,
                            const std::shared_ptr<ros::ServiceClient> controller_manager_client)
{

    ROS_INFO("Executing start pose task");

    // close gripper
    gripper->move(0.005, 0.1);

    // switch controller
    if (!helpers::switchControllers("effort_joint_trajectory_controller", "cartesian_impedance_controller", controller_manager_client))
    {
        return false;
    }
    else
        ROS_INFO("Switched controllers");

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "panda_link0";
    goal.pose            = tf2::toMsg(start_pose_);

    std::string err_string;
    float angle                = 0;
    const float originX        = goal.pose.position.x;
    const float originY        = goal.pose.position.y;
    constexpr float radius     = 0.12;
    constexpr float vel_factor = 0.2;

    constexpr size_t center_filter_size = 5;
    Eigen::Matrix<double, 3, center_filter_size> center_filter_buffer;
    size_t current_col = 0;
    geometry_msgs::TransformStamped center;

    // start filter
    std_msgs::Int64 startFilterMsg;
    startFilterMsg.data = 1;

    ros::Rate rate(2);
    while (ros::ok() && !tf_buffer.canTransform("panda_link0", "taskboard_center_stable", ros::Time(0)))
    {
        start_filter_center_.publish(startFilterMsg);
        ros::spinOnce();

        ROS_INFO_STREAM_DELAYED_THROTTLE(2, "Searching for taskboard");

        planning_interface::MotionPlanResponse plan_response = planner.planTrajectoryFromCurrentPose(goal, vel_factor);

        if (plan_response.error_code_.val != plan_response.error_code_.SUCCESS)  // TODO what now?
        {
            ROS_ERROR("Start pose task: failed to plan trajectory");
            return false;
        }

        // convert moveit-trajectory-msg to control_msgs
        auto goalTraj                = helpers::motionPlan2JointTrajectoryGoal(plan_response);
        goalTraj.goal_time_tolerance = ros::Duration(5);  // 5s tolerance

        ROS_INFO("Start pose task: executing trajectory");
        actionlib::SimpleClientGoalState goalState = follow_joint_traj_client->sendGoalAndWait(goalTraj, ros::Duration(60));
        if (goalState.state_ != goalState.SUCCEEDED)  // TODO what now?
        {
            ROS_ERROR_STREAM("Start pose task: Failed to execute trajectory -> goalState: " << goalState.toString());
            return false;
        }

        // append new poses

        float X              = originX + cos(angle) * radius;
        float Y              = originY + sin(angle) * radius;
        goal.pose.position.x = X;
        goal.pose.position.y = Y;
        angle += -0.2;

        ROS_INFO_STREAM_DELAYED_THROTTLE(2, "Waiting for taskboard_center " << err_string);
        rate.sleep();
    }

    Eigen::Isometry3d above_center_pose;
    above_center_pose = tf2::transformToEigen(tf_buffer.lookupTransform("panda_link0", "taskboard_center_stable", ros::Time(0))) * pose_above_center_;
    above_center_pose = above_center_pose * tf2::transformToEigen(tf_buffer.lookupTransform("camera", "panda_link8", ros::Time(0)));
    goal.pose         = tf2::toMsg(above_center_pose);

    // plan
    planning_interface::MotionPlanResponse plan_response = planner.planTrajectoryFromCurrentPose(goal, vel_factor);
    if (plan_response.error_code_.val != plan_response.error_code_.SUCCESS)  // TODO what now?
    {
        ROS_ERROR("Start pose task: failed to plan trajectory");
        return false;
    }

    // convert moveit-trajectory-msg to control_msgs
    auto goalTraj                = helpers::motionPlan2JointTrajectoryGoal(plan_response);
    goalTraj.goal_time_tolerance = ros::Duration(5);  // 5s tolerance

    // execute
    ROS_INFO("Start pose task: moving to taskboard center");
    actionlib::SimpleClientGoalState goalState = follow_joint_traj_client->sendGoalAndWait(goalTraj, ros::Duration(60));
    if (goalState.state_ != goalState.SUCCEEDED)  //  TODO what now?
    {
        ROS_ERROR_STREAM("Start pose task: Failed to moving to center");
        return false;
    }

    while (ros::ok() && !tf_buffer.canTransform("panda_link0", "taskboard_refined_stable", ros::Time(0)))
    {
        rate.sleep();
        start_filter_refined_.publish(startFilterMsg);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Located taskboard");

    return true;
}

}  // namespace pandaria_ros