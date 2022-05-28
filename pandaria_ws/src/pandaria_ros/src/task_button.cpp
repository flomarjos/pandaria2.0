#include "task_button.h"
#include "helpers.hpp"
#include "yaml_parser.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

namespace pandaria_ros
{

TaskButton::TaskButton(ros::NodeHandle &node, const XmlRpc::XmlRpcValue &parameter)
{
    // Read button pose from parameters
    button_top_pose_1.setIdentity();
    button_top_pose_2.setIdentity();
    button_press_.setIdentity();
    safe_point_pose_.setIdentity();

    Eigen::VectorXd translation;
    if (yaml_parser::yamlReadMember(parameter, "translation_aboveButton1", &translation) && translation.rows() == 3)
        button_top_pose_1.translation() = translation;
    else
        ROS_ERROR("Failed to parse button1 translation");

    Eigen::VectorXd orientation;
    if (yaml_parser::yamlReadMember(parameter, "orientation_aboveButton1", &orientation) && orientation.rows() == 4)
    {
        button_top_pose_1.rotate(Eigen::Quaterniond(orientation.coeff(0), orientation.coeff(1), orientation.coeff(2), orientation.coeff(3)));
    }
    else
        ROS_ERROR("Failed to parse button1 orientation");

    if (yaml_parser::yamlReadMember(parameter, "translation_aboveButton2", &translation) && translation.rows() == 3)
        button_top_pose_2.translation() = translation;
    else
        ROS_ERROR("Failed to parse button2 translation");

    if (yaml_parser::yamlReadMember(parameter, "orientation_aboveButton2", &orientation) && orientation.rows() == 4)
    {
        button_top_pose_2.rotate(Eigen::Quaterniond(orientation.coeff(0), orientation.coeff(1), orientation.coeff(2), orientation.coeff(3)));
    }
    else
        ROS_ERROR("Failed to parse button2 orientation");

    // translation.setZero();
    // orientation.setZero();
    if (yaml_parser::yamlReadMember(parameter, "translation_press_button", &translation) && translation.rows() == 3)
        button_press_.translation() = translation;
    else
        ROS_ERROR("Failed to parse button press translation");

    if (yaml_parser::yamlReadMember(parameter, "orientation_press_button", &orientation) && orientation.rows() == 4)
    {
        button_press_.rotate(Eigen::Quaterniond(orientation.coeff(0), orientation.coeff(1), orientation.coeff(2), orientation.coeff(3)));
    }
    else
        ROS_ERROR("Failed to parse button press orientation");

    // translation.setZero();
    // orientation.setZero();
    if (yaml_parser::yamlReadMember(parameter, "translation_safe_point", &translation) && translation.rows() == 3)
        safe_point_pose_.translation() = translation;
    else
        ROS_ERROR("Failed to parse button safe point translation");

    if (yaml_parser::yamlReadMember(parameter, "orientation_safe_point", &orientation) && orientation.rows() == 4)
    {
        safe_point_pose_.rotate(Eigen::Quaterniond(orientation.coeff(0), orientation.coeff(1), orientation.coeff(2), orientation.coeff(3)));
    }
    else
        ROS_ERROR("Failed to parse button safe point orientation");

    goal_publisher_ = node.advertise<geometry_msgs::PoseStamped>("equilibrium_pose", 1);
}

/**
 * @brief Execute task
 *
 * @param tf_buffer Buffer which knows transforms from robot base to taskboard
 * @return true on success
 * @return false on failure
 */
bool TaskButton::execute(tf2_ros::Buffer &tf_buffer, Planner &planner, const std::shared_ptr<Gripper> &gripper,
                         const std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> follow_joint_traj_client,
                         const std::shared_ptr<ros::ServiceClient> controller_manager_client)
{
    ROS_INFO("Executing button task");
    std::string err_string;
    if (!tf_buffer.canTransform("panda_link0", "taskboard_refined_stable", ros::Time(0), ros::Duration(0), &err_string))
    {
        ROS_ERROR_STREAM("Failed to execute button task: " << err_string);
        return false;
    }

    // close gripper
    gripper->move(0.005, 0.1);

    Eigen::Isometry3d tf_goal;
    tf_goal = tf2::transformToEigen(tf_buffer.lookupTransform("panda_link0", "taskboard_refined_stable", ros::Time(0))) * button_top_pose_1;
    // tf_goal.translate(Eigen::Vector3d(0, 0, -0.12));

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "panda_link0";
    goal.pose            = tf2::toMsg(tf_goal);

    // plan trajectory
    planning_interface::MotionPlanResponse traj = planner.planTrajectoryFromCurrentPose(goal, 0.3, 0.2);
    if (traj.error_code_.val != traj.error_code_.SUCCESS)
    {
        // TODO what now?
        ROS_ERROR("Button task: failed to plan trajectory");
        return false;
    }

    // convert moveit-trajectory-msg to control_msgs
    moveit_msgs::RobotTrajectory moveitTraj;
    traj.trajectory_->getRobotTrajectoryMsg(moveitTraj);
    control_msgs::FollowJointTrajectoryGoal goalTraj;
    goalTraj.trajectory          = moveitTraj.joint_trajectory;
    goalTraj.goal_time_tolerance = ros::Duration(5);  // 5s tolerance

    ROS_INFO("Button task: executing trajectory");
    actionlib::SimpleClientGoalState goalState = follow_joint_traj_client->sendGoalAndWait(goalTraj, ros::Duration(60));
    if (goalState.state_ != goalState.SUCCEEDED)
    {
        // TODO what now?
        ROS_ERROR("Button task: Failed to execute trajectory");
        return false;
    }

    ros::Duration(1).sleep();

    tf_goal = tf2::transformToEigen(tf_buffer.lookupTransform("panda_link0", "taskboard_refined_stable", ros::Time(0))) * button_top_pose_2;
    // tf_goal.translate(Eigen::Vector3d(0, 0, -0.12));

    goal.header.frame_id = "panda_link0";
    goal.pose            = tf2::toMsg(tf_goal);

    // plan trajectory
    traj = planner.planTrajectoryFromCurrentPose(goal, 0.3, 0.2);
    if (traj.error_code_.val != traj.error_code_.SUCCESS)
    {
        // TODO what now?
        ROS_ERROR("Button task: failed to plan trajectory");
        return false;
    }

    // convert moveit-trajectory-msg to control_msgs

    traj.trajectory_->getRobotTrajectoryMsg(moveitTraj);
    goalTraj.trajectory          = moveitTraj.joint_trajectory;
    goalTraj.goal_time_tolerance = ros::Duration(5);  // 5s tolerance

    ROS_INFO("Button task: executing trajectory");
    goalState = follow_joint_traj_client->sendGoalAndWait(goalTraj, ros::Duration(60));
    if (goalState.state_ != goalState.SUCCEEDED)
    {
        // TODO what now?
        ROS_ERROR("Button task: Failed to execute trajectory");
        return false;
    }

    ros::Duration(1).sleep();

    // switch controller
    if (!helpers::switchControllers("cartesian_impedance_controller", "effort_joint_trajectory_controller", controller_manager_client))
    {
        ROS_ERROR("Failed to switched controllers");
        return false;
    }
    else
        ROS_INFO("Switched controllers");

    geometry_msgs::PoseStamped goal_cart;
    Eigen::Isometry3d res = tf2::transformToEigen(tf_buffer.lookupTransform("panda_link0", "taskboard_refined_stable", ros::Time(0))) * button_press_;
    goal_cart.pose        = tf2::toMsg(res);

    // hard-coded height
    std::cout << "-------------------goal_cart.pose.position.z:----------------------" << goal_cart.pose.position.z << std::endl;
    // Eigen::Quaterniond q(res.linear());
    // goal_cart.pose.orientation.y = q.y();
    // goal_cart.pose.orientation.z = q.z();
    // goal_cart.pose.orientation.w = q.w();

    goal_cart.header.frame_id = "panda_link0";
    goal_cart.header.stamp    = ros::Time::now();

    goal_publisher_.publish(goal_cart);

    // if (!gripper->waitForMovingToFinish())
    //     return false;

    // TODO do next task command (button task may not require any other commands)
    // ...

    ros::Duration(4).sleep();

    // switch controller
    if (!helpers::switchControllers("effort_joint_trajectory_controller", "cartesian_impedance_controller", controller_manager_client))
    {
        return false;
    }
    else
        ROS_INFO("Switched controllers");

    Eigen::Isometry3d tf_goal_back;
    tf_goal_back = tf2::transformToEigen(tf_buffer.lookupTransform("panda_link0", "taskboard_refined_stable", ros::Time(0))) * safe_point_pose_;

    geometry_msgs::PoseStamped goal_back;
    goal_back.header.frame_id = "panda_link0";
    goal_back.pose            = tf2::toMsg(tf_goal_back);

    planning_interface::MotionPlanResponse traj_back = planner.planTrajectoryFromCurrentPose(goal_back, 0.3, 0.1);

    // convert moveit-trajectory-msg to control_msgs
    moveit_msgs::RobotTrajectory moveitTraj_back;
    traj_back.trajectory_->getRobotTrajectoryMsg(moveitTraj_back);
    control_msgs::FollowJointTrajectoryGoal goalTraj_back;
    goalTraj_back.trajectory          = moveitTraj_back.joint_trajectory;
    goalTraj_back.goal_time_tolerance = ros::Duration(5);  // 5s tolerance

    ROS_INFO("Button task finished: going to safe point.");
    actionlib::SimpleClientGoalState goalState_back = follow_joint_traj_client->sendGoalAndWait(goalTraj_back, ros::Duration(60));

    if (goalState_back.state_ != goalState_back.SUCCEEDED)
    {
        ROS_ERROR("Button task: Failed to execute trajectory");
        return false;
    }

    return true;
}

}  // namespace pandaria_ros