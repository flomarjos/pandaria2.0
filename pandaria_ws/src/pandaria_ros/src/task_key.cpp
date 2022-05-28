#include "task_key.h"
#include "helpers.hpp"
#include "yaml_parser.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

namespace pandaria_ros
{

TaskKey::TaskKey(ros::NodeHandle &node, const XmlRpc::XmlRpcValue &parameter)
{
    // Read Key pose from parameters
    key_pick_supp_pose_.setIdentity();
    key_pick_pose_.setIdentity();
    key_mid_pose_.setIdentity();
    key_mid_pose_2.setIdentity();
    key_insert_pose_.setIdentity();

    // For Key Pick Task
    Eigen::VectorXd translation;
    if (yaml_parser::yamlReadMember(parameter, "translation_pick", &translation) && translation.rows() == 3)
        key_pick_pose_.translation() = translation;
    else
        ROS_ERROR("Failed to parse Key Pick translation");

    Eigen::VectorXd orientation;
    if (yaml_parser::yamlReadMember(parameter, "orientation_pick", &orientation) && orientation.rows() == 4)
    {
        key_pick_pose_.rotate(Eigen::Quaterniond(orientation.coeff(0), orientation.coeff(1), orientation.coeff(2), orientation.coeff(3)));
    }
    else
        ROS_ERROR("Failed to parse Key Pick orientation");

    // For Key Pick Support
    if (yaml_parser::yamlReadMember(parameter, "translation_pick_supp", &translation) && translation.rows() == 3)
        key_pick_supp_pose_.translation() = translation;
    else
        ROS_ERROR("Failed to parse Key Pick Supp translation");

    if (yaml_parser::yamlReadMember(parameter, "orientation_pick_supp", &orientation) && orientation.rows() == 4)
    {
        key_pick_supp_pose_.rotate(Eigen::Quaterniond(orientation.coeff(0), orientation.coeff(1), orientation.coeff(2), orientation.coeff(3)));
    }
    else
        ROS_ERROR("Failed to parse Key Pick Supp orientation");

    // For Key Pick Support
    if (yaml_parser::yamlReadMember(parameter, "translation_mid", &translation) && translation.rows() == 3)
        key_mid_pose_.translation() = translation;
    else
        ROS_ERROR("Failed to parse Key Mid translation");

    if (yaml_parser::yamlReadMember(parameter, "orientation_mid", &orientation) && orientation.rows() == 4)
    {
        key_mid_pose_.rotate(Eigen::Quaterniond(orientation.coeff(0), orientation.coeff(1), orientation.coeff(2), orientation.coeff(3)));
    }
    else
        ROS_ERROR("Failed to parse Key Mid orientation");

    // For Key Pick Support 2
    if (yaml_parser::yamlReadMember(parameter, "translation_mid2", &translation) && translation.rows() == 3)
        key_mid_pose_2.translation() = translation;
    else
        ROS_ERROR("Failed to parse Key Mid translation");

    if (yaml_parser::yamlReadMember(parameter, "orientation_mid2", &orientation) && orientation.rows() == 4)
    {
        key_mid_pose_2.rotate(Eigen::Quaterniond(orientation.coeff(0), orientation.coeff(1), orientation.coeff(2), orientation.coeff(3)));
    }
    else
        ROS_ERROR("Failed to parse Key Mid orientation");

    // For Key Insert Task
    if (yaml_parser::yamlReadMember(parameter, "translation_insert", &translation) && translation.rows() == 3)
        key_insert_pose_.translation() = translation;
    else
        ROS_ERROR("Failed to parse Key Insert translation");

    if (yaml_parser::yamlReadMember(parameter, "orientation_insert", &orientation) && orientation.rows() == 4)
    {
        key_insert_pose_.rotate(Eigen::Quaterniond(orientation.coeff(0), orientation.coeff(1), orientation.coeff(2), orientation.coeff(3)));
    }
    else
        ROS_ERROR("Failed to parse Key Insert orientation");

    goal_publisher_ = node.advertise<geometry_msgs::PoseStamped>("equilibrium_pose", 1);
}

/**
 * @brief Execute task
 *
 * @param tf_buffer Buffer which knows transforms from robot base to taskboard
 * @return true on success
 * @return false on failure
 */
bool TaskKey::execute(tf2_ros::Buffer &tf_buffer, Planner &planner, const std::shared_ptr<Gripper> &gripper,
                      const std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> follow_joint_traj_client,
                      const std::shared_ptr<ros::ServiceClient> controller_manager_client)
{
    ROS_INFO("Executing key pick supp task");
    std::string err_string;
    if (!tf_buffer.canTransform("panda_link0", "taskboard_refined_stable", ros::Time(0), ros::Duration(0), &err_string))
    {
        ROS_ERROR_STREAM("Failed to execute key task: " << err_string);
        return false;
    }

    // open gripper
    gripper->move(0.5, 0.1);
    ros::Duration(1).sleep();

    Eigen::Isometry3d tf_goal;
    auto tf_ref_pl0 = tf2::transformToEigen(tf_buffer.lookupTransform("panda_link0", "taskboard_refined_stable", ros::Time(0)));
    tf_goal         = tf_ref_pl0 * key_pick_supp_pose_;

    // plan trajectory to support point of the task
    {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "panda_link0";
        goal.pose            = tf2::toMsg(tf_goal);

        planning_interface::MotionPlanResponse traj = planner.planTrajectoryFromCurrentPose(goal, 0.3);

        if (traj.error_code_.val != traj.error_code_.SUCCESS)
        {
            // TODO what now?
            ROS_ERROR("Key task: failed to plan trajectory");
            return false;
        }

        // convert moveit-trajectory-msg to control_msgs
        moveit_msgs::RobotTrajectory moveitTraj;
        traj.trajectory_->getRobotTrajectoryMsg(moveitTraj);

        control_msgs::FollowJointTrajectoryGoal goalTraj;
        goalTraj.trajectory          = moveitTraj.joint_trajectory;
        goalTraj.goal_time_tolerance = ros::Duration(5);  // 5s tolerance

        ROS_INFO("Key task: executing key pick supp trajectory");
        actionlib::SimpleClientGoalState goalState = follow_joint_traj_client->sendGoalAndWait(goalTraj, ros::Duration(60));
        ros::Duration(1).sleep();

        if (goalState.state_ != goalState.SUCCEEDED)
        {
            // TODO what now?
            ROS_ERROR("Key task: Failed to execute key pick trajectory");
            return false;
        }
    }

    // plan trajectory to pick point
    {
        ROS_INFO("Executing key pick task");
        // update parameters
        Eigen::Isometry3d tf_goal_pick = tf_ref_pl0 * key_pick_pose_;

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "panda_link0";
        goal.pose            = tf2::toMsg(tf_goal_pick);
        // hard-coded height
        std::cout << "-------------------goal.pose.position.z:----------------------" << goal.pose.position.z << std::endl;
        goal.pose.position.z                             = 0.25;
        const double vel_factor                          = 0.3;
        const double acc_factor                          = 0.25;
        planning_interface::MotionPlanResponse traj_pick = planner.planTrajectoryFromCurrentPose(goal, vel_factor, acc_factor);

        if (traj_pick.error_code_.val != traj_pick.error_code_.SUCCESS)
        {
            // TODO what now?
            ROS_ERROR("Key task: failed to plan trajectory");
            return false;
        }

        // convert moveit-trajectory-msg to control_msgs
        moveit_msgs::RobotTrajectory moveitTraj_pick;
        traj_pick.trajectory_->getRobotTrajectoryMsg(moveitTraj_pick);

        control_msgs::FollowJointTrajectoryGoal goalTraj_pick;
        goalTraj_pick.trajectory          = moveitTraj_pick.joint_trajectory;
        goalTraj_pick.goal_time_tolerance = ros::Duration(5);

        ROS_INFO("Key task: executing key pick supp trajectory");
        actionlib::SimpleClientGoalState goalState_pick = follow_joint_traj_client->sendGoalAndWait(goalTraj_pick, ros::Duration(60));
        ros::Duration(2).sleep();
    }

    // get the key
    gripper->grasp(0.04, 30, 0.1, 0.01, 0.01);

    // go up to mid pose
    {
        ROS_INFO("Retrieving key from hole");
        // update parameters
        Eigen::Isometry3d tf_goal_mid = tf_ref_pl0 * key_mid_pose_;

        std::cout << "start to go to mid pose" << std::endl;

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "panda_link0";
        goal.pose            = tf2::toMsg(tf_goal_mid);
        // hard-coded height
        std::cout << "-------------------goal.pose.position.z:----------------------" << goal.pose.position.z << std::endl;
        // goal.pose.position.z                             = 0.25;
        const double vel_factor                          = 0.3;
        const double acc_factor                          = 0.25;
        planning_interface::MotionPlanResponse traj_pick = planner.planTrajectoryFromCurrentPose(goal, vel_factor, acc_factor);

        if (traj_pick.error_code_.val != traj_pick.error_code_.SUCCESS)
        {
            // TODO what now?
            ROS_ERROR("Key task: failed to plan trajectory");
            return false;
        }

        // convert moveit-trajectory-msg to control_msgs
        moveit_msgs::RobotTrajectory moveitTraj_pick;
        traj_pick.trajectory_->getRobotTrajectoryMsg(moveitTraj_pick);

        control_msgs::FollowJointTrajectoryGoal goalTraj_pick;
        goalTraj_pick.trajectory          = moveitTraj_pick.joint_trajectory;
        goalTraj_pick.goal_time_tolerance = ros::Duration(5);

        ROS_INFO("Key task: executing key pick supp trajectory");
        actionlib::SimpleClientGoalState goalState_pick = follow_joint_traj_client->sendGoalAndWait(goalTraj_pick, ros::Duration(60));

        ros::Duration(2).sleep();
    }

    // go up to mid pose 2
    {
        ROS_INFO("Retrieving key from hole");
        // update parameters
        Eigen::Isometry3d tf_goal_mid = tf_ref_pl0 * key_mid_pose_2;

        std::cout << "start to go to mid pose" << std::endl;

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "panda_link0";
        goal.pose            = tf2::toMsg(tf_goal_mid);
        // hard-coded height
        std::cout << "-------------------goal.pose.position.z:----------------------" << goal.pose.position.z << std::endl;
        // goal.pose.position.z                             = 0.25;
        const double vel_factor                          = 0.3;
        const double acc_factor                          = 0.25;
        planning_interface::MotionPlanResponse traj_pick = planner.planTrajectoryFromCurrentPose(goal, vel_factor, acc_factor);

        if (traj_pick.error_code_.val != traj_pick.error_code_.SUCCESS)
        {
            // TODO what now?
            ROS_ERROR("Key task: failed to plan trajectory");
            return false;
        }

        // convert moveit-trajectory-msg to control_msgs
        moveit_msgs::RobotTrajectory moveitTraj_pick;
        traj_pick.trajectory_->getRobotTrajectoryMsg(moveitTraj_pick);

        control_msgs::FollowJointTrajectoryGoal goalTraj_pick;
        goalTraj_pick.trajectory          = moveitTraj_pick.joint_trajectory;
        goalTraj_pick.goal_time_tolerance = ros::Duration(5);

        ROS_INFO("Key task: executing key pick supp trajectory");
        actionlib::SimpleClientGoalState goalState_pick = follow_joint_traj_client->sendGoalAndWait(goalTraj_pick, ros::Duration(60));

        ros::Duration(2).sleep();
    }

    // go to key lock
    {
        ROS_INFO("Retrieving key from hole");
        // update parameters
        Eigen::Isometry3d tf_goal_insert = tf_ref_pl0 * key_insert_pose_;

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "panda_link0";
        goal.pose            = tf2::toMsg(tf_goal_insert);
        // hard-coded height
        std::cout << "-------------------goal_pick.pose.position.z:----------------------" << goal.pose.position.z << std::endl;
        goal.pose.position.z                             = 0.25;
        const double vel_factor                          = 0.3;
        const double acc_factor                          = 0.25;
        planning_interface::MotionPlanResponse traj_pick = planner.planTrajectoryFromCurrentPose(goal, vel_factor, acc_factor);

        if (traj_pick.error_code_.val != traj_pick.error_code_.SUCCESS)
        {
            // TODO what now?
            ROS_ERROR("Key task: failed to plan trajectory");
            return false;
        }

        // convert moveit-trajectory-msg to control_msgs
        moveit_msgs::RobotTrajectory moveitTraj_pick;
        traj_pick.trajectory_->getRobotTrajectoryMsg(moveitTraj_pick);

        control_msgs::FollowJointTrajectoryGoal goalTraj_pick;
        goalTraj_pick.trajectory          = moveitTraj_pick.joint_trajectory;
        goalTraj_pick.goal_time_tolerance = ros::Duration(5);

        ROS_INFO("Key task: executing key pick supp trajectory");
        actionlib::SimpleClientGoalState goalState_pick = follow_joint_traj_client->sendGoalAndWait(goalTraj_pick, ros::Duration(60));
        ros::Duration(2).sleep();
    }

    // ROS_INFO("Key task: executing key insert trajectory");
    // actionlib::SimpleClientGoalState goalState = follow_joint_traj_client->sendGoalAndWait(goalTraj, ros::Duration(60));

    // switch controller
    if (!helpers::switchControllers("effort_joint_trajectory_controller", "cartesian_impedance_controller", controller_manager_client))
    {
        return false;
    }
    else
        ROS_INFO("Switched controllers");

    // Todo: Inserting

    if (!gripper->waitForMovingToFinish())
        return false;

    // TODO do next task command (Key task may not require any other commands)
    // ...

    return true;
}

}  // namespace pandaria_ros