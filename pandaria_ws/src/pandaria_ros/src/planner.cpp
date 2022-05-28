#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include "planner.h"

/**
 * - a MoveIt-planning-group is a group of joints (& links?), which is configured in a parameter file
 * - the "panda_arm" group includes all joints except the hand joints
 * - the planner plans for this group (i.e. only modies these joints) - the collision geometry of the hand is still considered
 */
const std::string PLANNING_GROUP = "panda_arm";

namespace pandaria_ros
{

/**
 * @brief The planner object automatically monitors the current robot state and word objects. Use it to plan trajectories.
 *
 * @param node_handle
 */
Planner::Planner(ros::NodeHandle &node_handle) : visual_tools_("panda_link0")
{

    // the planning scene contains information about the world (i.e. robot state, obstacles)
    // the planning scene MONITOR manages and updates this planning scene from the correct ROS-topics
    std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>();

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description", tf_buffer);
    planning_scene_monitor_->startWorldGeometryMonitor("/collision_object", "/planning_scene_world");
    planning_scene_monitor_->stopSceneMonitor();
    // planning_scene_monitor_->startSceneMonitor("/planning_scene");
    planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE,
                                                          "/monitored_planning_scene");
    planning_scene_monitor_->providePlanningSceneService("/get_planning_scene");

    // load robot and set default pose
    const robot_model::RobotModelConstPtr robot_model      = planning_scene_monitor_->getRobotModel();
    const moveit::core::JointModelGroup *joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);

    // create planning pipeline
    this->plannning_pipeline_ =
        std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node_handle, "planning_plugin", "request_adapters");

    // Visualization
    display_publisher_ = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
}

void Planner::waitForRobotState()
{
    if (planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now()))
    {
        ROS_ERROR("Did not receive robot state.");
    }
}

/**
 * @brief Plan a trajectory from the current robot state (determined via the topic /joint_states) to \arg endPose.
 * Here, no path constraints are enforced. If you want path constraints (e.g. orientation) use
 * \ref Planner::planTrajectoryFromCurrentPose(const planning_interface::MotionPlanRequest req)
 *
 * @param endPose the goal pose
 * @return planning_interface::MotionPlanResponse result, valid if(response.error_code_.val == response.error_code_.SUCCESS)
 */
planning_interface::MotionPlanResponse Planner::planTrajectoryFromCurrentPose(const geometry_msgs::PoseStamped endPose, const double vel_factor,
                                                                              const double acc_factor)
{
    // this->waitForRobotState();
    planning_interface::MotionPlanResponse res;

    // create motion planning request
    planning_interface::MotionPlanRequest req;
    req.group_name                      = PLANNING_GROUP;
    req.allowed_planning_time           = 20;  // increase planning time (default is 1s) => better results // TODO is necessary?
    req.max_velocity_scaling_factor     = vel_factor;
    req.max_acceleration_scaling_factor = acc_factor;

    // tolerance in m and radians
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // add goal constraint
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("panda_link8", endPose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -2.0;
    req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z = 2.0;

    return this->planTrajectoryFromCurrentPose(req);
}

/**
 * @brief Plan a trajectory from the current robot state (determined via the topic /joint_states) to \arg endPose
 *
 * @param req
 * @return planning_interface::MotionPlanResponse
 */
planning_interface::MotionPlanResponse Planner::planTrajectoryFromCurrentPose(const planning_interface::MotionPlanRequest req)
{
    planning_interface::MotionPlanResponse res;
    planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor_->getPlanningScene();

    // planning_interface::PlanningContextPtr context = planner_instance_->getPlanningContext(planning_scene, req, res.error_code_);
    this->plannning_pipeline_->generatePlan(planning_scene, req, res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
    }

    this->visualizePlanningResult(res, PLANNING_GROUP);
    return res;
}

/**
 * @brief Visualize a motion-plan-response in rviz.
 *
 * @param res
 * @param planning_group
 */
void Planner::visualizePlanningResult(const planning_interface::MotionPlanResponse &res, const std::string &planning_group)
{
    namespace rvt = rviz_visual_tools;

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.2;
    visual_tools_.publishText(text_pose, "Motion Planning Response", rvt::WHITE, rvt::XLARGE);

    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);

    const robot_state::JointModelGroup *joint_model_group = planning_scene_monitor_->getRobotModel()->getJointModelGroup(planning_group);
    visual_tools_.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools_.trigger();
    display_publisher_.publish(display_trajectory);
}

}  // namespace pandaria_ros