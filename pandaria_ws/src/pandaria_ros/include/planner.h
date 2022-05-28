#pragma once

#include "moveit/planning_interface/planning_interface.h"
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/node_handle.h>

namespace pandaria_ros
{

/**
 * @brief The planner object automatically monitors the current robot state and word objects. Use it to plan trajectories.
 * Additonally it publishes it current "planning-state" (i.e. its' world) to rviz.
 *
 */
class Planner
{
  public:
    Planner(ros::NodeHandle &node);
    planning_interface::MotionPlanResponse planTrajectoryFromCurrentPose(const geometry_msgs::PoseStamped endPose, const double vel_factor = 0.3,
                                                                         const double acc_factor = 0.5);
    void waitForRobotState();

  private:
    void visualizePlanningResult(const planning_interface::MotionPlanResponse &res, const std::string &planning_group);
    planning_interface::MotionPlanResponse planTrajectoryFromCurrentPose(const planning_interface::MotionPlanRequest req);

    moveit_visual_tools::MoveItVisualTools visual_tools_;
    planning_pipeline::PlanningPipelinePtr plannning_pipeline_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    ros::Publisher display_publisher_;
};

}  // namespace pandaria_ros