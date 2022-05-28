#pragma once

#include "control_msgs/FollowJointTrajectoryAction.h"
#include "gripper.h"
#include "task.h"
#include <actionlib/client/simple_action_client.h>
#include <memory>
#include <ros/node_handle.h>
#include <string>

namespace pandaria_ros
{

/**
 * @brief Node coordinating all tasks
 */
class TaskCoordinator
{
  public:
    TaskCoordinator(const std::string &name);

  private:
    /// Handle to node
    ros::NodeHandle node_handle_;

    /// Task controllers
    std::vector<std::unique_ptr<Task>> tasks_;
    std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> follow_joint_traj_client_;
    std::shared_ptr<ros::ServiceClient> controller_manager_client_;
    std::shared_ptr<Gripper> gripper_;
};

}  // namespace pandaria_ros