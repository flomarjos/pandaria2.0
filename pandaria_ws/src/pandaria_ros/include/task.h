#pragma once

#include "control_msgs/FollowJointTrajectoryAction.h"
#include "gripper.h"
#include "planner.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/node_handle.h>
#include <string>
#include <tf2_ros/buffer.h>

namespace pandaria_ros
{

/**
 * @brief Superclass of task controllers
 */
class Task
{
  public:
    virtual bool execute(tf2_ros::Buffer &tf_buffer, Planner &planner, const std::shared_ptr<Gripper> &gripper,
                         const std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>,
                         const std::shared_ptr<ros::ServiceClient>) = 0;
};

}  // namespace pandaria_ros