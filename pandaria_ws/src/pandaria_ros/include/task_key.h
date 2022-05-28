#pragma once

#include "XmlRpcValue.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "task.h"
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <ros/node_handle.h>
#include <string>
#include <tf2_ros/buffer.h>

namespace pandaria_ros
{

/**
 * @brief Task controller for key pick and insert
 */
class TaskKey : public Task
{
  public:
    TaskKey(ros::NodeHandle &node, const XmlRpc::XmlRpcValue &parameters);

    bool execute(tf2_ros::Buffer &tf_buffer, Planner &planner, const std::shared_ptr<Gripper> &gripper,
                 const std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>,
                 const std::shared_ptr<ros::ServiceClient>) override;

  private:
    /// position of key to be picked
    Eigen::Isometry3d key_pick_pose_;
    Eigen::Isometry3d key_pick_supp_pose_;
    Eigen::Isometry3d key_mid_pose_;
    Eigen::Isometry3d key_mid_pose_2;
    Eigen::Isometry3d key_insert_pose_;

    /// Publisher
    ros::Publisher goal_publisher_;
};

}  // namespace pandaria_ros