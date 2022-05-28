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
 * @brief Task controller for button pressing task
 */
class TaskButton : public Task
{
  public:
    TaskButton(ros::NodeHandle &node, const XmlRpc::XmlRpcValue &parameters);

    bool execute(tf2_ros::Buffer &tf_buffer, Planner &planner, const std::shared_ptr<Gripper> &gripper,
                 const std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>,
                 const std::shared_ptr<ros::ServiceClient>) override;

  private:
    /// position of button to be pressed; z-axis is directed towards push direction
    Eigen::Isometry3d button_top_pose_1;
    Eigen::Isometry3d button_top_pose_2;
    Eigen::Isometry3d button_press_;
    Eigen::Isometry3d safe_point_pose_;

    /// Publisher
    ros::Publisher goal_publisher_;
};

}  // namespace pandaria_ros