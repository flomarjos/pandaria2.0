#pragma once

#include "XmlRpcValue.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "task.h"
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <ros/node_handle.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace pandaria_ros
{

/**
 * @brief Task controller to move robot to start pose
 */
class TaskStartPose : public Task
{
  public:
    TaskStartPose(ros::NodeHandle &node, const XmlRpc::XmlRpcValue &parameters);

    bool execute(tf2_ros::Buffer &tf_buffer, Planner &planner, const std::shared_ptr<Gripper> &gripper,
                 const std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>,
                 const std::shared_ptr<ros::ServiceClient>) override;

  private:
    /// position of button to be pressed; z-axis is directed towards push direction
    Eigen::Isometry3d start_pose_;
    Eigen::Isometry3d pose_above_center_;

    /// Publisher
    ros::Publisher start_filter_center_;
    ros::Publisher start_filter_refined_;
    ros::Publisher goal_publisher_;

    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
};

}  // namespace pandaria_ros