#include "task_coordinator.h"
#include "gripper.h"
#include "planner.h"
#include "ros/service_client.h"
#include "task_button.h"
#include "task_key.h"
#include "task_start_pose.h"
#include "yaml_parser.hpp"
#include <controller_manager_msgs/SwitchController.h>
#include <memory>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pandaria_ros
{

TaskCoordinator::TaskCoordinator(const std::string &name) : node_handle_(name)
{
    // init planner
    Planner planner(node_handle_);

    // create action client(s) -- one for each controller we are going to use
    follow_joint_traj_client_ = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(
        "/effort_joint_trajectory_controller/follow_joint_trajectory");

    if (!follow_joint_traj_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR("Follow-joint-trajectory action server not available");
    };

    // client for switching between controllers
    controller_manager_client_ = std::make_shared<ros::ServiceClient>(
        node_handle_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller"));

    // wait until updated robot state was received
    planner.waitForRobotState();

    // init gripper
    gripper_ = std::make_shared<Gripper>();
    // gripper_->homeAndWait();  // simulatiously moving the robot and the gripper is somehow not allowed/possible by franka?!

    ROS_INFO_STREAM("TaskCoordinator Node running:" << name);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // read task config // TODO: read all tasks (not just one)
    XmlRpc::XmlRpcValue task_config;
    node_handle_.getParam("/task_configuration", task_config);
    XmlRpc::XmlRpcValue task_parameter;
    if (yaml_parser::yamlReadMember(task_config, "start_pose_task", &task_parameter))
    {
        tasks_.push_back(std::make_unique<TaskStartPose>(node_handle_, task_parameter));
        task_parameter.clear();
        tasks_.front()->execute(tf_buffer, planner, gripper_, follow_joint_traj_client_, controller_manager_client_);
    }

    if (yaml_parser::yamlReadMember(task_config, "button_task", &task_parameter))
    {
        tasks_.push_back(std::make_unique<TaskButton>(node_handle_, task_parameter));
    }

    if (yaml_parser::yamlReadMember(task_config, "key_task", &task_parameter))
    {
        tasks_.push_back(std::make_unique<TaskKey>(node_handle_, task_parameter));
    }

    // planner.waitForRobotState();
    // wait for transform of taskboard (replace later with routine to localise taskboard)
    std::string err_string;
    while (ros::ok() && !tf_buffer.canTransform("panda_link0", "taskboard_reference", ros::Time::now(), ros::Duration(1), &err_string))
    {
        ROS_INFO_STREAM_DELAYED_THROTTLE(2, "Waiting for taskboard locaization" << err_string);
        err_string.clear();
        ros::spinOnce();
    }

    // execute all tasks
    for (size_t i = 1; i < tasks_.size(); ++i)
    {
        if (!tasks_.at(i)->execute(tf_buffer, planner, gripper_, follow_joint_traj_client_, controller_manager_client_))
            std::cout << "nothing " << std::endl;
    }
}

}  // namespace pandaria_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_coordinator");
    auto node = std::make_shared<pandaria_ros::TaskCoordinator>("task_coordinator");
    ros::spin();
    ros::shutdown();
    return 0;
}