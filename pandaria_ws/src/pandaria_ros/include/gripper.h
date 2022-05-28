#pragma once

#include "actionlib/client/simple_action_client.h"
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

namespace pandaria_ros
{

/**
 * @brief Wrapper class for controlling the franka hand
 *
 */
class Gripper
{
  public:
    Gripper();
    void home();
    bool waitForHomingToFinish();
    bool homeAndWait();
    void move(const double width, const double move_speed);
    bool waitForMovingToFinish();
    bool moveAndWait(const double width, const double move_speed);
    bool grasp(const double width, const double grasp_force, const double move_speed, const double max_delta_width_inner,
               const double max_delta_width_outer);
    bool stop();

  private:
    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client_;
    actionlib::SimpleActionClient<franka_gripper::StopAction> stop_client_;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client_;
    actionlib::SimpleActionClient<franka_gripper::HomingAction> home_client_;
};

}  // namespace pandaria_ros