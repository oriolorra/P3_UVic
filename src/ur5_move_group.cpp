#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "path.h"

void executePath( moveit::planning_interface::MoveGroup &arm, moveit::planning_interface::MoveGroup::Plan &plan, std::vector<geometry_msgs::Pose>& waypoints){
  moveit_msgs::RobotTrajectory trajectory_msg;

  double fraction = arm.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg, false);
  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(arm.getCurrentState()->getRobotModel(), "manipulator");

  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory_msg);

  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // Fourth compute computeTimeStamps
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);
  // Check trajectory_msg for velocities not empty
  //std::cout << trajectory_msg << std::endl;

  plan.trajectory_ = trajectory_msg;
  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);

  if(fraction == 1){
    ROS_INFO("EXECUTING....");
      arm.execute(plan);
  }
}



int main(int argc, char **argv)
{
  Path path;

  ros::init(argc, argv, "demo_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");
  moveit::planning_interface::MoveGroup::Plan my_plan;
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(10);
  group.setNumPlanningAttempts(5);

  group.setStartStateToCurrentState();

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose startingPose = group.getCurrentPose().pose;

  ROS_INFO("GET TRAJECTORY");
  sleep(2);

  if(path.getCircleTrajectoryEEPlane(startingPose, 0.1, 0.0, 180, 10.0, 1,  waypoints))
    executePath(group, my_plan, waypoints);

  startingPose = waypoints.back();
  waypoints.clear();
  if(path.getLineTrajectoryEEPlane(startingPose, 0.05, -1, 0.01, waypoints))
    executePath(group, my_plan, waypoints);

  startingPose = waypoints.back();
  waypoints.clear();
  if(path.getCircleTrajectoryEEPlane(startingPose, 0.1, 180, 360, 1.0, 1,  waypoints))
    executePath(group, my_plan, waypoints);

  startingPose = waypoints.back();
  waypoints.clear();
  if(path.getLineTrajectoryEEPlane(startingPose, 0.05, 1, 0.01, waypoints))
    executePath(group, my_plan, waypoints);

  ROS_INFO(" PATH DONE  ");
}
