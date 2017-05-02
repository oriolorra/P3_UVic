#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/Int32MultiArray.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <yaml-cpp/yaml.h> 
#include <string.h>
#include <sstream>
#include "path.h"

bool executePath( moveit::planning_interface::MoveGroup &arm, moveit::planning_interface::MoveGroup::Plan &plan, std::vector<geometry_msgs::Pose>& waypoints, std::string& moveit_group){
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
  bool success;

  std::string package_path = ros::package::getPath("p3_uvic");
  YAML::Node yaml_config  = YAML::LoadFile(package_path+"/config/trajectory.yaml");

  int trajectories_size = yaml_config["Totalsize"].as<int>();

  // Initialize trajectory waypoints array
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.clear();

  ros::init(argc, argv, "p3_uvic");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt! configuration for UR5
  std::string moveit_group = yaml_config["MoveItGroup"].as<std::string>();
  moveit::planning_interface::MoveGroup group(yaml_config["MoveItGroup"].as<std::string>());
  moveit::planning_interface::MoveGroup::Plan my_plan;
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(10);
  group.setNumPlanningAttempts(5);
  group.setEndEffectorLink("ee_link");
 
  // set start state of the simulated robot to the current state
  group.setStartStateToCurrentState();

  // start trajectory with the currrent state
  geometry_msgs::Pose startingPose = group.getCurrentPose().pose;

  for(int j=1; j <= trajectories_size; j++){

    //     COMPLETAR CODI
    int path_size; 

    for(int k = 1; k <= path_size; k++){

      //     COMPLETAR CODI
      std::string path_type;

      if(path_type.find("Circle") != std::string::npos){

        //     COMPLETAR CODI

      }else if (path_type.find("Line") != std::string::npos){

        //     COMPLETAR CODI

      }else if (path_type.find("Pose") != std::string::npos){
         
        //     COMPLETAR CODI

         group.move();
      }
    }

    //     COMPLETAR CODI
  
  }

  return 0;
}
