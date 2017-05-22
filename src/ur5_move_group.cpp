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

  //Find path from this package
  std::string package_path = ros::package::getPath("p3_uvic");

  //Load trajectory file to be read
  YAML::Node yaml_config  = YAML::LoadFile(package_path+"/config/trajectory.yaml");

  //Get parameter Totalsize from YAML file
  int trajectories_size = yaml_config["Totalsize"].as<int>();

  // Initialize trajectory waypoints array
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.clear();

  // Initialize ROS
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

  //Loop for Trajectories
  for(int j=1; j <= trajectories_size; j++){

    //Get all trajectory 
    YAML::Node trajectory_node = yaml_config["Trajectory" + std::to_string(j)];

    //Get size of trajectory
    int path_size = trajectory_node["size"].as<int>(); 

    bool isPose = false;

    //Loop for Paths
    for(int k = 1; k <= path_size; k++){

      //Get path
      auto path_node = trajectory_node["Path" + std::to_string(k)];

      //Get type of the path
      std::string path_type = path_node["Type"].as<std::string>();

      if(path_type.find("Circle") != std::string::npos){

        // Read parameters from YAML
        // Call 'getCircleTrajectoryEEPlane' function with the right parameters
        
        // Save waypoints
        //...

        //Make 'statingPose', the last point of waypoints
        //...

      }else if (path_type.find("Line") != std::string::npos){

        // Read parameters from YAML
        // Call 'getLineTrajectoryEEPlane' function with the right parameters
        
        // Save waypoints
        //...

        //Make 'statingPose', the last point of waypoints
        //...

      }else if (path_type.find("Pose") != std::string::npos){
        
        // group.setNamedTarget(_name_of_the_pose_);
        
        //Move robot to Pose
        group.move();

        //If the program enter here, should not execute 'executePath()' 
        //....
      }
    }

    //Execute trajectory 
    if(!isPose) executePath(group, my_plan, waypoints, moveit_group);

    //Update pose of the robot 
    group.setStartStateToCurrentState();
    
    //Get current state of the robot, to start next trajectory on the right place
    startingPose = group.getCurrentPose().pose;

    waypoints.clear();
  
  }

  return 0;
}
