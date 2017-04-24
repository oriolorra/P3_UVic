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
  bool success;
  std::string package_path = ros::package::getPath("P3_UVic");

  YAML::Node yaml_config  = YAML::LoadFile(package_path+"/config/trajectory.yaml");

  int trajectories_size = yaml_config["Totalsize"].as<int>();

  ROS_INFO_STREAM("3");

  std::vector<geometry_msgs::Pose> waypoints;

        ros::init(argc, argv, "demo_review");
  ros::NodeHandle nh;

  ros::Publisher pub_di = nh.advertise<std_msgs::Int32MultiArray>("/b2r_beckhoff/digital_commands",10);
        ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string moveit_group = yaml_config["MoveItGroup"].as<std::string>();
  moveit::planning_interface::MoveGroup group(yaml_config["MoveItGroup"].as<std::string>());
  moveit::planning_interface::MoveGroup::Plan my_plan;
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(10);
  group.setNumPlanningAttempts(5);
  group.setEndEffectorLink("tcp_link");

  group.setStartStateToCurrentState();
  geometry_msgs::Pose startingPose = group.getCurrentPose().pose;
  ROS_INFO_STREAM("Starting POSE: " << startingPose.position.x << "  " << startingPose.position.y << "  " << startingPose.position.z);
  waypoints.clear();

  for(int j=1; j <= trajectories_size; j++){

    auto trajectory_node = yaml_config["Trajectory" + std::to_string(j)];
    int path_size = trajectory_node["size"].as<int>();

    for(int k = 1; k <= path_size; k++){

      auto path_node = trajectory_node["Path" + std::to_string(k)];
      std::string path_type = path_node["Type"].as<std::string>();

      if(path_type.find("Circle") != std::string::npos){
        double radius = path_node["Radius"].as<double>();
        double start_angle = path_node["StartAngle"].as<double>();
        double end_angle = path_node["EndAngle"].as<double>();
        double step = path_node["AngleStep"].as<double>();
        int clockwise = (start_angle >= end_angle ) ? -1: 1;

        ROS_INFO_STREAM("CIRCLE: " << radius << "  " << start_angle << "  " << end_angle << "  " << step << " " << clockwise);
        path.getCircleTrajectoryEEPlane(startingPose, radius, start_angle, end_angle, step, clockwise,  waypoints);

        startingPose = waypoints.back();

      }else if (path_type.find("Line") != std::string::npos){

        double lenght = path_node["Lenght"].as<double>();
        int direction = path_node["Direction"].as<double>();             //   1,-1 -> x,-x      2,-2 -> y,-y      3,-3 -> z,-z
        double step = path_node["LenghtStep"].as<double>();

        ROS_INFO_STREAM("LINE : " << lenght << "  " << direction << "  " << step );
        path.getLineTrajectoryEEPlane(startingPose, lenght, direction, step, waypoints);

        startingPose = waypoints.back();

      }else if (path_type.find("DI") != std::string::npos){
        std_msgs::Int32MultiArray di_msg;

        di_msg.data.push_back(path_node["Emergency_stop"].as<int>());
        di_msg.data.push_back(path_node["Aspiration_enable"].as<int>());
        di_msg.data.push_back(path_node["Tool_enable"].as<int>());
        di_msg.data.push_back(path_node["Cablereel_enable"].as<int>());
        di_msg.data.push_back(path_node["Cablereel_direction"].as<int>());

        pub_di.publish(di_msg);
      }
    }

    executePath(group, my_plan, waypoints, moveit_group);

    group.setStartStateToCurrentState();
    startingPose = group.getCurrentPose().pose;
    waypoints.clear();
    ROS_INFO_STREAM("Starting POSE: " << startingPose.position.x << "  " << startingPose.position.y << "  " << startingPose.position.z);
  }

  return 0;
}
