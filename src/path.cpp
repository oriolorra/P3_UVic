#include "path.h"


#define PI 3.14159265

Path::Path(){}
Path::~Path(){}

//  x = cx + r * cos(alpha) sin(yaw) * sin(pitch)
//  y = cy + r * cos(alpha) * cos(yaw)
//  z = cz + r * sin(alpha) * cos(pitch)
bool Path::getCircleTrajectoryEEPlane(const geometry_msgs::Pose &startingPose, const double radius, const double startingAngleDeg, const double endAngleDeg, const double stepDeg, const int clockwise, std::vector<geometry_msgs::Pose> &waypoints){

  double steps = (endAngleDeg-startingAngleDeg)/stepDeg;
  double angle = startingAngleDeg;

  //center of circle from end effector -> cx, cy, cz ????
  //*** Eigen::Vector3d center(cx, cy, cz);

  //Getting orientation end effector -> quaternion-??
  //*** Eigen::Matrix3d R = Eigen::Quaterniond(quaternionW, quaternionX, quaternionY, quaternionZ).toRotationMatrix();
  
  //startingpose to eigen vector -> x,y,z ??
  //*** Eigen::Vector3d startingPoseBase(xBase, yBase, zBase);

  //check if steps has decimals
  double intpart;
  if( modf( steps, &intpart) != 0 ) return false;


  for(int i = 0; i <= steps; i++){

    //Calculate point in EndEfector coordinates, then rotate it and sum to startingpose
    //*** Eigen::Vector3d pointEE(eex, eey, eez);
    //*** Eigen::Vector3d pointBase = RotationMatrix*pointEE + startingpose;

    geometry_msgs::Pose target_pose;
    //*** target_pose.position.x = ;
    //*** target_pose.position.y = ;
    //*** target_pose.position.z = ;
    //*** target_pose.orientation.x = ;
    //*** target_pose.orientation.y = ;
    //*** target_pose.orientation.z = ;
    //*** target_pose.orientation.w = ;
    waypoints.push_back(target_pose);

    if(angle >= endAngleDeg) break;

    angle += stepDeg;
  }


  return true;
}

bool Path::getLineTrajectoryEEPlane(const geometry_msgs::Pose &startingPose, const double distance, const int direction, const double step, std::vector<geometry_msgs::Pose> &waypoints){

  double steps = distance/step;

  //Getting orientation end effector -> quaternion-??  
  //*** Eigen::Matrix3d R = Eigen::Quaterniond(quaternionW, quaternionX, quaternionY, quaternionZ).toRotationMatrix();
  
  //startingpose to eigen vector -> x,y,z ??
  //***Eigen::Vector3d startingPoseBase(xBase, yBase, zBase);


  //check if steps has decimals
  double intpart;
  if( modf( steps, &intpart) != 0 ) return false;

   for(int i = 0; i <= steps; i++){

    //Calculate point in EndEfector coordinates, then rotate it and sum to startingpose
    //*** Eigen::Vector3d pointEE(eex, eey, eez);
    //*** Eigen::Vector3d pointBase = RotationMatrix*pointEE + startingpose;

     geometry_msgs::Pose target_pose;
    //*** target_pose.position.x = ;
    //*** target_pose.position.y = ;
    //*** target_pose.position.z = ;
    //*** target_pose.orientation.x = ;
    //*** target_pose.orientation.y = ;
    //*** target_pose.orientation.z = ;
    //*** target_pose.orientation.w = ;
     waypoints.push_back(target_pose);
   }

  return true;
}

