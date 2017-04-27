#include "path.h"


#define PI 3.14159265

Path::Path(){}
Path::~Path(){}

//  x = cx + r * cos(a)
//  z = cz + r * sin(a)
// bool Path::getCircleTrajectoryXZBase(const geometry_msgs::Pose& startingPose, const double radius, const double startingAngleDeg, const double endAngleDeg, const double stepDeg, const int clockwise, std::vector<geometry_msgs::Pose> &waypoints){

//   double steps = (endAngleDeg-startingAngleDeg)/stepDeg;

//   double originX = startingPose.position.x;
//   double originY = startingPose.position.y;
//   double originZ = startingPose.position.z;

//   double angle = startingAngleDeg;

//   double cx = originX - radius*cos(angle * PI/180);
//   double cz = originZ - radius*sin(angle * PI/180);

//   //check if steps has decimals
//   double intpart;
//   if( modf( steps, &intpart) != 0 ) return false;

//   for(int i = 0; i <= steps; i++){

//     geometry_msgs::Pose target_pose;
//     target_pose.position.x = cx + radius * cos(angle * PI/180 * clockwise);
//     target_pose.position.y = originY;
//     target_pose.position.z = cz + radius * sin(angle * PI/180 * clockwise);
//     target_pose.orientation.x = startingPose.orientation.x;
//     target_pose.orientation.y = startingPose.orientation.y;
//     target_pose.orientation.z = startingPose.orientation.z;
//     target_pose.orientation.w = startingPose.orientation.w;
//     waypoints.push_back(target_pose);

//     if(angle >= endAngleDeg) break;

//     angle += stepDeg;
//   }


//   return true;
// }
// //  x = cx + r * cos(a)
// //  y = cy + r * sin(a)
// bool Path::getCircleTrajectoryXYBase(const geometry_msgs::Pose &startingPose, const double radius, const double startingAngleDeg, const double endAngleDeg, const double stepDeg, const int clockwise, std::vector<geometry_msgs::Pose> &waypoints){

//   double steps = (endAngleDeg-startingAngleDeg)/stepDeg;

//   double originX = startingPose.position.x;
//   double originY = startingPose.position.y;
//   double originZ = startingPose.position.z;

//   double angle = startingAngleDeg;

//   double cx = originX - radius*cos(angle * PI/180);
//   double cy = originY - radius*sin(angle * PI/180);

//   //check if steps has decimals
//   double intpart;
//   if( modf( steps, &intpart) != 0 ) return false;

//   for(int i = 0; i <= steps; i++){

//     geometry_msgs::Pose target_pose;
//     target_pose.position.x = cx + radius * cos(angle * PI/180 * clockwise);
//     target_pose.position.y = cy + radius * sin(angle * PI/180 * clockwise);
//     target_pose.position.z = originZ;
//     target_pose.orientation.x = startingPose.orientation.x;
//     target_pose.orientation.y = startingPose.orientation.y;
//     target_pose.orientation.z = startingPose.orientation.z;
//     target_pose.orientation.w = startingPose.orientation.w;
//     waypoints.push_back(target_pose);

//     if(angle >= endAngleDeg) break;

//     angle += stepDeg;
//   }


//   return true;
// }
// //  z = cz + r * cos(a) -> cz
// //  y = cy + r * sin(a)
// bool Path::getCircleTrajectoryYZBase(const geometry_msgs::Pose &startingPose, const double radius, const double startingAngleDeg, const double endAngleDeg, const double stepDeg, const int clockwise, std::vector<geometry_msgs::Pose> &waypoints){

//   double steps = (endAngleDeg-startingAngleDeg)/stepDeg;

//   double originX = startingPose.position.x;
//   double originY = startingPose.position.y;
//   double originZ = startingPose.position.z;

//   double angle = startingAngleDeg;


//   double cy = originY - radius*cos(angle * PI/180);
//   double cz = originZ - radius*sin(angle * PI/180);

//   //check if steps has decimals
//   double intpart;
//   if( modf( steps, &intpart) != 0 ) return false;

//   for(int i = 0; i <= steps; i++){

//     geometry_msgs::Pose target_pose;
//     target_pose.position.x = originX;
//     target_pose.position.y = cy + radius * cos(angle * PI/180 * clockwise);
//     target_pose.position.z = cz + radius * sin(angle * PI/180 * clockwise);
//     target_pose.orientation.x = startingPose.orientation.x;
//     target_pose.orientation.y = startingPose.orientation.y;
//     target_pose.orientation.z = startingPose.orientation.z;
//     target_pose.orientation.w = startingPose.orientation.w;
//     waypoints.push_back(target_pose);

//     if(angle >= endAngleDeg) break;

//     angle += stepDeg;
//   }


//   return true;
// }

//  x = cx + r * cos(alpha) sin(yaw) * sin(pitch)
//  y = cy + r * cos(alpha) * cos(yaw)
//  z = cz + r * sin(alpha) * cos(pitch)
bool Path::getCircleTrajectoryEEPlane(const geometry_msgs::Pose &startingPose, const double radius, const double startingAngleDeg, const double endAngleDeg, const double stepDeg, const int clockwise, std::vector<geometry_msgs::Pose> &waypoints){

  double steps = (endAngleDeg-startingAngleDeg)/stepDeg;
  double angle = startingAngleDeg;

  //center of circle from end effector -> cx, cy, cz ????
  // Eigen::Vector3d center(cx, cy, cz);
  Eigen::Vector3d center(0.0, - radius * cos(angle * PI/180), - radius * sin(angle * PI/180));

  //Getting orientation end effector -> quaternion-??
  //Eigen::Matrix3d R = Eigen::Quaterniond(quaternionW, quaternionX, quaternionY, quaternionZ).toRotationMatrix();
  Eigen::Matrix3d R = Eigen::Quaterniond(startingPose.orientation.w, startingPose.orientation.x, startingPose.orientation.y, startingPose.orientation.z).toRotationMatrix();

  //startingpose to eigen vector -> x,y,z ??
  //Eigen::Vector3d startingPoseBase(xBase, yBase, zBase);
  Eigen::Vector3d startingPoseBase(startingPose.position.x, startingPose.position.y, startingPose.position.z);
  //check if steps has decimals
  double intpart;
  if( modf( steps, &intpart) != 0 ) return false;


  for(int i = 0; i <= steps; i++){

    //Calculate point in EndEfector coordinates, then rotate it and sum to startingpose
    // Eigen::Vector3d pointEE(eex, eey, eez);
    // Eigen::Vector3d pointBase = RotationMatrix*pointEE + startingpose;
    
    Eigen::Vector3d pointEE(center(0), center(1) + radius * cos(angle * PI/180 * clockwise), center(2) + radius * sin(angle * PI/180 * clockwise));
    Eigen::Vector3d pointBase = R*pointEE + startingPoseBase;

    geometry_msgs::Pose target_pose;
    // target_pose.position.x = ;
    // target_pose.position.y = ;
    // target_pose.position.z = ;
    // target_pose.orientation.x = ;
    // target_pose.orientation.y = ;
    // target_pose.orientation.z = ;
    // target_pose.orientation.w = ;
    target_pose.position.x = pointBase(0);
    target_pose.position.y = pointBase(1);
    target_pose.position.z = pointBase(2);
    target_pose.orientation.x = startingPose.orientation.x;
    target_pose.orientation.y = startingPose.orientation.y;
    target_pose.orientation.z = startingPose.orientation.z;
    target_pose.orientation.w = startingPose.orientation.w;
    waypoints.push_back(target_pose);

    if(angle >= endAngleDeg) break;

    angle += stepDeg;
  }


  return true;
}

// bool Path::getLineTrajectoryXBase(const geometry_msgs::Pose &startingPose, const double distance, const int direction, const double step, std::vector<geometry_msgs::Pose> &waypoints){

//   double steps = distance/step;

//   double originX = startingPose.position.x;
//   double originY = startingPose.position.y;
//   double originZ = startingPose.position.z;

//   //check if steps has decimals
//   double intpart;
//   if( modf( steps, &intpart) != 0 ) return false;

//    for(int i = 0; i <= steps; i++){

//      geometry_msgs::Pose target_pose;
//      target_pose.position.x = originX + (step * i * direction);
//      target_pose.position.y = originY;
//      target_pose.position.z = originZ;
//      target_pose.orientation.x = startingPose.orientation.x;
//      target_pose.orientation.y = startingPose.orientation.y;
//      target_pose.orientation.z = startingPose.orientation.z;
//      target_pose.orientation.w = startingPose.orientation.w;
//      waypoints.push_back(target_pose);
//    }

//   return true;
// }

// bool Path::getLineTrajectoryYBase(const geometry_msgs::Pose &startingPose, const double distance, const int direction, const double step, std::vector<geometry_msgs::Pose> &waypoints){

//   double steps = distance/step;

//   double originX = startingPose.position.x;
//   double originY = startingPose.position.y;
//   double originZ = startingPose.position.z;

//   //check if steps has decimals
//   double intpart;
//   if( modf( steps, &intpart) != 0 ) return false;

//    for(int i = 0; i <= steps; i++){

//      geometry_msgs::Pose target_pose;
//      target_pose.position.x = originX;
//      target_pose.position.y = originY + (step * i * direction);
//      target_pose.position.z = originZ;
//      target_pose.orientation.x = startingPose.orientation.x;
//      target_pose.orientation.y = startingPose.orientation.y;
//      target_pose.orientation.z = startingPose.orientation.z;
//      target_pose.orientation.w = startingPose.orientation.w;
//      waypoints.push_back(target_pose);
//    }

//   return true;
// }
// bool Path::getLineTrajectoryZBase(const geometry_msgs::Pose &startingPose, const double distance, const int direction, const double step, std::vector<geometry_msgs::Pose> &waypoints){

//   double steps = distance/step;

//   double originX = startingPose.position.x;
//   double originY = startingPose.position.y;
//   double originZ = startingPose.position.z;

//   //Getting orientation end effector
//   Eigen::Matrix3d R = Eigen::Quaterniond(startingPose.orientation.w, startingPose.orientation.x, startingPose.orientation.y, startingPose.orientation.z).toRotationMatrix();


//   //check if steps has decimals
//   double intpart;
//   if( modf( steps, &intpart) != 0 ) return false;

//    for(int i = 0; i <= steps; i++){

//      geometry_msgs::Pose target_pose;
//      target_pose.position.x = originX;
//      target_pose.position.y = originY;
//      target_pose.position.z = originZ + (step * i * direction);
//      target_pose.orientation.x = startingPose.orientation.x;
//      target_pose.orientation.y = startingPose.orientation.y;
//      target_pose.orientation.z = startingPose.orientation.z;
//      target_pose.orientation.w = startingPose.orientation.w;
//      waypoints.push_back(target_pose);
//    }

//   return true;
// }

bool Path::getLineTrajectoryEEPlane(const geometry_msgs::Pose &startingPose, const double distance, const int direction, const double step, std::vector<geometry_msgs::Pose> &waypoints){

  double steps = distance/step;


  //Getting orientation end effector -> quaternion-??  
  // Eigen::Matrix3d R = Eigen::Quaterniond(quaternionW, quaternionX, quaternionY, quaternionZ).toRotationMatrix();
  Eigen::Matrix3d R = Eigen::Quaterniond(startingPose.orientation.w, startingPose.orientation.x, startingPose.orientation.y, startingPose.orientation.z).toRotationMatrix();
 
  //startingpose to eigen vector -> x,y,z ??
  // Eigen::Vector3d startingPoseBase(xBase, yBase, zBase);
  Eigen::Vector3d startingPoseBase(startingPose.position.x, startingPose.position.y, startingPose.position.z);


  //check if steps has decimals
  double intpart;
  if( modf( steps, &intpart) != 0 ) return false;

   for(int i = 0; i <= steps; i++){

    //Calculate point in EndEfector coordinates, then rotate it and sum to startingpose
    // Eigen::Vector3d pointEE(eex, eey, eez);
    // Eigen::Vector3d pointBase = RotationMatrix*pointEE + startingpose;
    
     Eigen::Vector3d pointEE(0.0, 0.0, step * i * direction);
     Eigen::Vector3d pointBase = R*pointEE + startingPoseBase;

     geometry_msgs::Pose target_pose;
    // target_pose.position.x = ;
    // target_pose.position.y = ;
    // target_pose.position.z = ;
    // target_pose.orientation.x = ;
    // target_pose.orientation.y = ;
    // target_pose.orientation.z = ;
    // target_pose.orientation.w = ;
     target_pose.position.x = pointBase(0);
     target_pose.position.y = pointBase(1);
     target_pose.position.z = pointBase(2);
     target_pose.orientation.x = startingPose.orientation.x;
     target_pose.orientation.y = startingPose.orientation.y;
     target_pose.orientation.z = startingPose.orientation.z;
     target_pose.orientation.w = startingPose.orientation.w;
     waypoints.push_back(target_pose);
   }

  return true;
}

