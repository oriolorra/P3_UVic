#ifndef PATH_H
#define PATH_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

class Path{

  public:

    Path();
    ~Path();

    // bool getCircleTrajectoryXZBase(const geometry_msgs::Pose& startingPose,
    //                       const double radius,
    //                       const double startingAngleDeg,
    //                       const double endAngleDeg,
    //                       const double stepDeg,
    //                       const int clockwise,
    //                       std::vector<geometry_msgs::Pose> &waypoints);

    // bool getCircleTrajectoryYZBase(const geometry_msgs::Pose& startingPose,
    //                       const double radius,
    //                       const double startingAngleDeg,
    //                       const double endAngleDeg,
    //                       const double stepDeg,
    //                       const int clockwise,
    //                       std::vector<geometry_msgs::Pose> &waypoints);
    // bool getCircleTrajectoryXYBase(const geometry_msgs::Pose& startingPose,
    //                       const double radius,
    //                       const double startingAngleDeg,
    //                       const double endAngleDeg,
    //                       const double stepDeg,
    //                       const int clockwise,
    //                       std::vector<geometry_msgs::Pose> &waypoints);

    bool getCircleTrajectoryEEPlane(const geometry_msgs::Pose& startingPose,
                             const double radius,
                             const double startingAngleDeg,
                             const double endAngleDeg,
                             const double stepDeg,
                             const int clockwise,       // 1,-1 -> clk-unclk
                             std::vector<geometry_msgs::Pose> &waypoints);

    // bool getLineTrajectoryXBase(const geometry_msgs::Pose &startingPose,
    //                    const double distance,
    //                    const int direction,
    //                    const double step,
    //                    std::vector<geometry_msgs::Pose> &waypoints);

    // bool getLineTrajectoryYBase(const geometry_msgs::Pose &startingPose,
    //                    const double distance,
    //                    const int direction,
    //                    const double step,
    //                    std::vector<geometry_msgs::Pose> &waypoints);

    // bool getLineTrajectoryZBase(const geometry_msgs::Pose &startingPose,
    //                    const double distance,
    //                    const int direction,
    //                    const double step,
    //                    std::vector<geometry_msgs::Pose> &waypoints);

    bool getLineTrajectoryEEPlane(const geometry_msgs::Pose &startingPose,
                       const double distance,
                       const int direction,   // 1,-1 -> x,-x     2,-2 -> y,-y        3,-3 -> z,-z
                       const double step,
                       std::vector<geometry_msgs::Pose> &waypoints);
};
#endif // PATH_H
