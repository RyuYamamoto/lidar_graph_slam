#ifndef _DATA_STRUCT_
#define _DATA_STRUCT_

#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

struct Pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  Pose() : x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0) {}
  Pose(double x, double y, double z, double roll, double pitch, double yaw)
  : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw)
  {
  }
  void init()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
  }
  Pose operator+(Pose input_pose)
  {
    Pose output_pose;
    output_pose.x = this->x + input_pose.x;
    output_pose.y = this->y + input_pose.y;
    output_pose.z = this->z + input_pose.z;
    output_pose.roll = this->roll + input_pose.roll;
    output_pose.pitch = this->pitch + input_pose.pitch;
    output_pose.yaw = this->yaw + input_pose.yaw;
    return output_pose;
  }
  Pose operator-(Pose input_pose)
  {
    Pose output_pose;
    output_pose.x = this->x - input_pose.x;
    output_pose.y = this->y - input_pose.y;
    output_pose.z = this->z - input_pose.z;
    output_pose.roll = calcDiffForRadian(this->roll, input_pose.roll);
    output_pose.pitch = calcDiffForRadian(this->pitch, input_pose.pitch);
    output_pose.yaw = calcDiffForRadian(this->yaw, input_pose.yaw);
    return output_pose;
  }
  double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
  {
    double diff_rad = lhs_rad - rhs_rad;
    if (M_PI <= diff_rad)
      diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
      diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
  }
};

#endif
