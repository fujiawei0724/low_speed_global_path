#pragma once

#include "Path.hpp"
#include <path_planning_msgs/Curve.h>
#include <path_planning_msgs/Path.h>
#include <path_planning_msgs/PathPoint.h>
#include <path_planning_msgs/MultiCurve.h>
#include <path_planning_msgs/BoundedCurve.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace PathPlanningUtilities
{
class CurveConverter
{
public:
  /**
   * PathPlanningUtilities::Curve to path_planning_msgs::Curve
  */
  static void toRosMessage(const PathPlanningUtilities::Curve& local_curve, path_planning_msgs::Curve& ros_curve);
  /**
   * PathPlanningUtilities::Curve to path_planning_msgs::Curve
  */
  static void fromRosMessage(const path_planning_msgs::CurveConstPtr& ros_curve, PathPlanningUtilities::Curve& local_curve);

  static void fromRosMessage(const path_planning_msgs::Curve& ros_curve, PathPlanningUtilities::Curve& local_curve);
};

class PathConverter
{
public:
  /**
   * PathPlanningUtilities::Curve to nav_msgs::Path
  */
  static void toRosMessage(const PathPlanningUtilities::Curve& local_curve, nav_msgs::Path& ros_path);

  /**
   * PathPlanningUtilities::Path to nav_msgs::Path
  */
  static void toRosMessage(const PathPlanningUtilities::Path& local_path, nav_msgs::Path& ros_path);

  /**
   * nav_msgs::Path to PathPlanningUtilities::Path
  */
  static void fromRosMessage(const nav_msgs::PathConstPtr& ros_path, PathPlanningUtilities::Path& local_path);

  /**
   * PathPlanningUtilities::Path to path_planning_msg::Path
   */
  static void toRosMessage(const PathPlanningUtilities::Path &local_path, path_planning_msgs::Path &ros_path);

  /**
   * path_planning_msgs::Path ti PathPlanningUtilities::Path
   */
  static void fromRosMessage(const path_planning_msgs::Path &ros_path, PathPlanningUtilities::Path &local_path);
};

class MultiCurveConverter
{
public:
  /**
   * PathPlanningUtilities::Curve vector to path_planning_msgs::MultiCurve
  */
  static void toRosMessage(const std::vector<PathPlanningUtilities::Curve>& local_multi_curve, path_planning_msgs::MultiCurve& ros_multi_curve);

  /**
   * path_planning_msgs::MultiCurve to PathPlanningUtilities::Curve vector
  */
  static void fromRosMessage(const path_planning_msgs::MultiCurveConstPtr& ros_multi_curve, std::vector<PathPlanningUtilities::Curve>& local_multi_curve);
};

class BoundedCurveConverter
{
public:
  /**
   * PathPlanningUtilities::BoundedCurve to path_planning_msgs::BoundedCurve
  */
  static void toRosMessage(const PathPlanningUtilities::BoundedCurve& local_bcurve, path_planning_msgs::BoundedCurve& ros_bcurve);
  /**
   * PathPlanningUtilities::BoundedCurve to path_planning_msgs::BoundedCurve
  */
  static void fromRosMessage(const path_planning_msgs::BoundedCurveConstPtr& ros_bcurve, PathPlanningUtilities::BoundedCurve& local_bcurve);
};

}