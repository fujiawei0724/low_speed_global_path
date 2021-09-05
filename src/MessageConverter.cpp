/*
 Copyright [2019] Songyi Zhang
*/

#include "MessageConverter.h"

#include <tf/tf.h>

namespace PathPlanningUtilities {

void CurveConverter::toRosMessage(const PathPlanningUtilities::Curve& local_curve, path_planning_msgs::Curve& ros_curve) {
  ros_curve.points.resize(local_curve.size());
  for (size_t i = 0; i < local_curve.size(); i++) {
    const PathPlanningUtilities::CurvePoint& local_curve_point = local_curve[i];
    path_planning_msgs::CurvePoint& ros_curve_point = ros_curve.points[i];

    ros_curve_point.x = local_curve_point.position_.x_;
    ros_curve_point.y = local_curve_point.position_.y_;
    ros_curve_point.theta = local_curve_point.theta_;
    ros_curve_point.kappa = local_curve_point.kappa_;
  }
  // ros_curve.is_last = false;
}

void CurveConverter::fromRosMessage(const path_planning_msgs::CurveConstPtr& ros_curve, PathPlanningUtilities::Curve& local_curve) {
  local_curve.resize(ros_curve->points.size());
  for (size_t i = 0; i < ros_curve->points.size(); i++) {
    const path_planning_msgs::CurvePoint& ros_curve_point = ros_curve->points[i];
    PathPlanningUtilities::CurvePoint& local_curve_point = local_curve[i];

    local_curve_point.position_.x_ = ros_curve_point.x;
    local_curve_point.position_.y_ = ros_curve_point.y;
    local_curve_point.theta_ = ros_curve_point.theta;
    local_curve_point.kappa_ = ros_curve_point.kappa;
  }
}

void CurveConverter::fromRosMessage(const path_planning_msgs::Curve& ros_curve, PathPlanningUtilities::Curve& local_curve) {
  local_curve.resize(ros_curve.points.size());
  for (size_t i = 0; i < ros_curve.points.size(); i++) {
    const path_planning_msgs::CurvePoint& ros_curve_point = ros_curve.points[i];
    PathPlanningUtilities::CurvePoint& local_curve_point = local_curve[i];

    local_curve_point.position_.x_ = ros_curve_point.x;
    local_curve_point.position_.y_ = ros_curve_point.y;
    local_curve_point.theta_ = ros_curve_point.theta;
    local_curve_point.kappa_ = ros_curve_point.kappa;
  }  
}

void PathConverter::toRosMessage(const PathPlanningUtilities::Curve& local_curve, nav_msgs::Path& ros_path) {
  ros_path.poses.resize(local_curve.size());
  for (size_t i = 0; i < local_curve.size(); i++) {
    const CurvePoint& local_curve_point = local_curve[i];
    geometry_msgs::Pose& ros_pose = ros_path.poses[i].pose;

    ros_pose.position.x = local_curve_point.position_.x_;
    ros_pose.position.y = local_curve_point.position_.y_;
    ros_pose.position.z = 0;
    ros_pose.orientation = tf::createQuaternionMsgFromYaw(local_curve_point.theta_);
  }
}

void PathConverter::toRosMessage(const PathPlanningUtilities::Path& local_path, nav_msgs::Path& ros_path) {
  ros_path.poses.resize(local_path.size());
  geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(0);
  for (size_t i = 0; i < local_path.size(); i++) {
    const Point2f& local_path_point = local_path[i];
    geometry_msgs::Pose& ros_pose = ros_path.poses[i].pose;

    ros_pose.position.x = local_path_point.x_;
    ros_pose.position.y = local_path_point.y_;
    ros_pose.position.z = 0;
    ros_pose.orientation = orientation;
  }
}

void PathConverter::fromRosMessage(const nav_msgs::PathConstPtr& ros_path, PathPlanningUtilities::Path& local_path) {
  local_path.resize(ros_path->poses.size());
  for (size_t i = 0; i < ros_path->poses.size(); i++) {
    const geometry_msgs::Pose& ros_pose = ros_path->poses[i].pose;
    Point2f& local_path_point = local_path[i];

    local_path_point.x_ = ros_pose.position.x;
    local_path_point.y_ = ros_pose.position.y;
  }
}

void PathConverter::toRosMessage(const PathPlanningUtilities::Path &local_path, path_planning_msgs::Path &ros_path) {
  ros_path.points.resize(local_path.size());
  for (size_t i = 0; i < local_path.size(); i++) {
    path_planning_msgs::PathPoint point;
    point.x = local_path[i].x_;
    point.y = local_path[i].y_;
    ros_path.points[i] = point;
  }
}

void PathConverter::fromRosMessage(const path_planning_msgs::Path &ros_path, PathPlanningUtilities::Path &local_path) {
  local_path.resize(ros_path.points.size());
  for (size_t i = 0; i < ros_path.points.size(); i++) {
    PathPlanningUtilities::Point2f point;
    point.x_ = ros_path.points[i].x;
    point.y_ = ros_path.points[i].y;
    local_path[i] = point;
  }
}

void MultiCurveConverter::toRosMessage(const std::vector<PathPlanningUtilities::Curve>& local_multi_curve, path_planning_msgs::MultiCurve& ros_multi_curve) {
  ros_multi_curve.curves.resize(local_multi_curve.size());
  for (size_t i = 0; i < local_multi_curve.size(); i++) {
    CurveConverter::toRosMessage(local_multi_curve[i], ros_multi_curve.curves[i]);
  }
}

void MultiCurveConverter::fromRosMessage(const path_planning_msgs::MultiCurveConstPtr& ros_multi_curve, std::vector<PathPlanningUtilities::Curve>& local_multi_curve) {
  local_multi_curve.resize(ros_multi_curve->curves.size());
  for (size_t i = 0; i < ros_multi_curve->curves.size(); i++) {
    path_planning_msgs::CurveConstPtr ros_curve(&ros_multi_curve->curves[i]);
    CurveConverter::fromRosMessage(ros_curve, local_multi_curve[i]);
  }
}

void BoundedCurveConverter::toRosMessage(const PathPlanningUtilities::BoundedCurve& local_bcurve, path_planning_msgs::BoundedCurve& ros_bcurve) {
  ros_bcurve.points.resize(local_bcurve.size());
  for (size_t i = 0; i < local_bcurve.size(); i++) {
    const PathPlanningUtilities::BoundedCurvePoint& local_bpoint = local_bcurve[i];
    path_planning_msgs::BoundedCurvePoint& ros_bpoint = ros_bcurve.points[i];

    ros_bpoint.center_point.x = local_bpoint.center_point_.position_.x_;
    ros_bpoint.center_point.y = local_bpoint.center_point_.position_.y_;
    ros_bpoint.center_point.theta = local_bpoint.center_point_.theta_;
    ros_bpoint.center_point.kappa = local_bpoint.center_point_.kappa_;
    ros_bpoint.left_distance = local_bpoint.left_distance_;
    ros_bpoint.right_distance = local_bpoint.right_distance_;
  }
}

void BoundedCurveConverter::fromRosMessage(const  path_planning_msgs::BoundedCurveConstPtr& ros_bcurve, PathPlanningUtilities::BoundedCurve& local_bcurve) {
  local_bcurve.resize(ros_bcurve->points.size());
  for (size_t i = 0; i < ros_bcurve->points.size(); i++) {
    const path_planning_msgs::BoundedCurvePoint& ros_bpoint = ros_bcurve->points[i];
    PathPlanningUtilities::BoundedCurvePoint& local_bpoint = local_bcurve[i];

    local_bpoint.center_point_.position_.x_ = ros_bpoint.center_point.x;
    local_bpoint.center_point_.position_.y_ = ros_bpoint.center_point.y;
    local_bpoint.center_point_.theta_ =  ros_bpoint.center_point.theta;
    local_bpoint.center_point_.kappa_ = ros_bpoint.center_point.kappa;
    local_bpoint.left_distance_ = ros_bpoint.left_distance;
    local_bpoint.right_distance_ = ros_bpoint.right_distance;
  }
}

}  // namespace PathPlanningUtilities
