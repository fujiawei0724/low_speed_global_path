/*
 * @Author: fjw 
 * @Date: 2021-04-23 10:13:11 
 * @Last Modified by: fjw
 * @Last Modified time: 2021-04-23 14:34:16
 */
#pragma once

#include "boost/filesystem.hpp"
#include <glog/logging.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
#include <self_test/self_test.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/SurroundReport.h>
#include <dbw_mkz_msgs/TurnSignal.h>
#include <dbw_mkz_msgs/TurnSignalCmd.h>
#include <vec_map_cpp_msgs/LocalLane.h>
#include <vec_map_cpp_msgs/VirtualObstacle.h>
#include <vec_map_cpp_msgs/GetGuidedCurves.h>
#include <vec_map_cpp_msgs/GetPredictedTrajectory.h>
#include <path_planning_msgs/BoundedCurve.h>
#include <path_planning_msgs/Curve.h>
#include <path_planning_msgs/CurvePoint.h>
#include <path_planning_msgs/MotionPlanningCurve.h>
#include <path_planning_msgs/Path.h>
#include <path_planning_msgs/PathPoint.h>
#include <traffic_light_msgs/traffic_lights.h>
#include <ibeo_lidar_msgs/object_filter_data.h>
#include <ibeo_lidar_msgs/object_filter.h>
#include <ibeo_lidar_msgs/Contour_box.h>
#include <mission_msgs/StartMain.h>
#include <mission_msgs/RequireTurnAround.h>
#include <control_msgs/CoreReport.h>
#include <std_srvs/Trigger.h>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <typeinfo>
#include <thread>
#include <mutex>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>           
#include <math.h>
#include <vector>
#include <queue>
#include <set>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <ctime>
#include "Path.hpp"
#include "PathGenerator.h"
#include "PathUtilities.h"
#include "Point.hpp"
#include "LineSegment.hpp"
#include "Obstacle.hpp"
#include "Lane.hpp"
#include "Tools.hpp"
#include "Visualization.hpp"
#include "MessageConverter.h"


