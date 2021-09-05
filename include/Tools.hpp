/*
    Copyright [2019] Jian ZhiQiang
*/

#pragma once

#include "Common.hpp"
#include "Point.hpp"
#include "Path.hpp"
#include "PathGenerator.h"
#include "Const.hpp"


// 工具函数集合
namespace Tools {
// 计算曲线的最大曲率点
size_t calcMaxKappaIndexForCurve(const PathPlanningUtilities::Curve &curve);

// 计算坐标系转换
PathPlanningUtilities::Point2f calcNewCoordinationPosition(const PathPlanningUtilities::VehicleState &new_coordination_origin, const PathPlanningUtilities::Point2f &position);

// 计算坐标系转换
PathPlanningUtilities::Point2f calcNewCoordinationPosition(const PathPlanningUtilities::CurvePoint &new_coordination_origin, const PathPlanningUtilities::Point2f &position);

// 将点从原始坐标系转化到新坐标系
void transferPointCoordinateSystem(const TransMatrix &transformer, const Eigen::Vector2d &point_in_origin_coordinate, Eigen::Vector2d *point_in_new_coordinate);

void transferThetaCoordinateSystem(const TransMatrix &transformer, double theta_in_origin_coordinate, double *theta_in_new_coordinate);

// 拉伸于收缩路径（好像没用了）
void shrinkPath(PathPlanningUtilities::Path *path, double ratio);

// 压缩路径
void zipPath(PathPlanningUtilities::Path *path, double max_offset);

// 计算规划距离(测试版 TOFIX)
double normalMotionDistance(double velocity, double deccelaration, double gap_distance);

// 计算障碍物占用区距离(测试版 TOFIX)
double normalObstacleVehicleOccupationDistance(double velocity, double deccelaration);

// 计算本车占用区距离（TOFIX）
double normalSubvehicleOccupationDistance(double velocity, double deccelaration, double gap_distance);

// 计算预见长度（测试版 TOFIX）
double normalForeseenDistance(double velocity, double deccelaration, double gap_distance);

// 计算行列式
double determinant(double v1, double v2, double v3, double v4);

// 判断两条线段是否发生相交
bool isLineSegmentInteracted(const path_planning_msgs::PathPoint &aa, const path_planning_msgs::PathPoint &bb, const path_planning_msgs::PathPoint &cc, const path_planning_msgs::PathPoint &dd);

// 判断两条曲线是否发生相交
bool isLinesInteracted(const std::vector<path_planning_msgs::PathPoint> &line_raw, const std::vector<path_planning_msgs::PathPoint> &line_judge, double gap = 1.0);

// 根据当前速度得到应有的加速度
double velocityToAccelerationReflection(double velocity);

// 计算法向加速度
double calcNormalAcceleration(double velocity, double kappa);

// 计算曲线上的最大法向加速度
double calcMaxNormalAccelerationForCurve(const PathPlanningUtilities::Curve &curve, double velocity);

// 根据最大法向加速度计算最大速度
double calcVelocityForMaxNormalAcceleration(double kappa);

// 计算法向加加速度
double calcNormalJerk(double kappa, double velocity, double acceleration);

// 根据最大法向加加速度计算最大加速度
double calcAccelerationForMaxNormalJerk(double kappa, double velocity);

// 最大法向加加速度计算最大速度
double calcVelocityForMaxNormalJerk(double curvature_change_rate);

// 计算曲线的最大角度
double getMaxThetaForCurve(const PathPlanningUtilities::Curve &curve);

// 计算当前位置离道路最近的点
size_t findNearestPositionIndexInCurve(const PathPlanningUtilities::Curve &curve, const PathPlanningUtilities::Point2f &current_position, size_t start_index = 0);

// 计算当前位置离车道最近的点
size_t findNearestPositionIndexInCoordination(const std::vector<PathPlanningUtilities::CoordinationPoint> &coordination_points, const PathPlanningUtilities::Point2f &current_position, size_t start_index = 0);

// 是否避障的规划距离不固定？（TODO）

// 寻找邻居状态
bool searchNeighborStates(std::vector<size_t> neighbors, size_t search);

//　终点集合展开
void goalSetGenerate(const PathPlanningUtilities::BoundedCurvePoint& goal, const double space, const int number, std::vector<PathPlanningUtilities::CurvePoint> *goal_set);

// 计算偏移点(好像没用了)
void pointAdjust(const PathPlanningUtilities::CurvePoint &origin_point, double adjust_distance, PathPlanningUtilities::CurvePoint *new_point);

void pointAdjust(const PathPlanningUtilities::VehicleState &origin_point, double adjust_distance, PathPlanningUtilities::VehicleState *new_point);

void pointAdjust(const PathPlanningUtilities::CurvePoint &origin_point, double adjust_distance, PathPlanningUtilities::Point2f *new_point);

// 根据车辆状态信息对起、终点进行位置调整(好像没用了)
void adjustStartAndGoalPoint(const PathPlanningUtilities::VehicleState &start_point, const PathPlanningUtilities::CurvePoint &goal_point, double vehicle_length, double rear_axis_center_scale, PathPlanningUtilities::VehicleState *adjusted_start_point, PathPlanningUtilities::CurvePoint *adjusted_goal_point);

// 调整路径点到车辆后轴中心上
void adjustCurvePoints(const PathPlanningUtilities::Curve &curve, double vehicle_length, double rear_axis_center_scale, PathPlanningUtilities::Path *adjusted_path);

// 保存路径到指定文件
void savePoints(const std::string &file_path, const PathPlanningUtilities::Path &path);

void savePoints(const std::string &file_path, const PathPlanningUtilities::Curve &curve);

// 创建日志文件夹
void resetLogFile(const std::string &file_path);

// 限制当前值在最大、最小值之间
double limitValue(double min_value, double max_value, double current_value);

// 计算等待的安全距离
double getWaitSafeDistance(double subvehicle_velocity, double obstacle_velocity);

// 计算等待的期望距离
double getWaitExpectedDistance(double subvehicle_velocity, double obstacle_velocity);

// 计算超越安全距离
double getOvertakeSafeDistance(double subvehicle_velocity, double obstacle_velocity);

// 计算超越期望距离
double getOvertakeExpectedDistance(double subvehicle_velocity, double obstacle_velocity);

// 一维数组进行kmeans聚类
std::vector<double> kMeans(const std::vector<double> &data, size_t k, std::vector<std::vector<size_t>> &clusters);

// 返回当前unix时间戳
std::string returnCurrentTimeAndDate();

// 给出几何朝向, 几何中心的曲率, 几何中心到后轴中心的距离, 计算后轴中心朝向
double centerYawToRearYaw(double center_yaw, double center_curvature, double rear_to_center_distance);

// 给出后轴中心朝向, 给出后轴中心曲率, 计算几何中心的朝向
double rearYawToCenterYaw(double rear_yaw, double rear_curvature, double rear_to_center_distance);

// 给出几何中心曲率, 几何中心到后轴中心的距离, 计算后轴中心曲率
double centerToRearCurvature(double center_curvature, double rear_to_center_distance);

// 给出后轴中心曲率, 后轴中心到后轴中心的距离, 计算几何中心曲率
double rearToCenterCurvature(double rear_curvature, double rear_to_center_distance);

// 中值滤波, window_size为奇数
void medianFilter(std::vector<double> &data, int window_size);

// 得到路径最大曲率变化率
double getCurveMaxCurvatureChangeRate(const PathPlanningUtilities::Curve &curve);

// 角度安全转换
double angleSafeConversion(double raw_angle);

// 障碍物散点边界补全
std::vector<PathPlanningUtilities::Point2f> constructObstacleBoundary(const
PathPlanningUtilities::Point2f &front_point, const PathPlanningUtilities::Point2f &rear_point, double resolution = 0.1);

// 计算acos
double safeAcos(double x);


// 采样函数
template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{

    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0) { return linspaced; }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for(int i=0; i < num-1; ++i){
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end);
    return linspaced;
}

double calcDistanceBetweenPoint2f(const PathPlanningUtilities::Point2f &point_1, const PathPlanningUtilities::Point2f &point_2);

// calculate theta information from scatter point
double calcThetaInfo(const PathPlanningUtilities::Point2f &cur_point, const PathPlanningUtilities::Point2f &pre_point);

// calculate curvature information from scatter points
double calcCurvatureInfo(const PathPlanningUtilities::Point2f &pre_point, const PathPlanningUtilities::Point2f &cur_point, const PathPlanningUtilities::Point2f &next_point);

// delete path points for hybrid astar test
PathPlanningUtilities::Path deletePathPoint(const PathPlanningUtilities::Path &initial_path, double index_gap);

};  // namespace Tools

