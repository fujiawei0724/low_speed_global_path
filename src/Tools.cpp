/*
    Copyright [2019] Jian ZhiQiang
*/

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include "Common.hpp"


// 判断一个double类型的数是否等于0
bool Tools::isZero(double value) {
    if (std::fabs(value) <= EPS) {
        return true;
    } else {
        return false;
    }
}

// 判断前一个double是否大于后一个double
bool Tools::isLarge(double value_1, double value_2) {
    if (value_1 > value_2 && std::fabs(value_1 - value_2) > EPS) {
        return true;
    } else {
        return false;
    }
}

// 判断前一个double是否小于后一个double
bool Tools::isSmall(double value_1, double value_2) {
    if (value_1 < value_2 && std::fabs(value_1 - value_2) > EPS) {
        return true;
    } else {
        return false;
    }
}

// 判断前一个double是否等于后一个double
bool Tools::isEqual(double value_1, double value_2) {
    if (std::fabs(value_1 - value_2) <= EPS) {
        return true;
    } else {
        return false;
    }
}


// 根据当前速度得到应有的加速度
// double Tools::velocityToAccelerationReflection(double velocity) {
//     double acceleration;
//     if (isSmall(velocity, 10.0)) {
//         acceleration = 0.5 + velocity * 0.05;
//     } else if (isSmall(velocity, 15.0)) {
//         acceleration = 1.0;
//     } else if (isSmall(velocity, 30.0)) {
//         acceleration = 1.75 - velocity * 0.05;
//     } else {
//         acceleration = 0.25;
//     }
//     return acceleration;
// }
// 根据当前速度得到应有的加速度
// double Tools::velocityToAccelerationReflection(double velocity) {
//     double acceleration;
//     if (isSmall(velocity, 8.0)) {
//         acceleration = 0.8 + velocity * 0.05;
//     } else if (isSmall(velocity, 10.0)) {
//         acceleration = 1.2;
//     } else if (isSmall(velocity, 30.0)) {
//         acceleration = 1.7 - velocity * 0.05;
//     } else {
//         acceleration = 0.2;
//     }
//     return acceleration;
// }
// 根据当前速度得到应有的加速度
double Tools::velocityToAccelerationReflection(double velocity) {
    double acceleration;
    if (isSmall(velocity, 6.667)) {
        acceleration = 0.9 + velocity * 0.0675;
    } else if (isSmall(velocity, 16.667)) {
        acceleration = 1.35;
    } else if (isSmall(velocity, 30.0)) {
        acceleration = 2.7 - velocity * 0.081;
    } else {
        acceleration = 0.27;
    }
    return acceleration;
}

// 计算法向加速度
double Tools::calcNormalAcceleration(double velocity, double kappa) {
    // 曲率不能为负值
    kappa = std::fabs(kappa);
    return velocity * velocity * kappa;
}

// 计算曲线的最大曲率点
size_t Tools::calcMaxKappaIndexForCurve(const PathPlanningUtilities::Curve &curve) {
    double max_kappa = 0.0;
    size_t index = 0;
    for (size_t i = 0; i < curve.size(); i++) {
        double kappa = std::fabs(curve[i].kappa_);
        if (!isSmall(kappa, max_kappa)) {
            max_kappa = kappa;
            index = i;
        }
    }
    return index;
}

// 计算曲线上的最大法向加速度
double Tools::calcMaxNormalAccelerationForCurve(const PathPlanningUtilities::Curve &curve, double velocity) {
    size_t index = calcMaxKappaIndexForCurve(curve);
    double kappa = curve[index].kappa_;
    return calcNormalAcceleration(velocity, kappa);
}

// 根据最大法向加速度计算最大速度
double Tools::calcVelocityForMaxNormalAcceleration(double kappa) {
    // 曲率不能为负值
    kappa = std::fabs(kappa);
    kappa = kappa + EPS;
    return sqrt(MAX_NORMAL_ACCELERATION / kappa);
}

// 计算法向加加速度
double Tools::calcNormalJerk(double kappa, double velocity, double acceleration) {
    // 曲率不能为负
    kappa = std::fabs(kappa);
    return 2.0 * kappa * velocity * acceleration;
}

// 根据最大法向加加速度计算最大加速度
double Tools::calcAccelerationForMaxNormalJerk(double kappa, double velocity) {
    // 曲率不能为负
    kappa = std::fabs(kappa);
    kappa = kappa + EPS;
    return MAX_NORMAL_JERK / (2.0 * kappa * velocity);
}

// 最大法向加加速度计算最大速度
double Tools::calcVelocityForMaxNormalJerk(double curvature_change_rate) {
    curvature_change_rate = std::abs(curvature_change_rate) + EPS;
    return sqrt(MAX_NORMAL_JERK / (2.0 * curvature_change_rate));
}

// 计算曲线中存在的最大角度变化
double Tools::getMaxThetaForCurve(const PathPlanningUtilities::Curve &curve) {
    double max_theta = 0.0;
    for (size_t i = 0; i < curve.size(); i++) {
        double theta = std::fabs(curve[i].theta_ - curve[0].theta_);
        if (isLarge(theta, max_theta)) {
            max_theta = theta;
        }
    }
    return max_theta;
}


// 将点从原始坐标系转化到新坐标系
void Tools::transferPointCoordinateSystem(const TransMatrix &transformer,
                                          const Eigen::Vector2d &point_in_origin_coordinate,
                                          Eigen::Vector2d *point_in_new_coordinate) {
    *point_in_new_coordinate = transformer.rotation_ * point_in_origin_coordinate + transformer.trans_;
}

void Tools::transferThetaCoordinateSystem(const TransMatrix &transformer,
                                          double theta_in_origin_coordinate,
                                          double *theta_in_new_coordinate) {
    double sin_theta_of_sd = transformer.rotation_(1, 0)*cos(theta_in_origin_coordinate) + transformer.rotation_(1, 1)*sin(theta_in_origin_coordinate);
    double cos_theta_of_sd = -transformer.rotation_(1, 0)*sin(theta_in_origin_coordinate) + transformer.rotation_(1, 1)*cos(theta_in_origin_coordinate);
    *theta_in_new_coordinate = atan2(sin_theta_of_sd, cos_theta_of_sd);
}

// 拉伸于收缩路径
void Tools::shrinkPath(PathPlanningUtilities::Path *path, double ratio) {
    PathPlanningUtilities::Point2f start_point = (*path)[0];
    for (size_t i = 0; i < (*path).size(); i++) {
        (*path)[i].x_ = ((*path)[i].x_ - start_point.x_)*ratio + start_point.x_;
    }
}

// 压缩路径
void Tools::zipPath(PathPlanningUtilities::Path *path, double max_offset) {
    for (size_t i = 0; i < path->size(); i++) {
        double offset_y = (*path)[i].y_ - (*path)[0].y_;
        if (isSmall(offset_y, max_offset / 2.0))
        (*path)[i].y_ = max_offset * tanh(offset_y / max_offset) + (*path)[0].y_;
    }

}

// 计算规划距离(测试版 TOFIX)
double Tools::normalMotionDistance(double velocity,
                                   double deccelaration,
                                   double gap_distance) {
    return 1.5 * velocity + velocity * velocity/(2 * deccelaration) + gap_distance;
}

// 障碍物占用区距离（测试版 TOFIX）
double Tools::normalObstacleVehicleOccupationDistance(double velocity,
                                                      double deccelaration) {
    return  4.5 * velocity + velocity * velocity/(2 * deccelaration) + 2.0;
}

// 计算本车占用区距离（TOFIX）
double Tools::normalSubvehicleOccupationDistance(double velocity, 
                                                 double deccelaration, 
                                                 double gap_distance) {
    return  2.0 * velocity + velocity * velocity/(2 * 2.0) + gap_distance;
}

// 预见长度（测试版 TOFIX）
double Tools::normalForeseenDistance(double velocity,
                                     double deccelaration,
                                     double gap_distance) {
    return  2.0 * velocity + velocity * velocity/(2 * deccelaration) + 2.0;
}

// 计算行列式
double Tools::determinant(double v1, double v2, double v3, double v4) {
    return (v1 * v4 - v2 * v3);
}

// 判断线段是否相交
bool Tools::isLineSegmentInteracted(const path_planning_msgs::PathPoint &aa, const path_planning_msgs::PathPoint &bb, const path_planning_msgs::PathPoint &cc, const path_planning_msgs::PathPoint &dd) {
    double delta = determinant(bb.x-aa.x, cc.x-dd.x, bb.y-aa.y, cc.y-dd.y);
    if (isZero(delta)) {
        // delta=0，表示两线段重合或平行
        return false;
    }
    double namenda = determinant(cc.x-aa.x, cc.x-dd.x, cc.y-aa.y, cc.y-dd.y) / delta;
    if (isLarge(namenda, 1.0) || isSmall(namenda, 0.0)) {
        return false;
    }
    double miu = determinant(bb.x-aa.x, cc.x-aa.x, bb.y-aa.y, cc.y-aa.y) / delta;
    if (isLarge(miu, 1.0) || isSmall(miu, 0.0)) {
        return false;
    }
    return true;
}

// 判断两条曲线是否发生相交
bool Tools::isLinesInteracted(const std::vector<path_planning_msgs::PathPoint> &line_raw, const std::vector<path_planning_msgs::PathPoint> &line_judge, double gap) {
    for (size_t i = 0; i < line_raw.size() - 1; i++) {
        path_planning_msgs::PathPoint line1_point1 = line_raw[i];
        path_planning_msgs::PathPoint line1_point2 = line_raw[i + 1];
        for (size_t j = 0; j < line_judge.size() - 1; j++) {
            path_planning_msgs::PathPoint line2_point1 = line_judge[j];
            path_planning_msgs::PathPoint line2_point2 = line_judge[j + 1];
            // 计算线段是否相交
            if (isLineSegmentInteracted(line1_point1, line1_point2, line2_point1, line2_point2)) {
                return true;
            }
        }
    }
    return false;
}

// 计算坐标系转换
PathPlanningUtilities::Point2f Tools::calcNewCoordinationPosition(const PathPlanningUtilities::CurvePoint &new_coordination_origin, const PathPlanningUtilities::Point2f &position) {
    PathPlanningUtilities::Point2f new_position;
    new_position.x_ = (position.x_ - new_coordination_origin.position_.x_) * cos(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * sin(new_coordination_origin.theta_);
    new_position.y_ = -(position.x_ - new_coordination_origin.position_.x_) * sin(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * cos(new_coordination_origin.theta_);
    return new_position;
}

// 计算坐标系转换
PathPlanningUtilities::Point2f Tools::calcNewCoordinationPosition(const PathPlanningUtilities::VehicleState &new_coordination_origin, const PathPlanningUtilities::Point2f &position) {
    PathPlanningUtilities::Point2f new_position;
    new_position.x_ = (position.x_ - new_coordination_origin.position_.x_) * cos(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * sin(new_coordination_origin.theta_);
    new_position.y_ = -(position.x_ - new_coordination_origin.position_.x_) * sin(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * cos(new_coordination_origin.theta_);
    return new_position;
}

// 计算当前位置离道路最近的点
size_t Tools::findNearestPositionIndexInCurve(const PathPlanningUtilities::Curve &curve, const PathPlanningUtilities::Point2f &current_position, size_t start_index) {
    size_t current_position_index = curve.size() - 1;
    for (size_t i = start_index; i < curve.size(); i++) {
        PathPlanningUtilities::Point2f local_position = calcNewCoordinationPosition(curve[i], current_position);
        if (!isLarge(local_position.x_, 0.0)) {
            // 当前位置在路径起始点后面
            current_position_index = i;
            break;
        }
    }
    return current_position_index;
}

// 计算当前位置离车道最近的点
size_t Tools::findNearestPositionIndexInCoordination(const std::vector<PathPlanningUtilities::CoordinationPoint> &coordination_points, const PathPlanningUtilities::Point2f &current_position, size_t start_index) {
    size_t current_position_index = coordination_points.size() - 1;
    for (size_t i = start_index; i < coordination_points.size(); i++) {
        PathPlanningUtilities::Point2f local_position = calcNewCoordinationPosition(coordination_points[i].worldpos_, current_position);
        if (isSmall(local_position.x_, 0.0)) {
            // 当前位置在路径起始点后面
            current_position_index = i;
            break;
        }
    }
    return current_position_index;
}

// 是否避障的规划距离不固定？（TODO）

// 寻找邻居状态
bool Tools::searchNeighborStates(std::vector<size_t> neighbors, size_t search) {
    bool check = false;
    for (size_t i = 0; i < neighbors.size(); i++) {
        if (neighbors[i] == search) {
            check = true;
        }
    }
    return check;
}

//　终点集合展开
void Tools::goalSetGenerate(const PathPlanningUtilities::BoundedCurvePoint& goal, const double space, const int number, std::vector<PathPlanningUtilities::CurvePoint> *goal_set) {
    std::vector<PathPlanningUtilities::CurvePoint>().swap(*goal_set);
    for (int i = 0; i< 2 * number + 1; i++) {
        PathPlanningUtilities::CurvePoint wanted_goal;
        bool put_in = true;
        if (isLarge((number - i)*space, - goal.right_distance_) && isSmall((number - i)*space, goal.left_distance_)) {
            wanted_goal.position_.x_ = goal.center_point_.position_.x_;
            wanted_goal.position_.y_ = goal.center_point_.position_.y_ + (number - i)*space;
            wanted_goal.theta_ = goal.center_point_.theta_;
            wanted_goal.kappa_ = goal.center_point_.kappa_;
        } else {
            put_in = false;
        }
        if (put_in) {
            goal_set->push_back(wanted_goal);
        }
    }
}

// 计算偏移点(好像没用了)
void Tools::pointAdjust(const PathPlanningUtilities::CurvePoint &origin_point, double adjust_distance, PathPlanningUtilities::CurvePoint *new_point) {
    new_point->theta_ = origin_point.theta_;
    new_point->kappa_ = origin_point.kappa_;
    new_point->position_.x_ = origin_point.position_.x_ + adjust_distance * cos(origin_point.theta_);
    new_point->position_.y_ = origin_point.position_.y_ + adjust_distance * sin(origin_point.theta_);
}

void Tools::pointAdjust(const PathPlanningUtilities::VehicleState &origin_point, double adjust_distance, PathPlanningUtilities::VehicleState *new_point) {
    new_point->theta_ = origin_point.theta_;
    new_point->kappa_ = origin_point.kappa_;
    new_point->position_.x_ = origin_point.position_.x_ + adjust_distance * cos(origin_point.theta_);
    new_point->position_.y_ = origin_point.position_.y_ + adjust_distance * sin(origin_point.theta_);
}

void Tools::pointAdjust(const PathPlanningUtilities::CurvePoint &origin_point, double adjust_distance, PathPlanningUtilities::Point2f *new_point) {
    new_point->x_ = origin_point.position_.x_ + adjust_distance * cos(origin_point.theta_);
    new_point->y_ = origin_point.position_.y_ + adjust_distance * sin(origin_point.theta_);
}

// 根据车辆状态信息对起、终点进行位置调整(好像没用了)
void Tools::adjustStartAndGoalPoint(const PathPlanningUtilities::VehicleState &start_point, const PathPlanningUtilities::CurvePoint &goal_point, double vehicle_length, double rear_axis_center_scale, PathPlanningUtilities::VehicleState *adjusted_start_point, PathPlanningUtilities::CurvePoint *adjusted_goal_point) {
    double start_distance = vehicle_length * rear_axis_center_scale - vehicle_length;
    double goal_distance = vehicle_length * rear_axis_center_scale;
    pointAdjust(start_point, start_distance, adjusted_start_point);
    pointAdjust(goal_point, goal_distance, adjusted_goal_point);
}

// 调整路径点到车辆后轴中心上(好像没用了)
void Tools::adjustCurvePoints(const PathPlanningUtilities::Curve &curve, double vehicle_length, double rear_axis_center_scale, PathPlanningUtilities::Path *adjusted_path) {
    adjusted_path->resize(curve.size());
    for (size_t i = 0; i < curve.size(); i++) {
        double scale = 1.0 - rear_axis_center_scale - static_cast<double>(i)/static_cast<double>(curve.size() - 1);
        double distance = scale * vehicle_length;
        pointAdjust(curve[i], distance, &((*adjusted_path)[i]));
    }
}

// 保存路径到指定文件
void Tools::savePoints(const std::string &file_path, const PathPlanningUtilities::Path &path) {
    //　如果文件存在则删除
    if (access(file_path.c_str(), 0) != -1) {
        remove(file_path.c_str());
    }
    // 写入文件
    std::ofstream os_file;
    os_file.open(file_path.c_str(), std::ios::app);
    for (size_t i = 0; i < path.size(); i++) {
        os_file << path[i].x_ << "," << path[i].y_ << std::endl;
    }
    os_file.close();
}

void Tools::savePoints(const std::string &file_path, const PathPlanningUtilities::Curve &curve) {
    //　如果文件存在则删除
    if (access(file_path.c_str(), 0) != -1) {
        remove(file_path.c_str());
    }
    // 写入文件
    std::ofstream os_file;
    os_file.open(file_path.c_str(), std::ios::app);
    for (size_t i = 0; i < curve.size(); i++) {
        os_file << curve[i].position_.x_ << "," << curve[i].position_.y_ << std::endl;
    }
    os_file.close();
}

// 创建日志文件夹
void Tools::resetLogFile(const std::string &file_path) {
    if (access(file_path.c_str(), 0) == -1) {
        // 如果文件夹不存在则创建文件夹
        mkdir(file_path.c_str(), S_IRWXU|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH);
    }
}

// 限制当前值在最大、最小值之间
double Tools::limitValue(double min_value, double max_value, double current_value) {
    return std::max(min_value, std::min(max_value, current_value));
}

// 计算等待的安全距离
double Tools::getWaitSafeDistance(double subvehicle_velocity, double obstacle_velocity) {
    // 在本车进行等待时，判断所需的安全距离
    double safe_distance;
    // 随后计算前方车刹车时，不会发生碰撞所需距离
    double decceleration_safe_distance = 10.0 + subvehicle_velocity * subvehicle_velocity / (2.0 * MAX_DECCELERATION) - obstacle_velocity * obstacle_velocity / (2.0 * MAX_DECCELERATION);
    // 需要与前车保持的安全距离
    double keep_distance = 10.0 + 0.5 * subvehicle_velocity + pow(subvehicle_velocity - obstacle_velocity, 2.0) *(Tools::isLarge(subvehicle_velocity, obstacle_velocity) ? 1.0 : -1.0);
    // 得到安全距离
    safe_distance = std::max(std::max(keep_distance, decceleration_safe_distance), 10.0 + 0.3 * subvehicle_velocity);
    return safe_distance;
}

// 计算等待的期望距离
double Tools::getWaitExpectedDistance(double subvehicle_velocity, double obstacle_velocity) {
    // 在本车进行等待时，判断所期望的安全距离
    double safe_distance;
    // 随后计算前方车刹车时，不会发生碰撞所需距离
    double decceleration_safe_distance = 10.0 + subvehicle_velocity * subvehicle_velocity / (2.0 * MAX_DECCELERATION) - obstacle_velocity * obstacle_velocity / (2.0 * MAX_DECCELERATION);
    // 需要与前车保持的安全距离
    double keep_distance = 10.0 + 3.6 * subvehicle_velocity + pow(subvehicle_velocity - obstacle_velocity, 2.0) * (Tools::isLarge(subvehicle_velocity, obstacle_velocity) ? 1.0 : -1.0) + 12.0 * std::min(0.0, subvehicle_velocity - obstacle_velocity);
    double expand_safe_distance = (10.0 + 0.5 * subvehicle_velocity + pow(subvehicle_velocity - obstacle_velocity, 2.0) *(Tools::isLarge(subvehicle_velocity, obstacle_velocity) ? 1.0 : -1.0)) * 1.5;
    // 得到安全距离
    safe_distance = std::max(std::max(std::max(keep_distance, decceleration_safe_distance), expand_safe_distance), 15.0 + 0.45 * subvehicle_velocity);
    return safe_distance;
}

// 计算超越安全距离
double Tools::getOvertakeSafeDistance(double subvehicle_velocity, double obstacle_velocity) {
    // 在本车进行等待时，判断所期望的安全距离
    double safe_distance;
    // 随后计算本车车刹车时，不会发生碰撞所需距离
    double decceleration_safe_distance = 10.0 + obstacle_velocity * obstacle_velocity / (2.0 * MAX_DECCELERATION) -  subvehicle_velocity * subvehicle_velocity / (2.0 * MAX_DECCELERATION);
    // 需要与后车保持的安全距离
    double keep_distance = 10.0 + 0.5 * obstacle_velocity + pow(std::max(obstacle_velocity - subvehicle_velocity, 0.0), 2.0);
    // 得到安全距离
    safe_distance = std::max(std::max(keep_distance, decceleration_safe_distance), 10.0 + 0.5 * subvehicle_velocity);
    return safe_distance;
}

// 计算超越期望距离
double Tools::getOvertakeExpectedDistance(double subvehicle_velocity, double obstacle_velocity) {
    // 在本车进行等待时，判断所期望的安全距离
    double safe_distance;
    // 随后计算本车车刹车时，不会发生碰撞所需距离
    double decceleration_safe_distance = 15.0 + obstacle_velocity * obstacle_velocity / (2.0 * MAX_DECCELERATION) -  subvehicle_velocity * subvehicle_velocity / (2.0 * MAX_DECCELERATION);
    // 需要与后车保持的安全距离
    double keep_distance = 15.0 + 0.75 * obstacle_velocity + 1.5 * pow(std::max(obstacle_velocity - subvehicle_velocity, 0.0), 2.0);
    // 得到安全距离
    safe_distance = std::max(std::max(keep_distance, decceleration_safe_distance), 15.0 + 0.75 * subvehicle_velocity);
    return safe_distance;
}




// 一维数组进行kmeans聚类
std::vector<double> Tools::kMeans(const std::vector<double> &data, size_t k, std::vector<std::vector<size_t>> &clusters) {
    size_t data_number = data.size();
    // 求解k个聚类中心的值
    std::vector<double> centers;
    centers.resize(k);
    // 判断进行聚类的数量
    if (data_number <= k) {
        // 不进行聚类
        std::vector<std::vector<double>> groups;
        std::vector<std::vector<size_t>> clusters_candidate;
        groups.resize(k);
        clusters_candidate.resize(k);
        for (size_t i = 0; i < data_number; i++) {
            clusters_candidate[0].push_back(i);
            groups[0].push_back(data[i]);
        }
        // 计算每一组的中心
        std::vector<double> new_centers(k, 0.0);
        for (size_t i = 0; i < k; i++) {
            if (groups[i].size() > 0) {
                new_centers[i] = std::accumulate(std::begin(groups[i]), std::end(groups[i]), 0.0) / static_cast<double>(groups[i].size());
            }
        }
        centers = new_centers;
        clusters = clusters_candidate;
        return centers;
    }
    // 给出初始值
    std::vector<double> sort_data = data;
    sort(sort_data.begin(), sort_data.end());
    for (size_t i = 0; i < k; i ++) {
        centers[i] = sort_data[i * floor(static_cast<double>(sort_data.size() - 1) / static_cast<double>(k))];
    }

    // 计算离群点
    std::vector<size_t> seperate_points;
    // 求解均值
    double mean_value = std::accumulate(std::begin(data), std::end(data), 0.0);
    // 求解标准差
    double accum = 0;
    std::for_each (std::begin(data), std::end(data), [&](const double d) {
        accum  += (d - mean_value)*(d - mean_value);
    });
    double stdev = sqrt(accum / (data.size()-1));

    for (size_t i = 0; i < data.size(); i++) {
        if (Tools::isLarge(data[i], accum + 2 * stdev)) {
            seperate_points.push_back(i);
        }
    }

    // 开始循环迭代
    while (true) {
        // 首先对数据进行分组
        std::vector<std::vector<double>> groups;
        std::vector<std::vector<size_t>> clusters_candidate;
        groups.resize(k);
        clusters_candidate.resize(k);
        for (size_t i = 0; i < data.size(); i++) {
            // 判断是否是离群点
            if (std::find(seperate_points.begin(), seperate_points.end(), i) == seperate_points.end()) {
                double min_distances = 100000.0;
                size_t min_index = 0;
                for (size_t j = 0; j < k; j++) {
                    if (Tools::isSmall(fabs(centers[j] - data[i]), min_distances)) {
                        min_distances = fabs(centers[j] - data[i]);
                        min_index = j;
                    }
                }
                groups[min_index].push_back(data[i]);
                clusters_candidate[min_index].push_back(i);
            }
       
        }

        // 计算每一组的中心
        std::vector<double> new_centers;
        new_centers.resize(k);
        for (size_t i = 0; i < k; i++) {
            new_centers[i] = std::accumulate(std::begin(groups[i]), std::end(groups[i]), 0.0) / static_cast<double>(groups[i].size());
        }

        // 比较中心是否出现更新
        double update = 0.0;
        for (size_t i = 0; i < k; i++) {
            update += fabs(new_centers[i] - centers[i]);
        }

        if (Tools::isLarge(update, 10e-4)) {
            // 出现更新
            centers = new_centers;
            sort(centers.begin(), centers.end());
        } else {
            // 没有更新
            // 将离群点加入到最差的集合中
            for (auto seperate_point: seperate_points) {
                clusters_candidate[clusters_candidate.size() - 1].push_back(seperate_point);
            }
            clusters = clusters_candidate;
            break;
        }
    }
    return centers;
}

// std::string Tools::returnCurrentTimeAndDate() {
//     std::time_t t = std::time(0);
//     std::stringstream ss;
//     ss << t;
//     return ss.str();
// };

//返回当前时刻
std::string Tools::returnCurrentTimeAndDate() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d%X");
    return ss.str();
}

// 给出几何朝向, 几何中心的曲率, 几何中心到后轴中心的距离, 计算后轴中心朝向
double Tools::centerYawToRearYaw(double center_yaw, double center_curvature, double rear_to_center_distance) {
    double rear_yaw = center_yaw - asin(center_curvature * rear_to_center_distance);
    return rear_yaw;
};

// 给出后轴中心朝向, 给出后轴中心曲率, 计算几何中心的朝向
double Tools::rearYawToCenterYaw(double rear_yaw, double rear_curvature, double rear_to_center_distance) {
    double center_yaw = rear_yaw + atan(rear_curvature * rear_to_center_distance);
    return center_yaw;
}

// 给出几何中心曲率, 几何中心到后轴中心的距离, 计算后轴中心曲率
double Tools::centerToRearCurvature(double center_curvature, double rear_to_center_distance) {
    return center_curvature / sqrt(1 - pow(center_curvature * rear_to_center_distance, 2));
}

// 给出后轴中心曲率, 后轴中心到后轴中心的距离, 计算几何中心曲率
double Tools::rearToCenterCurvature(double rear_curvature, double rear_to_center_distance) {
    return rear_curvature / sqrt(1 + pow(rear_curvature * rear_to_center_distance, 2));
}

// 中值滤波, window_size为奇数
void Tools::medianFilter(std::vector<double> &data, int window_size) {
    int half_size = window_size / 2;
    for (int i = 0; i < static_cast<int>(data.size()); i++) {
        // 找出对应点的窗
        std::vector<double> window_data;
        for (int j = i - half_size; j < i + half_size; j++) {
            window_data.push_back(data[std::min(static_cast<int>(data.size()) - 1, std::max(j, 0))]);
        }
        // 寻找窗的中值
        const auto median_it = window_data.begin() + window_data.size() / 2;
        std::nth_element(window_data.begin(), median_it , window_data.end());
        auto median = *median_it;
        data[i] = median;
    }
}

// 得到路径最大曲率变化率
double Tools::getCurveMaxCurvatureChangeRate(const PathPlanningUtilities::Curve &curve) {
    // 计算曲率变化率
    std::vector<double> curvature_change_rates;
    for (size_t i = 0; i < curve.size() - 1; i++) {
        double distance = PathPlanningUtilities::calcDistance(curve[i].position_, curve[i + 1]. position_);
        double curvature_change_rate = (curve[i + 1].kappa_ - curve[i].kappa_) / distance;
        curvature_change_rates.push_back(curvature_change_rate);
    }
    // 进行中值滤波
    medianFilter(curvature_change_rates, 5);
    // 计算最大值
    double max_curvature_change_rate = 0.0;
    for (auto curvature_change_rate: curvature_change_rates) {
        if (Tools::isLarge(std::abs(curvature_change_rate), std::abs(max_curvature_change_rate))) {
            max_curvature_change_rate = curvature_change_rate;
        }
    }
    return max_curvature_change_rate;
}

// 角度变换，确保角度不超出范围
double Tools::angleSafeConversion(double raw_angle) {
    double changed_angle = raw_angle;
    while (changed_angle > PI) {
        changed_angle -= 2.0 * PI;
    }
    while (changed_angle <= -2.0 * PI) {
        changed_angle += 2.0 * PI;
    }
    return changed_angle;
}

// 在两点之间进行补点(构造障碍物边界)
std::vector<PathPlanningUtilities::Point2f> Tools::constructObstacleBoundary(const
PathPlanningUtilities::Point2f &front_point, const PathPlanningUtilities::Point2f &rear_point, double resolution) {
    std::vector<PathPlanningUtilities::Point2f> obstacle_boundary_line;
    obstacle_boundary_line.emplace_back(front_point);
    obstacle_boundary_line.emplace_back(rear_point);
    bool interpolation_finished = false;
    while (!interpolation_finished) {
        interpolation_finished = true;
        for (size_t i = 1; i < obstacle_boundary_line.size(); i++) {
            // 计算两点之间的距离
            double distance = PathPlanningUtilities::calcDistance(obstacle_boundary_line[i - 1], obstacle_boundary_line[i]);
            if (Tools::isLarge(distance, 2.0 * resolution)) {
                // 在两点之间插入一个点
                PathPlanningUtilities::Point2f new_point;
                new_point.x_ = (obstacle_boundary_line[i - 1].x_ + obstacle_boundary_line[i].x_) * 0.5;
                new_point.y_ = (obstacle_boundary_line[i - 1].y_ + obstacle_boundary_line[i].y_) * 0.5;
                obstacle_boundary_line.insert(obstacle_boundary_line.begin() + i, new_point);
                interpolation_finished = false;
                break;
            }
        }
    }
    return obstacle_boundary_line;
}

// 计算acos
double Tools::safeAcos (double x) {
    if (Tools::isSmall(x, -0.999)) {
        x = -0.999;
    } else if (Tools::isLarge(x, 0.999)) {
        x = 0.999;
    }
    return acos(x) ;
}

// 计算两个点之间的距离
double Tools::calcDistanceBetweenPoint2f(const PathPlanningUtilities::Point2f &point_1, const PathPlanningUtilities::Point2f &point_2) {
    double edge_length_1 = std::fabs(point_1.x_ - point_2.x_);
    double edge_length_2 = std::fabs(point_1.y_ - point_2.y_);
    double length = sqrt(edge_length_1 * edge_length_1 + edge_length_2 * edge_length_2);
    return length;
}

// calculate theta information from scatter point
double Tools::calcThetaInfo(const PathPlanningUtilities::Point2f &cur_point, const PathPlanningUtilities::Point2f &pre_point) {
    double diff_y = cur_point.y_ - pre_point.y_;
    double diff_x = cur_point.x_ - pre_point.x_;
    return atan2(diff_y, diff_x);
}

// calculate curvature information from scatter points
double Tools::calcCurvatureInfo(const PathPlanningUtilities::Point2f &pre_point, const PathPlanningUtilities::Point2f &cur_point, const PathPlanningUtilities::Point2f &next_point) {
    double pre_point_x = pre_point.x_, pre_point_y = pre_point.y_;
    double cur_point_x = cur_point.x_, cur_point_y = cur_point.y_;
    double next_point_x = next_point.x_, next_point_y = next_point.y_;
    double a_1 = (next_point_x - pre_point_x) / 2.0;
    double b_1 = (next_point_y - pre_point_y) / 2.0;
    double a_2 = (next_point_x + pre_point_x) / 2.0 - cur_point_x;
    double b_2 = (next_point_y + pre_point_y) / 2.0 - cur_point_y;
    double k = (2.0 * (a_1 * b_2 - a_2 * b_1)) / (pow(pow(a_1, 2.0) + pow(b_1, 2), 1.5));
    return k;
}

// delete path points for hybrid astar test
PathPlanningUtilities::Path Tools::deletePathPoint(const PathPlanningUtilities::Path &initial_path, double index_gap) {
    PathPlanningUtilities::Path processed_path;
    for (size_t i = 0; i < initial_path.size(); i += index_gap) {
        processed_path.emplace_back(initial_path[i]);
    }
    return processed_path;
}

