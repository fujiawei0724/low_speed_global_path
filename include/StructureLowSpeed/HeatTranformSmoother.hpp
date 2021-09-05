/*
 * @Author: fjw 
 * @Date: 2021-04-28 16:44:06 
 * @Last Modified by: fjw
 * @Last Modified time: 2021-04-28 16:59:18
 */

#pragma once

#include "StructureLowSpeed/Config.hpp"
#include "StructureLowSpeed/LocalPath.hpp"
#include "Tools.hpp"
#include "StructureLowSpeed/KDTree.hpp"

namespace GlobalPlanningBackend{

class HeatTransformSmoother {
public:
    HeatTransformSmoother(int iter_num = 60) {
        this->iter_num_ = iter_num;
    }

    PathPlanningUtilities::Path smoothing(const PathPlanningUtilities::Path &raw_path) {
        int iter = 0;
        int path_length = raw_path.size();
        PathPlanningUtilities::Path smoothed_path = raw_path;
        while (iter < this->iter_num_) {
            // Initiate direction
            Eigen::MatrixXd directions = Eigen::MatrixXd::Zero(path_length, 2);
            // Calculate direction for each point
            for (int i = 1; i < path_length - 1; i++) {
                // 首先从路径中取出3个点
                Eigen::MatrixXd point_m1 = this->pointToArray(smoothed_path[i - 1]);
                Eigen::MatrixXd point = this->pointToArray(smoothed_path[i]);
                Eigen::MatrixXd point_p1 = this->pointToArray(smoothed_path[i + 1]);
                // 计算曲率梯度
                Eigen::MatrixXd cur_gradient_item = this->calcSmoothItem(point_m1, point, point_p1);
                // 加入总梯度
                directions.row(i) += directions.row(0);
            }
            // Update path
            for (int i = 1; i < path_length - 1; i++) {
                smoothed_path[i].x_ -= directions.coeff(i, 0);
                smoothed_path[i].y_ -= directions.coeff(i, 1);
            }
            iter += 1;
        }
        return smoothed_path;
    }

private:
    // Calculate direction
    Eigen::MatrixXd calcSmoothItem(const Eigen::MatrixXd &point_m1, const Eigen::MatrixXd &point, const Eigen::MatrixXd &point_p1) {
        Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(1, 2);
        gradient.row(0) = 1.0 * (point_m1 + point_p1 - 2.0 * point);
        return gradient;
    }

    // 点转矩阵
    Eigen::MatrixXd pointToArray(const PathPlanningUtilities::Point2f &point) {
        Eigen::MatrixXd result(1, 2);
        result.row(0) << point.x_, point.y_;
        return result;
    }


    int iter_num_;
};
};