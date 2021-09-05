/*
 * @Author: fjw 
 * @Date: 2021-06-24 11:01:30 
 * @Last Modified by: fjw
 * @Last Modified time: 2021-06-24 11:17:55
 */

#pragma once



#include "StructureLowSpeed/Config.hpp"
#include "StructureLowSpeed/LocalPath.hpp"
#include "StructureLowSpeed/KDTree.hpp"
#include "Tools.hpp"
#include "Point.hpp"
#include "Path.hpp"
#include "PathGenerator.h"
#include "LineSegment.hpp"
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <typeinfo>
#include <thread>
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

namespace GlobalPlanningFrontend {

class ThetaStructureGuideNode {
public:
    ThetaStructureGuideNode(PathPlanningUtilities::Point2f point, double theta) {
        this->point_ = point;
        this->theta_ = theta;
        this->cost_ = 0.0;
        this->pre_index_ = -1;
    };
    ThetaStructureGuideNode() {

    };
    ~ThetaStructureGuideNode() {

    };

    int this_lateral_index_; // ego point laternal index
    int this_theta_index_; // ego point theta index
    int pre_index_; // pre point theta index
    double theta_; // actual theta 
    double cost_; // path cost
    PathPlanningUtilities::Point2f point_; // actual position
    PathPlanningUtilities::Path path_; // actual path
    std::vector<double> path_derivations_; // actual path derivations


};


class ThetaStructureGuidePlanner {
public:
    ThetaStructureGuidePlanner(double theta_gap = 15.0 / 180.0 * M_PI, double laternal_gap = 1.5, double longitudinal_gap = 5.0) {
        this->theta_gap_ = theta_gap;
        this->laternal_gap_ = laternal_gap;
        this->longitudinal_gap_ = longitudinal_gap;
        this->max_theta_ = 60.0 / 180.0 * M_PI;
        this->reference_line_gap_ = 0.1;
        this->max_distance_to_obstacle_ = 12.0;
    };
    ~ThetaStructureGuidePlanner() {

    };

    // structure guide planning with angle 
    int planning(const std::vector<PathPlanningUtilities::CoordinationPoint> &reference_line, const KDTree &kdtree, const PathPlanningUtilities::Point2f &start_point, double start_derivation, const std::shared_ptr<KDTree> &history_reference_line_kdtree_ptr, PathPlanningUtilities::Path &initial_path) {
        // 首先确定纵向规划距离
        int remain_reference_path_length = reference_line.size();
        int valid_global_length = remain_reference_path_length;
        LOG(INFO) << "结构化道路剩余规划路径长度为" << valid_global_length; 
        // std::cout << "结构化道路剩余规划路径长度为" << valid_global_length << std::endl;
        if (remain_reference_path_length > 601) {
            valid_global_length = 600;
        }

        // 然后确定横向采样距离，简化计算认为规划道路横向宽度不改变
        double lateral_sampled_distance = std::max(reference_line[0].max_height_, -reference_line[0].min_height_);
        // 计算单侧采样点数量
        int half_sample_points_num = std::ceil(lateral_sampled_distance / this->laternal_gap_);
        LOG(INFO) << "单侧横向采样点数量为" << half_sample_points_num;
        // std::cout << "单侧横向采样点数量为" << half_sample_points_num << std::endl;
        // int laternal_sample_points_num = 2 * half_sample_points_num + 1;

        // get goal point derivation
        double goal_derivation = reference_line.back().worldpos_.theta_;
        // calculate sample theta number for each point
        int half_sample_theta_num = std::round(this->max_theta_ / this->theta_gap_);
        int sample_theta_num = 2 * half_sample_theta_num + 1;

        // construct node search table
        std::vector<std::vector<ThetaStructureGuideNode>> search_table;
        for (int i = 0; i <= valid_global_length; i += this->longitudinal_gap_ / this->reference_line_gap_) { 
            PathPlanningUtilities::CoordinationPoint reference_point = reference_line[i];
            std::vector<ThetaStructureGuideNode> point_column;
            for (int j = -half_sample_points_num; j <= half_sample_points_num; j++) {
                double d = j * this->laternal_gap_;
                // generate laternal sample point
                PathPlanningUtilities::Point2f sample_point;
                sample_point.x_ = reference_point.worldpos_.position_.x_ + d * cos(reference_point.worldpos_.theta_ + M_PI * 0.5);
                sample_point.y_ = reference_point.worldpos_.position_.y_ + d * sin(reference_point.worldpos_.theta_ + M_PI * 0.5);
                // theta sample 
                for (int k = -half_sample_theta_num; k <= half_sample_theta_num; k++) {
                    double sample_point_derivation = goal_derivation + k * this->theta_gap_;
                    ThetaStructureGuideNode sample_node = ThetaStructureGuideNode(sample_point, sample_point_derivation);
                    point_column.emplace_back(sample_node);
                }
            }
            search_table.emplace_back(point_column);
        }

        // get the nearest point to the robot
        int laternal_index = half_sample_points_num * sample_theta_num;
        double min_distance = 2.0 * lateral_sampled_distance;
        for (size_t i = 0; i < search_table[0].size(); i += sample_theta_num) {
            if (sqrt((start_point.x_ - search_table[0][i].point_.x_) * (start_point.x_ - search_table[0][i].point_.x_) + (start_point.y_ - search_table[0][i].point_.y_) * (start_point.y_ - search_table[0][i].point_.y_)) < min_distance) {
                min_distance = sqrt((start_point.x_ - search_table[0][i].point_.x_) * (start_point.x_ - search_table[0][i].point_.x_) + (start_point.y_ - search_table[0][i].point_.y_) * (start_point.y_ - search_table[0][i].point_.y_));
                laternal_index = i;
            }
        }

        // consider theta
        int vehicle_initial_index = laternal_index + half_sample_theta_num;
        double min_theta_diff = 120.0 / 180.0 * M_PI;
        for (int j = laternal_index; j < laternal_index + sample_theta_num; j++) {
            if (fabs(start_derivation - search_table[0][j].theta_) < min_theta_diff) {
                min_theta_diff = fabs(start_derivation - search_table[0][j].theta_);
                vehicle_initial_index = j;
            }
        }

        // start search 
        for (size_t longitudinal_index = 1; longitudinal_index < search_table.size(); longitudinal_index++) {
            if (longitudinal_index == 1) {
                for (size_t j = 0; j < search_table[longitudinal_index].size(); j++) {
                    // generate path 
                    std::pair<PathPlanningUtilities::Path, std::vector<double>> path_info = this->generatePath(search_table[longitudinal_index - 1][vehicle_initial_index], search_table[longitudinal_index][j]);
                    search_table[longitudinal_index][j].path_ = path_info.first;
                    
                    // calculate cost
                    search_table[longitudinal_index][j].cost_ = this->calcEdgeCost(path_info.first, path_info.second, kdtree, history_reference_line_kdtree_ptr, longitudinal_index, j, sample_theta_num, half_sample_points_num);

                    // set pre index 
                    search_table[longitudinal_index][j].pre_index_ = vehicle_initial_index;
                }
            } else {
                for (size_t j = 0; j < search_table[longitudinal_index].size(); j++) {
                    // generate and store path, calculate cost
                    std::vector<PathPlanningUtilities::Path> paths;
                    std::vector<double> path_costs;
                    paths.resize(search_table[longitudinal_index - 1].size());
                    path_costs.resize(search_table[longitudinal_index - 1].size());
                    for (size_t k = 0; k < search_table[longitudinal_index - 1].size(); k++) {
                        // get path
                        std::pair<PathPlanningUtilities::Path, std::vector<double>> path_info = this->generatePath(search_table[longitudinal_index - 1][k], search_table[longitudinal_index][j]);

                        // get cost
                        double path_cost = search_table[longitudinal_index - 1][k].cost_ + this->calcEdgeCost(path_info.first, path_info.second, kdtree, history_reference_line_kdtree_ptr, longitudinal_index, j, sample_theta_num, half_sample_points_num);

                        // store
                        paths[k] = path_info.first;
                        path_costs[k] = path_cost;
                    }

                    // find min cost
                    auto min_index = std::min_element(path_costs.begin(), path_costs.end()) - path_costs.begin();
                    double min_cost = path_costs[min_index];
                    PathPlanningUtilities::Path min_cost_path = paths[min_index];

                    // set path, cost, pre index 
                    search_table[longitudinal_index][j].path_ = min_cost_path;
                    search_table[longitudinal_index][j].cost_ = min_cost;
                    search_table[longitudinal_index][j].pre_index_ = min_index;
                }
            }
        }

        // iterate costs
        std::vector<double> final_costs;
        final_costs.resize(search_table.back().size());
        for (size_t i = 0; i < search_table.back().size(); i++) {
            final_costs[i] = search_table.back()[i].cost_;
        }

        // get optimal cost index
        int optimal_index = std::min_element(final_costs.begin(), final_costs.end()) - final_costs.begin();

        // generate final path
        PathPlanningUtilities::Path raw_path;
        int longitudinal_index = search_table.size() - 1;
        while (optimal_index != -1) {
            PathPlanningUtilities::Path path_segment = search_table[longitudinal_index][optimal_index].path_;
            reverse(path_segment.begin(), path_segment.end());
            raw_path.insert(raw_path.end(), path_segment.begin(), path_segment.end());
            optimal_index = search_table[longitudinal_index][optimal_index].pre_index_;
            longitudinal_index -= 1;

        }
        reverse(raw_path.begin(), raw_path.end());
        assert(longitudinal_index == 0);

        // interpolation
        bool interpolation_finished = false;
        while (!interpolation_finished) {
            interpolation_finished = true;
            for (size_t i = 1; i < raw_path.size(); i++) {
                // 计算两点之间的距离
                double distance = PathPlanningUtilities::calcDistance(raw_path[i - 1], raw_path[i]);
                if (Tools::isLarge(distance, 0.1)) {
                    // 在两点之间插入一个点
                    PathPlanningUtilities::Point2f new_point;
                    new_point.x_ = (raw_path[i - 1].x_ + raw_path[i].x_) * 0.5;
                    new_point.y_ = (raw_path[i - 1].y_ + raw_path[i].y_) * 0.5;
                    raw_path.insert(raw_path.begin() + i, new_point);
                    interpolation_finished = false;
                    break;
                }
            }
        }


        initial_path = raw_path;
        return 1;

    }

private:

    // generate cubic path 
    std::pair<PathPlanningUtilities::Path, std::vector<double>> generatePath(const ThetaStructureGuideNode &pre_node, const ThetaStructureGuideNode &current_node) {
        // cubic spline generate path
        CubicSpline cubic_path = CubicSpline(pre_node.point_.x_, pre_node.point_.y_, pre_node.theta_, current_node.point_.x_, current_node.point_.y_, current_node.theta_);
        std::pair<PathPlanningUtilities::Path, std::vector<double>> path_information = cubic_path.getPath();
        return path_information;
    }

    // calculate edge cost
    double calcEdgeCost(const PathPlanningUtilities::Path &this_path, const std::vector<double> &path_derivations, const KDTree &kdtree, const std::shared_ptr<KDTree> &history_reference_line_kdtree_ptr, int longitudinal_index, int point_index, int theta_sample_num, int half_laternal_sample_num) {
        // distance cost
        double distance_cost = Tools::calcDistanceBetweenPoint2f(this_path.front(), this_path.back());
        
        // theta change cost 
        double theta_change_cost = fabs(path_derivations.front() - path_derivations.back());

        // obstacle cost
        double obstacle_cost = this->calcObstacleCost(this_path, path_derivations, kdtree);

        // distance to center line
        double center_distance_cost = fabs(half_laternal_sample_num - point_index / theta_sample_num);

        // consistence cost
        double consistence_cost = 0.0;
        if (history_reference_line_kdtree_ptr != nullptr && longitudinal_index <= 2) {
            // get the point calculated
            PathPlanningUtilities::Point2f calculate_point = this_path.back();
            std::vector<std::pair<float, float>> neighbors;
            std::vector<float> sq_distances;
            history_reference_line_kdtree_ptr->findKNeighbor(calculate_point.x_, calculate_point.y_, &neighbors, &sq_distances, 1);
            consistence_cost = sqrt(sq_distances[0]);
        }

        // TOFIX: define the detailed parameters
        return distance_cost + theta_change_cost + obstacle_cost + center_distance_cost + 0.1 * consistence_cost;
    } 

    // calculate obstacle cost
    double calcObstacleCost(const PathPlanningUtilities::Path &this_path, const std::vector<double> &path_derivations, const KDTree &kdtree) {
        double cost = 0.0;
        double vehicle_length = 4.9;
        double d = vehicle_length / 2.0;
        double r = 1.2;
        // 得到disc参数后,判断机器人与障碍物的最小距离
        double min_distance = this->max_distance_to_obstacle_;
        size_t index_gap = this_path.size() / 3;
        for (size_t i = index_gap; i <= index_gap * 2; i += index_gap) {
            // get derivation 
            double theta = path_derivations[i];
            // 计算disc中心
            PathPlanningUtilities::Point2f center_disc = this_path[i];
            PathPlanningUtilities::Point2f front_disc;
            front_disc.x_ = this_path[i].x_ + d * cos(theta);
            front_disc.y_ = this_path[i].y_ + d * sin(theta);
            PathPlanningUtilities::Point2f front_disc_1;
            front_disc_1.x_ = this_path[i].x_ + 0.333 * d * cos(theta);
            front_disc_1.y_ = this_path[i].y_ + 0.333 * d * sin(theta);
            PathPlanningUtilities::Point2f front_disc_2;
            front_disc_2.x_ = this_path[i].x_ + 0.666 * d * cos(theta);
            front_disc_2.y_ = this_path[i].y_ + 0.666 * d * sin(theta);
            // PathPlanningUtilities::Point2f front_disc_3;
            // front_disc_3.x_ = straight_line[i].x_ + 0.6 * d * cos(theta);
            // front_disc_3.y_ = straight_line[i].y_ + 0.6 * d * sin(theta);
            // PathPlanningUtilities::Point2f front_disc_4;
            // front_disc_4.x_ = straight_line[i].x_ + 0.8 * d * cos(theta);
            // front_disc_4.y_ = straight_line[i].y_ + 0.8 * d * sin(theta);
            PathPlanningUtilities::Point2f rear_disc;
            rear_disc.x_ = this_path[i].x_ - d * cos(theta);
            rear_disc.y_ = this_path[i].y_ - d * sin(theta);
            PathPlanningUtilities::Point2f rear_disc_1;
            rear_disc_1.x_ = this_path[i].x_ - 0.333 * d * cos(theta);
            rear_disc_1.y_ = this_path[i].y_ - 0.333 * d * sin(theta);
            PathPlanningUtilities::Point2f rear_disc_2;
            rear_disc_2.x_ = this_path[i].x_ - 0.666 * d * cos(theta);
            rear_disc_2.y_ = this_path[i].y_ - 0.666 * d * sin(theta);
            // PathPlanningUtilities::Point2f rear_disc_3;
            // rear_disc_3.x_ = straight_line[i].x_ - 0.6 * d * cos(theta);
            // rear_disc_3.y_ = straight_line[i].y_ - 0.6 * d * sin(theta);
            // PathPlanningUtilities::Point2f rear_disc_4;
            // rear_disc_4.x_ = straight_line[i].x_ - 0.8 * d * cos(theta);
            // rear_disc_4.y_ = straight_line[i].y_ - 0.8 * d * sin(theta);
            // 进行距离判断
            std::vector<std::pair<float, float>> results;
            std::vector<float> sq_distances;
            kdtree.findKNeighbor(center_disc.x_, center_disc.y_, &results, &sq_distances, 1);
            if (Tools::isSmall(std::max(0.0, sqrt(sq_distances[0]) - r), min_distance)) {
                min_distance = std::max(0.0, sqrt(sq_distances[0]) - r);
            }

            kdtree.findKNeighbor(front_disc.x_, front_disc.y_, &results, &sq_distances, 1);
            if (Tools::isSmall(std::max(0.0, sqrt(sq_distances[0]) - r), min_distance)) {
                min_distance = std::max(0.0, sqrt(sq_distances[0]) - r);
            }
            kdtree.findKNeighbor(front_disc_1.x_, front_disc_1.y_, &results, &sq_distances, 1);
            if (Tools::isSmall(std::max(0.0, sqrt(sq_distances[0]) - r), min_distance)) {
                min_distance = std::max(0.0, sqrt(sq_distances[0]) - r);
            }
            kdtree.findKNeighbor(front_disc_2.x_, front_disc_2.y_, &results, &sq_distances, 1);
            if (Tools::isSmall(std::max(0.0, sqrt(sq_distances[0]) - r), min_distance)) {
                min_distance = std::max(0.0, sqrt(sq_distances[0]) - r);
            }
            // kdtree.findKNeighbor(front_disc_3.x_, front_disc_3.y_, &results, &sq_distances, 1);
            // if (Tools::isSmall(std::max(0.0, sqrt(sq_distances[0]) - r), min_distance)) {
            //     min_distance = std::max(0.0, sqrt(sq_distances[0]) - r);
            // }
            // kdtree.findKNeighbor(front_disc_4.x_, front_disc_4.y_, &results, &sq_distances, 1);
            // if (Tools::isSmall(std::max(0.0, sqrt(sq_distances[0]) - r), min_distance)) {
            //     min_distance = std::max(0.0, sqrt(sq_distances[0]) - r);
            // }  

            kdtree.findKNeighbor(rear_disc.x_, rear_disc.y_, &results, &sq_distances, 1);
            if (Tools::isSmall(std::max(0.0, sqrt(sq_distances[0]) - r), min_distance)) {
                min_distance = std::max(0.0, sqrt(sq_distances[0]) - r);
            }
            kdtree.findKNeighbor(rear_disc_1.x_, rear_disc_1.y_, &results, &sq_distances, 1);
            if (Tools::isSmall(std::max(0.0, sqrt(sq_distances[0]) - r), min_distance)) {
                min_distance = std::max(0.0, sqrt(sq_distances[0]) - r);
            }
            kdtree.findKNeighbor(rear_disc_2.x_, rear_disc_2.y_, &results, &sq_distances, 1);
            if (Tools::isSmall(std::max(0.0, sqrt(sq_distances[0]) - r), min_distance)) {
                min_distance = std::max(0.0, sqrt(sq_distances[0]) - r);
            }
            // kdtree.findKNeighbor(rear_disc_3.x_, rear_disc_3.y_, &results, &sq_distances, 1);
            // if (Tools::isSmall(std::max(0.0, sqrt(sq_distances[0]) - r), min_distance)) {
            //     min_distance = std::max(0.0, sqrt(sq_distances[0]) - r);
            // }
            // kdtree.findKNeighbor(rear_disc_4.x_, rear_disc_4.y_, &results, &sq_distances, 1);
            // if (Tools::isSmall(std::max(0.0, sqrt(sq_distances[0]) - r), min_distance)) {
            //     min_distance = std::max(0.0, sqrt(sq_distances[0]) - r);
            // }

        }
        // DEBUG test obstacle cost function
        // cost = exp(-sqrt(1.0 / this->max_distance_to_obs_ * min_distance)) / (1.0 / this->max_distance_to_obs_ * min_distance);
        if (min_distance == 0.0) {
            cost = 100000.0;
        } else {
            cost = this->max_distance_to_obstacle_ - min_distance;
        }
        return cost;
    }


    double theta_gap_; // angle sample gap
    double laternal_gap_; // laternal position sample gap
    double longitudinal_gap_; // longitudinal position sample gap
    double max_theta_; // max angle derivation from goal angle
    double reference_line_gap_; // center line continuous points' distance
    double max_distance_to_obstacle_; // max distance to obstacle
};

};
