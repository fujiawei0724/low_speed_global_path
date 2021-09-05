/*
    Copyright [2020] Jian ZhiQiang
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

// 结构化道路引导
namespace GlobalPlanningFrontend {

// structure node in search graph
class graphNode {
 public:
    // constructor
    graphNode(PathPlanningUtilities::Point2f path_point, int pre_index = -1) {
        this->pre_index_ = pre_index;
        this->path_point_ = path_point;
    };
    // destructor
    ~graphNode() {

    };

    // set previous index information
    void setPreIndex(int pre_index) {
        this->pre_index_ = pre_index;
    }
    // get previous index information
    int getPreIndex() const {
        return this->pre_index_;
    }
    // get path point
    PathPlanningUtilities::Point2f getPathPoint() const {
        return this->path_point_;
    }

    double curvature_;
    double curvature_change_rate_;
    int pre_index_;
    PathPlanningUtilities::Point2f path_point_;
};

// 根据结构化道路中心线生成蔽障路径
class StructureGuidance {
 public:
    // 构造函数
    StructureGuidance() {
        this->longitudinal_gap_ = 1.0; // 无效值
        this->lateral_gap_ = 0.2; 
        this->lateral_max_offset_ = 2.0;
        this->max_distance_to_obs_ = Config::global_planning_max_distance_to_obs_;
    };

    // 析构函数
    ~StructureGuidance() {

    };
    
    // structure guide planning using data structure graph
    int expeditiousSearching(const std::vector<PathPlanningUtilities::CoordinationPoint> &reference_line, const KDTree &kdtree, const PathPlanningUtilities::Point2f &start_point, const std::shared_ptr<KDTree> &history_reference_line_kdtree_ptr, PathPlanningUtilities::Path &initial_path) {
        // generate sampled points and graph node
        // 首先确定纵向规划距离
        std::vector<std::vector<graphNode>> points_table;
        int remain_reference_path_length = reference_line.size();
        int valid_global_length = remain_reference_path_length;
        LOG(INFO) << "结构化道路剩余规划路径长度为" << valid_global_length; 
        // std::cout << "结构化道路剩余规划路径长度为" << valid_global_length << std::endl;
        if (remain_reference_path_length > 1501) {
            valid_global_length = 1500;
        }
        // 然后确定横向采样距离，简化计算认为规划道路横向宽度不改变
        double lateral_sampled_distance = std::max(reference_line[0].max_height_, -reference_line[0].min_height_);
        // 计算单侧采样点数量
        int half_sample_points_num = std::ceil(lateral_sampled_distance / 0.6);
        // LOG(INFO) << "单侧横向采样点数量为" << half_sample_points_num;
        // std::cout << "单侧横向采样点数量为" << half_sample_points_num << std::endl;
        for (int i = 0; i <= valid_global_length; i += 60) { 
            PathPlanningUtilities::CoordinationPoint reference_point = reference_line[i];
            std::vector<graphNode> point_column;
            for (int j = -half_sample_points_num; j <= half_sample_points_num; j++) {
                double d = j * 0.6;
                PathPlanningUtilities::Point2f sample_point;
                sample_point.x_ = reference_point.worldpos_.position_.x_ + d * cos(reference_point.worldpos_.theta_ + PI * 0.5);
                sample_point.y_ = reference_point.worldpos_.position_.y_ + d * sin(reference_point.worldpos_.theta_ + PI * 0.5);
                graphNode graph_node = graphNode(sample_point);
                point_column.push_back(graph_node);
            }
            points_table.push_back(point_column);
        }
        // LOG(INFO) << "纵向采样点数量为" << points_table.size();
        // std::cout << "纵向采样点数量为" << points_table.size() << std::endl;
        // initiate dp table
        // clock_t construct_dp_start_time = clock();
        std::vector<std::vector<double>> dynamic_programming_table(points_table.size(), std::vector<double>(points_table[0].size(), 0.0));
        // get the nearest point to the robot, and add initial point to the path
        int index = half_sample_points_num;
        this->half_sample_points_num_ = half_sample_points_num;
        double min_distance = lateral_sampled_distance;
        for (size_t i = 0; i < points_table[0].size(); i++) {
            if (sqrt((start_point.x_ - points_table[0][i].getPathPoint().x_) * (start_point.x_ - points_table[0][i].getPathPoint().x_) + (start_point.y_ - points_table[0][i].getPathPoint().y_) * (start_point.y_ - points_table[0][i].getPathPoint().y_)) < min_distance) {
                min_distance = sqrt((start_point.x_ - points_table[0][i].getPathPoint().x_) * (start_point.x_ - points_table[0][i].getPathPoint().x_) + (start_point.y_ - points_table[0][i].getPathPoint().y_) * (start_point.y_ - points_table[0][i].getPathPoint().y_));
                index = i;
            }
        }
        // set information for initial point
        points_table[0][index].curvature_ = 0.0;
        points_table[0][index].curvature_change_rate_ = 0.0;
        // generate dp table and add the previous index information
        for (size_t i = 1; i < dynamic_programming_table.size(); i++) {
            if (i == 1) {
                for (size_t j = 0; j < dynamic_programming_table[0].size(); j++) {
                    dynamic_programming_table[i][j] = this->calcEdgeCost(0, index, i, j, points_table, kdtree, history_reference_line_kdtree_ptr);
                    points_table[i][j].setPreIndex(index);
                } 
            } else {
                for (size_t j = 0; j < dynamic_programming_table[0].size(); j++) {
                    // calculate cost
                    std::vector<double> costs(dynamic_programming_table[0].size(), 0.0);
                    for (size_t k = 0; k < dynamic_programming_table[0].size(); k++) {
                        if (dynamic_programming_table[i - 1][k] >= 100000.0) {
                            costs[k] = 100000.0;
                        } else {
                            costs[k] = dynamic_programming_table[i - 1][k] + this->calcEdgeCost(i - 1, k, i, j, points_table, kdtree, history_reference_line_kdtree_ptr);
                        }
                    }

                    // find min index and recod information
                    auto min_index = std::min_element(costs.begin(), costs.end()) - costs.begin();
                    double min_cost = costs[min_index];
                    dynamic_programming_table[i][j] = min_cost;
                    points_table[i][j].setPreIndex(min_index);

                }
            }
        }
        // clock_t construct_dp_end_time = clock();
        // std::cout << "construct dp time consume is: " << static_cast<double>(construct_dp_end_time - construct_dp_start_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;
        // generate final path
        PathPlanningUtilities::Path tmp_path;
        // add the last point
        auto min_iter = std::min_element(dynamic_programming_table.back().begin(), dynamic_programming_table.back().end());
        int optimal_point_index = std::distance(dynamic_programming_table.back().begin(), min_iter);
        // find the min cost path
        int column_num = points_table.size() - 1;
        while (optimal_point_index != -1) {
            tmp_path.push_back(points_table[column_num][optimal_point_index].getPathPoint());
            optimal_point_index = points_table[column_num][optimal_point_index].getPreIndex();
            column_num -= 1;
        }
        PathPlanningUtilities::Path raw_path = tmp_path;
        reverse(raw_path.begin(), raw_path.end());
        // // add remain reference line
        // if (remain_reference_path_length > 401) {
        //     PathPlanningUtilities::Path remain_reference_path;
        //     for (size_t i = 401; i < reference_line.size(); i++) {
        //         remain_reference_path.push_back(reference_line[i].worldpos_.position_);
        //     }
        //     raw_path.insert(raw_path.end(), remain_reference_path.begin(), remain_reference_path.end());
        // } else {
        //     raw_path.push_back(reference_line[reference_line.size() - 1].worldpos_.position_);
        // }
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

    // DEBUG: Add visualization
    int expeditiousSearching(const std::vector<PathPlanningUtilities::CoordinationPoint> &reference_line, const KDTree &kdtree, const PathPlanningUtilities::Point2f &start_point, const std::shared_ptr<KDTree> &history_reference_line_kdtree_ptr, PathPlanningUtilities::Path &initial_path, const ros::Publisher &publisher) {
        // generate sampled points and graph node
        // 首先确定纵向规划距离
        std::vector<std::vector<graphNode>> points_table;
        int remain_reference_path_length = reference_line.size();
        int valid_global_length = remain_reference_path_length;
        LOG(INFO) << "结构化道路剩余规划路径长度为" << valid_global_length; 
        // std::cout << "结构化道路剩余规划路径长度为" << valid_global_length << std::endl;
        if (remain_reference_path_length > 1501) {
            valid_global_length = 1500;
        }
        // 然后确定横向采样距离，简化计算认为规划道路横向宽度不改变
        double lateral_sampled_distance = std::max(reference_line[0].max_height_, -reference_line[0].min_height_);
        // 计算单侧采样点数量
        int half_sample_points_num = std::ceil(lateral_sampled_distance / 0.6);
        // LOG(INFO) << "单侧横向采样点数量为" << half_sample_points_num;
        // std::cout << "单侧横向采样点数量为" << half_sample_points_num << std::endl;
        for (int i = 0; i <= valid_global_length; i += 60) { 
            PathPlanningUtilities::CoordinationPoint reference_point = reference_line[i];
            std::vector<graphNode> point_column;
            visualization_msgs::MarkerArray sampling_point_marker_array;
            for (int j = -half_sample_points_num; j <= half_sample_points_num; j++) {
                double d = j * 0.6;
                PathPlanningUtilities::Point2f sample_point;
                sample_point.x_ = reference_point.worldpos_.position_.x_ + d * cos(reference_point.worldpos_.theta_ + PI * 0.5);
                sample_point.y_ = reference_point.worldpos_.position_.y_ + d * sin(reference_point.worldpos_.theta_ + PI * 0.5);
                graphNode graph_node = graphNode(sample_point);
                point_column.push_back(graph_node);
                sampling_point_marker_array.markers.push_back(VisualizationMethods::visualizeSphere(sample_point.x_, sample_point.y_, 0.1, VisualizationMethods::color(255.0/255.0, 0.0/255.0, 255.0/255.0, 1), 5000000 + i * 10 + j));
            }
            publisher.publish(sampling_point_marker_array);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            points_table.push_back(point_column);
        }
        // LOG(INFO) << "纵向采样点数量为" << points_table.size();
        // std::cout << "纵向采样点数量为" << points_table.size() << std::endl;
        // initiate dp table
        // clock_t construct_dp_start_time = clock();
        std::vector<std::vector<double>> dynamic_programming_table(points_table.size(), std::vector<double>(points_table[0].size(), 0.0));
        // get the nearest point to the robot, and add initial point to the path
        int index = half_sample_points_num;
        this->half_sample_points_num_ = half_sample_points_num;
        double min_distance = lateral_sampled_distance;
        for (size_t i = 0; i < points_table[0].size(); i++) {
            if (sqrt((start_point.x_ - points_table[0][i].getPathPoint().x_) * (start_point.x_ - points_table[0][i].getPathPoint().x_) + (start_point.y_ - points_table[0][i].getPathPoint().y_) * (start_point.y_ - points_table[0][i].getPathPoint().y_)) < min_distance) {
                min_distance = sqrt((start_point.x_ - points_table[0][i].getPathPoint().x_) * (start_point.x_ - points_table[0][i].getPathPoint().x_) + (start_point.y_ - points_table[0][i].getPathPoint().y_) * (start_point.y_ - points_table[0][i].getPathPoint().y_));
                index = i;
            }
        }
        // set information for initial point
        points_table[0][index].curvature_ = 0.0;
        points_table[0][index].curvature_change_rate_ = 0.0;
        // generate dp table and add the previous index information
        for (size_t i = 1; i < dynamic_programming_table.size(); i++) {
            if (i == 1) {
                for (size_t j = 0; j < dynamic_programming_table[0].size(); j++) {
                    dynamic_programming_table[i][j] = this->calcEdgeCost(0, index, i, j, points_table, kdtree, history_reference_line_kdtree_ptr);
                    points_table[i][j].setPreIndex(index);
                } 
            } else {
                for (size_t j = 0; j < dynamic_programming_table[0].size(); j++) {
                    // calculate cost
                    std::vector<double> costs(dynamic_programming_table[0].size(), 0.0);
                    for (size_t k = 0; k < dynamic_programming_table[0].size(); k++) {
                        if (dynamic_programming_table[i - 1][k] >= 100000.0) {
                            costs[k] = 100000.0;
                        } else {
                            costs[k] = dynamic_programming_table[i - 1][k] + this->calcEdgeCost(i - 1, k, i, j, points_table, kdtree, history_reference_line_kdtree_ptr);
                        }
                    }

                    // find min index and recod information
                    auto min_index = std::min_element(costs.begin(), costs.end()) - costs.begin();
                    double min_cost = costs[min_index];
                    dynamic_programming_table[i][j] = min_cost;
                    points_table[i][j].setPreIndex(min_index);

                }
            }
        }
        // clock_t construct_dp_end_time = clock();
        // std::cout << "construct dp time consume is: " << static_cast<double>(construct_dp_end_time - construct_dp_start_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;
        // generate final path
        PathPlanningUtilities::Path tmp_path;
        // add the last point
        auto min_iter = std::min_element(dynamic_programming_table.back().begin(), dynamic_programming_table.back().end());
        int optimal_point_index = std::distance(dynamic_programming_table.back().begin(), min_iter);
        // find the min cost path
        int column_num = points_table.size() - 1;
        while (optimal_point_index != -1) {
            tmp_path.push_back(points_table[column_num][optimal_point_index].getPathPoint());
            optimal_point_index = points_table[column_num][optimal_point_index].getPreIndex();
            column_num -= 1;
        }
        PathPlanningUtilities::Path raw_path = tmp_path;
        reverse(raw_path.begin(), raw_path.end());
        // // add remain reference line
        // if (remain_reference_path_length > 401) {
        //     PathPlanningUtilities::Path remain_reference_path;
        //     for (size_t i = 401; i < reference_line.size(); i++) {
        //         remain_reference_path.push_back(reference_line[i].worldpos_.position_);
        //     }
        //     raw_path.insert(raw_path.end(), remain_reference_path.begin(), remain_reference_path.end());
        // } else {
        //     raw_path.push_back(reference_line[reference_line.size() - 1].worldpos_.position_);
        // }
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




    // calculate point to point cost
    double calcEdgeCost(int x_1, int y_1, int x_2, int y_2, std::vector<std::vector<graphNode>> &points_table, const KDTree &kdtree, const std::shared_ptr<KDTree> &history_reference_line_kdtree_ptr) {
        double half_sample_points_num = static_cast<double>(this->half_sample_points_num_);
        double w_d = 0.85;
        double long_edges_cost = fabs(y_1 - y_2) / (2.0 * half_sample_points_num);
        double lateral_offset_cost = (fabs(y_1 - half_sample_points_num) / half_sample_points_num + fabs(y_2 - half_sample_points_num) / half_sample_points_num) / 2.0;
        // 加入一致性损失
        double consistency_cost = 0.0;
        if (history_reference_line_kdtree_ptr != nullptr) {
            // get the point is going to be calculated
            PathPlanningUtilities::Point2f calculate_point = points_table[x_2][y_2].getPathPoint();
            std::vector<std::pair<float, float>> neighbors;
            std::vector<float> sq_distances;
            history_reference_line_kdtree_ptr->findKNeighbor(calculate_point.x_, calculate_point.y_, &neighbors, &sq_distances, 1);
            consistency_cost = sqrt(sq_distances[0]);
        }
        // if (consistency_cost != 0.0) {
        //     std::cout << "The consistency cost is " << consistency_cost << std::endl;
        // }
        // 加入转角损失
        double twist_cost = 0.0;
        if (x_2 >= 2) {
            int pre_two_index = points_table[x_1][y_1].getPreIndex();
            if ((y_2 - y_1) * (y_1 - pre_two_index) < 0) {
                twist_cost = fabs((y_2 - y_1) * (y_1 - pre_two_index)) / (2.0 * half_sample_points_num);
            }
        }
        
        double obstacle_cost = this->calcObstacleCost(points_table[x_1][y_1].getPathPoint(), points_table[x_2][y_2].getPathPoint(), kdtree);
        points_table[x_2][y_2].curvature_ = fabs(y_1 - y_2) / (2.0 * half_sample_points_num);
        points_table[x_2][y_2].curvature_change_rate_ = fabs(points_table[x_2][y_2].curvature_ - points_table[x_1][y_1].curvature_) / fabs(y_2 - y_1);
        double curvature_change_cost = points_table[x_2][y_2].curvature_change_rate_;
        
        
        double cost = long_edges_cost * w_d + lateral_offset_cost * (1.0 - w_d) + consistency_cost * 0.1 + obstacle_cost + twist_cost * 1.0;
        
        
        return cost;
    }
    
    // calculate obstacle cost
    double calcObstacleCost(const PathPlanningUtilities::Point2f &point_1, const PathPlanningUtilities::Point2f &point_2, const KDTree &kdtree) {
        double cost = 0.0;
        // generate straight line
        PathPlanningUtilities::Path straight_line;
        straight_line.push_back(point_1);
        straight_line.push_back(point_2);
        for (size_t i = 1; i < 10; i++) {
            PathPlanningUtilities::Point2f sample_point;
            sample_point.x_ = point_1.x_ * (i / 10.0) + point_2.x_ * ((10 - i) / 10.0);
            sample_point.y_ = point_1.y_ * (i / 10.0) + point_2.y_ * ((10 - i) / 10.0);
            straight_line.push_back(sample_point);
        }
        double theta = atan2(point_2.y_ - point_1.y_, point_2.x_ - point_1.x_);
        double vehicle_length = 4.9;
        double d = vehicle_length / 2.0;
        double r = 1.3;
        // 得到disc参数后,判断机器人与障碍物的最小距离
        double min_distance = this->max_distance_to_obs_;
        size_t index_gap = straight_line.size() / 3;
        for (size_t i = index_gap; i <= index_gap * 2; i += index_gap) {
            // 计算disc中心
            PathPlanningUtilities::Point2f center_disc = straight_line[i];
            PathPlanningUtilities::Point2f front_disc;
            front_disc.x_ = straight_line[i].x_ + d * cos(theta);
            front_disc.y_ = straight_line[i].y_ + d * sin(theta);
            PathPlanningUtilities::Point2f front_disc_1;
            front_disc_1.x_ = straight_line[i].x_ + 0.333 * d * cos(theta);
            front_disc_1.y_ = straight_line[i].y_ + 0.333 * d * sin(theta);
            PathPlanningUtilities::Point2f front_disc_2;
            front_disc_2.x_ = straight_line[i].x_ + 0.666 * d * cos(theta);
            front_disc_2.y_ = straight_line[i].y_ + 0.666 * d * sin(theta);
            // PathPlanningUtilities::Point2f front_disc_3;
            // front_disc_3.x_ = straight_line[i].x_ + 0.6 * d * cos(theta);
            // front_disc_3.y_ = straight_line[i].y_ + 0.6 * d * sin(theta);
            // PathPlanningUtilities::Point2f front_disc_4;
            // front_disc_4.x_ = straight_line[i].x_ + 0.8 * d * cos(theta);
            // front_disc_4.y_ = straight_line[i].y_ + 0.8 * d * sin(theta);
            PathPlanningUtilities::Point2f rear_disc;
            rear_disc.x_ = straight_line[i].x_ - d * cos(theta);
            rear_disc.y_ = straight_line[i].y_ - d * sin(theta);
            PathPlanningUtilities::Point2f rear_disc_1;
            rear_disc_1.x_ = straight_line[i].x_ - 0.333 * d * cos(theta);
            rear_disc_1.y_ = straight_line[i].y_ - 0.333 * d * sin(theta);
            PathPlanningUtilities::Point2f rear_disc_2;
            rear_disc_2.x_ = straight_line[i].x_ - 0.666 * d * cos(theta);
            rear_disc_2.y_ = straight_line[i].y_ - 0.666 * d * sin(theta);
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
            cost = std::max(0.0, 1.3 - min_distance);
        }
        return cost;
    }



    double longitudinal_gap_;  // 纵向采样间隔[米]
    double lateral_gap_;  // 横向采样间隔[米]
    double lateral_max_offset_;  // 最大横向采样偏移[米]
    double max_distance_to_obs_; //global planning max distance to obstacle
    double half_sample_points_num_; // 单侧采样点数量
};

};
