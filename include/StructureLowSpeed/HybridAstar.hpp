/*
 * @Author: fjw 
 * @Date: 2021-06-20 21:59:12 
 * @Last Modified by: fjw
 * @Last Modified time: 2021-06-22 22:28:42
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
#include <unordered_map>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <ctime>

namespace GlobalPlanningFrontend  {


class HybridAstarNode {
public:
    HybridAstarNode(int x_index, int y_index, int yaw_index, bool direction, PathPlanningUtilities::Path path, std::vector<double> yaw_list, std::vector<bool> directions, double steer, double cost, double heuristic, int pre_index) {
        this->x_index_ = x_index;
        this->y_index_ = y_index;
        this->yaw_index_ = yaw_index;
        this->direction_ = direction;
        this->path_ = path;
        this->yaw_list_ = yaw_list;
        this->directions_ = directions;
        this->steer_ = steer;
        this->cost_ = cost;
        this->heuristic_ = heuristic;
        this->pre_index_ = pre_index;
        this->is_open_ = false;
        this->is_closed_ = false;
    };
    HybridAstarNode() {
        this->is_open_ = false;
        this->is_closed_ = false;
    };
    ~HybridAstarNode() {

    };

    // 重载大小比较
    bool operator<(const HybridAstarNode &node) const {
        return Tools::isSmall(this->cost_ + this->heuristic_, node.cost_ + node.heuristic_);
    };

    bool operator>(const HybridAstarNode &node) const {
        return Tools::isLarge(this->cost_ + this->heuristic_, node.cost_ + node.heuristic_);
    }

    void open() {
        this->is_open_ = true;
        this->is_closed_ = false;
    }
    
    void close() {
        this->is_open_ = false;
        this->is_closed_ = true;
    }



    bool is_open_;
    bool is_closed_;
    int x_index_; // position x
    int y_index_; // position y
    int yaw_index_; 
    bool direction_; // true means forward, false means backward
    PathPlanningUtilities::Path path_; // path from previous point to this point
    std::vector<double> yaw_list_; // yaw positions from previous point to this point
    std::vector<bool> directions_; // directions from previous point to this point
    double steer_; // steer 
    double cost_; // point cost
    double heuristic_; // heuristic cost
    int pre_index_; // previous point index
};

class GridMapConfig {
public:
    GridMapConfig() {

    };
    ~GridMapConfig() {

    };

    constexpr static double reso_xy_ = 0.5;
    constexpr static double reso_yaw_ = 15.0 / 180.0 * M_PI;
    constexpr static int wx_ = 1000;
    constexpr static int wy_ = 400;
    constexpr static int wyaw_ = std::round(M_PI / reso_yaw_) - std::round(-M_PI / reso_yaw_) + 1;

};

class HybridAstarPlanner {
public:
    HybridAstarPlanner() {

    };
    ~HybridAstarPlanner() {

    };

    int planning(const PathPlanningUtilities::Point2f &start_point, double start_derivation, const PathPlanningUtilities::Point2f &goal_point, double goal_derivation, const KDTree &kd_tree, PathPlanningUtilities::Path &final_path) {
        // construct start point and end point
        PathPlanningUtilities::Point2f start_point_copy = start_point;
        PathPlanningUtilities::Point2f goal_point_copy = goal_point;
        PathPlanningUtilities::Path start_point_path = {start_point_copy};
        PathPlanningUtilities::Path goal_point_path = {goal_point_copy};
        std::vector<double> start_point_path_yaw = {start_derivation};
        std::vector<double> goal_point_path_yaw = {goal_derivation};
        std::vector<bool> start_point_path_direction = {true};
        std::vector<bool> goal_point_path_direction = {true};
        
        // config 11600 and 3000 mean the offset
        HybridAstarNode start_node = HybridAstarNode(std::round((start_point.x_ + 11600.0) / GridMapConfig::reso_xy_), std::round((start_point.y_ + 3000.0) / GridMapConfig::reso_xy_), std::round(start_derivation / GridMapConfig::reso_xy_), true, start_point_path, start_point_path_yaw, start_point_path_direction, 0.0, 0.0, 0.0, -1);
        
        HybridAstarNode goal_node = HybridAstarNode(std::round((goal_point.x_ + 11600.0) / GridMapConfig::reso_xy_), std::round((goal_point.y_ + 3000.0) / GridMapConfig::reso_xy_), std::round(goal_derivation / GridMapConfig::reso_yaw_), true, goal_point_path, goal_point_path_yaw, goal_point_path_direction, 0.0, DBL_MAX, 0.0, -1);

        // calculate distance between start node and goal node
        double start_goal_distance = Tools::calcDistanceBetweenPoint2f(start_point, goal_point);

        // construct open set and node recorder
        std::unordered_map<int, HybridAstarNode> node_recorder;
        std::priority_queue<HybridAstarNode, std::vector<HybridAstarNode>, std::greater<HybridAstarNode>> open_set;

        // calculate information for start node
        start_node.heuristic_ = this->calcHeuristic(start_node, goal_node);
        start_node.open();

        // add start node to search list
        node_recorder[this->calcIndex(start_node)] = start_node;
        open_set.emplace(start_node);

        // start search 
        while (!open_set.empty()) {
            // get current node and delete it from open set
            HybridAstarNode current_node = open_set.top();
            open_set.pop();

            // add it to close set
            int current_index = this->calcIndex(current_node);
            node_recorder[current_index].close();

            // calculate distance between current node and goal node 
            double current_goal_distance = Tools::calcDistanceBetweenPoint2f(current_node.path_.back(), goal_point);

            if (current_goal_distance / start_goal_distance <= 0.2) {
                // judge whether exist a path to goal point
                PathPlanningUtilities::Path path_to_goal;
                std::vector<double> yaw_list;
                if (this->existPathToGoal(current_node, goal_node, path_to_goal, yaw_list, kd_tree)) {
                    // exist a path to goal
                    goal_node.path_ = path_to_goal;
                    goal_node.yaw_list_ = yaw_list;
                    goal_node.pre_index_ = current_index;
                    break;
                }
            }

            // if there isn't a path to goal without collision, search neighbor node
            std::vector<HybridAstarNode> neighbor_nodes = this->getNeighbors(current_node, goal_node);
            // std::cout << "neighbor nodes size: " << neighbor_nodes.size() << std::endl;
            for (auto &neighbor_node: neighbor_nodes) {
                // calculate neighbor index 
                int neighbor_index = this->calcIndex(neighbor_node);
                // judge whether neighbor node in close set
                if (node_recorder[neighbor_index].is_closed_ == true) {
                    continue;
                }
                // if there are collisions
                if (this->judgeCollision(neighbor_node.path_, neighbor_node.yaw_list_, kd_tree)) {
                    continue;
                }
                // judge whether neighbor node in open set
                if (node_recorder[neighbor_index].is_open_) {
                    if (neighbor_node < node_recorder[neighbor_index]) {
                        neighbor_node.open();
                        node_recorder[neighbor_index] = neighbor_node;
                        open_set.emplace(neighbor_node);
                    }
                } else {
                    neighbor_node.open();
                    node_recorder[neighbor_index] = neighbor_node;
                    open_set.emplace(neighbor_node);
                }
            }

        }

        if (goal_node.pre_index_ == -1) {
            // if there is not a result
            return -1;
        }

        // generate final path
        PathPlanningUtilities::Path raw_path = this->findFinalPath(goal_node, node_recorder);

        // // interpolation
        // bool interpolation_finished = false;
        // while (!interpolation_finished) {
        //     interpolation_finished = true;
        //     for (size_t i = 1; i < raw_path.size(); i++) {
        //         // 计算两点之间的距离
        //         double distance = PathPlanningUtilities::calcDistance(raw_path[i - 1], raw_path[i]);
        //         if (Tools::isLarge(distance, 0.1)) {
        //             // 在两点之间插入一个点
        //             PathPlanningUtilities::Point2f new_point;
        //             new_point.x_ = (raw_path[i - 1].x_ + raw_path[i].x_) * 0.5;
        //             new_point.y_ = (raw_path[i - 1].y_ + raw_path[i].y_) * 0.5;
        //             raw_path.insert(raw_path.begin() + i, new_point);
        //             interpolation_finished = false;
        //             break;
        //         }
        //     }
        // }

        final_path = raw_path;
        return 1;
    }

private:

    // calculate index
    int calcIndex(const HybridAstarNode &node) {
        return node.x_index_ + node.y_index_ * GridMapConfig::wx_ + node.yaw_index_ * GridMapConfig::wx_ * GridMapConfig::wy_;
    }

    // calculate heuristic
    double calcHeuristic(const HybridAstarNode &current_node, const HybridAstarNode &goal_node, double weight = 1.0) {
        PathPlanningUtilities::Point2f current_point = current_node.path_.back();
        PathPlanningUtilities::Point2f goal_point = goal_node.path_.back();
        return sqrt(pow(current_point.x_ - goal_point.x_, 2.0) + pow(current_point.y_ - goal_point.y_, 2.0)) * weight;
    }

    // judge exist path to goal
    bool existPathToGoal(const HybridAstarNode &current_node, const HybridAstarNode &goal_node, PathPlanningUtilities::Path &path_to_goal, std::vector<double> &yaw_list, const KDTree &kd_tree) {
        // generate a cubic spline from current point to goal point
        CubicSpline cubic_spline = CubicSpline(current_node.path_.back().x_, current_node.path_.back().y_, current_node.yaw_list_.back(), goal_node.path_.back().x_, goal_node.path_.back().y_, goal_node.yaw_list_.back(), 0.1);
        std::pair<PathPlanningUtilities::Path, std::vector<double>> path_information = cubic_spline.getPath();
        PathPlanningUtilities::Path cubic_path = path_information.first;
        std::vector<double> path_yaw = path_information.second;
        cubic_path.erase(cubic_path.begin());
        path_yaw.erase(path_yaw.begin());
        
        // judge whether exist collision
        if (this->judgeCollision(cubic_path, path_yaw, kd_tree)) {
            return false;
        } else {
            path_to_goal = cubic_path;
            yaw_list = path_yaw;
            return true;
        }
    }

    // judge collision with obstacles
    bool judgeCollision(const PathPlanningUtilities::Path &judge_path, const std::vector<double> &path_yaw, const KDTree &kdtree) {
        double vehicle_length = 4.9;
        double d = vehicle_length / 2.0;
        double r = 1.2;
        // 得到disc参数后,判断车辆与障碍物的最小距离
        double min_distance = 12.0;
        // select one point from each 10 points
        for (size_t i = 0; i < judge_path.size(); i += 10) {
            // 计算disc中心
            double theta = path_yaw[i];
            PathPlanningUtilities::Point2f center_disc = judge_path[i];
            PathPlanningUtilities::Point2f front_disc;
            front_disc.x_ = judge_path[i].x_ + d * cos(theta);
            front_disc.y_ = judge_path[i].y_ + d * sin(theta);
            PathPlanningUtilities::Point2f front_disc_1;
            front_disc_1.x_ = judge_path[i].x_ + 0.333 * d * cos(theta);
            front_disc_1.y_ = judge_path[i].y_ + 0.333 * d * sin(theta);
            PathPlanningUtilities::Point2f front_disc_2;
            front_disc_2.x_ = judge_path[i].x_ + 0.666 * d * cos(theta);
            front_disc_2.y_ = judge_path[i].y_ + 0.666 * d * sin(theta);
            // PathPlanningUtilities::Point2f front_disc_3;
            // front_disc_3.x_ = straight_line[i].x_ + 0.6 * d * cos(theta);
            // front_disc_3.y_ = straight_line[i].y_ + 0.6 * d * sin(theta);
            // PathPlanningUtilities::Point2f front_disc_4;
            // front_disc_4.x_ = straight_line[i].x_ + 0.8 * d * cos(theta);
            // front_disc_4.y_ = straight_line[i].y_ + 0.8 * d * sin(theta);
            PathPlanningUtilities::Point2f rear_disc;
            rear_disc.x_ = judge_path[i].x_ - d * cos(theta);
            rear_disc.y_ = judge_path[i].y_ - d * sin(theta);
            PathPlanningUtilities::Point2f rear_disc_1;
            rear_disc_1.x_ = judge_path[i].x_ - 0.333 * d * cos(theta);
            rear_disc_1.y_ = judge_path[i].y_ - 0.333 * d * sin(theta);
            PathPlanningUtilities::Point2f rear_disc_2;
            rear_disc_2.x_ = judge_path[i].x_ - 0.666 * d * cos(theta);
            rear_disc_2.y_ = judge_path[i].y_ - 0.666 * d * sin(theta);
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
        if (min_distance == 0.0) {
            return true;
        } else {
            return false;
        }
    }

    // generate motions, first item is forward or backward, second item is steer 
    std::vector<std::pair<bool, double>> motions() {
        // construct steers and directions
        std::vector<double> steers = myLinspace(-this->max_steer_, this->max_steer_, this->num_steer_);
        if (std::find(steers.begin(), steers.end(), 0.0) == steers.end()) {
            steers.emplace_back(0.0);
        }
        sort(steers.begin(), steers.end());
        std::vector<bool> directions = {true, false};

        // generate motions
        std::vector<std::pair<bool, double>> motions;
        for (auto steer: steers) {
            for (auto direction: directions) {
                motions.emplace_back(std::make_pair(direction, steer));
            }
        }
        return motions;
    }

    // calculate next position information after executing a motion
    std::pair<PathPlanningUtilities::Point2f, double> move(const PathPlanningUtilities::Point2f &current_position, double current_yaw, double distance, double steer) {
        PathPlanningUtilities::Point2f new_position;
        new_position.x_ = current_position.x_ + distance * cos(current_yaw);
        new_position.y_ = current_position.y_ + distance * sin(current_yaw);
        double new_yaw = current_yaw + this->pi_2_pi(distance * tan(steer) / this->WB_);
        return std::make_pair(new_position, new_yaw);
    }

    // convert angle to the range from -PI to PI
    double pi_2_pi(double theta) {
        double changed_angle = theta;
        while (changed_angle > M_PI) {
            changed_angle -= 2.0 * M_PI;
        }
        while (changed_angle <= -M_PI) {
            changed_angle += 2.0 * M_PI;
        }
        return changed_angle;
    }

    // generate neughbors from a specific node
    std::vector<HybridAstarNode> getNeighbors(const HybridAstarNode &current_node, const HybridAstarNode &goal_node) {
        // calculate move distance
        double arc_l = 1.5 * GridMapConfig::reso_xy_;
        
        std::vector<HybridAstarNode> neighbors;
        // generate motions
        std::vector<std::pair<bool, double>> motions = this->motions();
        for (auto motion: motions) {
            // generate path from this motion
            PathPlanningUtilities::Path this_motion_path;
            std::vector<double> path_yaw;
            // get current position and yaw
            PathPlanningUtilities::Point2f current_position = current_node.path_.back();
            double current_yaw = current_node.yaw_list_.back();
            // generate and iterate motion distance
            std::vector<double> distances = myArange(0.0, arc_l + this->motion_resolution_, this->motion_resolution_);
            for (auto distance: distances) {
                std::pair<PathPlanningUtilities::Point2f, double> tmp_point_info = this->move(current_position, current_yaw, motion.first ? this->motion_resolution_ : -this->motion_resolution_, motion.second);
                current_position = tmp_point_info.first;
                current_yaw = tmp_point_info.second;
                this_motion_path.emplace_back(current_position);
                path_yaw.emplace_back(current_yaw);
            }
            // calculate path cost
            double cost = arc_l + this->steer_cost_ * fabs(motion.second) + this->steer_change_cost_ * fabs(current_node.steer_ - motion.second) + current_node.cost_;
            // construct new node 
            std::vector<bool> neighbor_directions = {true};
            HybridAstarNode neighbor_node = HybridAstarNode(std::round((this_motion_path.back().x_ + 11600.0) / GridMapConfig::reso_xy_), std::round((this_motion_path.back().y_ + 3000.0) / GridMapConfig::reso_xy_), std::round(path_yaw.back() / GridMapConfig::reso_yaw_), true, this_motion_path, path_yaw, neighbor_directions, motion.second, cost, this->calcHeuristic(current_node, goal_node), this->calcIndex(current_node));
            neighbors.emplace_back(neighbor_node);
        }
        return neighbors;
    }

    // generate final path
    PathPlanningUtilities::Path findFinalPath(const HybridAstarNode &goal_node, std::unordered_map<int, HybridAstarNode> &node_recorder) {
        PathPlanningUtilities::Path raw_path;
        HybridAstarNode current_node = goal_node;
        while (current_node.pre_index_ != -1) {
            PathPlanningUtilities::Path current_node_path = current_node.path_;
            assert(current_node_path.size() > 0);
            reverse(current_node_path.begin(), current_node_path.end());
            raw_path.insert(raw_path.end(), current_node_path.begin(), current_node_path.end());
            current_node = node_recorder[current_node.pre_index_];
            assert(current_node.is_closed_);
            // if (current_node.pre_index_ == -1) {
            //     break;
            // }
        }
        reverse(raw_path.begin(), raw_path.end());
        return raw_path;
    }
    




    double max_steer_ = 0.6; // max steer change for every node
    int num_steer_ = 20; // the sample number of steer
    double WB_ = 3.0; // rear to front wheel
    double motion_resolution_ = 0.1; // motion resolution
    double steer_cost_ = 1.0; // path steer cost
    double steer_change_cost_ = 5.0; // path steer change cost
};

};