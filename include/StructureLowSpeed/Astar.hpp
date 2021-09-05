/*
    Copyright [2020] Jian ZhiQiang
*/

#ifndef ASTAR_H_
#define ASTAR_H_

#include "StructureLowSpeed/Config.hpp"
#include "StructureLowSpeed/KDTree.hpp"
#include "StructureLowSpeed/LocalPath.hpp"
#include "Common.hpp"
#include "Tools.hpp"
#include "Point.hpp"
#include "Path.hpp"
#include "PathGenerator.h"
#include "LineSegment.hpp"
#include "Visualization.hpp"
#include "KDTree.hpp"
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


// A星引导规划
namespace GlobalPlanningFrontend {

// 节点类
class Node {
 public:
    // 构造函数
    Node(int x, int y, int pre_index = -1, double cost = 0.0, double heuristic = 0.0) {
        this->x_ = x;
        this->y_ = y;
        this->pre_index_ = pre_index;
        this->cost_ = cost;
        this->heuristic_ = heuristic;
        this->is_open_ = false;
        this->is_closed_ = false;
    };

    // 构造函数
    Node(){
        this->is_open_ = false;
        this->is_closed_ = false;
    };

    // 析构函数
    ~Node(){};

    // 重载大小比较
    bool operator<(const Node &node) const {
        return Tools::isSmall(this->cost_ + this->heuristic_, node.cost_ + node.heuristic_);
    };

    bool operator>(const Node &node) const {
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

    int x_;  // 节点宽度下标
    int y_;  // 节点高度下标
    int pre_index_;  // 上一个节点的下标
    double cost_;  // 节点的代价
    double heuristic_;  // 节点的启发
    bool is_open_;  // 是否属于open集合
    bool is_closed_;  // 是否属于closed集合
};

// A星规划器
class AstarPlanner {
 public:
    // 构造函数
    AstarPlanner(double robot_size){
        this->robot_size_ = robot_size;
        this->motions_.push_back(std::make_pair(-1, -1));
        this->motions_.push_back(std::make_pair(-1, 0));
        this->motions_.push_back(std::make_pair(-1, 1));
        this->motions_.push_back(std::make_pair(0, -1));
        this->motions_.push_back(std::make_pair(0, 1));
        this->motions_.push_back(std::make_pair(1, -1));
        this->motions_.push_back(std::make_pair(1, 0));
        this->motions_.push_back(std::make_pair(1, 1));
    };

    // 析构函数
    ~AstarPlanner(){};

    // 进行规划
    int planning(const PathPlanningUtilities::Point2f &start_point, const PathPlanningUtilities::Point2f &goal_point, const GridMap &grid_map, const KDTree &kd_tree, PathPlanningUtilities::Path &path){
        // 进行A星搜索
        // 初始化节点记录器
        std::vector<Node> node_recorder;
        node_recorder.resize(grid_map.getWidth() * grid_map.getHeight());
        // 初始化搜索记录器
        std::priority_queue<Node, std::vector<Node>, std::greater<Node> > open_set;  // 未搜索节点
        // 构建起始节点和目标节点
        std::pair<int, int> start_in_grid = grid_map.getGridMapCoordinate(start_point.x_, start_point.y_);
        std::pair<int, int> goal_in_grid = grid_map.getGridMapCoordinate(goal_point.x_, goal_point.y_);
        Node start_node = Node(start_in_grid.first, start_in_grid.second);
        Node goal_node = Node(goal_in_grid.first, goal_in_grid.second);
        // 计算起点的代价和启发
        start_node.cost_ = 0.0;
        start_node.heuristic_ = this->calcHeuristic(start_node, goal_node);
        // 将起始节点加入搜索列表
        start_node.open();
        node_recorder[grid_map.getIndex(start_node.x_, start_node.y_)] = start_node;
        open_set.push(start_node);
        // 开始搜索
        while (!open_set.empty()) {
            // 得到搜索列表中最小的index作为当前节点,并从开集合中删除
            Node current_node = open_set.top();
            open_set.pop();
            // 将当前节点加入闭集合,
            int current_index = grid_map.getIndex(current_node.x_, current_node.y_);
            node_recorder[current_index].close();
            // 判断当前节点是否为终点
            if (current_node.x_ == goal_node.x_ && current_node.y_ == goal_node.y_) {
                // 当前节点为终点,结束搜索
                goal_node = current_node;
                break;
            }
            // 得到当前点到终点的连线段
            PathPlanningUtilities::Point2f current_point = grid_map.getCartesianCoordinate(current_node.x_, current_node.y_);
            PathPlanningUtilities::LineSegment line_segment = PathPlanningUtilities::LineSegment(current_point, goal_point);
            if (Tools::isSmall(line_segment.length(), 0.33 * PathPlanningUtilities::calcDistance(start_point, goal_point))) {
                // 判断障碍物点到线段距离是否小于阈值
                bool is_goal_reachable = true;
                for (auto obstacle_point: kd_tree.getObstacles().points) {
                    double distance = line_segment.pointToLineSegmentDistance(obstacle_point.x, obstacle_point.y);
                    if (!Tools::isLarge(distance, this->robot_size_)) {
                        is_goal_reachable = false;
                    }
                }
                if (is_goal_reachable) {
                    goal_node.pre_index_ = current_index;
                    break;
                }
            }
            // 如果当前节点不是终点,搜索其的邻居
            for (auto motion: this->motions_) {
                // 计算邻居节点
                Node neighbor_node = Node(current_node.x_ + motion.first, current_node.y_ + motion.second, current_index);
                // 得到邻居节点下标
                int neighbor_index = grid_map.getIndex(neighbor_node.x_, neighbor_node.y_);
                // 判断邻居节点是否在闭集合内
                if (node_recorder[neighbor_index].is_closed_ == true) {
                    // 存在于闭集合内
                    continue;
                }
                // 判断邻居是否超出边界
                if(!grid_map.isVerify(neighbor_node.x_, neighbor_node.y_)) {
                    continue;
                }
                // 判断此邻居是否与障碍物碰撞
                bool is_neighbor_collide = false;
                int range = static_cast<int>(this->robot_size_ / Config::grid_resolution_);
                for (int i = - range; i < range + 1; i++) {
                    for (int j = -range; j < range + 1; j++) {
                        if (!grid_map.isVerify(neighbor_node.x_ + i, neighbor_node.y_ + j)) {
                            is_neighbor_collide = true;
                            goto end;
                        } else if (this->judgeCollision(grid_map.getCartesianCoordinate(neighbor_node.x_, neighbor_node.y_), kd_tree)) {
                            is_neighbor_collide = true;
                            goto end;
                        }
                    }
                }
                end:
                if (is_neighbor_collide) {
                    continue;
                }
                // 计算损失增量,包括两个部分,走过的距离和离障碍物的距离
                double motion_cost = sqrt(motion.first * motion.first + motion.second * motion.second);
                // 得到邻居的损失
                neighbor_node.cost_ = current_node.cost_ + motion_cost;
                // 得到邻居的启发
                neighbor_node.heuristic_ = this->calcHeuristic(neighbor_node, goal_node);
                // 判断邻居节点是否在开集合内
                if (node_recorder[neighbor_index].is_open_ == true) {
                    // 在开集合内
                    if (neighbor_node < node_recorder[neighbor_index]) {
                        neighbor_node.open();
                        node_recorder[neighbor_index] = neighbor_node;
                        open_set.push(neighbor_node);
                    }
                } else {
                    // 不在开集合内
                    // 加入开集合
                    neighbor_node.open();
                    node_recorder[neighbor_index] = neighbor_node;
                    open_set.push(neighbor_node);
                }
            }
        }
        // 开始生成最终路径
        // 判断是否生成了最终路径
        if (goal_node.pre_index_ == -1) {
            // 没有找到路径
            return -1;
        }
        // 生成了最终路径
        PathPlanningUtilities::Path raw_path;
        Node current_node = goal_node;
        while (current_node.pre_index_ != -1) {
            PathPlanningUtilities::Point2f point = grid_map.getCartesianCoordinate(current_node.x_, current_node.y_);
            raw_path.push_back(point);
            current_node = node_recorder[current_node.pre_index_];
            assert(current_node.is_closed_);
        }
        // 反转路径后输出
        reverse(raw_path.begin(),raw_path.end());
        // 给raw_path补点
        bool interpolation_finished = false;
        while (!interpolation_finished) {
            interpolation_finished = true;
            for (size_t i = 1; i < raw_path.size(); i++) {
                // 计算两点之间的距离
                double distance = PathPlanningUtilities::calcDistance(raw_path[i - 1], raw_path[i]);
                if (Tools::isLarge(distance, 2.0 * Config::grid_resolution_)) {
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
        path = raw_path;
        return 1;
    };

    // 进行规划(带盲区和偏向性)
    int planning(const PathPlanningUtilities::Point2f &start_point, const PathPlanningUtilities::Point2f &goal_point, const GridMap &grid_map, const GridMap &blind_grid_map, const KDTree &kd_tree, const std::shared_ptr<KDTree> &history_reference_line_kdtree_ptr, PathPlanningUtilities::Path &path){
        // 进行A星搜索
        // 初始化节点记录器
        std::vector<Node> node_recorder;
        node_recorder.resize(grid_map.getWidth() * grid_map.getHeight());
        // 初始化搜索记录器
        std::priority_queue<Node, std::vector<Node>, std::greater<Node> > open_set;  // 未搜索节点
        // 构建起始节点和目标节点
        std::pair<int, int> start_in_grid = grid_map.getGridMapCoordinate(start_point.x_, start_point.y_);
        std::pair<int, int> goal_in_grid = grid_map.getGridMapCoordinate(goal_point.x_, goal_point.y_);
        Node start_node = Node(start_in_grid.first, start_in_grid.second);
        Node goal_node = Node(goal_in_grid.first, goal_in_grid.second);
        // 计算起点的代价和启发
        start_node.cost_ = 0.0;
        start_node.heuristic_ = this->calcHeuristic(start_node, goal_node);
        // 将起始节点加入搜索列表
        start_node.open();
        node_recorder[grid_map.getIndex(start_node.x_, start_node.y_)] = start_node;
        open_set.push(start_node);
        // 开始搜索
        while (!open_set.empty()) {
            // 得到搜索列表中最小的index作为当前节点,并从开集合中删除
            Node current_node = open_set.top();
            open_set.pop();
            // 将当前节点加入闭集合,
            int current_index = grid_map.getIndex(current_node.x_, current_node.y_);
            node_recorder[current_index].close();
            // 判断当前节点是否为终点
            if (current_node.x_ == goal_node.x_ && current_node.y_ == goal_node.y_) {
                // 当前节点为终点,结束搜索
                goal_node = current_node;
                break;
            }
            // 得到当前点到终点的连线段
            PathPlanningUtilities::Point2f current_point = grid_map.getCartesianCoordinate(current_node.x_, current_node.y_);
            PathPlanningUtilities::LineSegment line_segment = PathPlanningUtilities::LineSegment(current_point, goal_point);
            if (Tools::isSmall(line_segment.length(), 0.33 * PathPlanningUtilities::calcDistance(start_point, goal_point))) {
                // 判断障碍物点到线段距离是否小于阈值
                bool is_goal_reachable = true;
                for (auto obstacle_point: kd_tree.getObstacles().points) {
                    double distance = line_segment.pointToLineSegmentDistance(obstacle_point.x, obstacle_point.y);
                    if (!Tools::isLarge(distance, this->robot_size_)) {
                        is_goal_reachable = false;
                    }
                }
                if (is_goal_reachable) {
                    goal_node.pre_index_ = current_index;
                    break;
                }
            }
            // 如果当前节点不是终点,搜索其的邻居
            for (auto motion: this->motions_) {
                // 计算邻居节点
                Node neighbor_node = Node(current_node.x_ + motion.first, current_node.y_ + motion.second, current_index);
                // 得到邻居节点下标
                int neighbor_index = grid_map.getIndex(neighbor_node.x_, neighbor_node.y_);
                // 判断邻居节点是否在闭集合内
                if (node_recorder[neighbor_index].is_closed_ == true) {
                    // 存在于闭集合内
                    continue;
                }
                // 判断邻居是否超出边界
                if(!grid_map.isVerify(neighbor_node.x_, neighbor_node.y_)) {
                    continue;
                }
                // 判断此邻居是否与障碍物碰撞
                bool is_neighbor_collide = false;
                int range = static_cast<int>(this->robot_size_ / Config::grid_resolution_);
                for (int i = - range; i < range + 1; i++) {
                    for (int j = -range; j < range + 1; j++) {
                        if (!grid_map.isVerify(neighbor_node.x_ + i, neighbor_node.y_ + j)) {
                            is_neighbor_collide = true;
                            goto end;
                        } else if (grid_map.isOccupied(grid_map.getIndex(neighbor_node.x_ + i, neighbor_node.y_ + j))) {
                            is_neighbor_collide = true;
                            goto end;
                        }
                    }
                }
                end:
                if (is_neighbor_collide) {
                    continue;
                }
                // 计算损失增量,包括三个部分,走过的距离,离历史参考点的距离,是否处于盲区
                // 走过距离的损失
                double motion_cost = sqrt(motion.first * motion.first + motion.second * motion.second);
                // 离历史参考的损失
                double consistency_cost = 0.0;
                if (history_reference_line_kdtree_ptr != nullptr) {
                    // 计算邻居对应的真实坐标
                    PathPlanningUtilities::Point2f neighbor_point = grid_map.getCartesianCoordinate(neighbor_node.x_, neighbor_node.y_);
                    // 计算最近的参考点之间的距离
                    std::vector<std::pair<float, float>> neighbors;
                    std::vector<float> sq_distances;
                    history_reference_line_kdtree_ptr->findKNeighbor(neighbor_point.x_, neighbor_point.y_, &neighbors, &sq_distances, 1);
                    consistency_cost = sqrt(sq_distances[0]);
                }
                // 盲区损失
                double blind_cost = 0.0;
                if (blind_grid_map.isOccupied(blind_grid_map.getIndex(neighbor_node.x_, neighbor_node.y_))) {
                    // 如果是盲区,进行损失增加
                    blind_cost = Config::blind_cost_increment;
                }
                // 得到邻居的损失
                neighbor_node.cost_ = current_node.cost_ + motion_cost + consistency_cost + blind_cost;
                // 得到邻居的启发
                neighbor_node.heuristic_ = this->calcHeuristic(neighbor_node, goal_node);
                // 判断邻居节点是否在开集合内
                if (node_recorder[neighbor_index].is_open_ == true) {
                    // 在开集合内
                    if (neighbor_node < node_recorder[neighbor_index]) {
                        neighbor_node.open();
                        node_recorder[neighbor_index] = neighbor_node;
                        open_set.push(neighbor_node);
                    }
                } else {
                    // 不在开集合内
                    // 加入开集合
                    neighbor_node.open();
                    node_recorder[neighbor_index] = neighbor_node;
                    open_set.push(neighbor_node);
                }
            }
        }
        // 开始生成最终路径
        // 判断是否生成了最终路径
        if (goal_node.pre_index_ == -1) {
            // 没有找到路径
            return -1;
        }
        // 生成了最终路径
        PathPlanningUtilities::Path raw_path;
        Node current_node = goal_node;
        while (current_node.pre_index_ != -1) {
            PathPlanningUtilities::Point2f point = grid_map.getCartesianCoordinate(current_node.x_, current_node.y_);
            raw_path.push_back(point);
            current_node = node_recorder[current_node.pre_index_];
            assert(current_node.is_closed_);
        }
        // 反转路径后输出
        reverse(raw_path.begin(),raw_path.end());
        // 给raw_path补点
        bool interpolation_finished = false;
        while (!interpolation_finished) {
            interpolation_finished = true;
            for (size_t i = 1; i < raw_path.size(); i++) {
                // 计算两点之间的距离
                double distance = PathPlanningUtilities::calcDistance(raw_path[i - 1], raw_path[i]);
                if (Tools::isLarge(distance, 2.0 * Config::grid_resolution_)) {
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
        path = raw_path;
        return 1;
    };
 
 private:
    // 计算启发函数(使用的启发函数为到终点的距离)
    double calcHeuristic(const Node &current_node, const Node &goal_node, double weight= 1.0) {
        return sqrt((current_node.x_ - goal_node.x_) * (current_node.x_ - goal_node.x_) + (current_node.y_ - goal_node.y_) * (current_node.y_ - goal_node.y_)) * weight;
    }

    // judge whether collision
    bool judgeCollision(const PathPlanningUtilities::Point2f point, const KDTree &kd_tree) {
        // 进行距离判断
        std::vector<std::pair<float, float>> results;
        std::vector<float> sq_distances;
        kd_tree.findKNeighbor(point.x_, point.y_, &results, &sq_distances, 1);
        double min_distance = sqrt(sq_distances[0]) - this->robot_size_;
        if (min_distance >= 0.0) {
            return false;
        } else {
            return true;
        }
    }

    double robot_size_;  // 机器人大小
    std::vector<std::pair<int, int>> motions_;  // 行为
};

};

#endif
