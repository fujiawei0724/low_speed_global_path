/*
    Copyright [2020] Jian ZhiQiang
*/

#pragma once

#include "Point.hpp"
#include "Path.hpp"

// 障碍物类，用于保存障碍物的信息，主要包含障碍物的位置、大小、速度、属性、预测轨迹等
// 根据障碍物的信息，可以进行决策和安全性判断。


class Obstacle {
 public:
    // 构造函数和析构函数，构造函数必须显式调用，必须包含障碍物信息为障碍物的id，id为区分不同障碍物的唯一标识符
    explicit Obstacle(size_t id) {
        this->id_ = id;
    }

    ~Obstacle() {}

    // 获取障碍物ID
    size_t getID() const {
        return this->id_;
    }

    // 设置障碍物中心点位置
    void setObstaclePosition(const PathPlanningUtilities::Point2f &position) {
        this->position_ = position;
    }

    // 获取障碍物中性点位置
    PathPlanningUtilities::Point2f getObstaclePosition() const {
        return this->position_;
    }

    // 设置障碍物形状
    void setObstacleShape(double width, double length) {
        this->width_ = width;
        this->length_ = length;
    }

    // 获取障碍物形状
    double getObstacleWidth() const {
        return this->width_;
    }

    double getObstacleLength() const {
        return this->length_;
    }

    // 设置障碍物朝向
    void setObstacleOrientation(double orientation) {
        this->orientation_ = orientation;
    }

    // 获取障碍物朝向
    double getObstacleOrientation() const {
        return this->orientation_;
    }

    // 设置障碍物速度
    void setObstacleVelocity(double velocity) {
        this->velocity_ = velocity;
    }

    // 获取障碍物速度
    double getObstacleVelocity() const {
        return this->velocity_;
    }

    // 设置障碍物速度方向
    void setObstacleVelocityDirection(double velocity_direction) {
        this->velocity_direction_ = velocity_direction;
    }

    // 获取障碍物速度方向
    double getObstacleVelocityDirection() const {
        return this->velocity_direction_;
    }

    // 设置障碍物的占用区域宽度
    void setObstacleOccupationWidth(double occupation_width) {
        this->occupation_width_ = occupation_width;
    }

    // 获取障碍物占用区域宽度
    double getObstacleOccupationWidth() const {
        return this->occupation_width_;
    }

    // 设置障碍物加速度（目前可能不需要）
    void setObstacleAcceleration(double acceleration) {
        this->acceleration_ = acceleration;
    }

    // 获取障碍物加速度（目前可能不需要）
    double getObstacleAcceleration() const {
        return this->acceleration_;
    }

    // 设置预测轨迹集合
    void setPredictedTrajectorySet(const std::vector<PathPlanningUtilities::Curve> &predicted_trajectory_set) {
        this->predicted_trajectory_set_ = predicted_trajectory_set;
    }

    // 获取预测轨迹数量
    size_t getPredictedTrajectoryNumber() const {
        return this->predicted_trajectory_set_.size();
    }

    // 获取预测轨迹集合
    const std::vector<PathPlanningUtilities::Curve> &getPredictedTrajectorySet() const {
        return this->predicted_trajectory_set_;
    }

    // 获取一条预测轨迹
    const PathPlanningUtilities::Curve &getPredictedTrajectory(size_t index) const {
        return this->predicted_trajectory_set_[index];
    }

    // 设置障碍物类别
    void setObstacleClass(size_t class_name) {
        this->class_name_ = class_name;
    }

    // 获取障碍物类别
    size_t getObstacleClass() const {
        return this->class_name_;
    }

    // 清空历史轨迹（目前可能不需要）
    void clearObstacleHistoryPosition() {
        std::vector<PathPlanningUtilities::Point2f>().swap(this->history_position_set_);
    }

    // 添加历史轨迹点（目前可能不需要）
    void addObstacleHistoryPosition(const PathPlanningUtilities::Point2f &history_position) {
        this->history_position_set_.push_back(history_position);
    }

    // 获取历史轨迹（目前可能不需要）
    const std::vector<PathPlanningUtilities::Point2f> &getObstacleHistoryPositionSet() const {
        return this->history_position_set_;
    }

 private:
    size_t id_;  // 障碍物的唯一标识符
    PathPlanningUtilities::Point2f position_;  // 障碍物中心在世界坐标系下的位置信息
    double width_;  // 障碍物矩形边框的宽度
    double length_;  // 障碍物矩形边框的长度
    double orientation_;  // 障碍物朝向
    double velocity_;  // 障碍物的速度
    double velocity_direction_;  // 障碍物速度方向
    double occupation_width_;  // 障碍物占用区域的宽度
    double acceleration_;  // 障碍物的加速度（未定）
    std::vector<PathPlanningUtilities::Curve> predicted_trajectory_set_;  // 障碍物预测的轨迹
    size_t class_name_;  // 障碍物的类别信息（未定）
    std::vector<PathPlanningUtilities::Point2f> history_position_set_;  // 障碍物的历史位置（未定）
};



