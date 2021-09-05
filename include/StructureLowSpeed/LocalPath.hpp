/*
    Copyright [2020] Jian ZhiQiang
*/
#pragma once


#include "Point.hpp"
#include "Path.hpp"
#include "Common.hpp"
#include "glog/logging.h"

// actualize arange
template<typename T>
static std::vector<double> myArange(T start_in, T end_in, T gap_in) {
    std::vector<double> arange;
    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in) - EPS;
    double gap = static_cast<double>(gap_in);
    if (end < start) {
        return arange;
    }
    int length = static_cast<int>((end - start) / gap) + 1;
    arange.resize(length);
    for (int i = 0; i < length; i++) {
        arange[i] = i * gap + start;
    }
    return arange;
}

// actualize dot
static double myDot(const std::vector<double> &x, const std::vector<double> &y) {
    assert(x.size() == y.size());
    double res = 0.0;
    for (size_t i = 0; i < x.size(); i++) {
        res += x[i] * y[i];
    }
    return res;
}

// actualize linspace
template<typename T>
static std::vector<double> myLinspace(T start_in, T end_in, int num_in) {

    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0) {
        return linspaced;
    }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);
    linspaced.resize(num);
    for(int i = 0; i < num - 1; i++){
        linspaced[i] = start + delta * i;
    }
    linspaced[num - 1] = end;
    return linspaced;
}

// 局部规划路径信息
class LocalPathInfo {
 public:
    // 构造函数
    LocalPathInfo() {}

    LocalPathInfo(const PathPlanningUtilities::Curve &curve, const std::vector<double> &curvature_change_rates, const std::vector<double> &arc_lengths, const std::vector<size_t> &reference_indexes, double offset) {
        this->curve_ = curve;
        this->curvature_change_rates_ = curvature_change_rates;
        this->arc_lengths_ = arc_lengths;
        this->reference_indexes_ = reference_indexes;
        this->offset_ = offset;
    }

    // 析构函数
    ~LocalPathInfo() {}

    PathPlanningUtilities::Curve curve_;  // 路径
    std::vector<double> curvature_change_rates_;  // 曲率变化率
    std::vector<double> arc_lengths_;  // 弧长
    std::vector<size_t> reference_indexes_;  // 相对参考线的下标
    double offset_;  // 离参考线偏移量
};

// 栅格地图类
class GridMap {
 public:
    // 构造函数
    GridMap(const nav_msgs::OccupancyGrid &occupancy_grid){
        for (auto meta_data: occupancy_grid.data) {
            if (meta_data > 50) {   
                this->data_.push_back(true);
            } else {
                this->data_.push_back(false);
            }
        }
        this->width_ = occupancy_grid.info.width;
        this->height_ = occupancy_grid.info.height;
        this->resolution_ = occupancy_grid.info.resolution;
        this->root_x_ = occupancy_grid.info.origin.position.x;
        this->root_y_ = occupancy_grid.info.origin.position.y;
        this->root_theta_ = 2.0 * atan2(occupancy_grid.info.origin.orientation.z, occupancy_grid.info.origin.orientation.w);
    };

    GridMap() {};

    // 析构函数
    ~GridMap() {};

    // 修改分辨率
    void changeResolution(double new_resolution) {
        // 计算比例系数
        double ratio = new_resolution / this->resolution_;
        // 计算新的宽和高
        int new_width = static_cast<int>(this->width_ / ratio);
        int new_height = static_cast<int>(this->height_ / ratio);
        // 得到新的栅格数据
        std::vector<bool> new_data(new_width * new_height, false);
        for (int i = 0; i < this->width_; i++) {
            for (int j = 0; j < this->height_; j++) {
                // 计算是否被占据
                int index = this->getIndex(i, j);
                if (this->isOccupied(index)) {
                    // 如果被占据,判断它对应新栅格的下标
                    int new_index = static_cast<int>(i / ratio) + static_cast<int>(j / ratio) * new_width;
                    new_data[new_index] = true;
                }
            }
        }
        // 更新数据
        this->resolution_ = new_resolution;
        this->width_ = new_width;
        this->height_ = new_height;
        this->data_ = new_data;
        std::cout << "new width: " << this->width_ << ", new height: " << this->height_ << std::endl;
        std::cout << "data size: " << this->data_.size() << std::endl;
    }

    // 获取对应栅格是否被占据
    bool isOccupied(int index) const {
        return data_[index];
    };

    // 求对应点的栅格下标
    int getIndex(int x, int y) const {
        return x + y * this->width_;
    };

    // 计算栅格坐标
    std::pair<int, int> getXY(int index) const {
        int x = index % this->width_;
        int y = index / this->width_;
        return std::make_pair(x, y);
    };

    // 判断是否抄错边界
    bool isVerify(int x, int y) const {
        if (x >= 0 && x < this->width_ && y >= 0 && y < this->height_) {
            return true;
        } else {
            return false;
        }
    }

    // 计算对应的栅格坐标
    std::pair<int, int> getGridMapCoordinate(double px, double py) const {
        PathPlanningUtilities::CurvePoint curve_point;
        curve_point.position_.x_ = this->root_x_;
        curve_point.position_.y_ = this->root_y_;
        curve_point.theta_ = this->root_theta_;
        PathPlanningUtilities::Point2f point;
        point.x_ = px;
        point.y_ = py;
        PathPlanningUtilities::Point2f new_point = this->calcNewCoordinationPosition(curve_point, point);
        int x = new_point.x_ / this->resolution_;
        int y = new_point.y_ / this->resolution_;
        return std::pair<int, int>(x, y);
    };
    
    // 计算真实坐标
    PathPlanningUtilities::Point2f getCartesianCoordinate(int x, int y) const {
        PathPlanningUtilities::CurvePoint curve_point;
        curve_point.position_.x_ = this->root_x_;
        curve_point.position_.y_ = this->root_y_;
        curve_point.theta_ = this->root_theta_;
        PathPlanningUtilities::Point2f point;
        point.x_ = static_cast<double>(x * this->resolution_);
        point.y_ = static_cast<double>(y * this->resolution_);
        PathPlanningUtilities::Point2f raw_point = this->calcOldCoordinationPosition(curve_point, point);
        return raw_point;
    };

    // 转化成ros消息
    nav_msgs::OccupancyGrid toRosMessage() const{
        nav_msgs::OccupancyGrid occupancy_grid;
        geometry_msgs::Pose pose;
        pose.position.x = this->root_x_;
        pose.position.y = this->root_y_;
        pose.orientation.z = sin(this->root_theta_ * 0.5);
        pose.orientation.w = cos(this->root_theta_ * 0.5);
        std::vector<int8_t> out_data;
        for (auto meta_data: this->data_) {
            if (meta_data) {
                out_data.push_back(100);
            } else {
                out_data.push_back(0);
            }
        }
        occupancy_grid.data = out_data;
        occupancy_grid.info.resolution = this->resolution_;
        occupancy_grid.info.width = this->width_;
        occupancy_grid.info.height = this->height_;
        occupancy_grid.info.origin = pose;
        occupancy_grid.info.map_load_time = ros::Time::now();
        occupancy_grid.header.frame_id = "world";
        occupancy_grid.header.stamp = ros::Time::now();
        return occupancy_grid;
    };

    // 得到栅格地图的宽度
    int getWidth() const {
        return this->width_;
    };

    // 得到栅格地图的高度
    int getHeight() const {
        return this->height_;
    };

    // 得到分辨率
    double getResolution() const {
        return this->resolution_;
    }

    // 清空栅格中全部内容
    void clear() {
        for (size_t i = 0; i < this->data_.size(); i++) {
            this->data_[i] = false;
        }
    }

    // 设置栅格地图
    void set(size_t index, bool value) {
        this->data_[index] = value;
    }

 private:

    // 计算坐标系转换
    PathPlanningUtilities::Point2f calcNewCoordinationPosition(const PathPlanningUtilities::CurvePoint &new_coordination_origin, const PathPlanningUtilities::Point2f &position) const {
        PathPlanningUtilities::Point2f new_position;
        new_position.x_ = (position.x_ - new_coordination_origin.position_.x_) * cos(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * sin(new_coordination_origin.theta_);
        new_position.y_ = -(position.x_ - new_coordination_origin.position_.x_) * sin(new_coordination_origin.theta_) + (position.y_ - new_coordination_origin.position_.y_) * cos(new_coordination_origin.theta_);
        return new_position;
    }
    
    // 计算坐标系反转换
    PathPlanningUtilities::Point2f calcOldCoordinationPosition(const PathPlanningUtilities::CurvePoint &new_coordination_origin, const PathPlanningUtilities::Point2f &position) const {
        PathPlanningUtilities::Point2f old_position;
        old_position.x_ = new_coordination_origin.position_.x_ + position.x_ * cos(new_coordination_origin.theta_) - position.y_ * sin(new_coordination_origin.theta_);
        old_position.y_ = new_coordination_origin.position_.y_ + position.x_ * sin(new_coordination_origin.theta_) + position.y_ * cos(new_coordination_origin.theta_);
        return old_position;
    }
    int width_;  // 栅格地图的宽度(格数)
    int height_;  // 栅格地图的高度(格数)
    double resolution_;  // 栅格地图的分辨率
    double root_x_;  // 栅格地图根节点x坐标
    double root_y_;  // 栅格地图根节点y坐标
    double root_theta_;  // 栅格地图根节点朝向
    std::vector<bool> data_;  // 栅格地图数据
};


class CubicSpline {
public:
    CubicSpline(double sx, double sy, double syaw, double gx, double gy, double gyaw, double gap = 0.1) {
        this->arc_l_ = sqrt(pow(sx - gx, 2.0) + pow(sy - gy, 2.0));
        this->x_param_ = this->calcParam(sx, cos(syaw), gx, cos(gyaw), this->arc_l_);
        this->y_param_ = this->calcParam(sy, sin(syaw), gy, sin(gyaw), this->arc_l_);
        this->gap_ = gap;
    };
    ~CubicSpline() {

    };

    // get path
    std::pair<PathPlanningUtilities::Path, std::vector<double>> getPath() {
        std::vector<double> samples = myArange(0.0, this->arc_l_ + 0.1, this->gap_);
        PathPlanningUtilities::Path cubic_path;
        std::vector<double> path_yaw;
        for (auto sample: samples) {
            PathPlanningUtilities::Point2f this_point;
            std::vector<double> sample_param = {1.0, sample, pow(sample, 2.0), pow(sample, 3.0)};
            std::vector<double> sample_derivative_param = {0.0, 1.0, 2.0 * sample, 3.0 * pow(sample, 2.0)};
            double position_x = myDot(this->x_param_, sample_param);
            double position_y = myDot(this->y_param_, sample_param);
            this_point.x_ = position_x;
            this_point.y_ = position_y;
            cubic_path.emplace_back(this_point);
            double point_yaw = atan2(myDot(this->y_param_, sample_derivative_param), myDot(this->x_param_, sample_derivative_param));
            path_yaw.emplace_back(point_yaw);
        }
        return std::make_pair(cubic_path, path_yaw);
    }

private:
    // calculate paramters
    std::vector<double> calcParam(double start, double start_derivation, double goal, double goal_derivation, double arc_l) {
        double a0 = start;
        double a1 = start_derivation;
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);
        A(0, 0) = pow(arc_l, 2.0);
        A(0, 1) = pow(arc_l, 3.0);
        A(1, 0) = 2.0 * arc_l;
        A(1, 1) = 3.0 * pow(arc_l, 2.0);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2, 1);
        B(0, 0) = goal - a0 - a1 * arc_l;
        B(1, 0) = goal_derivation - a1;
        Eigen::MatrixXd result = A.colPivHouseholderQr().solve(B);
        return {a0, a1, result(0, 0), result(1, 0)};
    }


    double arc_l_;
    std::vector<double> x_param_;
    std::vector<double> y_param_;
    double gap_; // distance between two contiguous points
};



