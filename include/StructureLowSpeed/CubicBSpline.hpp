/*
    Copyright [2020] Jian ZhiQiang
*/
#pragma once



#include "Point.hpp"
#include "Path.hpp"
#include "Tools.hpp"
#include <vector>
#include <set>
#include <stdlib.h>
#include <unistd.h>
#include <algorithm>
#include <cassert>



// 三次BSpline曲线拟合
class CubicBSpline {
 public:
    // 构造函数
    CubicBSpline(const PathPlanningUtilities::Path &path) {
        PathPlanningUtilities::Path points;
        points.resize(path.size() + 2);
        for (size_t i = 0; i < path.size(); i++) {
            points[i + 1] = path[i];
        }
        size_t point_num = points.size();
        // 首先确定输入点的数量大于3
        assert(point_num > 3);
        // 对第一个点和最后一个点进行额外处理
        points[0].x_ = 2.0 * path[0].x_ - path[1].x_;
        points[0].y_ = 2.0 * path[0].y_ - path[1].y_;
        points[point_num - 1].x_ = 2.0 * path[path.size() - 1].x_ - path[path.size() - 2].x_;
        points[point_num - 1].y_ = 2.0 * path[path.size() - 1].y_ - path[path.size() - 2].y_;
        // 确定分段数
        this->segment_num_ = point_num - 3;
        this->x_coefficients_.resize(this->segment_num_);
        this->y_coefficients_.resize(this->segment_num_);
        // 开始遍历点
        for (size_t i = 0; i < point_num - 3; i++) {
            // 得到进行单段B-spline生成的4个点
            // 并以此进行x参数的计算
            this->x_coefficients_[i].resize(4);
            this->x_coefficients_[i][0] = 1.0 / 6.0 * (points[i].x_ + 4.0 * points[i + 1].x_ + points[i + 2].x_);
            this->x_coefficients_[i][1] = -0.5 * (points[i].x_ - points[i + 2].x_);
            this->x_coefficients_[i][2] = 0.5 * (points[i].x_ - 2.0 * points[i + 1].x_ + points[i + 2].x_);
            this->x_coefficients_[i][3] = -1.0 / 6.0 * (points[i].x_ - 3.0 * points[i + 1].x_ + 3.0 * points[i + 2].x_ - points[i + 3].x_);
            // 进行y参数的计算
            this->y_coefficients_[i].resize(4);
            this->y_coefficients_[i][0] = 1.0 / 6.0 * (points[i].y_ + 4.0 * points[i + 1].y_ + points[i + 2].y_);
            this->y_coefficients_[i][1] = -0.5 * (points[i].y_ - points[i + 2].y_);
            this->y_coefficients_[i][2] = 0.5 * (points[i].y_ - 2.0 * points[i + 1].y_ + points[i + 2].y_);
            this->y_coefficients_[i][3] = -1.0 / 6.0 * (points[i].y_ - 3.0 * points[i + 1].y_ + 3.0 * points[i + 2].y_ - points[i + 3].y_);
        }
    };

    // 析构函数
    ~CubicBSpline() {};

    // 获得总段数
    int getSegmentNum() const {
        return this->segment_num_;
    }

    // 得到采样点
    PathPlanningUtilities::Point2f getPoint(double u) const  {
        PathPlanningUtilities::Point2f point;
        point.x_ = this->xValue(u);
        point.y_ = this->yValue(u);
        return point;
    }

    // 得到朝向
    double getYaw(double u) const  {
        double dx = this->xDerivative(u);
        double dy = this->yDerivative(u);
        return atan2(dy, dx);
    }

    // 得到曲率
    double getCurvature(double u) const {
        double dx = this->xDerivative(u);
        double dy = this->yDerivative(u);
        double ddx = this->xSecondDerivative(u);
        double ddy = this->ySecondDerivative(u);
        return (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 1.5);
    }

    // 获得参数曲线上一个采样点
    PathPlanningUtilities::CurvePoint getCurvePoint(double u) const {
        PathPlanningUtilities::CurvePoint curve_point;
        curve_point.position_ = this->getPoint(u);
        curve_point.theta_ = this->getYaw(u);
        curve_point.kappa_ = this->getCurvature(u);
        return curve_point;
    }

 private:
    // 判断输入是否有效
    double inputVerify(double u) const  {
        if (Tools::isSmall(u, 0.0)) {
            return 0.0;
        } else if (Tools::isLarge(u, static_cast<double>(this->segment_num_))){
            return static_cast<double>(this->segment_num_);
        } else {
            return u;
        }
    }
    
    // 判断输入是在哪一段, 参数是多少
    std::pair<int, double> getSegmentInfo(double u) const  {
        u = this->inputVerify(u);
        for (int i = 0; i < this->segment_num_; i++) {
            if (Tools::isSmall(u, static_cast<double>(i + 1))) {
                double remain = u - i;
                std::pair<int, double> result(i, remain);
                return result;
            }
        }
        std::pair<int, double> result(this->segment_num_ - 1, 1.0);
        return result;
    }

    // 三次多项式x计算
    double xValue(double u) const  {
        u = this->inputVerify(u);
        // 得到分段和参数
        auto segment_info = this->getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return this->x_coefficients_[index][0] + this->x_coefficients_[index][1] * u + this->x_coefficients_[index][2] * u * u + this->x_coefficients_[index][3] * u * u * u;
    }

    // 三次多项式y计算
    double yValue(double u) const  {
        u = this->inputVerify(u);
        // 得到分段和参数
        auto segment_info = this->getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return this->y_coefficients_[index][0] + this->y_coefficients_[index][1] * u + this->y_coefficients_[index][2] * u * u + this->y_coefficients_[index][3] * u * u * u;
    }

    // 三次多项式导数x计算
    double xDerivative(double u) const  {
        u = this->inputVerify(u);
        // 得到分段和参数
        auto segment_info = this->getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return this->x_coefficients_[index][1] + 2.0 * this->x_coefficients_[index][2] * u + 3.0 * this->x_coefficients_[index][3] * u * u;
    }

    // 三次多项式导数y计算
    double yDerivative(double u) const  {
        u = this->inputVerify(u);
        // 得到分段和参数
        auto segment_info = this->getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return this->y_coefficients_[index][1] + 2.0 * this->y_coefficients_[index][2] * u + 3.0 * this->y_coefficients_[index][3] * u * u;
    }

    // 三次多项式二阶导数x计算
    double xSecondDerivative(double u) const  {
        u = this->inputVerify(u);
        // 得到分段和参数
        auto segment_info = this->getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return 2.0 * this->x_coefficients_[index][2] + 6.0 * this->x_coefficients_[index][3] * u;
    }

    // 三次多项式二阶导数x计算
    double ySecondDerivative(double u) const  {
        u = this->inputVerify(u);
        // 得到分段和参数
        auto segment_info = this->getSegmentInfo(u);
        u = segment_info.second;
        int index = segment_info.first;
        return 2.0 * this->y_coefficients_[index][2] + 6.0 * this->y_coefficients_[index][3] * u;
    }

    int segment_num_;  // 分段的数量
    std::vector<std::vector<double>> x_coefficients_;  // B-spline的x参数
    std::vector<std::vector<double>> y_coefficients_;  // B-spline的y参数
};

