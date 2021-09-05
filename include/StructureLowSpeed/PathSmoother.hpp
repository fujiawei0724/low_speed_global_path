/*
    Copyright [2020] Jian ZhiQiang
*/
#pragma once



#include "StructureLowSpeed/Config.hpp"
#include "StructureLowSpeed/LocalPath.hpp"
#include "Tools.hpp"
#include "StructureLowSpeed/KDTree.hpp"

namespace GlobalPlanningBackend{

// 路径平滑器
class PathSmoother {
 public:
    // 构造函数
    PathSmoother(double max_distance, double max_curvature, double obs_weight = 1.0, double cur_weight = 10.0, double smoo_weight = 0.1, double dis_weight = 1.0, double cur_change_rate_weight = 10.0) {
        this->obs_weight_ = obs_weight;
        this->cur_weight_ = cur_weight;
        this->smoo_weight_ = smoo_weight;
        this->dis_weight_ = dis_weight;
        this->cur_change_rate_weight_ = cur_change_rate_weight;
        this->max_curvature_ = max_curvature;
        this->max_distance_ = max_distance;
    };

    // 析构函数
    ~PathSmoother() {};

    // DEBUG: Add visualization
    PathPlanningUtilities::Path smoothing(const PathPlanningUtilities::Path &raw_path, const KDTree &kd_tree, const ros::Publisher &publisher, bool with_remove = true) {
        // 路径平滑考虑到三个因素,离障碍物的距离,曲率大小和采样点间隔变化
        PathPlanningUtilities::Path smoothed_path = raw_path;
        std::vector<double> raw_path_arc_length = this->calcPathArcLength(raw_path);
        // 采用的方法是梯度下降法
        // 首先确定最大迭代次数
        int max_iter = Config::smooth_iter_maximum_;
        // 当前迭代次数
        int iter_num = 0;
        // 开始迭代
        while (iter_num < max_iter) {
            // 路径计算损失
            // 路径上点的数量
            int path_length = smoothed_path.size();
            // 计算路径弧长
            std::vector<double> path_arc_lengths = this->calcPathArcLength(smoothed_path);
            // 计算起点与障碍物之间的距离
            std::vector<std::pair<float, float>> start_neighbors;
            std::vector<float> start_sq_distances;
            kd_tree.findKNeighbor(smoothed_path[0].x_, smoothed_path[0].y_, &start_neighbors, &start_sq_distances, 1);
            // 计算终点离障碍物之间的距离
            std::vector<std::pair<float, float>> goal_neighbors;
            std::vector<float> goal_sq_distances;
            kd_tree.findKNeighbor(smoothed_path[path_length - 1].x_, smoothed_path[path_length - 1].y_, &goal_neighbors, &goal_sq_distances, 1);
            // 初始化更新梯度
            Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(path_length, 2);
            // 首先计算障碍物损失梯度
            Eigen::MatrixXd obstacle_gradient = Eigen::MatrixXd::Zero(path_length, 2);
            if (Tools::isLarge(this->obs_weight_, 0.0)){
                for (int i = 0; i < path_length; i++) {
                    // 首先从路径中取出1个点
                    PathPlanningUtilities::Point2f point = smoothed_path[i];
                    // 计算障碍物梯度
                    double distance_param = this->calcObstacleDistanceParamAccordingToArcLength(sqrt(start_sq_distances[0]), sqrt(goal_sq_distances[0]), path_arc_lengths[path_length - 1], Config::influence_range_, path_arc_lengths[i], this->max_distance_);
                    // std::cout <<"sort " << i << ", distance param " << distance_param << std::endl;
                    // LOG(INFO) << "sort " << i << ", distance param " << distance_param;
                    Eigen::MatrixXd obstacle_gradient_item = this->calcObstacleItem(point, kd_tree, distance_param);
                    // 加入总梯度
                    obstacle_gradient.row(i) += obstacle_gradient_item;
                }
            }

            // 计算路程损失梯度
            Eigen::MatrixXd distance_gradient = Eigen::MatrixXd::Zero(path_length, 2);
            if (Tools::isLarge(this->dis_weight_, 0.0)){
                // 计算距离损失系数
                double coef = path_arc_lengths[path_arc_lengths.size() - 1] - raw_path_arc_length[raw_path_arc_length.size() - 1];
                for (int i = 1; i < path_length - 1; i++) {
                    // 首先从路径中取出3个点
                    Eigen::MatrixXd point_m1 = this->pointToArray(smoothed_path[i - 1]);
                    Eigen::MatrixXd point = this->pointToArray(smoothed_path[i]);
                    Eigen::MatrixXd point_p1 = this->pointToArray(smoothed_path[i + 1]);
                    // 计算路程梯度
                    Eigen::MatrixXd dis_gradient_item = this->calcDistanceItem(point_m1, point, point_p1, coef);
                    // 加入总梯度
                    distance_gradient.row(i) += dis_gradient_item;
                }
            }

            // 计算曲率带来的梯度
            Eigen::MatrixXd curvature_gradient = Eigen::MatrixXd::Zero(path_length, 2);
            // 记录曲率相关信息用以计算曲率变化率梯度
            std::vector<double> curvatures(path_length, 0.0);
            std::vector<Eigen::MatrixXd> curvature_items(path_length, Eigen::MatrixXd::Zero(3, 2));
            if (Tools::isLarge(this->cur_weight_, 0.0)) {
                for (int i = 1; i < path_length - 1; i++) {
                    // 首先从路径中取出3个点
                    Eigen::MatrixXd point_m1 = this->pointToArray(smoothed_path[i - 1]);
                    Eigen::MatrixXd point = this->pointToArray(smoothed_path[i]);
                    Eigen::MatrixXd point_p1 = this->pointToArray(smoothed_path[i + 1]);
                    // 计算曲率梯度
                    Eigen::MatrixXd cur_gradient_item = this->calcCurvatureItemTest(point_m1, point, point_p1);
                    // 加入总梯度
                    curvature_gradient.row(i - 1) += cur_gradient_item.row(0);
                    curvature_gradient.row(i) += cur_gradient_item.row(1);
                    curvature_gradient.row(i + 1) += cur_gradient_item.row(2);
                    curvatures[i] = this->calcCurvature(point_m1, point, point_p1);
                    curvature_items[i] = cur_gradient_item;
                }
            }

            // 计算曲率变化率带来的梯度
            Eigen::MatrixXd curvature_change_rate_gradient = Eigen::MatrixXd::Zero(path_length, 2);
            if (Tools::isLarge(this->cur_weight_, 0.0) && Tools::isLarge(this->cur_change_rate_weight_, 0.0)) {
                for (int i = 2; i < path_length - 2; i++) {
                    Eigen::MatrixXd curvature_change_rate_item = this->calcCurvatureChangeRateItem(i, curvatures, curvature_items);
                    curvature_change_rate_gradient.row(i - 2) += curvature_change_rate_item.row(0);
                    curvature_change_rate_gradient.row(i - 1) += curvature_change_rate_item.row(1);
                    curvature_change_rate_gradient.row(i) += curvature_change_rate_item.row(2);
                    curvature_change_rate_gradient.row(i + 1) += curvature_change_rate_item.row(3);
                    curvature_change_rate_gradient.row(i + 2) += curvature_change_rate_item.row(4);
                }
            }

            // 计算平滑带来的梯度
            Eigen::MatrixXd smoo_gradient = Eigen::MatrixXd::Zero(path_length, 2);
            if (Tools::isLarge(this->smoo_weight_, 0.0)) {
                for (int i = 1; i < path_length - 1; i++) {
                    // 首先从路径中取出3个点
                    Eigen::MatrixXd point_m1 = this->pointToArray(smoothed_path[i - 1]);
                    Eigen::MatrixXd point = this->pointToArray(smoothed_path[i]);
                    Eigen::MatrixXd point_p1 = this->pointToArray(smoothed_path[i + 1]);
                    // 计算平滑度梯度
                    Eigen::MatrixXd smoo_gradient_item = this->calcSmoothItem(point_m1, point, point_p1);
                    // 加入总梯度
                    smoo_gradient.row(i - 1) += smoo_gradient_item.row(0);
                    smoo_gradient.row(i) += smoo_gradient_item.row(1);
                    smoo_gradient.row(i + 1) += smoo_gradient_item.row(2);
                }
            }

            // 计算更新梯度
            gradient = this->obs_weight_ * obstacle_gradient + this->dis_weight_ * distance_gradient + this->cur_weight_ * curvature_gradient + this->cur_change_rate_weight_ * curvature_change_rate_gradient + this->smoo_weight_ * smoo_gradient;

            // 可选学习率
            int lr_length = 9;
            double lr_list[lr_length] = {0, 0.0005, 0.001, 0.005, 0.0075, 0.01, 0.02, 0.05, 0.1};

            // 确定学习率
            double lr = 0.0;

            // 计算学习率
            std::vector<double> values;
            for (int i = 0; i < lr_length; i++) {
                PathPlanningUtilities::Path predict_path;
                predict_path.resize(path_length);
                for (int j = 0; j < path_length; j++) {
                    predict_path[j].x_ = smoothed_path[j].x_ - lr_list[i] * gradient.coeff(j, 0);
                    predict_path[j].y_ = smoothed_path[j].y_ - lr_list[i] * gradient.coeff(j, 1);
                }
                // 计算损失
                double cost = 0.0;
                if (Tools::isLarge(this->obs_weight_, 0.0)) {
                    cost += this->obs_weight_ * this->calcObstacleCost(predict_path, kd_tree, this->max_distance_);
                }
                if (Tools::isLarge(this->dis_weight_, 0.0)) {
                    std::vector<double> predict_path_arc_lengths = this->calcPathArcLength(predict_path);
                    cost += this->dis_weight_ * this->calcDistanceCost(raw_path_arc_length[raw_path_arc_length.size() - 1], predict_path_arc_lengths[predict_path_arc_lengths.size() - 1]);
                }
                if (Tools::isLarge(this->cur_weight_, 0.0)) {
                    double tmp_store = this->cur_weight_ * this->calcCurvatureCost(predict_path);
                    cost += tmp_store;
                    if (this->obs_weight_ == 0.0) {
                        // LOG(INFO) << "in the no. " << iter_num << " with the lr is: " << lr_list[i] << " the cost caused by curvature is " << tmp_store; 
                    }
                }
                if (Tools::isLarge(this->cur_change_rate_weight_, 0.0)) {
                    cost += this->cur_change_rate_weight_ * this->calcCurvatureRateCost(predict_path);
                }
                if (Tools::isLarge(this->smoo_weight_, 0.0)) {
                    cost += this->smoo_weight_ * this->calcSmoothCost(predict_path);
                }
                values.push_back(cost);
            }
            // 得到损失最小的学习率
            lr = lr_list[std::min_element(values.begin(), values.end()) - values.begin()];


            // std::cout << "cost is " << *(std::min_element(values.begin(), values.end())) << ", lr is " << lr << std::endl;
            // LOG(INFO) << " in the no. " << iter_num << " cost is " << *(std::min_element(values.begin(), values.end())) << ", lr is " << lr;
            // if (Tools::isSmall(lr, 0.0000001)) {
            //     // 到达局部极小值
            //     break;
            // }
            
            // // Debug
            // if (iter_num >= 10 && values[0] < 0.1) {
            //     break;
            // }

            
            // 更新路径
            for (int i = 1; i < path_length - 1; i++) {
                smoothed_path[i].x_ -= lr * gradient.coeff(i, 0);
                smoothed_path[i].y_ -= lr * gradient.coeff(i, 1);
            }

            // 删除路径聚集点
            if (with_remove) {
                this->clusteredPointRemove(smoothed_path, path_arc_lengths[path_arc_lengths.size() - 1]);
                this->detachedPointAdd(smoothed_path, path_arc_lengths[path_arc_lengths.size() - 1]);
            }


            // Visualizing optimized path in each round
            visualization_msgs::MarkerArray optimizing_path_marker_array;
            optimizing_path_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(smoothed_path, VisualizationMethods::color(153.0 / 255.0, 51.0 / 255.0, 250.0 / 255.0, 1.0), 19500000, 0.1));
            publisher.publish(optimizing_path_marker_array);
            std::this_thread::sleep_for(std::chrono::milliseconds(150));

            // 迭代次数增加
            iter_num++;
        }
        return smoothed_path;
    }

    // 进行路径平滑(正式)
    PathPlanningUtilities::Path smoothing(const PathPlanningUtilities::Path &raw_path, const KDTree &kd_tree, bool with_remove = true) {
        // 路径平滑考虑到三个因素,离障碍物的距离,曲率大小和采样点间隔变化
        PathPlanningUtilities::Path smoothed_path = raw_path;
        std::vector<double> raw_path_arc_length = this->calcPathArcLength(raw_path);
        // 采用的方法是梯度下降法
        // 首先确定最大迭代次数
        int max_iter = Config::smooth_iter_maximum_;
        // 当前迭代次数
        int iter_num = 0;
        // 开始迭代
        while (iter_num < max_iter) {
            // 路径计算损失
            // 路径上点的数量
            int path_length = smoothed_path.size();
            // 计算路径弧长
            std::vector<double> path_arc_lengths = this->calcPathArcLength(smoothed_path);
            // 计算起点与障碍物之间的距离
            std::vector<std::pair<float, float>> start_neighbors;
            std::vector<float> start_sq_distances;
            kd_tree.findKNeighbor(smoothed_path[0].x_, smoothed_path[0].y_, &start_neighbors, &start_sq_distances, 1);
            // 计算终点离障碍物之间的距离
            std::vector<std::pair<float, float>> goal_neighbors;
            std::vector<float> goal_sq_distances;
            kd_tree.findKNeighbor(smoothed_path[path_length - 1].x_, smoothed_path[path_length - 1].y_, &goal_neighbors, &goal_sq_distances, 1);
            // 初始化更新梯度
            Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(path_length, 2);
            // 首先计算障碍物损失梯度
            Eigen::MatrixXd obstacle_gradient = Eigen::MatrixXd::Zero(path_length, 2);
            if (Tools::isLarge(this->obs_weight_, 0.0)){
                for (int i = 0; i < path_length; i++) {
                    // 首先从路径中取出1个点
                    PathPlanningUtilities::Point2f point = smoothed_path[i];
                    // 计算障碍物梯度
                    double distance_param = this->calcObstacleDistanceParamAccordingToArcLength(sqrt(start_sq_distances[0]), sqrt(goal_sq_distances[0]), path_arc_lengths[path_length - 1], Config::influence_range_, path_arc_lengths[i], this->max_distance_);
                    // std::cout <<"sort " << i << ", distance param " << distance_param << std::endl;
                    // LOG(INFO) << "sort " << i << ", distance param " << distance_param;
                    Eigen::MatrixXd obstacle_gradient_item = this->calcObstacleItem(point, kd_tree, distance_param);
                    // 加入总梯度
                    obstacle_gradient.row(i) += obstacle_gradient_item;
                }
            }

            // 计算路程损失梯度
            Eigen::MatrixXd distance_gradient = Eigen::MatrixXd::Zero(path_length, 2);
            if (Tools::isLarge(this->dis_weight_, 0.0)){
                // 计算距离损失系数
                double coef = path_arc_lengths[path_arc_lengths.size() - 1] - raw_path_arc_length[raw_path_arc_length.size() - 1];
                for (int i = 1; i < path_length - 1; i++) {
                    // 首先从路径中取出3个点
                    Eigen::MatrixXd point_m1 = this->pointToArray(smoothed_path[i - 1]);
                    Eigen::MatrixXd point = this->pointToArray(smoothed_path[i]);
                    Eigen::MatrixXd point_p1 = this->pointToArray(smoothed_path[i + 1]);
                    // 计算路程梯度
                    Eigen::MatrixXd dis_gradient_item = this->calcDistanceItem(point_m1, point, point_p1, coef);
                    // 加入总梯度
                    distance_gradient.row(i) += dis_gradient_item;
                }
            }

            // 计算曲率带来的梯度
            Eigen::MatrixXd curvature_gradient = Eigen::MatrixXd::Zero(path_length, 2);
            // 记录曲率相关信息用以计算曲率变化率梯度
            std::vector<double> curvatures(path_length, 0.0);
            std::vector<Eigen::MatrixXd> curvature_items(path_length, Eigen::MatrixXd::Zero(3, 2));
            if (Tools::isLarge(this->cur_weight_, 0.0)) {
                for (int i = 1; i < path_length - 1; i++) {
                    // 首先从路径中取出3个点
                    Eigen::MatrixXd point_m1 = this->pointToArray(smoothed_path[i - 1]);
                    Eigen::MatrixXd point = this->pointToArray(smoothed_path[i]);
                    Eigen::MatrixXd point_p1 = this->pointToArray(smoothed_path[i + 1]);
                    // 计算曲率梯度
                    Eigen::MatrixXd cur_gradient_item = this->calcCurvatureItemTest(point_m1, point, point_p1);
                    // 加入总梯度
                    curvature_gradient.row(i - 1) += cur_gradient_item.row(0);
                    curvature_gradient.row(i) += cur_gradient_item.row(1);
                    curvature_gradient.row(i + 1) += cur_gradient_item.row(2);
                    curvatures[i] = this->calcCurvature(point_m1, point, point_p1);
                    curvature_items[i] = cur_gradient_item;
                }
            }

            // 计算曲率变化率带来的梯度
            Eigen::MatrixXd curvature_change_rate_gradient = Eigen::MatrixXd::Zero(path_length, 2);
            if (Tools::isLarge(this->cur_weight_, 0.0) && Tools::isLarge(this->cur_change_rate_weight_, 0.0)) {
                for (int i = 2; i < path_length - 2; i++) {
                    Eigen::MatrixXd curvature_change_rate_item = this->calcCurvatureChangeRateItem(i, curvatures, curvature_items);
                    curvature_change_rate_gradient.row(i - 2) += curvature_change_rate_item.row(0);
                    curvature_change_rate_gradient.row(i - 1) += curvature_change_rate_item.row(1);
                    curvature_change_rate_gradient.row(i) += curvature_change_rate_item.row(2);
                    curvature_change_rate_gradient.row(i + 1) += curvature_change_rate_item.row(3);
                    curvature_change_rate_gradient.row(i + 2) += curvature_change_rate_item.row(4);
                }
            }

            // 计算平滑带来的梯度
            Eigen::MatrixXd smoo_gradient = Eigen::MatrixXd::Zero(path_length, 2);
            if (Tools::isLarge(this->smoo_weight_, 0.0)) {
                for (int i = 1; i < path_length - 1; i++) {
                    // 首先从路径中取出3个点
                    Eigen::MatrixXd point_m1 = this->pointToArray(smoothed_path[i - 1]);
                    Eigen::MatrixXd point = this->pointToArray(smoothed_path[i]);
                    Eigen::MatrixXd point_p1 = this->pointToArray(smoothed_path[i + 1]);
                    // 计算平滑度梯度
                    Eigen::MatrixXd smoo_gradient_item = this->calcSmoothItem(point_m1, point, point_p1);
                    // 加入总梯度
                    smoo_gradient.row(i - 1) += smoo_gradient_item.row(0);
                    smoo_gradient.row(i) += smoo_gradient_item.row(1);
                    smoo_gradient.row(i + 1) += smoo_gradient_item.row(2);
                }
            }

            // 计算更新梯度
            gradient = this->obs_weight_ * obstacle_gradient + this->dis_weight_ * distance_gradient + this->cur_weight_ * curvature_gradient + this->cur_change_rate_weight_ * curvature_change_rate_gradient + this->smoo_weight_ * smoo_gradient;

            // 可选学习率
            int lr_length = 9;
            double lr_list[lr_length] = {0, 0.0005, 0.001, 0.005, 0.0075, 0.01, 0.02, 0.05, 0.1};

            // 确定学习率
            double lr = 0.0;

            // 计算学习率
            std::vector<double> values;
            for (int i = 0; i < lr_length; i++) {
                PathPlanningUtilities::Path predict_path;
                predict_path.resize(path_length);
                for (int j = 0; j < path_length; j++) {
                    predict_path[j].x_ = smoothed_path[j].x_ - lr_list[i] * gradient.coeff(j, 0);
                    predict_path[j].y_ = smoothed_path[j].y_ - lr_list[i] * gradient.coeff(j, 1);
                }
                // 计算损失
                double cost = 0.0;
                if (Tools::isLarge(this->obs_weight_, 0.0)) {
                    cost += this->obs_weight_ * this->calcObstacleCost(predict_path, kd_tree, this->max_distance_);
                }
                if (Tools::isLarge(this->dis_weight_, 0.0)) {
                    std::vector<double> predict_path_arc_lengths = this->calcPathArcLength(predict_path);
                    cost += this->dis_weight_ * this->calcDistanceCost(raw_path_arc_length[raw_path_arc_length.size() - 1], predict_path_arc_lengths[predict_path_arc_lengths.size() - 1]);
                }
                if (Tools::isLarge(this->cur_weight_, 0.0)) {
                    double tmp_store = this->cur_weight_ * this->calcCurvatureCost(predict_path);
                    cost += tmp_store;
                    if (this->obs_weight_ == 0.0) {
                        // LOG(INFO) << "in the no. " << iter_num << " with the lr is: " << lr_list[i] << " the cost caused by curvature is " << tmp_store; 
                    }
                }
                if (Tools::isLarge(this->cur_change_rate_weight_, 0.0)) {
                    cost += this->cur_change_rate_weight_ * this->calcCurvatureRateCost(predict_path);
                }
                if (Tools::isLarge(this->smoo_weight_, 0.0)) {
                    cost += this->smoo_weight_ * this->calcSmoothCost(predict_path);
                }
                values.push_back(cost);
            }
            // 得到损失最小的学习率
            lr = lr_list[std::min_element(values.begin(), values.end()) - values.begin()];


            // std::cout << "cost is " << *(std::min_element(values.begin(), values.end())) << ", lr is " << lr << std::endl;
            // LOG(INFO) << " in the no. " << iter_num << " cost is " << *(std::min_element(values.begin(), values.end())) << ", lr is " << lr;
            if (Tools::isSmall(lr, 0.0000001)) {
                // 到达局部极小值
                break;
            }
            
            // // Debug
            // if (iter_num >= 10 && values[0] < 0.1) {
            //     break;
            // }

            
            // 更新路径
            for (int i = 1; i < path_length - 1; i++) {
                smoothed_path[i].x_ -= lr * gradient.coeff(i, 0);
                smoothed_path[i].y_ -= lr * gradient.coeff(i, 1);
            }

            // 删除路径聚集点
            if (with_remove) {
                this->clusteredPointRemove(smoothed_path, path_arc_lengths[path_arc_lengths.size() - 1]);
                this->detachedPointAdd(smoothed_path, path_arc_lengths[path_arc_lengths.size() - 1]);
            }

            // 迭代次数增加
            iter_num++;
        }
        return smoothed_path;
    }

    // 角度滤波(双边滤波器)
    PathPlanningUtilities::Curve yawBlurring(const PathPlanningUtilities::Path &path, double sigma_s, double sigma_yaw) {
        // 最终结果
        std::vector<double> blurred_yaw;
        blurred_yaw.resize(path.size());
        // 首先计算路径每个点的arc length
        std::vector<double> arc_lengths;
        arc_lengths.push_back(0.0);
        for (size_t i = 1; i < path.size(); i++) {
            double distance = PathPlanningUtilities::calcDistance(path[i], path[i - 1]);
            arc_lengths.push_back(arc_lengths[arc_lengths.size() - 1] + distance);
        }
        assert(arc_lengths.size() == path.size());
        // 计算每个点的朝向
        std::vector<double> yaws;
        for (size_t i = 0; i < path.size() - 1; i++) {
            double yaw = atan2(path[i + 1].y_ - path[i].y_, path[i + 1].x_ - path[i].x_);
            yaws.push_back(yaw);
        }
        double final_yaw = yaws[yaws.size()-1] + (yaws[yaws.size()-1] - yaws[yaws.size()-2]) / (arc_lengths[arc_lengths.size()-2] - arc_lengths[arc_lengths.size()-3]) * (arc_lengths[arc_lengths.size()-1] - arc_lengths[arc_lengths.size()-2]);
        yaws.push_back(final_yaw);
        assert(yaws.size() == path.size());
        // 确定双边滤波器的窗长度
        double window_size = sigma_s * 5.0;
        // 开始进行滤波
        // 遍历路径每个点
        for (int i = 0; i < static_cast<int>(path.size()); i++) {
            // 求以当前点为中点,在窗内的点
            std::vector<int> neighbor_index;
            neighbor_index.push_back(i);
            bool p_enable = true;
            bool m_enable = true;
            for (int j = 1; j < static_cast<int>(path.size()); j++) {
                // 判断邻居是否有效
                if (i + j >=0 && i + j < static_cast<int>(path.size()) && p_enable) {
                    // 邻居有效,计算当前点到邻居的距离是否小于窗长
                    double distance = PathPlanningUtilities::calcDistance(path[i], path[i + j]);
                    if (Tools::isSmall(distance, window_size)) {
                        neighbor_index.push_back(i + j);
                    } else {
                        p_enable = false;
                    }
                } else {
                    p_enable = false;
                }
                // 判断邻居是否有效
                if (i - j >=0 && i - j < static_cast<int>(path.size()) && m_enable) {
                    // 邻居有效,计算当前点到邻居的距离是否小于窗长
                    double distance = PathPlanningUtilities::calcDistance(path[i], path[i - j]);
                    if (Tools::isSmall(distance, window_size)) {
                        neighbor_index.push_back(i - j);
                    } else {
                        m_enable = false;
                    }
                } else {
                    m_enable = false;
                }
                // 判断是否结束循环
                if (!p_enable && !m_enable) {
                    break;
                }
            }
            // 将邻居进行排序
            std::sort(neighbor_index.begin(), neighbor_index.end());
            assert(neighbor_index.size() > 0);
            // 找到邻居后,进行双边滤波
            double I = 0.0;
            double W = 0.0;
            for (auto index: neighbor_index) {
                double gs = this->gaussian(arc_lengths[index], arc_lengths[i], sigma_s);
                double gy = this->gaussian(yaws[index], yaws[i], sigma_yaw);
                double weight = gs * gy;
                I += yaws[index] * weight;
                W += weight;
            }
            blurred_yaw[i] = I / W;
        }
        // 构建返回值
        PathPlanningUtilities::Curve result;
        result.resize(path.size());
        for (size_t i = 0; i < path.size(); i++) {
            result[i].position_ = path[i];
            result[i].theta_ = blurred_yaw[i];
            if (i > 1) {
                result[i].kappa_ = (blurred_yaw[i] - blurred_yaw[i - 1]) / (arc_lengths[i] - arc_lengths[i - 1]);
            }
        }
        result[0].kappa_ = result[1].kappa_;
        return result;
    }

 private:
    // 路径转矩阵
    Eigen::MatrixXd pathToArray(const PathPlanningUtilities::Path &path) {
        Eigen::MatrixXd result(path.size(), 2);
        for (size_t i = 0; i < path.size(); i++) {
            result.row(i) << path[i].x_, path[i].y_;
        }
        return result;
    }

    // 点转矩阵
    Eigen::MatrixXd pointToArray(const PathPlanningUtilities::Point2f &point) {
        Eigen::MatrixXd result(1, 2);
        result.row(0) << point.x_, point.y_;
        return result;
    }

    // 计算障碍物梯度
    Eigen::MatrixXd calcObstacleItem(const PathPlanningUtilities::Point2f &point, const KDTree &kd_tree, double expected_distance) {
        // 找出最近障碍物和最近距离
        std::vector<std::pair<float, float>> nearest_obstacle;
        std::vector<float> sq_distance;
        int is_find = kd_tree.findKNeighbor(point.x_, point.y_, &nearest_obstacle, &sq_distance, 1);
        if (is_find < 0) {
            // 没有找到
            assert(false);
        }
        // 计算梯度
        Eigen::MatrixXd point_array = this->pointToArray(point);
        Eigen::MatrixXd nearest_obstacle_point(1, 2);
        nearest_obstacle_point << nearest_obstacle[0].first, nearest_obstacle[0].second;
        double min_distance = sqrt(sq_distance[0]);
        // std::cout << "point" << point_array << std::endl;
        // std::cout << "obs" << nearest_obstacle_point << std::endl;
        // std::cout << "min dis" << min_distance << std::endl;
        Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(1, 2);
        gradient = - 1.0 / expected_distance * (exp(-min_distance / expected_distance) / (min_distance / expected_distance) + exp(-min_distance / expected_distance) / pow(min_distance / expected_distance, 2)) / min_distance * (point_array - nearest_obstacle_point);
        
        // std::cout << "grad" << gradient << std::endl;
        // LOG(INFO) << "obstacle gradient" << gradient;
        return gradient;
    }

    // 计算平滑度梯度
    Eigen::MatrixXd calcSmoothItem(const Eigen::MatrixXd &point_m1, const Eigen::MatrixXd &point, const Eigen::MatrixXd &point_p1) {
        Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(3, 2);
        gradient.row(0) = 0.0 * 2.0 * (point_m1 + point_p1 - 2.0 * point);
        gradient.row(1) = -1.0 * (point_m1 + point_p1 - 2.0 * point);
        gradient.row(2) = 0.0 * 2.0 * (point_m1 + point_p1 - 2.0 * point);
        // LOG(INFO) << "smooth gradient values in the first row " << gradient(0, 0) << " " << gradient(0, 1);
        // LOG(INFO) << "smooth gradient values in the second row " << gradient(1, 0) << " " << gradient(1, 1);
        // LOG(INFO) << "smooth gradient values in the third row " << gradient(2, 0) << " " << gradient(2, 1);
        return gradient;
    }

    // 计算曲率梯度
    Eigen::MatrixXd calcCurvatureItem(const Eigen::MatrixXd &point_m1, const Eigen::MatrixXd &point, const Eigen::MatrixXd &point_p1) {
        Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(3, 2);
        // 计算前一个点到中间点的向量
        Eigen::MatrixXd delta_x = point - point_m1;
        double abs_delta_x = delta_x.norm();
        // 计算中间点到后一个点的向量
        Eigen::MatrixXd delta_px = point_p1 - point;
        double abs_delta_px = delta_px.norm();
        // 计算角度变化量
        double delta_phi = Tools::safeAcos((delta_x * delta_px.transpose()).value() / (abs_delta_x * abs_delta_px));
        assert(abs_delta_x > 0.0 && abs_delta_px > 0.0);
        // 计算曲率
        double curvature = delta_phi / abs_delta_x;
        // std::cout << "curvature" << curvature << std::endl;
        // 计算梯度
        // std::cout << "delta_phi" << delta_phi << std::endl;
        // std::cout << "sqrt(1 - pow(cos(delta_phi), 2.0))" << sqrt(1 - pow(cos(delta_phi), 2.0)) << std::endl;
        if (Tools::isLarge(fabs(curvature), this->max_curvature_)) {
            // double u = 2. * delta_phi * -1. / sqrt(1 - pow(cos(delta_phi), 2.0));
            // Eigen::MatrixXd p1 = this->ort(delta_x, -delta_px) / (abs_delta_x * abs_delta_px);
            // Eigen::MatrixXd p2 = this->ort(-delta_px, delta_x) / (abs_delta_x * abs_delta_px);
            // gradient.row(0) = u * p1;
            // gradient.row(1) = u * (-p1 - p2);
            // gradient.row(2) = u * p2;
            double u = 1. / abs_delta_x * -1. / sqrt(1 - pow(cos(delta_phi), 2.0));
            Eigen::MatrixXd p1 = this->ort(delta_x, -delta_px) / (abs_delta_x * abs_delta_px);
            Eigen::MatrixXd p2 = this->ort(-delta_px, delta_x) / (abs_delta_x * abs_delta_px);
            gradient.row(0) = 0.01 * 2.0 * curvature * (u * p2 + delta_phi / pow(abs_delta_x, 3) * delta_x);
            gradient.row(1) = 0.01 * 2.0 * curvature * (u * (-p1 - p2) - delta_phi / pow(abs_delta_x, 3) * delta_x);
            gradient.row(2) = 0.01 * 2.0 * curvature * (u * p1);
            // std::cout << "gradient" << gradient << std::endl;
        }

        return gradient;
    }
    
    // 计算长度梯度
    Eigen::MatrixXd calcDistanceItem(const Eigen::MatrixXd &point_m1, const Eigen::MatrixXd &point, const Eigen::MatrixXd &point_p1, double coef) {
        Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(1, 2);
        // 计算前一个点到中间点的向量
        Eigen::MatrixXd delta_x = point - point_m1;
        double abs_delta_x = delta_x.norm();
        // 计算中间点到后一个点的向量
        Eigen::MatrixXd delta_px = point_p1 - point;
        double abs_delta_px = delta_px.norm();
        gradient = 2.0 * coef * (delta_x / abs_delta_x - delta_px / abs_delta_px);
        return gradient;
    }

    // 计算曲率变化率梯度
    Eigen::MatrixXd calcCurvatureChangeRateItem(int center_point_index, const std::vector<double> &curvatures, const std::vector<Eigen::MatrixXd> &curvature_items) {
        Eigen::MatrixXd cur_change_rate_item = Eigen::MatrixXd::Zero(5, 2);
        double pre_curvature = curvatures[center_point_index - 1];
        double this_curvature = curvatures[center_point_index];
        double next_curvature = curvatures[center_point_index + 1];
        Eigen::MatrixXd pre_cur_item = curvature_items[center_point_index - 1];
        Eigen::MatrixXd this_cur_item = curvature_items[center_point_index];
        Eigen::MatrixXd next_cur_item = curvature_items[center_point_index + 1];
        double coef = 2.0 * (2.0 * this_curvature - pre_curvature - next_curvature);
        cur_change_rate_item.row(0) = -pre_cur_item.row(0);
        cur_change_rate_item.row(1) = -pre_cur_item.row(1) + 2.0 * this_cur_item.row(0);
        cur_change_rate_item.row(2) = -pre_cur_item.row(2) + 2.0 * this_cur_item.row(1) - next_cur_item.row(0);
        cur_change_rate_item.row(3) = 2.0 * this_cur_item.row(2) - next_cur_item.row(1);
        cur_change_rate_item.row(4) = -next_cur_item.row(2);
        cur_change_rate_item *= coef / 1000.0;
        // LOG(INFO) << "gradient values in the first row " << cur_change_rate_item(0, 0) << " " << cur_change_rate_item(0, 1);
        // LOG(INFO) << "gradient values in the second row " << cur_change_rate_item(1, 0) << " " << cur_change_rate_item(1, 1);
        // LOG(INFO) << "gradient values in the third row " << cur_change_rate_item(2, 0) << " " << cur_change_rate_item(2, 1);
        // LOG(INFO) << "gradient values in the fourth row " << cur_change_rate_item(3, 0) << " " << cur_change_rate_item(3, 1);
        // LOG(INFO) << "gradient values in the fifth row " << cur_change_rate_item(4, 0) << " " << cur_change_rate_item(4, 1);
        return cur_change_rate_item;
    }


    // 计算障碍物损失(exp(-x)/x)
    double calcObstacleCost(const PathPlanningUtilities::Path &path, const KDTree &kd_tree, double expected_distance) {
        double cost = 0.0;
        // 遍历路径中的点
        for (size_t i = 0; i < path.size(); i++) {
            // 计算kd树最与当前点最近的距离
            std::vector<std::pair<float, float>> nearest_obses;
            std::vector<float> sq_distances;
            // TOFIX, cannot find the source of nan, just avoid error
            if (std::isnan(path[i].x_) || std::isnan(path[i].y_) || path[i].x_ >= DBL_MAX || path[i].x_ <= -DBL_MAX || path[i].y_ >= DBL_MAX || path[i].y_ <= -DBL_MAX) {
                continue;
            }
            kd_tree.findKNeighbor(path[i].x_, path[i].y_, &nearest_obses, &sq_distances, 1);
            // if (Tools::isSmall(sqrt(sq_distances[0]), this->max_distance_)) {
            //     // 如果距离小于最大距离
            //     cost += 1.0 / sq_distances[0];
            // }
            cost += exp(-1.0 / expected_distance * sqrt(sq_distances[0])) / (1.0 / expected_distance * sqrt(sq_distances[0]));
        }
        return cost;
    }

    // 计算平滑度损失
    double calcSmoothCost(const PathPlanningUtilities::Path &path) {
        double cost = 0.0;
        for (size_t i = 1; i < path.size() - 1; i++) {
            Eigen::MatrixXd point_m1 = this->pointToArray(path[i - 1]);
            Eigen::MatrixXd point = this->pointToArray(path[i]);
            Eigen::MatrixXd point_p1 = this->pointToArray(path[i + 1]);
            Eigen::MatrixXd delta_x = point - point_m1;
            Eigen::MatrixXd delta_px = point_p1 - point;
            cost += ((delta_px - delta_x) * (delta_px - delta_x).transpose()).value();
        }
        return cost;
    }

    // 计算曲率损失
    double calcCurvatureCost(const PathPlanningUtilities::Path &path) {
        double cost = 0.0;
        for (size_t i = 1; i < path.size() - 1; i++) {
            Eigen::MatrixXd point_m1 = this->pointToArray(path[i - 1]);
            Eigen::MatrixXd point = this->pointToArray(path[i]);
            Eigen::MatrixXd point_p1 = this->pointToArray(path[i + 1]);
            Eigen::MatrixXd delta_x = point - point_m1;
            Eigen::MatrixXd delta_px = point_p1 - point;
            // std::cout << "path[i - 1]" << path[i - 1].x_ << ", " << path[i - 1].y_ << std::endl;
            // std::cout << "path[i]" << path[i].x_ << ", " << path[i].y_ << std::endl;
            // std::cout << "path[i + 1]" << path[i + 1].x_ << ", " << path[i + 1].y_ << std::endl;
            // std::cout << "point_m1" << point_m1 << std::endl;
            // std::cout << "point" << point << std::endl;
            // std::cout << "point_p1" << point_p1 << std::endl;
            // std::cout << "delta_x" << delta_x << std::endl;
            // std::cout << "delta_px" << delta_px << std::endl;
            double abs_delta_x = delta_x.norm();
            double abs_delta_px = delta_px.norm();
            // std::cout << "abs_delta_x" << abs_delta_x << "abs_delta_px" << abs_delta_px << std::endl;
            // std::cout << "(delta_x * delta_px.transpose()).value()" << (delta_x * delta_px.transpose()).value() << std::endl;
            // std::cout << "(delta_x * delta_px.transpose()).value() / (abs_delta_x * abs_delta_px)" << (delta_x * delta_px.transpose()).value() / (abs_delta_x * abs_delta_px) << std::endl;
            // 得到角度偏移量
            double delta_phi = Tools::safeAcos((delta_x * delta_px.transpose()).value() / (abs_delta_x * abs_delta_px));
            // std::cout << "delta_phi" << delta_phi << std::endl;
            // 计算曲率
            assert(abs_delta_x > 0.0 && abs_delta_px > 0.0);
            double curvature = delta_phi / abs_delta_x;
            if (Tools::isLarge(fabs(curvature), this->max_curvature_)) {

                cost += (curvature * curvature - this->max_curvature_ * this->max_curvature_);
            }
        }
        return cost;
    }

    // 计算距离长度损失
    double calcDistanceCost(double expected_arc_length, double arc_length) {
        double cost = (arc_length - expected_arc_length) * (arc_length - expected_arc_length);
        return cost;
    }

    // 计算曲率变化率损失
    double calcCurvatureRateCost(const PathPlanningUtilities::Path &path) {
        double cost = 0.0;
        // 计算曲率
        std::vector<double> curvatures(path.size(), 0.0);
        for (size_t i = 1; i < path.size() - 1; i++) {
            Eigen::MatrixXd point_m1 = this->pointToArray(path[i - 1]);
            Eigen::MatrixXd point = this->pointToArray(path[i]);
            Eigen::MatrixXd point_p1 = this->pointToArray(path[i + 1]);
            double curvature = this->calcCurvature(point_m1, point, point_p1);
            curvatures[i] = curvature;
        }
        // 计算曲率变化率
        for (size_t j = 2; j < path.size() - 2; j++) {
            double cost_item = (2.0 * curvatures[j] - curvatures[j - 1] - curvatures[j + 1]);
            cost += cost_item;
        }
        return cost;
    }

    // 判断路径上的点是否出现聚集,如果出现,则删除聚集点
    void clusteredPointRemove(PathPlanningUtilities::Path &path, double arc_length) {
        // 首先计算路径上点间距的平均值和标准差
        double dis_mu = arc_length / (path.size() - 1);
        double dis_sigma = 0.0;
        // 记录距离
        std::vector<double> distances;
        for (size_t i = 0; i < path.size() - 1; i++) {
            double distance = PathPlanningUtilities::calcDistance(path[i], path[i + 1]);
            distances.push_back(distance);
            dis_sigma += (distance - dis_mu) * (distance - dis_mu);
        }
        dis_sigma = sqrt(dis_sigma / (path.size() - 1));
        // std::cout << "mean: " << dis_mu << ", deviation: " << dis_sigma << std::endl;
        // LOG(INFO) << "mean: " << dis_mu << ", deviation: " << dis_sigma;
        // 根据标准差和平均值进行删除点(第一个点和最后一个点不变)
        std::vector<size_t> delete_indexes;
        for (size_t i = 1; i < path.size() - 1; i++) {
            double distance_before = distances[i - 1];
            double distance_after = distances[i];
            if (Tools::isSmall(distance_before, dis_mu - dis_sigma) || Tools::isSmall(distance_after, dis_mu - dis_sigma)) {
                if (std::find(delete_indexes.begin(), delete_indexes.end(), i - 1) == delete_indexes.end()) {
                    delete_indexes.push_back(i);
                }
            }
        }
        // 进行删除
        for (int i = delete_indexes.size() - 1;i >= 0; i--) {
            // std::cout << "delete cluster point" << std::endl;
            PathPlanningUtilities::Path::iterator it = path.begin() + delete_indexes[i];
            path.erase(it);
        }
        // std::cout << "after remove path length " << path.size() << std::endl;
    }

    // 判断路径上的点是否出现偏离,如果出现,则增加点
    void detachedPointAdd(PathPlanningUtilities::Path &path, double arc_length) {
        // 首先计算路径上点间距的平均值和标准差
        double dis_mu = arc_length / (path.size() - 1);
        double dis_sigma = 0.0;
        // 记录距离
        std::vector<double> distances;
        for (size_t i = 0; i < path.size() - 1; i++) {
            double distance = PathPlanningUtilities::calcDistance(path[i], path[i + 1]);
            distances.push_back(distance);
            dis_sigma += (distance - dis_mu) * (distance - dis_mu);
        }
        dis_sigma = sqrt(dis_sigma / (path.size() - 1));
        // 根据标准差和平均值进行加点
        std::vector<size_t> add_indexes;
        for (size_t i = 1; i < path.size(); i++) {
            double distance_before = distances[i - 1];
            if (Tools::isLarge(distance_before, dis_mu + dis_sigma)) {
                add_indexes.push_back(i);
            }
        }
        // 开始进行加点
        PathPlanningUtilities::Path new_path;
        for(size_t i = 0; i < path.size();) {
            // 判断i是不是要加点
            if (add_indexes.size() > 0) {
                if (i == add_indexes[0]) {
                    // 要加点
                    // 计算中心点
                    PathPlanningUtilities::Point2f new_point;
                    new_point.x_ = (path[i - 1].x_ + path[i].x_) * 0.5;
                    new_point.y_ = (path[i - 1].y_ + path[i].y_) * 0.5;
                    new_path.push_back(new_point);
                    // 删除添加点
                    add_indexes.erase(add_indexes.begin());
                } else {
                    // 不用加点
                    new_path.push_back(path[i]);
                    i++;
                }
            } else {
                new_path.push_back(path[i]);
                i++;
            }
        }
        path = new_path;
    }


    // 计算向量垂直
    Eigen::MatrixXd ort(const Eigen::MatrixXd &vector1, const Eigen::MatrixXd &vector2) {
        return vector1 - (vector1 * vector2.transpose()).value() * vector2 / (vector2.norm() * vector2.norm());
    }

    // 高斯函数
    double gaussian(double x, double mu, double sigma) {
        return (1.0 / sqrt(2.0 * PI * sigma * sigma)) * exp(- (x - mu) * (x - mu) / (2.0 * sigma * sigma));
    }

    // 计算路径的弧长
    std::vector<double> calcPathArcLength(const PathPlanningUtilities::Path &path) {
        double total_distance = 0.0;
        std::vector<double> result;
        result.push_back(total_distance);
        for (size_t i = 0;i < path.size() - 1; i++) {
            total_distance += PathPlanningUtilities::calcDistance(path[i], path[i + 1]);
            result.push_back(total_distance);
        }
        return result;
    }

    // 障碍物距离影响参数随弧长的变化
    double calcObstacleDistanceParamAccordingToArcLength(double start_distance, double goal_distance, double total_arc_length, double influence_range, double current_arc_lenth, double max_distance) {
        start_distance = std::min(0.05 * start_distance, max_distance);
        goal_distance = std::min(0.05 * goal_distance, max_distance);
        // 判断总弧长是否大于两倍影响距离
        if (Tools::isLarge(total_arc_length, 2.0 * influence_range)) {
            // 大于两倍距离
            if (Tools::isSmall(current_arc_lenth, influence_range)) {
                // 起点影响范围内
                // LOG(INFO) << "Influenced by start point, parameter is: " << current_arc_lenth / influence_range * (max_distance - start_distance) + start_distance;
                return current_arc_lenth / influence_range * (max_distance - start_distance) + start_distance;
            } else if (Tools::isLarge(current_arc_lenth, total_arc_length - influence_range)) {
                // 终点影响范围内
                // LOG(INFO) << "Influenced by end point, parameter is: " << (current_arc_lenth - total_arc_length) / influence_range * (goal_distance - max_distance) + goal_distance;
                // return (current_arc_lenth - total_arc_length) / influence_range * (goal_distance - max_distance) + goal_distance;
                // remove the influence from target point
                return max_distance;
            } else {
                // 无影响范围内
                // LOG(INFO) << "No influence, parameter is: " << max_distance;
                return max_distance;
            }
        } else {
            // 小于两倍距离
            // 始终处于起点或终点的影响范围内
            LOG(INFO) << "Influenced by start and end point, parameter is: " << current_arc_lenth / total_arc_length * (goal_distance - start_distance) + start_distance;
            return current_arc_lenth / total_arc_length * (goal_distance - start_distance) + start_distance;
        }
    }

    // The codes below is for debugging the new calculation method for curvature gradient item
    // construct matrix
    Eigen::MatrixXd valuesToArray(double value_1, double value_2) {
        Eigen::MatrixXd result(1, 2);
        result.row(0) << value_1, value_2;
        return result;
    }

    // calculate curvature gradient
    Eigen::MatrixXd calcCurvatureItemTest(const Eigen::MatrixXd &point_m1, const Eigen::MatrixXd &point, const Eigen::MatrixXd &point_p1) {
        Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(3, 2);
        // calculate curvature and judge whether need to optimize
        double curvature = this->calcCurvature(point_m1, point, point_p1);
        if ((curvature < this->max_curvature_ && curvature >= 0.0) || (curvature <= 0.0 && curvature > -1.0 * this->max_curvature_)) {
            return gradient;
        }
        // std::cout << "start calculate gradient item" << std::endl;
        // get the x, y values
        double pre_point_x = point_m1(0);
        double pre_point_y = point_m1(1);
        double this_point_x = point(0);
        double this_point_y = point(1);
        double next_point_x = point_p1(0);
        double next_point_y = point_p1(1);
        // calculate intermediate values
        double a_1 = (next_point_x - pre_point_x) / 2.0;
        double b_1 = (next_point_y - pre_point_y) / 2.0;
        double a_2 = (next_point_x + pre_point_x) / 2.0 - this_point_x;
        double b_2 = (next_point_y + pre_point_y) / 2.0 - this_point_y;
        // std::cout << a_1 << a_2 << b_1 << b_2 <<std::endl;
        // calculate gradient portion
        Eigen::MatrixXd delta_k_pre_point = this->valuesToArray(((-1.0 * b_2 - b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 1.5) + 3.0 * (a_1 * b_2 - a_2 * b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 0.5) * a_1) / pow(pow(a_1, 2) + pow(b_1, 2), 3), ((a_1 + a_2) * pow(pow(a_1, 2) + pow(b_1, 2), 1.5) + 3.0 * (a_1 * b_2 - a_2 * b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 0.5) * b_1) / pow(pow(a_1, 2) + pow(b_1, 2), 3));
        Eigen::MatrixXd delta_k_this_point = this->valuesToArray((next_point_y - pre_point_y) / pow(pow(a_1, 2) + pow(b_1, 2), 1.5), (pre_point_x - next_point_x) / pow(pow(a_1, 2) + pow(b_1, 2), 1.5));
        Eigen::MatrixXd delta_k_next_point = this->valuesToArray(((b_2 - b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 1.5) - 3.0 * (a_1 * b_2 - a_2 * b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 0.5) * a_1) / pow(pow(a_1, 2) + pow(b_1, 2), 3), ((a_1 - a_2) * pow(pow(a_1, 2) + pow(b_1, 2), 1.5) - 3.0 * (a_1 * b_2 - a_2 * b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 0.5) * b_1) / pow(pow(a_1, 2) + pow(b_1, 2), 3));
        // gradient.row(0) = curvature * delta_k_pre_point * ((0.01 * (delta_k_pre_point.norm() / delta_k_this_point.norm())) / (fabs(curvature) * delta_k_pre_point.norm()));
        // gradient.row(1) = curvature * delta_k_this_point * (0.01 / (fabs(curvature) * delta_k_this_point.norm()));
        // gradient.row(2) = curvature * delta_k_next_point * ((0.01 * (delta_k_next_point.norm() / delta_k_this_point.norm())) / (fabs(curvature) * delta_k_next_point.norm()));


        gradient.row(0) = 2.0 * curvature * delta_k_pre_point / (1000.0 * sqrt(4.0 * curvature * curvature + 8.0));
        gradient.row(1) = 2.0 * curvature * delta_k_this_point / (1000.0 * sqrt(4.0 * curvature * curvature + 8.0));
        gradient.row(2) = 2.0 * curvature * delta_k_next_point / (1000.0 * sqrt(4.0 * curvature * curvature + 8.0));

        // LOG(INFO) << "curvature gradient values in the first row " << gradient(0, 0) << " " << gradient(0, 1);
        // LOG(INFO) << "curvature gradient values in the second row " << gradient(1, 0) << " " << gradient(1, 1);
        // LOG(INFO) << "curvature gradient values in the third row " << gradient(2, 0) << " " << gradient(2, 1);
        return gradient;
    }

    // calculate curvature
    double calcCurvature(const Eigen::MatrixXd &point_m1, const Eigen::MatrixXd &point, const Eigen::MatrixXd &point_p1) {
        // get the x, y values
        double pre_point_x = point_m1(0);
        double pre_point_y = point_m1(1);
        double this_point_x = point(0);
        double this_point_y = point(1);
        double next_point_x = point_p1(0);
        double next_point_y = point_p1(1);
        // calculate intermediate values
        double a_1 = (next_point_x - pre_point_x) / 2.0;
        double b_1 = (next_point_y - pre_point_y) / 2.0;
        double a_2 = (next_point_x + pre_point_x) / 2.0 - this_point_x;
        double b_2 = (next_point_y + pre_point_y) / 2.0 - this_point_y;
        // calculate curvature
        double k = (2.0 * (a_1 * b_2 - a_2 * b_1)) / (pow(pow(a_1, 2) + pow(b_1, 2), 1.5));
        return k;
    }

    double obs_weight_;  // 障碍物损失权重
    double cur_weight_;  // 曲率损失权重
    double smoo_weight_;  // 平滑损失权重
    double dis_weight_;  // 距离损失权重
    double cur_change_rate_weight_; // 曲率变化率损失
    double obs_max_distance_;  // 离障碍物最大距离
    double max_curvature_;  // 最大曲率
    double max_distance_;  // 离障碍物最大距离
};

// DEBUG: optimize curvature
class OptimizeCurvature {
 public:
    // Constructor
    OptimizeCurvature(double max_curvature = 2.4) {
        this->max_curvature_ = max_curvature;
    }
    // Destructor
    ~OptimizeCurvature() {}

    // optimzie function
    PathPlanningUtilities::Path optimize(const PathPlanningUtilities::Path &raw_path) {
        PathPlanningUtilities::Path optimized_path = raw_path;
        int max_iter = 100;
        int iter_num = 0;
        while (iter_num < max_iter) {
            // 计算路径弧长
            std::vector<double> path_arc_lengths = this->calcPathArcLength(optimized_path);
            int path_length = optimized_path.size();
            // initiate the gradient
            Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(path_length, 2);
            Eigen::MatrixXd curvature_gradient = Eigen::MatrixXd::Zero(path_length, 2);
            std::vector<double> curvature_store = std::vector<double> (path_length, 0.0);
            for (int i = 1; i < path_length - 1; i++) {
                // 首先从路径中取出3个点
                Eigen::MatrixXd point_m1 = this->pointToArray(optimized_path[i - 1]);
                Eigen::MatrixXd point = this->pointToArray(optimized_path[i]);
                Eigen::MatrixXd point_p1 = this->pointToArray(optimized_path[i + 1]);
                auto &tmp_curvature = curvature_store[i];
                // 计算曲率梯度
                Eigen::MatrixXd cur_gradient_item = this->calcCurvatureItem(point_m1, point, point_p1, tmp_curvature);
                // 加入总梯度
                curvature_gradient.row(i - 1) += cur_gradient_item.row(0);
                curvature_gradient.row(i) += cur_gradient_item.row(1);
                curvature_gradient.row(i + 1) += cur_gradient_item.row(2);
            }
            LOG(INFO) << "the most curvature in the path is:" << *std::max_element(curvature_store.begin(), curvature_store.end());
            std::cout << "the most curvature in the path is:" << *std::max_element(curvature_store.begin(), curvature_store.end()) << std::endl;
            // if the curvature has meet the requirements, stop the circle
            if (*std::max_element(curvature_store.begin(), curvature_store.end()) < this->max_curvature_) {
                break;
            }
            gradient = curvature_gradient;

            // calculate the best for the circle
            // 可选学习率
            int lr_length = 7;
            double lr_list[lr_length] = {0, 0.0005, 0.001, 0.005, 0.01, 0.05, 0.1};

            // 确定学习率
            double lr = 0.0;

            // 计算学习率
            std::vector<double> values;
            for (int i = 0; i < lr_length; i++) {
                PathPlanningUtilities::Path predict_path;
                predict_path.resize(path_length);
                for (int j = 0; j < path_length; j++) {
                    predict_path[j].x_ = optimized_path[j].x_ - lr_list[i] * gradient.coeff(j, 0);
                    predict_path[j].y_ = optimized_path[j].y_ - lr_list[i] * gradient.coeff(j, 1);
                }
                // 计算损失
                double cost = 0.0;
                cost += this->calcCurvatureCost(predict_path);
                values.push_back(cost);
            }
            // 得到损失最小的学习率
            lr = lr_list[std::min_element(values.begin(), values.end()) - values.begin()];
            // std::cout << "cost is " << *(std::min_element(values.begin(), values.end())) << ", lr is " << lr << std::endl;
            
            LOG(INFO) << "The selected learning rate is :" << lr;
            if (Tools::isSmall(lr, 0.0000001)) {
                // 到达局部极小值
                break;
            }

            for (int i = 1; i < path_length - 1; i++) {
                optimized_path[i].x_ -= lr * gradient.coeff(i, 0);
                optimized_path[i].y_ -= lr * gradient.coeff(i, 1);

            }
            // delete clustering points
            this->clusteredPointRemove(optimized_path, path_arc_lengths[path_arc_lengths.size() - 1]);
            this->detachedPointAdd(optimized_path, path_arc_lengths[path_arc_lengths.size() - 1]);
            iter_num += 1;         
        }
        return optimized_path;
    }
 private:
    // path to matrix
    Eigen::MatrixXd pathToArray(const PathPlanningUtilities::Path &path) {
        Eigen::MatrixXd result(path.size(), 2);
        for (size_t i = 0; i < path.size(); i++) {
            result.row(i) << path[i].x_, path[i].y_;
        }
        return result;
    }

    // point to matrix
    Eigen::MatrixXd pointToArray(const PathPlanningUtilities::Point2f &point) {
        Eigen::MatrixXd result(1, 2);
        result.row(0) << point.x_, point.y_;
        return result;
    }

    // construct matrix
    Eigen::MatrixXd valuesToArray(double value_1, double value_2) {
        Eigen::MatrixXd result(1, 2);
        result.row(0) << value_1, value_2;
        return result;
    }

    // calculate curvature gradient
    Eigen::MatrixXd calcCurvatureItem(const Eigen::MatrixXd &point_m1, const Eigen::MatrixXd &point, const Eigen::MatrixXd &point_p1, double &tmp_curvature) {
        Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(3, 2);
        // calculate curvature and judge whether need to optimize
        double curvature = this->calcCurvature(point_m1, point, point_p1);
        tmp_curvature = fabs(curvature);
        if ((curvature < this->max_curvature_ && curvature >= 0.0) || (curvature <= 0.0 && curvature > -1.0 * this->max_curvature_)) {
            return gradient;
        }
        // std::cout << "start calculate gradient item" << std::endl;
        // get the x, y values
        double pre_point_x = point_m1(0);
        double pre_point_y = point_m1(1);
        double this_point_x = point(0);
        double this_point_y = point(1);
        double next_point_x = point_p1(0);
        double next_point_y = point_p1(1);
        // calculate intermediate values
        double a_1 = (next_point_x - pre_point_x) / 2.0;
        double b_1 = (next_point_y - pre_point_y) / 2.0;
        double a_2 = (next_point_x + pre_point_x) / 2.0 - this_point_x;
        double b_2 = (next_point_y + pre_point_y) / 2.0 - this_point_y;
        // std::cout << a_1 << a_2 << b_1 << b_2 <<std::endl;
        // calculate gradient portion
        Eigen::MatrixXd delta_k_pre_point = this->valuesToArray(((-1.0 * b_2 - b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 1.5) + 3.0 * (a_1 * b_2 - a_2 * b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 0.5) * a_1) / pow(pow(a_1, 2) + pow(b_1, 2), 3), ((a_1 + a_2) * pow(pow(a_1, 2) + pow(b_1, 2), 1.5) + 3.0 * (a_1 * b_2 - a_2 * b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 0.5) * b_1) / pow(pow(a_1, 2) + pow(b_1, 2), 3));
        Eigen::MatrixXd delta_k_this_point = this->valuesToArray((next_point_y - pre_point_y) / pow(pow(a_1, 2) + pow(b_1, 2), 1.5), (pre_point_x - next_point_x) / pow(pow(a_1, 2) + pow(b_1, 2), 1.5));
        Eigen::MatrixXd delta_k_next_point = this->valuesToArray(((b_2 - b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 1.5) - 3.0 * (a_1 * b_2 - a_2 * b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 0.5) * a_1) / pow(pow(a_1, 2) + pow(b_1, 2), 3), ((a_1 - a_2) * pow(pow(a_1, 2) + pow(b_1, 2), 1.5) - 3.0 * (a_1 * b_2 - a_2 * b_1) * pow(pow(a_1, 2) + pow(b_1, 2), 0.5) * b_1) / pow(pow(a_1, 2) + pow(b_1, 2), 3));
        // construct gradient and limit the norm
        gradient.row(0) = curvature * delta_k_pre_point * (2.0 / (fabs(curvature) * delta_k_pre_point.norm()));
        gradient.row(1) = curvature * delta_k_this_point * (4.0 / (fabs(curvature) * delta_k_this_point.norm()));
        gradient.row(2) = curvature * delta_k_next_point * (2.0 / (fabs(curvature) * delta_k_next_point.norm()));
        // gradient.row(0) = curvature * delta_k_pre_point * ((4.0 * (delta_k_pre_point.norm() / delta_k_this_point.norm())) / (fabs(curvature) * delta_k_pre_point.norm()));
        // gradient.row(1) = curvature * delta_k_this_point * (4.0 / (fabs(curvature) * delta_k_this_point.norm()));
        // gradient.row(2) = curvature * delta_k_next_point * ((4.0 * (delta_k_next_point.norm() / delta_k_this_point.norm())) / (fabs(curvature) * delta_k_next_point.norm()));
        // LOG(INFO) << "gradient values in the first row " << gradient(0, 0) << " " << gradient(0, 1);
        // LOG(INFO) << "gradient values in the second row " << gradient(1, 0) << " " << gradient(1, 1);
        // LOG(INFO) << "gradient values in the third row " << gradient(2, 0) << " " << gradient(2, 1);
        return gradient;
    }

    // calculate curvature
    double calcCurvature(const Eigen::MatrixXd &point_m1, const Eigen::MatrixXd &point, const Eigen::MatrixXd &point_p1) {
        // get the x, y values
        double pre_point_x = point_m1(0);
        double pre_point_y = point_m1(1);
        double this_point_x = point(0);
        double this_point_y = point(1);
        double next_point_x = point_p1(0);
        double next_point_y = point_p1(1);
        // calculate intermediate values
        double a_1 = (next_point_x - pre_point_x) / 2.0;
        double b_1 = (next_point_y - pre_point_y) / 2.0;
        double a_2 = (next_point_x + pre_point_x) / 2.0 - this_point_x;
        double b_2 = (next_point_y + pre_point_y) / 2.0 - this_point_y;
        // calculate curvature
        double k = (2.0 * (a_1 * b_2 - a_2 * b_1)) / (pow(pow(a_1, 2) + pow(b_1, 2), 1.5));
        return k;
    }

    // 判断路径上的点是否出现聚集,如果出现,则删除聚集点
    void clusteredPointRemove(PathPlanningUtilities::Path &path, double arc_length) {
        // 首先计算路径上点间距的平均值和标准差
        double dis_mu = arc_length / (path.size() - 1);
        double dis_sigma = 0.0;
        // 记录距离
        std::vector<double> distances;
        for (size_t i = 0; i < path.size() - 1; i++) {
            double distance = PathPlanningUtilities::calcDistance(path[i], path[i + 1]);
            distances.push_back(distance);
            dis_sigma += (distance - dis_mu) * (distance - dis_mu);
        }
        dis_sigma = sqrt(dis_sigma / (path.size() - 1));
        // std::cout << "mean: " << dis_mu << ", deviation: " << dis_sigma << std::endl;
        // 根据标准差和平均值进行删除点(第一个点和最后一个点不变)
        std::vector<size_t> delete_indexes;
        for (size_t i = 1; i < path.size() - 1; i++) {
            double distance_before = distances[i - 1];
            double distance_after = distances[i];
            if (Tools::isSmall(distance_before, dis_mu - dis_sigma) || Tools::isSmall(distance_after, dis_mu - dis_sigma)) {
                if (std::find(delete_indexes.begin(), delete_indexes.end(), i - 1) == delete_indexes.end()) {
                    delete_indexes.push_back(i);
                }
            }
        }
        // 进行删除
        for (int i = delete_indexes.size() - 1;i >= 0; i--) {
            // std::cout << "delete cluster point" << std::endl;
            PathPlanningUtilities::Path::iterator it = path.begin() + delete_indexes[i];
            path.erase(it);
        }
        // std::cout << "after remove path length " << path.size() << std::endl;
    }
    
    // 判断路径上的点是否出现偏离,如果出现,则增加点
    void detachedPointAdd(PathPlanningUtilities::Path &path, double arc_length) {
        // 首先计算路径上点间距的平均值和标准差
        double dis_mu = arc_length / (path.size() - 1);
        double dis_sigma = 0.0;
        // 记录距离
        std::vector<double> distances;
        for (size_t i = 0; i < path.size() - 1; i++) {
            double distance = PathPlanningUtilities::calcDistance(path[i], path[i + 1]);
            distances.push_back(distance);
            dis_sigma += (distance - dis_mu) * (distance - dis_mu);
        }
        dis_sigma = sqrt(dis_sigma / (path.size() - 1));
        // 根据标准差和平均值进行加点
        std::vector<size_t> add_indexes;
        for (size_t i = 1; i < path.size(); i++) {
            double distance_before = distances[i - 1];
            if (Tools::isLarge(distance_before, dis_mu + dis_sigma)) {
                add_indexes.push_back(i);
            }
        }
        // 开始进行加点
        PathPlanningUtilities::Path new_path;
        for(size_t i = 0; i < path.size();) {
            // 判断i是不是要加点
            if (add_indexes.size() > 0) {
                if (i == add_indexes[0]) {
                    // 要加点
                    // 计算中心点
                    PathPlanningUtilities::Point2f new_point;
                    new_point.x_ = (path[i - 1].x_ + path[i].x_) * 0.5;
                    new_point.y_ = (path[i - 1].y_ + path[i].y_) * 0.5;
                    new_path.push_back(new_point);
                    // 删除添加点
                    add_indexes.erase(add_indexes.begin());
                } else {
                    // 不用加点
                    new_path.push_back(path[i]);
                    i++;
                }
            } else {
                new_path.push_back(path[i]);
                i++;
            }
        }
        path = new_path;
    }

    // calculate arc length
    std::vector<double> calcPathArcLength(const PathPlanningUtilities::Path &path) {
        double total_distance = 0.0;
        std::vector<double> result;
        result.push_back(total_distance);
        for (size_t i = 0;i < path.size() - 1; i++) {
            total_distance += PathPlanningUtilities::calcDistance(path[i], path[i + 1]);
            result.push_back(total_distance);
        }
        return result;
    }

    // calculate curvature cost
    double calcCurvatureCost(const PathPlanningUtilities::Path &path) {
        double cost = 0.0;
        for (size_t i = 1; i < path.size() - 1; i++) {
            Eigen::MatrixXd point_m1 = this->pointToArray(path[i - 1]);
            Eigen::MatrixXd point = this->pointToArray(path[i]);
            Eigen::MatrixXd point_p1 = this->pointToArray(path[i + 1]);
            Eigen::MatrixXd delta_x = point - point_m1;
            Eigen::MatrixXd delta_px = point_p1 - point;
            // std::cout << "path[i - 1]" << path[i - 1].x_ << ", " << path[i - 1].y_ << std::endl;
            // std::cout << "path[i]" << path[i].x_ << ", " << path[i].y_ << std::endl;
            // std::cout << "path[i + 1]" << path[i + 1].x_ << ", " << path[i + 1].y_ << std::endl;
            // std::cout << "point_m1" << point_m1 << std::endl;
            // std::cout << "point" << point << std::endl;
            // std::cout << "point_p1" << point_p1 << std::endl;
            // std::cout << "delta_x" << delta_x << std::endl;
            // std::cout << "delta_px" << delta_px << std::endl;
            double abs_delta_x = delta_x.norm();
            double abs_delta_px = delta_px.norm();
            // std::cout << "abs_delta_x" << abs_delta_x << "abs_delta_px" << abs_delta_px << std::endl;
            // std::cout << "(delta_x * delta_px.transpose()).value()" << (delta_x * delta_px.transpose()).value() << std::endl;
            // std::cout << "(delta_x * delta_px.transpose()).value() / (abs_delta_x * abs_delta_px)" << (delta_x * delta_px.transpose()).value() / (abs_delta_x * abs_delta_px) << std::endl;
            // 得到角度偏移量
            double delta_phi = Tools::safeAcos((delta_x * delta_px.transpose()).value() / (abs_delta_x * abs_delta_px));
            // std::cout << "delta_phi" << delta_phi << std::endl;
            // 计算曲率
            assert(abs_delta_x > 0.0 && abs_delta_px > 0.0);
            double curvature = delta_phi / abs_delta_x;
            if (Tools::isLarge(fabs(curvature), this->max_curvature_)) {
                cost += (curvature * curvature - this->max_curvature_ * this->max_curvature_);
            }
        }
        return cost;
    }

    // class values
    double max_curvature_;
    double learning_rate_;
};
// 路径简化器
// 道格拉斯-普客路径简化算法
class DouglasPeuckerSimplify{
 public:
    // 构造函数
    DouglasPeuckerSimplify() {};

    // 析构函数
    ~DouglasPeuckerSimplify() {};

    // 进行简化
    PathPlanningUtilities::Path simplify(const PathPlanningUtilities::Path &path, double tolerance) {
        // 初始化数据
        this->raw_path_ = path;
        this->tolerance_ = tolerance;
        this->tags_.resize(path.size(), 0);
        // 开始简化
        this->douglasPeuckerReduction(0, path.size() - 1);
        PathPlanningUtilities::Path result;
        for (size_t i = 0; i < this->tags_.size(); i++) {
            if (this->tags_[i]) {
                result.push_back(this->raw_path_[i]);
            }
        }
        return result;
    }

 private:
    // 迭代函数
    void douglasPeuckerReduction(size_t first_index, size_t last_index) {
        // std::cout << "first_index " << first_index << ", last_index " << last_index << std::endl;
        // 得到分段中的最远距离
        double max_dist = 0.0;
        size_t max_index = first_index;
        for (size_t index = first_index; index < last_index; index++) {
            double distance = this->perpendicularDistance(this->raw_path_[first_index], this->raw_path_[last_index], this->raw_path_[index]);
            if (Tools::isLarge(distance, max_dist)) {
                max_dist = distance;
                max_index = index;
            }
        }
        // std::cout << "max index " << max_index << ", max dist " << max_dist << std::endl;
        // 判断最优距离是否大于阈值
        if (Tools::isLarge(max_dist, this->tolerance_) && max_index != first_index) {
            // 如果大于阈值,则进行分段,再次进行迭代
            this->tags_[max_index] = true;
            this->douglasPeuckerReduction(first_index, max_index);
            this->douglasPeuckerReduction(max_index, last_index);
        } else {
            // 如果小于阈值则去除中间所有点,只留起点和终点
            this->tags_[first_index] = true;
            this->tags_[last_index] = true;
        }
    }

    // 计算点到直线的距离
    double perpendicularDistance (const PathPlanningUtilities::Point2f &point_start, const PathPlanningUtilities::Point2f &point_end, const PathPlanningUtilities::Point2f &point) {
        // 点到直线的距离公式法
        PathPlanningUtilities::LineSegment line = PathPlanningUtilities::LineSegment(point_start, point_end);
        double dist = line.pointToLineSegmentDistance(point.x_, point.y_);
        return dist;
    }

    PathPlanningUtilities::Path raw_path_;  // 输入未简化路径
    std::vector<bool> tags_;  // 保留的点下标
    double tolerance_;  // 偏移忍受程度

};

};

