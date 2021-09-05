/*
 * @Author: fjw 
 * @Date: 2021-04-17 15:39:16 
 * @Last Modified by: fjw
 * @Last Modified time: 2021-06-22 22:25:53
 */


#include "LowSpeedNavigation.hpp"


// low speed navigation planning main thread
void LowSpeedNavigation::lowSpeedNavigationPlanningThread() {
    ros::Rate loop_rate(1);
    ros::Rate wait_rate(1);

    // Thread wait until data prepared
    while (ros::ok()) {
        this->current_vehicle_world_position_ready_mutex_.lock();
        this->current_vehicle_movement_ready_mutex_.lock();
        this->current_vehicle_kappa_ready_mutex_.lock();
        bool data_upload_flag = this->current_vehicle_world_position_ready_ && this->current_vehicle_movement_ready_ && this->current_vehicle_kappa_ready_ && this->VEHICLE_OCCUPANCY_GRID_READY_FLAG_;
        this->current_vehicle_world_position_ready_mutex_.unlock();
        this->current_vehicle_movement_ready_mutex_.unlock();
        this->current_vehicle_kappa_ready_mutex_.unlock();
        if (data_upload_flag) {
            break;
        }
        wait_rate.sleep();
    }

    std::cout << "++++++++++++++++++++++++++++++++ data prepare finished, start navigation planning ++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    LOG(INFO) << "++++++++++++++++++++++++++++++++ data prepare finished, start navigation planning ++++++++++++++++++++++++++++++++++++++++++";

    // Start low speed navigation path planning
    int planning_num = 0;
    int fail_num = 0;
    while (ros::ok()) {
        
        // TODO: add start planning judgement, while entering low speed mod, start low speed navigation path planning

        // Update vehicle information and obstacles information
        this->updateMapInformation();
        this->updateObstacleInformation();
        

        // 首先通过地图信息产生地图绝对中心线即每个点对应的边界
        std::vector<PathPlanningUtilities::CoordinationPoint> structure_center_line = this->generateCenterLine();

        // Generate center line for obstacles generation
        std::vector<PathPlanningUtilities::CoordinationPoint> obstacle_center_line = this->generateCenterLineForObstacle();


        // // 然后通过边界点产生虚拟障碍物
        // this->generateAllObstacles(structure_center_line);
        
        // 考虑转弯曲线边界障碍物生成
        this->generateAllObstaclesConsiderCurve(obstacle_center_line);
        // 构建障碍物散点集合
        std::vector<PathPlanningUtilities::Point2f> obstacles_points = this->generateObstalcesPoints();
        // 构造障碍物KDTree
        KDTree obstacle_kdtree = KDTree(obstacles_points);

        // Judge whether need replanning
        bool need_replanning = this->isNavigationPathNeedUpdate(obstacle_kdtree);

        // 生成全局路径 
        std::vector<PathPlanningUtilities::CoordinationPoint> reference_line = this->generateReferenceLine(structure_center_line, obstacle_kdtree);

        // Judge reference line suitable
        bool is_navigation_feasible = this->judgeNavigationSuitable(reference_line);
        
        // Calculate fail number
        planning_num += 1;
        if (!is_navigation_feasible) {
            fail_num += 1;
        }

        // Output fail rate info
        std::cout << "fail number: " << fail_num << std::endl;
        std::cout << "fail rate: " << static_cast<double>(fail_num) / static_cast<double>(planning_num) << std::endl;
        LOG(INFO) << "fail number: " << fail_num;
        LOG(INFO) << "fail rate: " << static_cast<double>(fail_num) / static_cast<double>(planning_num);

        // If navigation line feasible
        if (is_navigation_feasible) {
            this->candidate_reference_lines_.emplace_back(reference_line);
            while (this->candidate_reference_lines_.size() > 5) {
                this->candidate_reference_lines_.erase(this->candidate_reference_lines_.begin());
            }
        }

        // If needn't replanning
        if (!need_replanning) {
            loop_rate.sleep();
            continue;
        }

        // If need replanning
        // Select optimal candidate lines
        std::vector<PathPlanningUtilities::CoordinationPoint> final_reference_line;
        if (!this->candidate_reference_lines_.empty() && this->selectOptimalReferenceLine(final_reference_line, this->candidate_reference_lines_, obstacle_kdtree)) {
            assert(!final_reference_line.empty());
        } else {
            // No feasible candidate, keeping the previous planning navigation
            loop_rate.sleep();
            continue;
        }


        // 低速规划全局路径历史记录保存当前产生的路径
        this->history_reference_line_ = final_reference_line;

        // 可视化全局导航
        visualization_msgs::MarkerArray delete_marker_array, global_path_marker_array;
        delete_marker_array.markers.push_back(VisualizationMethods::visualizedeleteAllMarker(0));
        this->visualization_global_reference_pub_.publish(delete_marker_array);
        global_path_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(final_reference_line, VisualizationMethods::color(0.0, 0.0, 1.0, 1.0), 9000000, 0.1));
        this->visualization_global_reference_pub_.publish(global_path_marker_array);
        
        // 发布导航路径
        path_planning_msgs::BoundedCurve guided_curve_msg;
        guided_curve_msg.header.frame_id = "world";
        guided_curve_msg.header.stamp = ros::Time::now();
        for (auto reference_point: final_reference_line) {
            path_planning_msgs::BoundedCurvePoint point;
            point.left_distance = reference_point.max_height_;
            point.right_distance = -reference_point.min_height_;
            point.center_point.x = reference_point.worldpos_.position_.x_;
            point.center_point.y = reference_point.worldpos_.position_.y_;
            point.center_point.theta = reference_point.worldpos_.theta_;
            point.center_point.kappa = reference_point.worldpos_.kappa_;
            point.center_point.arc_length = reference_point.station_;
            guided_curve_msg.points.push_back(point);
        }
        this->guided_path_pub_.publish(guided_curve_msg);
        
        // 将全局导航路径的距离障碍物最近距离,曲率随弧长变化保存到csv文件中
        // 首先读取本工程的目录
        std::string file_path = ros::package::getPath("low_speed_navigation_planning");
        // 创建数据文件夹
        file_path += "/navigation_data_record/";
        Tools::resetLogFile(file_path);
        // 读取系统当前时钟作为文件名称
        file_path += Tools::returnCurrentTimeAndDate() + ".csv";
        std::ofstream file(file_path);
        if (file) {
            for (auto point: final_reference_line) {
                // calculate nearest distance
                std::vector<std::pair<float, float>> nearest_obses;
                std::vector<float> sq_distances;
                obstacle_kdtree.findKNeighbor(point.worldpos_.position_.x_, point.worldpos_.position_.y_, &nearest_obses, &sq_distances, 1);

                // record
                file << std::setprecision(14) << point.worldpos_.position_.x_ << "," << point.worldpos_.position_.y_ << "," << point.worldpos_.kappa_  << "," << sqrt(sq_distances[0]) << "\n";
            }
        }
        file.close();

        loop_rate.sleep();
    }
}

// Generate center line for obstacle generation
std::vector<PathPlanningUtilities::CoordinationPoint> LowSpeedNavigation::generateCenterLineForObstacle() {
    this->updateMapInformation();
    std::vector<PathPlanningUtilities::CoordinationPoint> final_center_line;
    // 确定规划距离
    // LOG(INFO) << "中心道路长度为" << this->center_lane_.getLaneCoordnation().size() / 10.0;
    double low_speed_planning_length = std::min(this->remain_distance_, 100.0);
    int center_points_num = static_cast<int>(low_speed_planning_length / 0.1);
    center_points_num += 200;
    // 获得当前定位
    this->current_vehicle_world_position_mutex_.lock();
    PathPlanningUtilities::VehicleState current_vehicle_world_position = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();

    // 单车道的情况
    std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = this->center_lane_.getLaneCoordnation();
    size_t start_index_of_lane = Tools::findNearestPositionIndexInCoordination(lane_coordination, current_vehicle_world_position.position_);
    for (size_t i = start_index_of_lane; i < start_index_of_lane + center_points_num; i++) {
        // final_center_line[i - start_index_of_lane] = lane_coordination[i];
        PathPlanningUtilities::CoordinationPoint edited_lane_coordination = lane_coordination[i];
        edited_lane_coordination.max_height_ += 0.0;
        edited_lane_coordination.min_height_ -= 0.0;
        final_center_line.emplace_back(edited_lane_coordination);
    }
    return final_center_line;
}

// 生成道路中心线与每个点距道路两端的距离
std::vector<PathPlanningUtilities::CoordinationPoint> LowSpeedNavigation::generateCenterLine() {
    this->updateMapInformation();
    std::vector<PathPlanningUtilities::CoordinationPoint> final_center_line;
    // 确定规划距离
    // LOG(INFO) << "中心道路长度为" << this->center_lane_.getLaneCoordnation().size() / 10.0;
    double low_speed_planning_length = std::min(this->remain_distance_, 100.0);
    int center_points_num = static_cast<int>(low_speed_planning_length / 0.1);
    final_center_line.resize(center_points_num);
    // 获得当前定位
    this->current_vehicle_world_position_mutex_.lock();
    PathPlanningUtilities::VehicleState current_vehicle_world_position = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();
    if (this->is_single_lane_) {
        // 单车道的情况
        std::vector<PathPlanningUtilities::CoordinationPoint> lane_coordination = this->center_lane_.getLaneCoordnation();
        size_t start_index_of_lane = Tools::findNearestPositionIndexInCoordination(lane_coordination, current_vehicle_world_position.position_);
        for (size_t i = start_index_of_lane; i < start_index_of_lane + center_points_num; i++) {
            // final_center_line[i - start_index_of_lane] = lane_coordination[i];
            PathPlanningUtilities::CoordinationPoint edited_lane_coordination = lane_coordination[i];
            edited_lane_coordination.max_height_ += 0.0;
            edited_lane_coordination.min_height_ -= 0.0;
            final_center_line[i - start_index_of_lane] = edited_lane_coordination;
        }
    } else if (this->left_lane_.getLaneExistance() && this->right_lane_.getLaneExistance()) {
        // 三车道都存在的情况
        std::vector<PathPlanningUtilities::CoordinationPoint> center_lane_coordination = this->center_lane_.getLaneCoordnation();
        size_t start_index_of_center_lane = Tools::findNearestPositionIndexInCoordination(center_lane_coordination, current_vehicle_world_position.position_);
        // 构造最终的中心参考线，直接使用三倍左右距离代替(后期可详细修正误差)，station暂时为无效值，与中心道路参考线保持一致即可
        for (size_t i = start_index_of_center_lane; i < start_index_of_center_lane + center_points_num; i++) {
            PathPlanningUtilities::CoordinationPoint this_center_line_point = center_lane_coordination[i];
            this_center_line_point.max_height_ = center_lane_coordination[i].max_height_ * 3.0;
            this_center_line_point.min_height_ = center_lane_coordination[i].min_height_ * 3.0;
            final_center_line[i - start_index_of_center_lane] = this_center_line_point;
        }
    } else if (this->left_lane_.getLaneExistance() && !this->right_lane_.getLaneExistance()) {
        // 只存在左侧车道和中间车道的情况
        std::vector<PathPlanningUtilities::CoordinationPoint> center_lane_coordination = this->center_lane_.getLaneCoordnation();
        size_t start_index_of_center_lane = Tools::findNearestPositionIndexInCoordination(center_lane_coordination, current_vehicle_world_position.position_);
        // 构造最终中心线，直接用中间车道中心点和左距离进行构造
        // 计算横向偏移
        double lateral_offset = center_lane_coordination[start_index_of_center_lane].max_height_;
        // 计算横向偏移方向
        double lateral_offset_angle = center_lane_coordination[start_index_of_center_lane].worldpos_.theta_ + PI * 0.5;
        for (size_t i = start_index_of_center_lane; i < start_index_of_center_lane + center_points_num; i++) {
            PathPlanningUtilities::CoordinationPoint reference_point = center_lane_coordination[i];
            PathPlanningUtilities::CoordinationPoint this_center_line_point;
            PathPlanningUtilities::CurvePoint this_center_line_curvepoint;
            this_center_line_curvepoint.position_.x_ = reference_point.worldpos_.position_.x_ + lateral_offset * std::cos(lateral_offset_angle);
            this_center_line_curvepoint.position_.y_ = reference_point.worldpos_.position_.y_ + lateral_offset * std::sin(lateral_offset_angle);
            this_center_line_curvepoint.theta_ = reference_point.worldpos_.theta_;
            this_center_line_curvepoint.kappa_ = reference_point.worldpos_.kappa_;
            this_center_line_point.worldpos_ = this_center_line_curvepoint;
            this_center_line_point.max_height_ = reference_point.max_height_ - reference_point.min_height_;
            this_center_line_point.min_height_ = reference_point.min_height_ - reference_point.max_height_;
            final_center_line[i - start_index_of_center_lane] = this_center_line_point;
        }
    } else if (!this->left_lane_.getLaneExistance() && this->right_lane_.getLaneExistance()) {
        // 只存在右侧车道和中间车道的情况
        std::vector<PathPlanningUtilities::CoordinationPoint> center_lane_coordination = this->center_lane_.getLaneCoordnation();
        size_t start_index_of_center_lane = Tools::findNearestPositionIndexInCoordination(center_lane_coordination, current_vehicle_world_position.position_);
        // 构造最终中心线，直接用中间车道中心点和右距离进行构造
        // 计算横向偏移
        double lateral_offset = center_lane_coordination[start_index_of_center_lane].min_height_;
        // 计算横向偏移方向
        double lateral_offset_angle = center_lane_coordination[start_index_of_center_lane].worldpos_.theta_ + PI * 0.5;
        for (size_t i = start_index_of_center_lane; i < start_index_of_center_lane + center_points_num; i++) {
            PathPlanningUtilities::CoordinationPoint reference_point = center_lane_coordination[i];
            PathPlanningUtilities::CoordinationPoint this_center_line_point;
            PathPlanningUtilities::CurvePoint this_center_line_curvepoint;
            this_center_line_curvepoint.position_.x_ = reference_point.worldpos_.position_.x_ + lateral_offset * std::cos(lateral_offset_angle);
            this_center_line_curvepoint.position_.y_ = reference_point.worldpos_.position_.y_ + lateral_offset * std::sin(lateral_offset_angle);
            this_center_line_curvepoint.theta_ = reference_point.worldpos_.theta_;
            this_center_line_curvepoint.kappa_ = reference_point.worldpos_.kappa_;
            this_center_line_point.worldpos_ = this_center_line_curvepoint;
            this_center_line_point.max_height_ = reference_point.max_height_ - reference_point.min_height_;
            this_center_line_point.min_height_ = reference_point.min_height_ - reference_point.max_height_;
            final_center_line[i - start_index_of_center_lane] = this_center_line_point;
        }
    } else {
        LOG(INFO) << "无法获得道路情况";
        std::cout << "无法获得道路情况" << std::endl;
        exit(0);
    }
    return final_center_line;
}


// 在道路两边产生虚拟障碍物并且和可感知到的障碍物汇总(用于障碍物距离优化)
void LowSpeedNavigation::generateAllObstacles(const std::vector<PathPlanningUtilities::CoordinationPoint> & structure_center_line) {
    // 将所有真实障碍物(包括仿真障碍物)加入结构化道路规划所需要的障碍物合集
    this->obstacle_mutex_.lock();
    this->structure_road_obstacles_mutex_.lock();
    this->structure_road_obstacles_.assign(this->obstacles_.begin(), this->obstacles_.end());
    this->obstacle_mutex_.unlock();
    // 沿着道路边界点生成障碍物，将生成的边界点障碍物的id从1000001开始编号
    int length = structure_center_line.size();
    int virtual_obstacle_id = 1000001;
    // 直接分别以道路两侧中心点为障碍墙中心点
    int base_index = length / 2;
    // 设置障碍物的长和宽
    double obstacle_wall_length = length * 0.1;
    double obstacle_wall_width = 0.05;
    // 确定参考点
    PathPlanningUtilities::CoordinationPoint reference_center_point = structure_center_line[base_index];
    // 确定左边界点
    PathPlanningUtilities::CurvePoint left_boundary_point;
    left_boundary_point.position_.x_ = reference_center_point.worldpos_.position_.x_ + reference_center_point.max_height_ * std::cos(reference_center_point.worldpos_.theta_ + PI * 0.5);
    left_boundary_point.position_.y_ = reference_center_point.worldpos_.position_.y_ + reference_center_point.max_height_ * std::sin(reference_center_point.worldpos_.theta_ + PI * 0.5);
    left_boundary_point.theta_ = reference_center_point.worldpos_.theta_;
    left_boundary_point.kappa_ = reference_center_point.worldpos_.kappa_;
    // 添加左侧边界障碍物，大小待定,类型为边界框障碍物
    Obstacle obs_left(virtual_obstacle_id);
    virtual_obstacle_id += 1;
    this->updateObstacleState(&obs_left, left_boundary_point.position_, obstacle_wall_width, obstacle_wall_length, left_boundary_point.theta_, 0.0, left_boundary_point.theta_, 0.0, vec_map_cpp_msgs::GetPredictedTrajectory::Request::UNKNOWN);
    obs_left.setObstacleOccupationWidth(obs_left.getObstacleWidth());
    PathPlanningUtilities::CurvePoint curve_point_left;
    curve_point_left.position_ = left_boundary_point.position_;
    curve_point_left.theta_ = left_boundary_point.theta_;
    curve_point_left.kappa_ = 0.0;
    PathPlanningUtilities::Curve curve_left;
    curve_left.emplace_back(curve_point_left);
    std::vector<PathPlanningUtilities::Curve> curve_set_left;
    curve_set_left.emplace_back(curve_left);
    obs_left.setPredictedTrajectorySet(curve_set_left);
    // 只有障碍物存在预测轨迹才加入到障碍物列表中
    if (obs_left.getPredictedTrajectoryNumber() != 0) {
        this->structure_road_obstacles_.emplace_back(obs_left);
    }
    // 确定右边界点并添加右边界障碍物
    PathPlanningUtilities::CurvePoint right_boundary_point;
    right_boundary_point.position_.x_ = reference_center_point.worldpos_.position_.x_ + reference_center_point.min_height_ * std::cos(reference_center_point.worldpos_.theta_ + PI * 0.5);
    right_boundary_point.position_.y_ = reference_center_point.worldpos_.position_.y_ + reference_center_point.min_height_ * std::sin(reference_center_point.worldpos_.theta_ + PI * 0.5);
    right_boundary_point.theta_ = reference_center_point.worldpos_.theta_;
    right_boundary_point.kappa_ = reference_center_point.worldpos_.kappa_;
    // 添加左侧边界障碍物，大小待定,类型为边界框障碍物
    Obstacle obs_right(virtual_obstacle_id);
    virtual_obstacle_id += 1;
    this->updateObstacleState(&obs_right, right_boundary_point.position_, obstacle_wall_width, obstacle_wall_length, right_boundary_point.theta_, 0.0, left_boundary_point.theta_, 0.0, vec_map_cpp_msgs::GetPredictedTrajectory::Request::UNKNOWN);
    obs_right.setObstacleOccupationWidth(obs_right.getObstacleWidth());
    PathPlanningUtilities::CurvePoint curve_point_right;
    curve_point_right.position_ = right_boundary_point.position_;
    curve_point_right.theta_ = right_boundary_point.theta_;
    curve_point_right.kappa_ = 0.0;
    PathPlanningUtilities::Curve curve_right;
    curve_right.emplace_back(curve_point_right);
    std::vector<PathPlanningUtilities::Curve> curve_set_right;
    curve_set_right.emplace_back(curve_right);
    obs_right.setPredictedTrajectorySet(curve_set_right);
    // 只有障碍物存在预测轨迹才加入到障碍物列表中
    if (obs_right.getPredictedTrajectoryNumber() != 0) {
        this->structure_road_obstacles_.emplace_back(obs_right);
    }
    this->structure_road_obstacles_mutex_.unlock();
}

// 生成道路两旁的虚拟障碍物(考虑道路非直线的情况)
void LowSpeedNavigation::generateAllObstaclesConsiderCurve(const std::vector<PathPlanningUtilities::CoordinationPoint> & structure_center_line) {

    // 将所有真实障碍物(包括仿真障碍物)加入结构化道路规划所需要的障碍物合集
    this->obstacle_mutex_.lock();
    this->structure_road_obstacles_mutex_.lock();
    // // Visualize percepted obstacles
    // VisualizationMethods::visualizeObstacles(this->obstacles_, this->visualization_global_reference_pub_);
    this->structure_road_obstacles_.assign(this->obstacles_.begin(), this->obstacles_.end());
    this->structure_road_obstacles_mutex_.unlock();
    this->obstacle_mutex_.unlock();
    // 沿着道路边界点生成障碍物，将生成的边界点障碍物的id从1000001开始编号
    int virtual_obstacle_id = 1000001;
    // 设置虚拟障碍物大小
    int obstacle_wall_width = 0.1;
    int obstacle_wall_length = 0.1;
    // std::cout << "structure_center_line: " << structure_center_line.size() << std::endl;
    // 生成障碍物并进行添加
    for (size_t i = 0; i < structure_center_line.size(); i++) {
        // 获得道路中心点参考点
        PathPlanningUtilities::CoordinationPoint reference_center_point = structure_center_line[i];
        // 获得左边界点
        PathPlanningUtilities::CurvePoint left_boundary_point;
        left_boundary_point.position_.x_ = reference_center_point.worldpos_.position_.x_ + reference_center_point.max_height_ * std::cos(reference_center_point.worldpos_.theta_ + PI * 0.5);
        left_boundary_point.position_.y_ = reference_center_point.worldpos_.position_.y_ + reference_center_point.max_height_ * std::sin(reference_center_point.worldpos_.theta_ + PI * 0.5);
        left_boundary_point.theta_ = reference_center_point.worldpos_.theta_;
        left_boundary_point.kappa_ = reference_center_point.worldpos_.kappa_;

        // 添加左侧边界障碍物，0.1m * 0.1m,类型为边界框障碍物
        Obstacle obs_left(virtual_obstacle_id);
        virtual_obstacle_id += 1;
        this->updateObstacleState(&obs_left, left_boundary_point.position_, obstacle_wall_width, obstacle_wall_length, left_boundary_point.theta_, 0.0, left_boundary_point.theta_, 0.0, vec_map_cpp_msgs::GetPredictedTrajectory::Request::UNKNOWN);
        obs_left.setObstacleOccupationWidth(obs_left.getObstacleWidth());
        PathPlanningUtilities::CurvePoint curve_point_left;
        curve_point_left.position_ = left_boundary_point.position_;
        curve_point_left.theta_ = left_boundary_point.theta_;
        curve_point_left.kappa_ = 0.0;
        PathPlanningUtilities::Curve curve_left;
        curve_left.emplace_back(curve_point_left);
        std::vector<PathPlanningUtilities::Curve> curve_set_left;
        curve_set_left.emplace_back(curve_left);
        obs_left.setPredictedTrajectorySet(curve_set_left);
        // 只有障碍物存在预测轨迹才加入到障碍物列表中
        this->structure_road_obstacles_mutex_.lock();
        this->structure_road_obstacles_.emplace_back(obs_left);
        this->structure_road_obstacles_mutex_.unlock();
        // 确定右边界点并添加右边界障碍物
        PathPlanningUtilities::CurvePoint right_boundary_point;
        right_boundary_point.position_.x_ = reference_center_point.worldpos_.position_.x_ + reference_center_point.min_height_ * std::cos(reference_center_point.worldpos_.theta_ + PI * 0.5);
        right_boundary_point.position_.y_ = reference_center_point.worldpos_.position_.y_ + reference_center_point.min_height_ * std::sin(reference_center_point.worldpos_.theta_ + PI * 0.5);
        right_boundary_point.theta_ = reference_center_point.worldpos_.theta_;
        right_boundary_point.kappa_ = reference_center_point.worldpos_.kappa_;
        // 添加左侧边界障碍物，大小待定,类型为边界框障碍物
        Obstacle obs_right(virtual_obstacle_id);
        virtual_obstacle_id += 1;
        this->updateObstacleState(&obs_right, right_boundary_point.position_, obstacle_wall_width, obstacle_wall_length, right_boundary_point.theta_, 0.0, right_boundary_point.theta_, 0.0, vec_map_cpp_msgs::GetPredictedTrajectory::Request::UNKNOWN);
        obs_right.setObstacleOccupationWidth(obs_right.getObstacleWidth());
        PathPlanningUtilities::CurvePoint curve_point_right;
        curve_point_right.position_ = right_boundary_point.position_;
        curve_point_right.theta_ = right_boundary_point.theta_;
        curve_point_right.kappa_ = 0.0;
        PathPlanningUtilities::Curve curve_right;
        curve_right.emplace_back(curve_point_right);
        std::vector<PathPlanningUtilities::Curve> curve_set_right;
        curve_set_right.emplace_back(curve_right);
        obs_right.setPredictedTrajectorySet(curve_set_right);
        // 只有障碍物存在预测轨迹才加入到障碍物列表中
        this->structure_road_obstacles_mutex_.lock();
        this->structure_road_obstacles_.emplace_back(obs_right);
        this->structure_road_obstacles_mutex_.unlock();
    }

}

// 构造KDTree,将已知障碍物构造成散点
std::vector<PathPlanningUtilities::Point2f> LowSpeedNavigation::generateObstalcesPoints() {
    // 获得构造散点的障碍物
    this->structure_road_obstacles_mutex_.lock();
    std::vector<Obstacle> generate_points_obstacles = this->structure_road_obstacles_;
    this->structure_road_obstacles_mutex_.unlock();
    // 获取车辆当前位置
    this->current_vehicle_world_position_mutex_.lock();
    PathPlanningUtilities::VehicleState current_vehicle_world_position = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();
    PathPlanningUtilities::Point2f vehicle_point = current_vehicle_world_position.position_;
    // 构造障碍物点集
    std::vector<PathPlanningUtilities::Point2f> obstacles_points;
    for (auto &this_obstacle: generate_points_obstacles) {
        // 先获得障碍物中心点,朝向和长宽
        PathPlanningUtilities::Point2f this_obstacle_center_point = this_obstacle.getObstaclePosition();
        // 如果障碍物与本车距离超过60m，直接忽略
        if (Tools::calcDistanceBetweenPoint2f(this_obstacle_center_point, vehicle_point) > 55.0) {
            continue;
        }
        double theta = this_obstacle.getObstacleOrientation();
        double length =this_obstacle.getObstacleLength();
        double width = this_obstacle.getObstacleWidth();
        // 计算障碍物的四个边界点
        // 先计算中心点与顶点之间的距离, 与四个顶点分别对应的旋转角度
        double distance = sqrt(std::pow(length, 2.0) + std::pow(width, 2.0)) / 2.0;
        double angle_1 = Tools::angleSafeConversion(atan2(width, length));
        double angle_2 = Tools::angleSafeConversion(PI - 2.0 * angle_1);
        double angle_3 = Tools::angleSafeConversion(angle_1 - PI);
        double angle_4 = -1.0 * angle_1;
        // 然后分别计算四个顶点
        PathPlanningUtilities::Point2f point_1, point_2, point_3, point_4;
        point_1.x_ = this_obstacle_center_point.x_ + distance * std::cos(theta + angle_1);
        point_1.y_ = this_obstacle_center_point.y_ + distance * std::sin(theta + angle_1);
        point_2.x_ = this_obstacle_center_point.x_ + distance * std::cos(theta + angle_2);
        point_2.y_ = this_obstacle_center_point.y_ + distance * std::sin(theta + angle_2);
        point_3.x_ = this_obstacle_center_point.x_ + distance * std::cos(theta + angle_3);
        point_3.y_ = this_obstacle_center_point.y_ + distance * std::sin(theta + angle_3);
        point_4.x_ = this_obstacle_center_point.x_ + distance * std::cos(theta + angle_4);
        point_4.y_ = this_obstacle_center_point.y_ + distance * std::sin(theta + angle_4);
        // 根据四个顶点产生障碍物边界并进行采样
        std::vector<PathPlanningUtilities::Point2f> obstacle_boundary_line_1 = Tools::constructObstacleBoundary(point_1, point_2, 0.05);
        std::vector<PathPlanningUtilities::Point2f> obstacle_boundary_line_2 = Tools::constructObstacleBoundary(point_2, point_3, 0.05);
        std::vector<PathPlanningUtilities::Point2f> obstacle_boundary_line_3 = Tools::constructObstacleBoundary(point_3, point_4, 0.05);
        std::vector<PathPlanningUtilities::Point2f> obstacle_boundary_line_4 = Tools::constructObstacleBoundary(point_4, point_1, 0.05);
        // 将障碍物轮廓点加入障碍物点集
        obstacles_points.insert(obstacles_points.end(), obstacle_boundary_line_1.begin(), obstacle_boundary_line_1.end());
        obstacles_points.insert(obstacles_points.end(), obstacle_boundary_line_2.begin(), obstacle_boundary_line_2.end());
        obstacles_points.insert(obstacles_points.end(), obstacle_boundary_line_3.begin(), obstacle_boundary_line_3.end());
        obstacles_points.insert(obstacles_points.end(), obstacle_boundary_line_4.begin(), obstacle_boundary_line_4.end());
    }
    return obstacles_points;
}

// 进行全局路径生成
std::vector<PathPlanningUtilities::CoordinationPoint> LowSpeedNavigation::generateReferenceLine(const std::vector<PathPlanningUtilities::CoordinationPoint> & structure_center_line, const KDTree &kdtree) {

    
    PathPlanningUtilities::Path unoptimized_path;
    int result = 0;
    while (result != 1) {
        // 首先确定车辆现在位置
        this->current_vehicle_world_position_mutex_.lock();
        PathPlanningUtilities::VehicleState current_vehicle_world_position = this->current_vehicle_world_position_;
        this->current_vehicle_world_position_mutex_.unlock();
        
        // determine start point and goal point information
        PathPlanningUtilities::Point2f start_position = current_vehicle_world_position.position_;
        double start_derivation = current_vehicle_world_position.theta_;
        PathPlanningUtilities::Point2f goal_position = structure_center_line.back().worldpos_.position_;
        double goal_derivation = structure_center_line.back().worldpos_.theta_;
        if (this->navigation_mode_ == 1) {
            // 然后通过结构化道路搜素产生一条初始全局导航
            // 使用结构化道路先搜索产生一条未优化路径，需要调整的是横向搜索范围
            GlobalPlanningFrontend::StructureGuidance structure_guidance = GlobalPlanningFrontend::StructureGuidance();
            // 判断历史路径的影响
            std::shared_ptr<KDTree> history_reference_line_kdtree_ptr = nullptr;
            if (this->history_reference_line_.size() > 0) {
                // if there is a history path
                std::vector<PathPlanningUtilities::Point2f> points;
                for (size_t i = 0; i < this->history_reference_line_.size(); i++) {
                    points.push_back(this->history_reference_line_[i].worldpos_.position_);
                }
                history_reference_line_kdtree_ptr.reset(new KDTree(points));
            }

            clock_t start_structure_guide_search_time = clock();
            result = structure_guidance.expeditiousSearching(structure_center_line, kdtree, start_position, history_reference_line_kdtree_ptr, unoptimized_path);
            clock_t end_structure_guide_search_time = clock();
            
            std::cout << "structure guide search time consume is: " << static_cast<double>(end_structure_guide_search_time - start_structure_guide_search_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;

            if (result == -1) {
                LOG(INFO) << "structure planning failed, replanning";
                std::cout << "structure planning failed, replanning" << std::endl;
            }
        } else if (this->navigation_mode_ == 2) {
            // construct hybrid a star search planner
            GlobalPlanningFrontend::HybridAstarPlanner hybrid_astar_planner = GlobalPlanningFrontend::HybridAstarPlanner();

            // get hybrid search result
            clock_t start_hybrid_astar_search_time = clock();
            result = hybrid_astar_planner.planning(start_position, start_derivation, goal_position, goal_derivation, kdtree, unoptimized_path);
            clock_t end_hybrid_astar_search_time = clock();

            std::cout << "hybrid astar search time consume is: " << static_cast<double>(end_hybrid_astar_search_time - start_hybrid_astar_search_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;

            if (result == -1) {
                LOG(INFO) << "hybrid astar search failed, replanning";
                std::cout << "hybrid astar search failed, replanning" << std::endl;
                sleep(0.5);
            }

        } else if (this->navigation_mode_ == 3) {
            // get grip map
            this->occupancy_grid_mutex_.lock();
            GridMap grid_map = this->occupancy_grid_;
            this->occupancy_grid_mutex_.unlock();

            // construct astar planner
            GlobalPlanningFrontend::AstarPlanner astar_planner = GlobalPlanningFrontend::AstarPlanner(1.0);
            
            clock_t start_astar_search_time = clock();
            result = astar_planner.planning(start_position, goal_position, grid_map, kdtree, unoptimized_path);
            clock_t end_astar_search_time = clock();
            
            std::cout << "astar search time consume is: " << static_cast<double>(end_astar_search_time - start_astar_search_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;

            if (result == -1) {
                LOG(INFO) << "astar search failed, replanning";
                std::cout << "astar search failed, replanning" << std::endl;
            }
        } else if (this->navigation_mode_ == 4) {
            // get theta structure guide planner
            GlobalPlanningFrontend::ThetaStructureGuidePlanner theta_structure_guide_planner = GlobalPlanningFrontend::ThetaStructureGuidePlanner();

            // get history path information
            std::shared_ptr<KDTree> history_reference_line_kdtree_ptr = nullptr;
            if (this->history_reference_line_.size() > 0) {
                // if there is a history path
                std::vector<PathPlanningUtilities::Point2f> points;
                for (size_t i = 0; i < this->history_reference_line_.size(); i++) {
                    points.push_back(this->history_reference_line_[i].worldpos_.position_);
                }
                history_reference_line_kdtree_ptr.reset(new KDTree(points));
            }

            // searching
            clock_t start_theta_structure_guide_search_time = clock();
            result = theta_structure_guide_planner.planning(structure_center_line, kdtree, start_position, start_derivation, history_reference_line_kdtree_ptr, unoptimized_path);
            clock_t end_theta_structure_guide_search_time = clock();
            
            std::cout << "theta structure guide search time consume is: " << static_cast<double>(end_theta_structure_guide_search_time - start_theta_structure_guide_search_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;

            if (result == -1) {
                LOG(INFO) << "theta structure planning failed, replanning";
                std::cout << "theta structure planning failed, replanning" << std::endl;
            }

        }
        if (result == -1) {
            this->front_end_failed_recorder_ += 1;
        }
    }

    // 首先读取本工程的目录
    std::string file_path_8 = ros::package::getPath("low_speed_navigation_planning");
    // 创建数据文件夹
    file_path_8 += "/front_end_raw_path/";
    Tools::resetLogFile(file_path_8);
    // 读取系统当前时钟作为文件名称
    file_path_8 += Tools::returnCurrentTimeAndDate() + ".csv";
    std::ofstream file_8(file_path_8);
    if (file_8) {
        for (auto point: unoptimized_path) {
            file_8 << std::setprecision(14) << point.x_ << "," << point.y_ << "\n";
        }
    }
    file_8.close();

    // // 可视化搜索结果
    // visualization_msgs::MarkerArray raw_path_marker_array;
    // raw_path_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(unoptimized_path, VisualizationMethods::color(0.0, 1.0, 0.0, 1.0), 9500000, 0.1));
    // this->visualization_global_reference_pub_.publish(raw_path_marker_array);

    
    
    // 接下来使用梯度下降算法进行路径优化
    GlobalPlanningBackend::PathSmoother path_smoother_obs = GlobalPlanningBackend::PathSmoother(Config::smooth_max_distance_to_obs_, Config::smooth_max_curvature_, 10.0, 0.05, 10.0, 1.0, 0.0);
    GlobalPlanningBackend::PathSmoother path_smoother_cur = GlobalPlanningBackend::PathSmoother(Config::smooth_max_distance_to_obs_, Config::smooth_max_curvature_, 0.0, 10.0, 10.0, 1.0, 1.0);
    
    // record optimization time
    clock_t start_optimization_time = clock();
    PathPlanningUtilities::Path smoothed_way_points = path_smoother_obs.smoothing(unoptimized_path, kdtree);

    // // 可视化优化障碍物距离结果
    // visualization_msgs::MarkerArray avoidance_obstacles_path_marker_array;
    // avoidance_obstacles_path_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(smoothed_way_points, VisualizationMethods::color(152.0 / 255.0, 245.0 / 255.0, 255.0 / 255.0, 1.0), 9550000, 0.1));
    // this->visualization_global_reference_pub_.publish(avoidance_obstacles_path_marker_array);

    // // 首先读取本工程的目录
    // std::string file_path_3 = ros::package::getPath("low_speed_navigation_planning");
    // // 创建数据文件夹
    // file_path_3 += "/unoptimized_path_data_record/";
    // Tools::resetLogFile(file_path_3);
    // // 读取系统当前时钟作为文件名称
    // file_path_3 += Tools::returnCurrentTimeAndDate() + ".csv";
    // std::ofstream file_3(file_path_3);
    // if (file_3) {
    //     for (auto point: smoothed_way_points) {
    //         file_3 << std::setprecision(14) << point.x_ << "," << point.y_ << "\n";
    //     }
    // }
    // file_3.close();

    GlobalPlanningBackend::DouglasPeuckerSimplify simplifier = GlobalPlanningBackend::DouglasPeuckerSimplify();
    if (this->navigation_mode_ != 2) {
        // 进行路径简化
        smoothed_way_points = simplifier.simplify(smoothed_way_points, 0.005);
    } else {
        smoothed_way_points = Tools::deletePathPoint(smoothed_way_points, 2);
    }

    // // 首先读取本工程的目录
    // std::string file_path_5 = ros::package::getPath("low_speed_navigation_planning");
    // // 创建数据文件夹
    // file_path_5 += "/simplified_path_data_record/";
    // Tools::resetLogFile(file_path_5);
    // // 读取系统当前时钟作为文件名称
    // file_path_5 += Tools::returnCurrentTimeAndDate() + ".csv";
    // std::ofstream file_5(file_path_5);
    // if (file_5) {
    //     for (auto point: smoothed_way_points) {
    //         file_5 << std::setprecision(14) << point.x_ << "," << point.y_ << "\n";
    //     }
    // }
    // file_5.close();

    smoothed_way_points = path_smoother_cur.smoothing(smoothed_way_points, kdtree);
    
    clock_t end_optimization_time = clock();
    std::cout << "optimization time consume is: " << static_cast<double>(end_optimization_time - start_optimization_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;


    // 首先读取本工程的目录
    std::string file_path_4 = ros::package::getPath("low_speed_navigation_planning");
    // 创建数据文件夹
    file_path_4 += "/optimized_path_data_record/";
    Tools::resetLogFile(file_path_4);
    // 读取系统当前时钟作为文件名称
    file_path_4 += Tools::returnCurrentTimeAndDate() + ".csv";
    std::ofstream file_4(file_path_4);
    if (file_4) {
        for (auto point: smoothed_way_points) {
            file_4 << std::setprecision(14) << point.x_ << "," << point.y_ << "\n";
        }
    }
    file_4.close();
    
    
    // 进行路径简化
    PathPlanningUtilities::Path simplified_smoothed_path;
    
    // // For demo vedio
    // visualization_msgs::MarkerArray optimized_path_marker_array;
    // optimized_path_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(smoothed_way_points, VisualizationMethods::color(153.0 / 255.0, 51.0 / 255.0, 250.0 / 255.0, 1.0), 19550000, 0.1));
    // this->visualization_global_reference_pub_.publish(optimized_path_marker_array);
    // std::this_thread::sleep_for(std::chrono::milliseconds(800));


    if (this->navigation_mode_ != 2) {
        simplified_smoothed_path = simplifier.simplify(smoothed_way_points, 0.11);

    } else {
        simplified_smoothed_path = smoothed_way_points;
    }

    if (this->navigation_mode_ != 2) {
        // 简化后进行插点(避免出现过长直线)
        bool interpolation_finished = false;
        double max_curvature = 1.0;
        while (!interpolation_finished) {
            interpolation_finished = true;
            for (size_t i = 1; i < simplified_smoothed_path.size(); i++) {
                // 计算两点之间的距离
                double distance = PathPlanningUtilities::calcDistance(simplified_smoothed_path[i - 1], simplified_smoothed_path[i]);
                if (Tools::isLarge(distance, 2.0 / max_curvature)) {
                    // 在两点之间插入一个点
                    PathPlanningUtilities::Point2f new_point;
                    new_point.x_ = (simplified_smoothed_path[i - 1].x_ + simplified_smoothed_path[i].x_) * 0.5;
                    new_point.y_ = (simplified_smoothed_path[i - 1].y_ + simplified_smoothed_path[i].y_) * 0.5;
                    simplified_smoothed_path.insert(simplified_smoothed_path.begin() + i, new_point);
                    interpolation_finished = false;
                    break;
                }
            }
        }

        // // For demo vedio
        // visualization_msgs::MarkerArray extracted_point_marker_array, delete_marker_array_2;
        // for (size_t i = 0; i < simplified_smoothed_path.size(); i++) {
        //     auto path_position = simplified_smoothed_path[i];
        //     extracted_point_marker_array.markers.push_back(VisualizationMethods::visualizeSphere(path_position.x_, path_position.y_, 0.1, VisualizationMethods::color(139.0 / 255.0, 0.0 / 255.0, 0.0 / 255.0, 1.0), 18550000 + i));
        // }
        // delete_marker_array_2.markers.push_back(VisualizationMethods::visualizedeleteMarker(19550000));
        // this->visualization_global_reference_pub_.publish(delete_marker_array_2);
        // this->visualization_global_reference_pub_.publish(extracted_point_marker_array);
        // std::this_thread::sleep_for(std::chrono::milliseconds(800));




        
        // 进行三次B-spline插值
        CubicBSpline bspline = CubicBSpline(simplified_smoothed_path);
        PathPlanningUtilities::Curve guided_curve;
        // 三次样条采样间隔
        double sample_gap = Config::navigation_point_margin_;
        // 采样点数
        int sample_num = static_cast<int>(static_cast<double>(bspline.getSegmentNum()) / sample_gap);
        // 采样参数
        std::vector<double> samples = Tools::linspace(0.0, static_cast<double>(bspline.getSegmentNum()), sample_num);
        
        for (size_t i = 0; i < samples.size(); i++) {
            double sample = samples[i];
            guided_curve.push_back(bspline.getCurvePoint(sample));
        }




        // 根据全局导航构造参考线
        std::vector<PathPlanningUtilities::CoordinationPoint> reference_line;
        double arc_length = 0.0;
        for (size_t i = 0; i < guided_curve.size(); i++) {
            if (i > 0) {
                arc_length += PathPlanningUtilities::calcDistance(guided_curve[i].position_, guided_curve[i - 1].position_);
            }
            PathPlanningUtilities::CoordinationPoint coordinate_point;
            coordinate_point.worldpos_ = guided_curve[i];
            coordinate_point.station_ = arc_length;
            reference_line.push_back(coordinate_point);
        }

        // // For demo video
        // visualization_msgs::MarkerArray interpolation_path_marker_array;
        // interpolation_path_marker_array.markers.push_back(VisualizationMethods::visualizeCurvesToMarker(reference_line, VisualizationMethods::color(0.0 / 255.0, 0.0 / 255.0, 255.0 / 255.0, 1.0), 17550000, 0.1));
        // this->visualization_global_reference_pub_.publish(interpolation_path_marker_array);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));


        return reference_line;

        // // 进行三次样条插值
        // tk::CubicSpline spline = tk::CubicSpline(simplified_smoothed_path);
        // PathPlanningUtilities::Curve guided_curve;
        // // 三次样条采样间隔
        // double sample_gap = Config::navigation_point_margin_;
        // // 三次样条长度
        // double spline_length = spline.getArcLengths().back();
        // LOG(INFO) << "三次样条曲线总长度为" << spline_length << std::endl;
        // // 采样点数量
        // int sample_num = static_cast<int>(spline_length / sample_gap);
        // // 采样参数
        // std::vector<double> samples = Tools::linspace(0.0, spline_length, sample_num);
        // for (size_t i = 0; i < samples.size(); i++) {
        //     double sample = samples[i];
        //     guided_curve.push_back(spline.getCurvePoint(sample));
        // }
        // // 根据全局导航构造参考线
        // std::vector<PathPlanningUtilities::CoordinationPoint> reference_line;
        // double arc_length = 0.0;
        // for (size_t i = 0; i < guided_curve.size(); i++) {
        //     if (i > 0) {
        //         arc_length += PathPlanningUtilities::calcDistance(guided_curve[i].position_, guided_curve[i - 1].position_);
        //     }
        //     PathPlanningUtilities::CoordinationPoint coordinate_point;
        //     coordinate_point.worldpos_ = guided_curve[i];
        //     coordinate_point.station_ = arc_length;
        //     reference_line.push_back(coordinate_point);
        // }
        // return reference_line;
    } else {
        // hybrid astar mode, add information through scatter point information
        int path_length = simplified_smoothed_path.size();
        std::vector<PathPlanningUtilities::CoordinationPoint> reference_line;
        reference_line.resize(path_length);

        // calculate midpoint
        double station = 0.0;
        for (int i = 1; i < path_length - 1; i++) {
            PathPlanningUtilities::CoordinationPoint this_coordination_point;
            PathPlanningUtilities::CurvePoint this_curve_point;
            this_curve_point.position_ = simplified_smoothed_path[i];
            this_curve_point.theta_ = Tools::calcThetaInfo(simplified_smoothed_path[i], simplified_smoothed_path[i - 1]);
            this_curve_point.kappa_ = Tools::calcCurvatureInfo(simplified_smoothed_path[i - 1], simplified_smoothed_path[i], simplified_smoothed_path[i + 1]);
            this_coordination_point.worldpos_ = this_curve_point;
            station += Tools::calcDistanceBetweenPoint2f(simplified_smoothed_path[i], simplified_smoothed_path[i - 1]);
            this_coordination_point.station_ = station;
            reference_line[i] = this_coordination_point;
        }

        // calculate start point
        PathPlanningUtilities::CoordinationPoint start_coordination_point;
        PathPlanningUtilities::CurvePoint start_curve_point;
        start_curve_point.position_ = simplified_smoothed_path[0];
        start_curve_point.theta_ = reference_line[1].worldpos_.theta_;
        start_curve_point.kappa_ = reference_line[1].worldpos_.kappa_;
        start_coordination_point.worldpos_ = start_curve_point;
        start_coordination_point.station_ = 0.0;
        reference_line[0] = start_coordination_point;

        // calculate end point 
        PathPlanningUtilities::CoordinationPoint end_coordination_point;
        PathPlanningUtilities::CurvePoint end_curve_point;
        end_curve_point.position_ = simplified_smoothed_path[path_length - 1];
        end_curve_point.theta_ = Tools::calcThetaInfo(simplified_smoothed_path[path_length - 1], simplified_smoothed_path[path_length - 2]);
        end_curve_point.kappa_ = reference_line[path_length - 2].worldpos_.kappa_;
        end_coordination_point.worldpos_ = end_curve_point;
        station += Tools::calcDistanceBetweenPoint2f(simplified_smoothed_path[path_length - 1], simplified_smoothed_path[path_length - 2]);
        end_coordination_point.station_ = station;
        reference_line[path_length - 1] = end_coordination_point;

        return reference_line;

    }

}

// 判断全局路径是否需要更新
bool LowSpeedNavigation::isNavigationPathNeedUpdate(const KDTree &kdtree) {
    // History path doesn't exist, replanning
    if (this->history_reference_line_.size() == 0) {
        return true;
    }
    // 获得当前定位
    this->current_vehicle_world_position_mutex_.lock();
    PathPlanningUtilities::VehicleState current_vehicle_world_position = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();
    // Get the corresponding index in history path
    std::vector<PathPlanningUtilities::CoordinationPoint> reference_line = this->history_reference_line_;
    size_t vehicle_index_in_path = Tools::findNearestPositionIndexInCoordination(reference_line, current_vehicle_world_position.position_, 0);

    // 通过剩余路径长度进行判断
    if (static_cast<double>(vehicle_index_in_path) / static_cast<double>(reference_line.size()) > 0.3) {
        return true;
    }
    // 通过障碍物距离进行评价
    double min_distance = 5.0;
    // 对产生的全局路径每20个取1个计算与障碍物最近距离
    for (size_t i = vehicle_index_in_path; i < std::min(reference_line.size() / 2 + vehicle_index_in_path, reference_line.size()); i += 20) {
        PathPlanningUtilities::Point2f calc_point = reference_line[i].worldpos_.position_;
        // 计算距离
        std::vector<std::pair<float, float>> results;
        std::vector<float> sq_distances;
        kdtree.findKNeighbor(calc_point.x_, calc_point.y_, &results, &sq_distances, 1);
        min_distance = std::min(min_distance, static_cast<double>(sq_distances[0]));
    }
    if (min_distance < 1.0) {
        return true;
    }
    return false;
}

// 判断产生的全局路径是否正常
bool LowSpeedNavigation::judgeNavigationSuitable(const std::vector<PathPlanningUtilities::CoordinationPoint> &reference_line) {
    // 首先获得车辆信息
    this->current_vehicle_world_position_mutex_.lock();
    PathPlanningUtilities::VehicleState current_vehicle_world_position = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();

    // // 判断当前机器人和导航路径的角度差
    // double yaw_deviation = fabs(current_vehicle_world_position.theta_ - reference_line[0].worldpos_.theta_);
    // if (Tools::isLarge(yaw_deviation, PI / 6.0)) {
    //     // 如果角度偏差大于阈值
    //     std::cout << "navigation failed because of the difference between vehicle theta and reference line initial theta" << std::endl;
    //     LOG(INFO) << "navigation failed because of the difference between vehicle theta and reference line initial theta";

    //     return false;
    // }

    // 判断生成的导航路径是否存在过大曲率
    double max_curvature = 0.0;
    for (auto coordination_point: reference_line) {
        if (Tools::isLarge(fabs(coordination_point.worldpos_.kappa_), max_curvature)) {
            max_curvature = fabs(coordination_point.worldpos_.kappa_);
        }
    }
    // 如果生成的导航路径最大曲率大于阈值,存在常量
    if (Tools::isLarge(max_curvature, 0.35)) {
        std::cout << "navigation failed because of the curvature exceeds limitation" << std::endl;
        LOG(INFO) << "navigation failed because of the curvature exceeds limitation";
        return false;
    }
    return true;
}

// Selector optimal navigation reference line
bool LowSpeedNavigation::selectOptimalReferenceLine(std::vector<PathPlanningUtilities::CoordinationPoint> &final_reference_line,std::vector<std::vector<PathPlanningUtilities::CoordinationPoint>> &candidate_reference_lines, const KDTree &kdtree) {
    // 获得当前定位
    this->current_vehicle_world_position_mutex_.lock();
    PathPlanningUtilities::VehicleState current_vehicle_world_position = this->current_vehicle_world_position_;
    this->current_vehicle_world_position_mutex_.unlock();
    // The metrics for discriminating are curvature and distance to obstacle
    std::vector<double> candidate_costs(candidate_reference_lines.size(), 0.0);
    for (size_t i = 0; i < candidate_costs.size(); i++) {
        std::vector<PathPlanningUtilities::CoordinationPoint> this_reference_line = candidate_reference_lines[i];
        size_t vehicle_index_in_path = Tools::findNearestPositionIndexInCoordination(this_reference_line, current_vehicle_world_position.position_, 0);
        // 通过障碍物距离进行评价
        double min_distance = 5.0;
        // 对产生的全局路径每20个取1个计算与障碍物最近距离
        for (size_t i = vehicle_index_in_path; i < std::min(this_reference_line.size() / 2 + vehicle_index_in_path, this_reference_line.size()); i += 20) {
            PathPlanningUtilities::Point2f calc_point = this_reference_line[i].worldpos_.position_;
            // 计算距离
            std::vector<std::pair<float, float>> results;
            std::vector<float> sq_distances;
            kdtree.findKNeighbor(calc_point.x_, calc_point.y_, &results, &sq_distances, 1);
            min_distance = std::min(min_distance, static_cast<double>(sq_distances[0]));
        }
        if (min_distance < 1.0) {
            candidate_costs[i] += 10000.0;
        }
        // Calculate max curvature
        double max_curvature = 0.0;
        for (auto coordination_point: this_reference_line) {
            if (Tools::isLarge(fabs(coordination_point.worldpos_.kappa_), max_curvature)) {
                max_curvature = fabs(coordination_point.worldpos_.kappa_);
            }
        }
        candidate_costs[i] += max_curvature * 10.0;
    }
    
    // Delete unfeasible reference line
    std::vector<std::vector<PathPlanningUtilities::CoordinationPoint>> new_candidate_reference_lines;
    std::vector<double> new_candidate_costs;
    for (size_t i = 0; i < candidate_costs.size(); i++) {
        if (candidate_costs[i] < 10000.0) {
            new_candidate_reference_lines.emplace_back(candidate_reference_lines[i]);
            new_candidate_costs.emplace_back(candidate_costs[i]);
        }
    }
    std::cout << "delete " << candidate_reference_lines.size() - new_candidate_reference_lines.size() << " pathes" << std::endl;
    candidate_reference_lines = new_candidate_reference_lines;

    // Calculate optimal reference line
    if (!new_candidate_reference_lines.empty()) {
        int index = std::min_element(new_candidate_costs.begin(), new_candidate_costs.end()) - new_candidate_costs.begin();
        std::cout << "select no." << index + 1 << " reference line" << std::endl;
        final_reference_line = new_candidate_reference_lines[index];
        return true;
    } else {
        return false;
    }
}