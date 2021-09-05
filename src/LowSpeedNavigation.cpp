/*
 * @Author: fjw 
 * @Date: 2021-04-17 15:38:20 
 * @Last Modified by: fjw
 * @Last Modified time: 2021-04-23 15:18:17
 */


#include "LowSpeedNavigation.hpp"


// Contact to ROS
void LowSpeedNavigation::initConnectionToRos() {

    // 获取tf
    this->tf_listener_ptr_ = new tf::TransformListener();

    // determine navigation mode 
    this->nh_.getParam("navigation_mode", this->navigation_mode_);
    
    // Initiate visualization node 
    std::string visualization_global_reference_topic;
    this->nh_.getParam("visualization_global_reference_topic", visualization_global_reference_topic);
    this->visualization_global_reference_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(visualization_global_reference_topic, 10);

    // 发布导航路径
    std::string guided_path_topic;
    this->nh_.getParam("guided_path_topic", guided_path_topic);
    this->guided_path_pub_ = this->nh_.advertise<path_planning_msgs::BoundedCurve>(guided_path_topic, 10);
    

    // 占据栅格地图topic
    std::string occupancy_grid_topic;
    this->nh_.getParam("occupancy_grid_topic", occupancy_grid_topic);
    this->occupancy_grid_sub_ = this->nh_.subscribe(occupancy_grid_topic, 1, &LowSpeedNavigation::getOccupancyGridMap, this);

    // Get location topic
    std::string odometry_topic;
    this->nh_.getParam("odometry_topic", odometry_topic);
    this->odom_sub_ = this->nh_.subscribe(odometry_topic, 1, &LowSpeedNavigation::updateVehiclePosition, this);

    // Get movement state topic 
    std::string movement_topic;
    this->nh_.getParam("movement_topic", movement_topic);
    this->movement_sub_ = this->nh_.subscribe(movement_topic, 1, &LowSpeedNavigation::updateVehicleMovement, this);

    // Get curvature topic
    std::string curvature_topic;
    this->nh_.getParam("curvature_topic", curvature_topic);
    this->curvature_sub_ = this->nh_.subscribe(curvature_topic, 1, &LowSpeedNavigation::updateVehicleCurvature, this);

    // 障碍物topic
    std::string obstacle_topic;
    this->nh_.getParam("obstacle_topic", obstacle_topic);
    this->obstacle_sub_ = this->nh_.subscribe(obstacle_topic, 1, &LowSpeedNavigation::getObstacles, this);

    // 初始化地图服务
    std::string map_service_name;
    this->nh_.getParam("map_service", map_service_name);
    ros::service::waitForService(map_service_name);
    this->map_service_client_ = this->nh_.serviceClient<vec_map_cpp_msgs::GetGuidedCurves>(map_service_name);

    // 初始化障碍物轨迹预测服务
    std::string obstacle_trajectory_prediction_service_name;
    this->nh_.getParam("obstacle_trajectory_prediction_service", obstacle_trajectory_prediction_service_name);
    ros::service::waitForService(obstacle_trajectory_prediction_service_name);
    this->obstacle_trajectory_prediction_service_client_ = this->nh_.serviceClient<vec_map_cpp_msgs::GetPredictedTrajectory>(obstacle_trajectory_prediction_service_name);
}

// 障碍物栅格地图函数,ros节点
void LowSpeedNavigation::getOccupancyGridMap(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    // 加载栅格地图
    this->occupancy_grid_mutex_.lock();
    this->occupancy_grid_ = GridMap((*msg));
    this->occupancy_grid_mutex_.unlock();
    // 修改标志位
    if (!this->VEHICLE_OCCUPANCY_GRID_READY_FLAG_) {
        this->VEHICLE_OCCUPANCY_GRID_READY_FLAG_ = true;
        ROS_INFO("OCCUPANCY GRID GOT");
        LOG(INFO) << "OCCUPANCY GRID GOT";
    }

}

// Update vehicle position information
void LowSpeedNavigation::updateVehiclePosition(const nav_msgs::Odometry::ConstPtr odometry_msg) {
    // Get the location information
    this->current_vehicle_world_position_mutex_.lock();
    this->current_vehicle_world_position_.position_.x_ = odometry_msg->pose.pose.position.x;
    this->current_vehicle_world_position_.position_.y_ = odometry_msg->pose.pose.position.y;
    tf::Quaternion quateration;
    double raw, pitch, theta;
    tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, quateration);
    tf::Matrix3x3(quateration).getRPY(raw, pitch, theta);
    this->current_vehicle_world_position_.theta_ = theta;
    this->current_vehicle_world_position_mutex_.unlock();
    // Ensure vehicle position update successful
    this->current_vehicle_world_position_ready_mutex_.lock();
    if (!this->current_vehicle_world_position_ready_) {
        this->current_vehicle_world_position_ready_ = true;
        ROS_INFO("VEHICLE POSITION GOT");
        LOG(INFO) << "VEHICLE POSITION GOT";
    }
    this->current_vehicle_world_position_ready_mutex_.unlock();
}

// Update vehicle movement information
void LowSpeedNavigation::updateVehicleMovement(const std_msgs::Float64::ConstPtr velocity_msg) {
    // Update vehicle velocity information
    this->current_vehicle_movement_mutex_.lock();
    this->current_vehicle_movement_.velocity_ = velocity_msg->data;
    this->current_vehicle_movement_.acceleration_ = 0.0;
    this->current_vehicle_movement_mutex_.unlock();
    // Ensure vehicle movement update successful
    this->current_vehicle_movement_ready_mutex_.lock();
    if (!this->current_vehicle_movement_ready_) {
        this->current_vehicle_movement_ready_ = true;
        ROS_INFO("VEHICLE MOVEMENT GOT");
        LOG(INFO) << "VEHICLE MOVEMENT GOT";
    }
    this->current_vehicle_movement_ready_mutex_.unlock();
}

// Update vehicle curvature
void LowSpeedNavigation::updateVehicleCurvature(const std_msgs::Float64::ConstPtr curvature_msg) {
    // Record vehicle curvature information
    this->current_vehicle_world_position_mutex_.lock();
    this->current_vehicle_world_position_.kappa_ = curvature_msg->data;
    this->current_vehicle_world_position_mutex_.unlock();
    this->current_vehicle_kappa_mutex_.lock();
    this->current_vehicle_kappa_ = curvature_msg->data;
    this->current_vehicle_kappa_mutex_.unlock();
    // Ensure vehicle curvature upload successful
    this->current_vehicle_kappa_ready_mutex_.lock();
    if (!this->current_vehicle_kappa_ready_) {
        this->current_vehicle_kappa_ready_ = true;
        ROS_INFO("VEHICLE CURVATURE GOT");
        LOG(INFO) << "VEHICLE CURVATURE GOT";
    }
    this->current_vehicle_kappa_ready_mutex_.unlock();
}

// Update map information
void LowSpeedNavigation::updateMapInformation() {
    do {
        // 初始化道路
        this->left_lane_ = Lane();
        this->right_lane_ = Lane();
        this->center_lane_ = Lane();
        // 调用地图服务，提供服务所需参数
        vec_map_cpp_msgs::GetGuidedCurves map_service;
        geometry_msgs::PoseStamped current_pose;
        current_pose.header.frame_id = "world";
        current_pose.header.stamp = ros::Time::now();
        this->current_vehicle_world_position_mutex_.lock();
        // // 以车头的中心点作为中心道路锚点
        // double vehicle_head_x, vehicle_head_y, vehicle_rear_axis_center_scale;
        // this->nh_.getParam("vehicle_rear_axis_center_scale", vehicle_rear_axis_center_scale);
        // vehicle_head_x = this->current_vehicle_world_position_.position_.x_ + vehicle_rear_axis_center_scale * this->vehicle_length_ * cos(this->current_vehicle_world_position_.theta_/2.0);
        // vehicle_head_y = this->current_vehicle_world_position_.position_.y_ + vehicle_rear_axis_center_scale * this->vehicle_length_ * sin(this->current_vehicle_world_position_.theta_/2.0);
        // current_pose.pose.position.x = vehicle_head_x;
        // current_pose.pose.position.y = vehicle_head_y;
        current_pose.pose.position.x = this->current_vehicle_world_position_.position_.x_;
        current_pose.pose.position.y = this->current_vehicle_world_position_.position_.y_;
        current_pose.pose.orientation.x = 0.0;
        current_pose.pose.orientation.y = 0.0;
        current_pose.pose.orientation.z = sin(this->current_vehicle_world_position_.theta_/2.0);
        current_pose.pose.orientation.w = cos(this->current_vehicle_world_position_.theta_/2.0);
        this->current_vehicle_world_position_mutex_.unlock();
        current_pose.pose.position.z = 0;
        // LOG(INFO) << "向地图请求的当前位置为" << std::setprecision(14) << current_pose.pose.position.x << "||" << std::setprecision(14) << current_pose.pose.position.y << "||" << std::setprecision(14) << std::atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w) * 2.0;
        // std::cout << "向地图请求的当前位置为" << std::setprecision(14) << current_pose.pose.position.x << "||" << std::setprecision(14) << current_pose.pose.position.y << "||" << std::setprecision(14) << std::atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w) * 2.0 << std::endl;
        map_service.request.current_pose = current_pose;
        bool current_pose_ignore_orientation;
        this->nh_.getParam("current_pose_ignore_orientation", current_pose_ignore_orientation);
        map_service.request.current_pose_orientation_unknown = current_pose_ignore_orientation;
        // this->destination_mutex_.lock();
        // map_service.request.goal_pose = this->destination_pose_;
        // this->destination_mutex_.unlock();
        // bool goal_pose_ignore_orientation;
        // this->nh_.getParam("goal_pose_ignore_orientation", goal_pose_ignore_orientation);
        // map_service.request.goal_pose_orientation_unknown = goal_pose_ignore_orientation;
        map_service.request.point_margin = LANE_GAP_DISTANCE;
        std::vector<size_t> ignore_traffic_obstacle_ids;
        for (size_t i = 0; i < this->traffic_rule_obstacles_raw_.size(); i++) {
            if (this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::INVALID) {
                ignore_traffic_obstacle_ids.push_back(this->traffic_rule_obstacles_raw_[i].id);
                LOG(INFO) << "删除的id为" << this->traffic_rule_obstacles_raw_[i].id;
            }
        }

        map_service.request.ignored_ids = ignore_traffic_obstacle_ids;
        
        int failed_call_map_service_num = 0;
        while (true) {
            bool is_success = this->map_service_client_.call(map_service);
            if (is_success) {
                break;
            }
            failed_call_map_service_num += 1;
            if (failed_call_map_service_num >= 100) {
                LOG(INFO) << "Called the map service failed 100 times , process has exited";
                std::cout << "Called the map service failed 100 times , process has exited" <<std::endl;
                exit(0);
            }
        }
        // Judge whether the goal point is unreachable
        if (map_service.response.status == vec_map_cpp_msgs::GetGuidedCurvesResponse::GOAL_UNREACHABLE) {
            LOG(INFO) << "The goal is unreachable";
            continue;
        }
        // 获取服务返回值，对于起点和终点的判断
        if (map_service.response.current_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::WRONG_ORIENTATION) {
            LOG(INFO) << "地图服务得到的起点方向错误";
            continue;
        } else if (map_service.response.current_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::OUT_MAP) {
            LOG(INFO) << "地图服务得到的起点在地图外";
            continue;
        }

        // 判断车道长度
        if (map_service.response.center_lane.geometry.points.size() < static_cast<size_t>(15.0 / LANE_GAP_DISTANCE)) {
            LOG(INFO) << "给出中间道路过短" << map_service.response.center_lane.geometry.points.size();
            continue;
        }



        // if (map_service.response.goal_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::WRONG_ORIENTATION) {
        //     LOG(INFO) << "地图服务得到的终点方向错误";
        // } else if (map_service.response.goal_pose_state == vec_map_cpp_msgs::GetGuidedCurvesResponse::OUT_MAP) {
        //     LOG(INFO) << "地图服务得到的终点在地图外";
        // }

        // if (map_service.response.goal_reachable == false) {
        //     LOG(INFO) << "地图服务得到的终点不可达";
        // }

        // 获取服务的返回值,首先是中间车道
        this->center_lane_.enable();
        // 判断是否需要连续换道
        if (map_service.response.multiple_lane_changes) {
            // 需要
            std::vector<double> max_speeds, min_speeds;
            for (auto max_speed: map_service.response.center_lane.max_speeds) {
                max_speeds.push_back(0.5 * max_speed);
            }
            this->center_lane_.setLaneVelocityLimitation(max_speeds);

            for (auto min_speed: map_service.response.center_lane.min_speeds) {
                min_speeds.push_back(0.5 * min_speed);
            }
            this->center_lane_.setLowVelocity(min_speeds);
        } else {
            // 不需要
            this->center_lane_.setLaneVelocityLimitation(map_service.response.center_lane.max_speeds);
            this->center_lane_.setLowVelocity(map_service.response.center_lane.min_speeds);
        }
        
        // 附加打灯情况
        this->center_lane_.setTurn(map_service.response.center_lane.turn);
        // for (size_t i = 0; i < map_service.response.center_lane.max_speeds.size(); i++) {
        //     std::cout << "asfwgax " << map_service.response.center_lane.min_speeds[i] << std::endl;
        // }
        this->center_lane_.generateLaneCenter(map_service.response.center_lane.geometry);
        this->center_lane_.generateLaneTransMatrix();
        // 判断左侧车道是否存在
        if (map_service.response.left_lane_exist) {
            // 判断左道长度
            if (map_service.response.left_lane.geometry.points.size() < static_cast<size_t>(15.0 / LANE_GAP_DISTANCE)) {
                LOG(INFO) << "给出左侧道路过短" << map_service.response.left_lane.geometry.points.size();
                continue;
            }
            this->left_lane_.enable();
            // 判断是否需要连续换道
            if (map_service.response.multiple_lane_changes) {
                // 需要
                std::vector<double> max_speeds, min_speeds;
                for (auto max_speed: map_service.response.left_lane.max_speeds) {
                    max_speeds.push_back(0.5 * max_speed);
                }
                this->left_lane_.setLaneVelocityLimitation(max_speeds);

                for (auto min_speed: map_service.response.left_lane.min_speeds) {
                    min_speeds.push_back(0.5 * min_speed);
                }
                this->left_lane_.setLowVelocity(min_speeds);
            } else {
                // 不需要
                this->left_lane_.setLaneVelocityLimitation(map_service.response.left_lane.max_speeds);
                this->left_lane_.setLowVelocity(map_service.response.left_lane.min_speeds);
            }
            this->left_lane_.generateLaneCenter(map_service.response.left_lane.geometry);
            this->left_lane_.generateLaneTransMatrix();
            // 附加打灯情况
            this->left_lane_.setTurn(map_service.response.left_lane.turn);
        } else {
            this->left_lane_.disable();
        }
        // 判断右侧车道是否存在
        if (map_service.response.right_lane_exist) {
            // 判断右道长度
            if (map_service.response.right_lane.geometry.points.size() < static_cast<size_t>(15.0 / LANE_GAP_DISTANCE)) {
                LOG(INFO) << "给出右侧道路过短" << map_service.response.right_lane.geometry.points.size();
                continue;
            }
            this->right_lane_.enable();
            // 判断是否需要连续换道
            if (map_service.response.multiple_lane_changes) {
                // 需要
                std::vector<double> max_speeds, min_speeds;
                for (auto max_speed: map_service.response.right_lane.max_speeds) {
                    max_speeds.push_back(0.5 * max_speed);
                }
                this->right_lane_.setLaneVelocityLimitation(max_speeds);

                for (auto min_speed: map_service.response.right_lane.min_speeds) {
                    min_speeds.push_back(0.5 * min_speed);
                }
                this->right_lane_.setLowVelocity(min_speeds);
            } else {
                // 不需要
                this->right_lane_.setLaneVelocityLimitation(map_service.response.right_lane.max_speeds);
                this->right_lane_.setLowVelocity(map_service.response.right_lane.min_speeds);
            }
            this->right_lane_.generateLaneCenter(map_service.response.right_lane.geometry);
            this->right_lane_.generateLaneTransMatrix();
            // 附加打灯情况
            this->right_lane_.setTurn(map_service.response.right_lane.turn);
        } else {
            this->right_lane_.disable();
        }
        // 确定引导道路类型
        this->guidance_type_ = map_service.response.guidance;
        // LOG(INFO) << "guided type: "<< this->guidance_type_;
        // std::cout << "guided type raw: "<< Lane::GuidanceType::ALL_AVAILABLE << std::endl;
        if (this->guidance_type_ == Lane::GuidanceType::CHANGE_LEFT) {
            this->right_lane_.disable();
        } else if (this->guidance_type_ == Lane::GuidanceType::CHANGE_RIGHT) {
            this->left_lane_.disable();
        }

        // 设置道路优先级
        // 确定中间道的优先级
        switch (this->guidance_type_) {
            case Lane::GuidanceType::CHANGE_LEFT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                break;
            case Lane::GuidanceType::KEEP_CENTER:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            case Lane::GuidanceType::CHANGE_RIGHT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                break;
            case Lane::GuidanceType::CENTER_LEFT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            case Lane::GuidanceType::CENTER_RIGHT:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            case Lane::GuidanceType::ALL_AVAILABLE:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                break;
            default:
                this->center_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                break;
        }
        // LOG(INFO) << "中间车道优先级" << this->center_lane_.getLanePriority();
        // 设置左道优先级
        if (this->left_lane_.getLaneExistance()) {
            // 如果左道存在
            switch (this->guidance_type_) {
                case Lane::GuidanceType::CHANGE_LEFT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                    break;
                case Lane::GuidanceType::KEEP_CENTER:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
                case Lane::GuidanceType::CHANGE_RIGHT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MIN);
                    break;
                case Lane::GuidanceType::CENTER_LEFT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                case Lane::GuidanceType::CENTER_RIGHT:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_LOW);
                    break;
                case Lane::GuidanceType::ALL_AVAILABLE:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                default:
                    this->left_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
            }
            LOG(INFO) << "左边车道优先级" << this->left_lane_.getLanePriority();
        }
        // 设置右道优先级
        if (this->right_lane_.getLaneExistance()) {
            switch (this->guidance_type_) {
                case Lane::GuidanceType::CHANGE_LEFT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MIN);
                    break;
                case Lane::GuidanceType::KEEP_CENTER:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
                case Lane::GuidanceType::CHANGE_RIGHT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MAX);
                    break;
                case Lane::GuidanceType::CENTER_LEFT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_LOW);
                    break;
                case Lane::GuidanceType::CENTER_RIGHT:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                case Lane::GuidanceType::ALL_AVAILABLE:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_HIGH);
                    break;
                default:
                    this->right_lane_.setLanePriority(PRIORITY_INIT_VALUE_MEDIAN);
                    break;
            }
            LOG(INFO) << "右边车道优先级" << this->right_lane_.getLanePriority();
        }

        // 当前位置最大限速
        this->expected_velocity_upper_bound_ = this->center_lane_.getLaneVelocityLimitation()[0];

        // 获取交通规则生成的障碍物
        this->traffic_rule_obstacles_raw_ = map_service.response.virtual_obstacles;
        // for (size_t i = 0; i < this->traffic_rule_obstacles_raw_.size(); i++) {
        //     if (this->traffic_rule_obstacles_raw_[i].points.size() > 0) {
        //         LOG(INFO) << " traffic obstacle is " << this->traffic_rule_obstacles_raw_[i].points[0].x << "||" << this->traffic_rule_obstacles_raw_[i].points[0].y;
        //         std::cout << " traffic obstacle is " << this->traffic_rule_obstacles_raw_[i].points[0].x << "||" << this->traffic_rule_obstacles_raw_[i].points[0].y << std::endl;
        //     } else {
        //         LOG(INFO) << "traffic obstacle size error";
        //         std::cout << "traffic obstacle size error" << std::endl;
        //     }

        // }

         // 判断是否为单车道
        if (!map_service.response.right_lane_exist && !map_service.response.left_lane_exist) {
            this->is_single_lane_ = true;
        } else {
            this->is_single_lane_ = false;
        }
        // std::cout << "hgugugu" << std::endl;

        // 获取是否允许避障标志位
        this->is_avoidance_capable_ = map_service.response.obstacle_avoidance_allowed;
        // this->is_avoidance_capable_ = true;
        // DEBUG
        // if (this->is_single_lane_) {
        //     this->is_avoidance_capable_ = false;
        // } else {
        //     this->is_avoidance_capable_ = true;
        // }
        // std::cout << "sgwbafsha" << std::endl;
        // 离终点的距离
        this->distance_to_goal_ = map_service.response.distance_to_goal;
        // TOFIX判断长度是否足够
        double shortest_distance = std::min(map_service.response.distance_to_stop, map_service.response.distance_to_goal);
        if (Tools::isLarge(shortest_distance, std::max(NAVIGATION_LENGTH_ENOUGH_MIN_VALUE, this->expected_velocity_upper_bound_ * NAVIGATION_LENGTH_ENOUGH_MIN_COEF))) {
            this->is_length_enough_ = true;
        } else {
            this->is_length_enough_ = false;
        }
        // this->is_length_enough_ = true;
        this->remain_distance_ = shortest_distance;
        LOG(INFO) << "可以自由行驶的距离还剩" << shortest_distance << "，是否足够" << this->is_length_enough_;
        // std::cout << "可以自由行驶的距离还剩" << shortest_distance << "，是否足够" << this->is_length_enough_ << std::endl;

        // if (map_service.response.extra_flags != "") {
        //     if (map_service.response.extra_flags == "HIGHWAY_DOWN_MIDDLE") {
        //         GLOBAL_IS_IN_CHECK_ = true;
        //         // std::cout << "在临检区" << std::endl;
        //     } else if (map_service.response.extra_flags == "BLIND_ZONE"){
        //         GLOBAL_IS_IN_SLOW_ = true;
        //         // std::cout << "不在临检区" << std::endl;
        //     } else if (map_service.response.extra_flags == "OVERTAKE") {
        //         GLOBAL_IS_IN_OVERTAKE_ = true;
        //     }
        // } else {
        //     GLOBAL_IS_IN_CHECK_ = false;
        //     GLOBAL_IS_IN_SLOW_ = false;
        //     GLOBAL_IS_IN_OVERTAKE_ = false;
        //     GLOBAL_IS_IN_CHECK_COUNT_FLAG_ = 0;
        // }
        break;
    } while (true);
}


// 障碍物信息callback函数，ros节点
void LowSpeedNavigation::getObstacles(const ibeo_lidar_msgs::object_filter_data::ConstPtr &msg) {
    this->perception_object_mutex_.lock();
    // 更新障碍物信息
    this->perception_objects_ = msg->objects;
    // 障碍物可视化(TOFIX)
    this->perception_object_mutex_.unlock();
}

// Update obstacle inforamtion
void LowSpeedNavigation::updateObstacleInformation() {
    // 判断是否存在历史轨迹点，如果列表中存在相同ID障碍物则添加历史轨迹，反之没有。
    // 添加障碍物状态信息
    // 如果速度大于最小值，表示障碍物有速度时，添加障碍物的预测轨迹信息；如果速度小于最小值则表示障碍物无速度，障碍物没有轨迹信息。
    this->obstacle_mutex_.lock();
    this->perception_object_mutex_.lock();
    std::vector<Obstacle>().swap(this->obstacles_);
    for (size_t i = 0; i < this->perception_objects_.size(); i++) {
        // std::cout << "new obstacle information is added" << std::endl;
        // LOG(INFO) << "new obstacle information is added";
        ibeo_lidar_msgs::object_filter obstacle = this->perception_objects_[i];
        // 判断是否处于机非混行区域
        // GLOBAL_IN_JIFEI_MUTEX_.lock();
        // bool is_in_jifei = GLOBAL_IS_IN_JIFEI_;
        // GLOBAL_IN_JIFEI_MUTEX_.unlock();
        // if (!is_in_jifei) {
        //     // 不在机非混行区
        // 首先判断是边界框障碍物还是轮廓点障碍物
        if (obstacle.ibeo_contours_box.size() > 0) {
            // 轮廓点障碍物（每两个轮廓点构成一个边界框，多个边对应一个id界框）
            for (size_t j = 0; j < obstacle.ibeo_contours_box.size(); j++) {
                ibeo_lidar_msgs::Contour_box contour_box = obstacle.ibeo_contours_box[j];
                Obstacle obs(obstacle.id);
                PathPlanningUtilities::Point2f position;
                position.x_ = contour_box.center.x;
                position.y_ = contour_box.center.y;
                double v_theta = contour_box.orientation;
                double v = 0.0;
                // 补全障碍物状态信息
                this->updateObstacleState(&obs, position, contour_box.width, contour_box.length, contour_box.orientation, v, v_theta, 0.0, vec_map_cpp_msgs::GetPredictedTrajectory::Request::UNKNOWN);
                // 打印障碍物信息
                // LOG(INFO) << "boundary points obstacle position is (" << position.x_ << ", " << position.y_ << "), orientation is " << obstacle.orientation << ", width is " << obstacle.width << ", length is " << obstacle.length;
                // 对于静态障碍物，其预测轨迹就是其所在的位置，占用宽度就是其宽度
                obs.setObstacleOccupationWidth(obs.getObstacleWidth());
                PathPlanningUtilities::CurvePoint curve_point;
                curve_point.position_ = position;
                curve_point.theta_ = contour_box.orientation;
                curve_point.kappa_ = 0.0;
                PathPlanningUtilities::Curve curve;
                curve.push_back(curve_point);
                std::vector<PathPlanningUtilities::Curve> curve_set;
                curve_set.push_back(curve);
                obs.setPredictedTrajectorySet(curve_set);
                // 只有障碍物存在预测轨迹才加入到障碍物列表中
                if (obs.getPredictedTrajectoryNumber() != 0) {
                    this->obstacles_.push_back(obs);
                } else {
                    LOG(INFO) << "此障碍物不存在预测轨迹，不加入障碍物列表";
                }
            }
        } else {
            // 边界框障碍物（边界框障碍物只会出现一个，并且一个框对应一个id）
            Obstacle obs(obstacle.id);
            PathPlanningUtilities::Point2f position;
            position.x_ = obstacle.center.x;
            position.y_ = obstacle.center.y;
            double v_theta = atan2(obstacle.velocity.y, obstacle.velocity.x);
            double v = obstacle.v;
            // 如果障碍物速度小于最小速度，视为静止
            if (Tools::isZero(v)) {
                v_theta = obstacle.orientation;
            } else if (Tools::isSmall(v, MIN_SPEED)) {
                v = 0.0;
            }
            // 打印障碍物信息
            // LOG(INFO) << "bounding box obstacle position is (" << position.x_ << ", " << position.y_ << "), orientation is " << obstacle.orientation << ", width is " << obstacle.width << ", length is " << obstacle.length << ", velocity is " << v;
            // 补全障碍物预测信息,静态物体没有预测轨迹
            if (!Tools::isZero(v)) {
                // 补全障碍物状态信息
                this->updateObstacleState(&obs, position, obstacle.width, obstacle.length, obstacle.orientation, v, v_theta, 0.0, vec_map_cpp_msgs::GetPredictedTrajectory::Request::UNKNOWN);
                // 如果速度较大,根据地图生成预测路径
                // this->updateObstaclePredictedInformation(&obs);
            } else {
                // 如果速度较小,根据速度方向生成预测路径
                // double predict_distance = 0.5 * Tools::normalObstacleVehicleOccupationDistance(obstacle.v, MAX_DECCELERATION);
                // obs.setObstacleOccupationWidth(obs.getObstacleWidth());
                // // 计算新的中心点
                // position.x_ = position.x_ + 0.5 * predict_distance * cos(v_theta);
                // position.y_ = position.y_ + 0.5 * predict_distance * sin(v_theta);
                // double new_width = obstacle.width;
                // double new_length = obstacle.length + predict_distance;
                // 补全障碍物状态信息
                this->updateObstacleState(&obs, position, obstacle.width, obstacle.length, v_theta, v, v_theta, 0.0, vec_map_cpp_msgs::GetPredictedTrajectory::Request::UNKNOWN);
                // 对于静态障碍物，其预测轨迹就是其所在的位置，占用宽度就是其宽度
                obs.setObstacleOccupationWidth(obs.getObstacleWidth());
                PathPlanningUtilities::CurvePoint curve_point;
                curve_point.position_ = position;
                curve_point.theta_ = v_theta;
                curve_point.kappa_ = 0.0;
                PathPlanningUtilities::Curve curve;
                curve.push_back(curve_point);
                std::vector<PathPlanningUtilities::Curve> curve_set;
                curve_set.push_back(curve);
                obs.setPredictedTrajectorySet(curve_set);
            }
            // 只有障碍物存在预测轨迹才加入到障碍物列表中
            if (obs.getPredictedTrajectoryNumber() != 0) {
                this->obstacles_.push_back(obs);
            } else {
                LOG(INFO) << "此障碍物不存在预测轨迹，不加入障碍物列表";
            }
        }
    } 
    this->perception_object_mutex_.unlock();

    // 障碍物信息可视化
    // VisualizationMethods::visualizeObstacles(this->obstacles_, this->vis_obstacle_pub_);
    // 打印新障碍物的信息
    // std::cout << "obstacle information updated, obstacle number is " << this->obstacles_.size() << std::endl;
    // for (size_t i = 0; i < this->obstacles_.size(); i++) {
    //     std::cout << "obstacle number " << i << ": " << std::endl;
    //     std::cout << "position: " << this->obstacles_[i].getObstaclePosition().x_ << "||" << this->obstacles_[i].getObstaclePosition().y_ << std::endl;
    //     std::cout << "velocity: " << this->obstacles_[i].getObstacleVelocity() << std::endl;
    //     std::cout << "predicted trajectory number is " << this->obstacles_[i].getPredictedTrajectoryNumber() << std::endl;
    //     for (size_t j = 0; j < this->obstacles_[i].getPredictedTrajectoryNumber(); j++) {
    //         std::cout << "-- trajectory " << j << " length is " << this->obstacles_[i].getPredictedTrajectory(j).size() << std::endl;
    //     }
    // }

    // LOG(INFO) << "obstacle information updated, obstacle number is " << this->obstacles_.size();
    // for (size_t i = 0; i < this->obstacles_.size(); i++) {
    //     LOG(INFO) << "obstacle number " << i << ": ";
    //     LOG(INFO) << "position: " << this->obstacles_[i].getObstaclePosition().x_ << "||" << this->obstacles_[i].getObstaclePosition().y_;
    //     LOG(INFO) << "velocity: " << this->obstacles_[i].getObstacleVelocity();
    //     LOG(INFO) << "predicted trajectory number is " << this->obstacles_[i].getPredictedTrajectoryNumber();
    //     for (size_t j = 0; j < this->obstacles_[i].getPredictedTrajectoryNumber(); j++) {
    //         LOG(INFO) << "-- trajectory " << j << " length is " << this->obstacles_[i].getPredictedTrajectory(j).size();
    //     }
    // }
    this->obstacle_mutex_.unlock();
}

// 补全障碍物的状态信息，其中acceleration和class_name为无用值，只要给占位符即可
void LowSpeedNavigation::updateObstacleState(Obstacle* obstacle, const PathPlanningUtilities::Point2f position, double width, double length, double orientation, double velocity, double velocity_direction, double acceleration, size_t class_name) {
    obstacle->setObstaclePosition(position);
    obstacle->setObstacleShape(width, length);
    obstacle->setObstacleOrientation(orientation);
    obstacle->setObstacleVelocity(velocity);
    obstacle->setObstacleVelocityDirection(velocity_direction);
    obstacle->setObstacleAcceleration(acceleration);
    obstacle->setObstacleClass(class_name);
}

// // 补全障碍物的预测信息、轨迹和位置(TOFIX)
// void LowSpeedNavigation::updateObstaclePredictedInformation(Obstacle* obstacle) {
//     // clock_t obstacle_service_start_time = clock();
//     // 地图服务所需参数，本车位置
//     geometry_msgs::PoseStamped vehicle_pose;
//     vehicle_pose.header.frame_id = "world";
//     vehicle_pose.header.stamp = ros::Time::now();
//     this->current_vehicle_world_position_mutex_.lock();
//     vehicle_pose.pose.position.x = this->current_vehicle_world_position_.position_.x_;
//     vehicle_pose.pose.position.y = this->current_vehicle_world_position_.position_.y_;
//     vehicle_pose.pose.position.z = 0.0;
//     vehicle_pose.pose.orientation.x = 0.0;
//     vehicle_pose.pose.orientation.y = 0.0;
//     vehicle_pose.pose.orientation.z = sin(this->current_vehicle_world_position_.theta_/2.0);
//     vehicle_pose.pose.orientation.w = cos(this->current_vehicle_world_position_.theta_/2.0);
//     this->current_vehicle_world_position_mutex_.unlock();
//     // 地图服务所需参数，位姿
//     geometry_msgs::PoseStamped obstacle_pose;
//     obstacle_pose.header.frame_id = "world";
//     obstacle_pose.header.stamp = ros::Time::now();
//     obstacle_pose.pose.position.x = obstacle->getObstaclePosition().x_;
//     obstacle_pose.pose.position.y = obstacle->getObstaclePosition().y_;
//     obstacle_pose.pose.position.z = 0.0;
//     obstacle_pose.pose.orientation.x = 0.0;
//     obstacle_pose.pose.orientation.y = 0.0;
//     obstacle_pose.pose.orientation.z = sin(obstacle->getObstacleOrientation()/2.0);
//     obstacle_pose.pose.orientation.w = cos(obstacle->getObstacleOrientation()/2.0);
//     // 是否需要无视车辆正后方障碍物
//     bool ignore_rear_side = true;
//     // 障碍物id
//     size_t id = obstacle->getID();
//     // 地图服务所需参数，计算长度，长度为障碍物以当前速度行驶3秒的距离(vt + (v*t + v^2/2a + constant_distance) + constant_distance)
//     double distance;
//     // 障碍物速度不为0时，取障碍物的刹车距离
//     if (obstacle->getObstacleClass() == vec_map_cpp_msgs::GetPredictedTrajectory::Request::PEDESTRIAN) {
//         distance = CONSTANT_DISTANCE;
//     } else {
//         distance = Tools::normalObstacleVehicleOccupationDistance(obstacle->getObstacleVelocity(), COMMON_DECCELERATION);
//     }
//     // 地图服务所需参数，形状
//     double width = obstacle->getObstacleWidth();
//     double length = obstacle->getObstacleLength();
//     // 地图服务所需参数，速度方向
//     double direction = obstacle->getObstacleVelocityDirection();
//     // 地图服务所需参数，点间距
//     double point_margin = OBSTACLE_MARGIN;
//     // 地图服务所需参数，类别(TOFIX)
//     double obstacle_class = obstacle->getObstacleClass();

//     // 调用地图服务（TOFIX）
//     vec_map_cpp_msgs::GetPredictedTrajectory predict_trajectory_service;
//     predict_trajectory_service.request.id = id;
//     predict_trajectory_service.request.ignore_rear_side = ignore_rear_side;
//     predict_trajectory_service.request.vehicle_pose = vehicle_pose;
//     predict_trajectory_service.request.current_pose = obstacle_pose;
//     predict_trajectory_service.request.width = width;
//     predict_trajectory_service.request.length = length;
//     predict_trajectory_service.request.speed_orientation = direction;
//     predict_trajectory_service.request.request_length = distance;
//     predict_trajectory_service.request.point_margin = point_margin;
//     predict_trajectory_service.request.type = obstacle_class;
//     predict_trajectory_service.request.current_pose_orientation_unknown = false;
//     this->obstacle_trajectory_prediction_service_client_.call(predict_trajectory_service);
//     // 地图服务返回state表示障碍物速度方向不正确或位置不正确或正常
//     int state = predict_trajectory_service.response.state;
//     // std::cout << "predict obstacle server result: " << is_out_map << std::endl;
//     // clock_t obstacle_service_end_time = clock();
//     // std::cout << "obstacle service time consuming is: " << (obstacle_service_end_time - obstacle_service_start_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;
//     if (state == vec_map_cpp_msgs::GetPredictedTrajectoryResponse::NORMAL) {
//         // 地图服务正常，返回轨迹和道路最小宽度
//         LOG(INFO) << "此障碍物轨迹预测正常";
//         obstacle->setObstacleOccupationWidth(std::max(predict_trajectory_service.response.road_width * OBSTACLE_OCCUPANCY_WIDTH_MAX_TO_LANE_WIDTH, obstacle->getObstacleWidth() + OBSTACLE_OCCUPANCY_WIDTH_EXPAND_CONST));
//         // clock_t obstacle_processing_start_time, obstacle_processing_end_time;
//         std::vector<path_planning_msgs::Curve> predicted_ros_curves = predict_trajectory_service.response.paths;
//         // std::cout << "predicted trajectory number from map server: " << predicted_ros_paths.size() << std::endl;
//         // for (size_t i = 0; i < predicted_ros_paths.size(); i++) {
//         //     std::cout << "predicted trajectory " << i << " point number is " << predicted_ros_paths[i].points.size() << std::endl;
//         //     std::cout << "predicted trajectory " << i << " distance is " << sqrt((predicted_ros_paths[i].points[0].x - predicted_ros_paths[i].points[predicted_ros_paths[i].points.size() - 1].x) * (predicted_ros_paths[i].points[0].x - predicted_ros_paths[i].points[predicted_ros_paths[i].points.size() - 1].x) + (predicted_ros_paths[i].points[0].y - predicted_ros_paths[i].points[predicted_ros_paths[i].points.size() - 1].y) * (predicted_ros_paths[i].points[0].y - predicted_ros_paths[i].points[predicted_ros_paths[i].points.size() - 1].y)) << std::endl;
//         //     for (size_t j = 0; j < predicted_ros_paths[i].points.size(); j++) {
//         //         std::cout << "point is " << predicted_ros_paths[i].points[j].x << "||" << predicted_ros_paths[i].points[j].y << std::endl;
//         //     }
//         // }
//         std::vector<PathPlanningUtilities::Curve> predicted_trajectories;
//         predicted_trajectories.resize(predicted_ros_curves.size());
//         // obstacle_processing_start_time = clock();
//         for (size_t i = 0; i < predicted_ros_curves.size(); i++) {
//             PathPlanningUtilities::Curve predicted_trajectory_curve;
//             PathPlanningUtilities::CurveConverter::fromRosMessage(predicted_ros_curves[i], predicted_trajectory_curve);
//             predicted_trajectories[i] = predicted_trajectory_curve;
//         }
//         obstacle->setPredictedTrajectorySet(predicted_trajectories);
//         // obstacle_processing_end_time = clock();
//         // std::cout << "obstacle trajectory processing time consuming is: " << (obstacle_processing_end_time - obstacle_processing_start_time) * 1000.0 / CLOCKS_PER_SEC << "ms" << std::endl;
//     } else if (state == vec_map_cpp_msgs::GetPredictedTrajectoryResponse::WRONG_ORIENTATION || state == vec_map_cpp_msgs::GetPredictedTrajectoryResponse::HIGH_UNCERTAINTY) {
//         // 障碍物方向与地图路径方向存在较大差异性
//         // TOFIX
//         LOG(INFO) << "此障碍物方向与其所处道路朝向存在较大差异或不可信";
//         // 得到占用宽度
//         obstacle->setObstacleOccupationWidth(obstacle->getObstacleWidth() + OBSTACLE_OCCUPANCY_WIDTH_EXPAND_CONST);
//         // 生成延障碍物速度方向的预测轨迹
//         std::vector<PathPlanningUtilities::Curve> predicted_trajectories;
//         PathPlanningUtilities::Curve predicted_trajectory_curve;
//         size_t predicted_trajectory_size = static_cast<size_t>(distance / OBSTACLE_MARGIN);
//         for (size_t i = 0; i < predicted_trajectory_size; i++) {
//             PathPlanningUtilities::CurvePoint curve_point;
//             curve_point.position_.x_ = obstacle->getObstaclePosition().x_ + i * OBSTACLE_MARGIN * cos(obstacle->getObstacleVelocityDirection());
//             curve_point.position_.y_ = obstacle->getObstaclePosition().y_ + i * OBSTACLE_MARGIN * sin(obstacle->getObstacleVelocityDirection());
//             curve_point.theta_ = obstacle->getObstacleVelocityDirection();
//             curve_point.kappa_ = 0.0;
//             predicted_trajectory_curve.push_back(curve_point);
//         }
//         predicted_trajectories.push_back(predicted_trajectory_curve);
//         obstacle->setPredictedTrajectorySet(predicted_trajectories);
//     } else if (state == vec_map_cpp_msgs::GetPredictedTrajectoryResponse::OUT_MAP || state == vec_map_cpp_msgs::GetPredictedTrajectoryResponse::IGNORED) {
//         // 障碍物位置在地图外面
//         // TOFIX
//         LOG(INFO) << "此障碍物处于地图外或被无视" << obstacle->getObstaclePosition().x_ << "||" << obstacle->getObstaclePosition().y_;
//     }
// }

// // 得到有效的交通障碍物列表
// void LowSpeedNavigation::updateValidateTrafficRuleInformation() {
//     // 首先清空之间的有效交通规则障碍物列表
//     std::vector<vec_map_cpp_msgs::VirtualObstacle>().swap(this->traffic_rule_obstacles_);
//     // 得到当前位置信息和速度信息
//     PathPlanningUtilities::VehicleState current_position_in_world;
//     this->current_vehicle_world_position_mutex_.lock();
//     current_position_in_world = this->current_vehicle_world_position_;
//     this->current_vehicle_world_position_mutex_.unlock();
//     PathPlanningUtilities::VehicleMovementState current_movement_state;
//     this->current_vehicle_movement_mutex_.lock();
//     current_movement_state = this->current_vehicle_movement_;
//     this->current_vehicle_movement_mutex_.unlock();
//     // 遍历原始交通障碍物列表
//     for (size_t i = 0; i < this->traffic_rule_obstacles_raw_.size(); i++) {
//         // 判断原始障碍物类型
//         if (this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::PERMANENT || this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::BEYOND_GOAL) {
//             // 如果是永久墙体，直接加入有效障碍物
//             this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
//         } else if (this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::NOT_PERMANENT) {
//             // 如果是非永久墙体，进行判断
//             if (this->NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG_) {
//                 // 当满足条件1. 当前状态是停车状态，且速度为0。2. 当前位置离空气墙中心很近。此空气墙无效化
//                 bool is_validate = true;
//                 if (this->current_state_.getStateName() == StateNames::STOP && Tools::isZero(this->current_state_.getVehicleCurrentMovement().velocity_)) {
//                     // 满足第一个条件后，开始判断第二个条件
//                     // 计算空气墙中心点
//                     PathPlanningUtilities::Point2f traffic_rule_obstacle_center;
//                     traffic_rule_obstacle_center.x_ = 0.0;
//                     traffic_rule_obstacle_center.y_ = 0.0;
//                     for (size_t j = 0; j < this->traffic_rule_obstacles_raw_[i].points.size(); j++) {
//                         traffic_rule_obstacle_center.x_ += this->traffic_rule_obstacles_raw_[i].points[j].x;
//                         traffic_rule_obstacle_center.y_ += this->traffic_rule_obstacles_raw_[i].points[j].y;
//                     }
//                     traffic_rule_obstacle_center.x_ = traffic_rule_obstacle_center.x_ / this->traffic_rule_obstacles_raw_[i].points.size();
//                     traffic_rule_obstacle_center.y_ = traffic_rule_obstacle_center.y_ / this->traffic_rule_obstacles_raw_[i].points.size();
//                     // 计算距离
//                     double distance = PathPlanningUtilities::calcDistance(current_position_in_world.position_, traffic_rule_obstacle_center);
//                     if (Tools::isSmall(distance, MAX_DISTANCE_TO_NOT_PERMANENT_TRAFFIC_RULE_TO_MAKE_INVALID)) {
//                         is_validate = false;
//                     }
//                 }
//                 if (is_validate) {
//                     this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
//                 } else {
//                     LOG(INFO) << "车辆当前处于停止状态，临时空气墙无效，并且将其从原始数据中删除";
//                     this->traffic_rule_obstacles_raw_[i].mode = vec_map_cpp_msgs::VirtualObstacle::INVALID;
//                 }
//             } else {
//                 LOG(INFO) << "临时空气墙未被使用，无效";
//             }

//         } else if (this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::CONDITION_DECISION) {
//             // 如果是交通灯，首先判断是否进行了使用
//             if (this->TRAFFIC_LIGHT_USAGE_FLAG_) {
//                 // 判断离灯距离
//                 PathPlanningUtilities::Point2f traffic_rule_center_point;
//                 traffic_rule_center_point.x_ = 0.0;
//                 traffic_rule_center_point.y_ = 0.0;
//                 for (size_t point_index = 0; point_index < this->traffic_rule_obstacles_raw_[i].points.size(); point_index++) {
//                     traffic_rule_center_point.x_ += this->traffic_rule_obstacles_raw_[i].points[point_index].x;
//                     traffic_rule_center_point.y_ += this->traffic_rule_obstacles_raw_[i].points[point_index].y;
//                 }
//                 traffic_rule_center_point.x_ = traffic_rule_center_point.x_ / static_cast<double>(this->traffic_rule_obstacles_raw_[i].points.size());
//                 traffic_rule_center_point.y_ = traffic_rule_center_point.y_ / static_cast<double>(this->traffic_rule_obstacles_raw_[i].points.size());
//                 LOG(INFO) << "traffic block center is" <<traffic_rule_center_point.x_ << " " << traffic_rule_center_point.y_;
//                 this->current_vehicle_world_position_mutex_.lock();
//                 double remain_distance = Tools::calcNewCoordinationPosition(this->current_vehicle_world_position_, traffic_rule_center_point).x_;
//                 LOG(INFO) << "current_position is " << this->current_vehicle_world_position_.position_.x_ << " " << this->current_vehicle_world_position_.position_.y_;
//                 double direction_distance = sqrt((this->current_vehicle_world_position_.position_.x_ - traffic_rule_center_point.x_)*(this->current_vehicle_world_position_.position_.x_ - traffic_rule_center_point.x_)+(this->current_vehicle_world_position_.position_.y_ - traffic_rule_center_point.y_)*(this->current_vehicle_world_position_.position_.y_ - traffic_rule_center_point.y_));
//                 this->current_vehicle_world_position_mutex_.unlock();
//                 if (Tools::isLarge(remain_distance, MAX_DISTANCE_TO_DETECT_TRAFFIC_LIGHT) || Tools::isLarge(direction_distance, MAX_DISTANCE_TO_DETECT_TRAFFIC_LIGHT)) {
//                     LOG(INFO) << "离红绿灯超过60米，不进行处理";
//                     continue;
//                 }
//                 LOG(INFO) << "traffic light remain distance is " << remain_distance;

//                 // 调用服务
//                 traffic_light_msgs::traffic_lights traffic_light_service;
//                 // 提供服务所需参数
//                 traffic_light_service.request.direction = this->traffic_rule_obstacles_raw_[i].traffic_light_mode;
//                 traffic_light_service.request.num = this->traffic_rule_obstacles_raw_[i].traffic_light_id;
//                 // 调用服务
//                 this->traffic_light_service_client_.call(traffic_light_service);
//                 LOG(INFO) << "traffic light number " << static_cast<int>(this->traffic_rule_obstacles_raw_[i].traffic_light_id) << " direction " << static_cast<int>(this->traffic_rule_obstacles_raw_[i].traffic_light_mode) << "is called";
//                 // 得到交通灯结果
//                 int traffic_light_result = traffic_light_service.response.move_signal;
//                 if (traffic_light_result == 1) {
//                     // 绿灯，空气墙无效
//                     LOG(INFO) << "方向" << this->traffic_rule_obstacles_raw_[i].traffic_light_mode << "前方绿灯，空气墙无效";
//                 } else {
//                     LOG(INFO) << "方向" << this->traffic_rule_obstacles_raw_[i].traffic_light_mode << "前方红灯，空气墙有效, time consume is" << traffic_light_service.response.consume_time;
//                     if (Tools::isLarge(traffic_light_service.response.consume_time, CONSUME_TIME_THRESHOLD_FOR_YELLOW_LIGHT_JUDGEMENT)) {
//                         LOG(INFO) << "黄灯阶段";
//                         // 判断红灯是否可以闯过去
//                         // double remain_time = std::max(2.8 - traffic_light_service.response.consume_time, 0.0);
//                         // std::cout << "remian time is" << remain_time << std::endl;                            
//                         // // 判断是否可以超过, 1.以最大加速度无法刹车停下，此时选择通过 2. 减速到3.0米/秒仍然可以通过，此时选择通过
//                         // this->current_vehicle_movement_mutex_.lock();
//                         // double current_velocity = this->current_vehicle_movement_.velocity_;
//                         // this->current_vehicle_movement_mutex_.unlock();
//                         // LOG(INFO) << "黄灯还剩余时间" << remain_time << "剩余距离为" << remain_distance << ",当前速度" << current_velocity;
//                         // double stop_distance = current_velocity * current_velocity / 5.0;
//                         // if (Tools::isLarge(remain_distance, stop_distance + 0.5 * this->vehicle_length_)) {
//                         //     // 能够以最大加速度刹车停下
//                         //     LOG(INFO) << "刹车能够刹下来, 给停车";
//                         //     this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
//                         // } else {
//                         //     // 以最大加速度无法刹车停下
//                         //     LOG(INFO) << "当前速度无法刹车停下";
//                         // }
                        
//                         // if (Tools::isLarge(travellable_distance, remain_distance + this->vehicle_length_)) {
//                         //     LOG(INFO) << "满足条件1，可以通过";
//                         // } else if (Tools::isLarge(stop_distance, remain_distance - 0.5 * this->vehicle_length_)){
//                         //     LOG(INFO) << "满足条件2，可以通过";
//                         // } else {
//                         //     LOG(INFO) << "不可以通过";
//                         //     this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
//                         // }
//                         // 第一种情况车已经到停止线上，直接走
//                         // 第二种情况车还没停止线，进行判断
//                         // 黄灯阶段逻辑梳理
//                         // 1. 已知数据：黄灯剩余时间，车辆中心离停止线距离，车长，车辆当前速度，车辆最低速度？
//                         // 2. 如果在剩余时间内车辆中心能够超过停止线，则直接忽略红灯。
//                         // 3. 如果不满足上诉条件，判断以最大加速度是否可以保持车头在停止线内，如果可以则停下。
//                         // 4. 如果不满足上诉条件，直接忽略红灯
//                         if (Tools::isSmall(remain_distance, VEHICLE_OUTOF_TRAFFIC_LINE_JUDGEMENT_COEF * this->vehicle_length_)) {
//                             // 车已经到停止线上，直接走
//                             LOG(INFO) << "车在线上，直接走";
//                         } else {
//                             // 黄灯的总时长
//                             double total_time = YELLOW_LIGHT_DURATION;
//                             // 计算黄的剩余的时长
//                             double remain_time = std::max(total_time - traffic_light_service.response.consume_time, 0.0);
//                             std::cout << "remain time is" << remain_time << std::endl;
//                             this->current_vehicle_movement_mutex_.lock();
//                             double current_velocity = this->current_vehicle_movement_.velocity_;
//                             this->current_vehicle_movement_mutex_.unlock();
//                             LOG(INFO) << "黄灯还剩余时间" << remain_time << "剩余距离为" << remain_distance << ",当前速度" << current_velocity;
//                             // 判断闯黄灯时能走过的距离,采用的是匀变速运动
//                             double max_travel_distance = current_velocity + std::min(current_velocity, MAX_VELOCITY_TO_OVERTAKE_TRAFFIC_LINE_IN_YELLOW_LIGHT) * 0.5 * remain_time;
//                             // 判断在黄灯前紧急停车需要的距离
//                             double max_stop_distance = current_velocity * current_velocity / MAX_ACCELERATION_IN_YELLOW_LIGHT;
//                             LOG(INFO) << "最大行驶距离为" << max_travel_distance;
//                             LOG(INFO) << "最大刹车距离为" << max_stop_distance;
//                             // 开始判断是刹车还是开过去
//                             if (Tools::isSmall(remain_distance - max_travel_distance, VEHICLE_OUTOF_TRAFFIC_LINE_JUDGEMENT_COEF * this->vehicle_length_)) {
//                                 // 可以开过去
//                                 LOG(INFO) << "当前速度可以开过去";
//                             } else {
//                                 if (Tools::isLarge(remain_distance - max_stop_distance, this->vehicle_length_ * 0.5)) {
//                                     LOG(INFO) << "当前速度可以停下";
//                                     this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
//                                 } else {
//                                     LOG(INFO) << "危险阶段，无法进行判断";
//                                     if (Tools::isLarge(current_velocity, MIN_VELOCITY_TO_OVERTAKE_TRAFFIC_LINE_IN_YELLOW_LIGHT_WITH_EMERGENCY)) {
//                                         // 车速大于5.0米，直接过
//                                     } else {
//                                         //车速小于5.0米，刹车
//                                         this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
//                                     }
//                                 }
//                             }
//                         }
//                     } else {
//                         LOG(INFO) << "非黄灯阶段";
//                         // // 判断车辆刹车停下是否会超出停止线
//                         // this->current_vehicle_movement_mutex_.lock();
//                         // double current_velocity = this->current_vehicle_movement_.velocity_;
//                         // this->current_vehicle_movement_mutex_.unlock();
//                         // if (Tools::isSmall(remain_distance, this->vehicle_length_ * 0.5) && !Tools::isZero(current_velocity)) {
//                         //     // 当前速度停下，车辆不会超出停止线
//                         //     LOG(INFO) << "车头已经超线，直接走";
//                         // } else {
//                         //     LOG(INFO) << "无法保持停车不超过车道线";
//                         //     this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
//                         // }
//                         // 如果车压在停止线上且速度大于2m/s，可以走
//                         this->current_vehicle_movement_mutex_.lock();
//                         double current_velocity = this->current_vehicle_movement_.velocity_;
//                         this->current_vehicle_movement_mutex_.unlock();
//                         if (Tools::isSmall(remain_distance, this->vehicle_length_ * VEHICLE_OUTOF_TRAFFIC_LINE_JUDGEMENT_COEF) && Tools::isLarge(current_velocity, MIN_VELOCITY_TO_OVERTAKE_TRAFFIC_LINE_IN_RED_LIGHT)) {
//                             // 走
//                             LOG(INFO) << "闯红灯";
//                         } else {
//                             this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
//                         }
                        
//                     }
//                 }
//             } else {
//                 LOG(INFO) << "没有使用交通灯，交通灯空气墙当做临时空气墙处理";
//                 if (this->NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG_) {
//                     // 当满足条件1. 当前状态是停车状态，且速度为0。2. 当前位置离空气墙中心很近。此空气墙无效化
//                     bool is_validate = true;
//                     if (this->current_state_.getStateName() == StateNames::STOP && Tools::isZero(this->current_state_.getVehicleCurrentMovement().velocity_)) {
//                         // 满足第一个条件后，开始判断第二个条件
//                         // 计算空气墙中心点
//                         PathPlanningUtilities::Point2f traffic_rule_obstacle_center;
//                         traffic_rule_obstacle_center.x_ = 0.0;
//                         traffic_rule_obstacle_center.y_ = 0.0;
//                         for (size_t j = 0; j < this->traffic_rule_obstacles_raw_[i].points.size(); j++) {
//                             traffic_rule_obstacle_center.x_ += this->traffic_rule_obstacles_raw_[i].points[j].x;
//                             traffic_rule_obstacle_center.y_ += this->traffic_rule_obstacles_raw_[i].points[j].y;
//                         }
//                         traffic_rule_obstacle_center.x_ = traffic_rule_obstacle_center.x_ / this->traffic_rule_obstacles_raw_[i].points.size();
//                         traffic_rule_obstacle_center.y_ = traffic_rule_obstacle_center.y_ / this->traffic_rule_obstacles_raw_[i].points.size();
//                         // 计算距离
//                         double distance = PathPlanningUtilities::calcDistance(current_position_in_world.position_, traffic_rule_obstacle_center);
//                         // std::cout << "miaomiaomiao" << distance << std::endl;
//                         // std::cout << "center_position is " << traffic_rule_obstacle_center.x_ << "||" << traffic_rule_obstacle_center.y_ << std::endl; 
//                         // LOG(INFO) << "miaomiaomiao" << distance;
//                         // LOG(INFO) << "center_position is " << traffic_rule_obstacle_center.x_ << "||" << traffic_rule_obstacle_center.y_;
//                         if (Tools::isSmall(distance, MAX_DISTANCE_TO_NOT_PERMANENT_TRAFFIC_RULE_TO_MAKE_INVALID)) {
//                             is_validate = false;
//                         }
//                     }
//                     if (is_validate) {
//                         this->traffic_rule_obstacles_.push_back(this->traffic_rule_obstacles_raw_[i]);
//                     } else {
//                         LOG(INFO) << "车辆当前处于停止状态，临时空气墙无效，并且将其从原始数据中删除";
//                         this->traffic_rule_obstacles_raw_[i].mode = vec_map_cpp_msgs::VirtualObstacle::INVALID;
//                     }
//                 } else {
//                     LOG(INFO) << "临时空气墙未被使用，无效";
//                 }
//             }
//         } else if (this->traffic_rule_obstacles_raw_[i].mode == vec_map_cpp_msgs::VirtualObstacle::INVALID){
//             // 空气墙模式为无效模式，不用处理
//             LOG(INFO) << "无效空气墙";
//         }
//     }
// }

// 启动ros订阅线程,50hz
void LowSpeedNavigation::listenRosMSGThread() {
    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Initiate and start planning thread
void LowSpeedNavigation::runLowSpeedNavigationPlanning() {
    std::thread ros_msgs_receiver_thread(&LowSpeedNavigation::listenRosMSGThread, this);
    std::thread navigation_planning_thread(&LowSpeedNavigation::lowSpeedNavigationPlanningThread, this);
    ros_msgs_receiver_thread.join();
    navigation_planning_thread.join();
}

