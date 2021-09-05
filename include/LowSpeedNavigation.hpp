/*
 * @Author: fjw 
 * @Date: 2021-04-17 11:56:39 
 * @Last Modified by: fjw
 * @Last Modified time: 2021-06-22 16:26:03
 */


#pragma once


#include "Common.hpp"
#include "StructureLowSpeed/LocalPath.hpp"
#include "StructureLowSpeed/PathSmoother.hpp"
#include "StructureLowSpeed/HeatTranformSmoother.hpp"
#include "StructureLowSpeed/StructureGuide.hpp"
#include "StructureLowSpeed/HybridAstar.hpp"
#include "StructureLowSpeed/Astar.hpp"
#include "StructureLowSpeed/ThetaStructureGuide.hpp"
#include "StructureLowSpeed/CubicBSpline.hpp"
#include "StructureLowSpeed/CubicSpline.hpp"
#include "StructureLowSpeed/Config.hpp"
#include "StructureLowSpeed/KDTree.hpp"



// Contact with ros
class LowSpeedNavigation {
public:
    // Contructor
    LowSpeedNavigation(const ros::NodeHandle &nh) {
        this->nh_ = nh;

        // Initiate contact to ROS
        this->initConnectionToRos();
    };
    
    // Destroctor
    ~LowSpeedNavigation() {

    };

    // Initiate and start thread
    void runLowSpeedNavigationPlanning();

private:

    // Contact to ROS
    void initConnectionToRos();

    // 启动ros订阅线程,50hz
    void listenRosMSGThread();

    // Low speed navigation main thread
    void lowSpeedNavigationPlanningThread();

    // Update location information
    void updateVehiclePosition(const nav_msgs::Odometry::ConstPtr odometry_msg);

    // Update vehicle movement information
    void updateVehicleMovement(const std_msgs::Float64::ConstPtr velocity_msg);
    
    // Update vehicle curvature
    void updateVehicleCurvature(const std_msgs::Float64::ConstPtr curvature_msg);

    // Update obstacle
    void getObstacles(const ibeo_lidar_msgs::object_filter_data::ConstPtr &msg);
    
    // get occupancy map
    void getOccupancyGridMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    
    // Update map information
    void updateMapInformation();

    // Update obstacle information
    void updateObstacleInformation();


    // 补全障碍物的状态信息，其中acceleration和class_name为无用值，只要给占位符即可
    void updateObstacleState(Obstacle* obstacle, const PathPlanningUtilities::Point2f position, double width, double length, double orientation, double velocity, double velocity_direction, double acceleration, size_t class_name);

    // // 补全障碍物的预测信息、轨迹和位置
    // void updateObstaclePredictedInformation(Obstacle* obstacle);

    // // 得到有效的交通障碍物列表
    // void updateValidateTrafficRuleInformation();

    // Judge whether need replanning 
    bool isNavigationPathNeedUpdate(const KDTree &kdtree);

    // Generate center line
    std::vector<PathPlanningUtilities::CoordinationPoint> generateCenterLine();

    // Generate center line for obstacle generation
    std::vector<PathPlanningUtilities::CoordinationPoint> generateCenterLineForObstacle();

    // 在道路两边产生虚拟障碍物并且和可感知到的障碍物汇总(用于障碍物距离优化)
    void generateAllObstacles(const std::vector<PathPlanningUtilities::CoordinationPoint> & structure_center_line);

    // 生成道路两旁的虚拟障碍物(考虑道路非直线的情况)
    void generateAllObstaclesConsiderCurve(const std::vector<PathPlanningUtilities::CoordinationPoint> & structure_center_line);

    // 构造KDTree,将已知障碍物构造成散点
    std::vector<PathPlanningUtilities::Point2f> generateObstalcesPoints();

    // 进行全局路径生成
    std::vector<PathPlanningUtilities::CoordinationPoint> generateReferenceLine(const std::vector<PathPlanningUtilities::CoordinationPoint> & structure_center_line, const KDTree &kdtree);


    // 判断产生的全局路径是否正常
    bool judgeNavigationSuitable(const std::vector<PathPlanningUtilities::CoordinationPoint> &reference_line);

    // Selector optimal navigation reference line
    bool selectOptimalReferenceLine(std::vector<PathPlanningUtilities::CoordinationPoint> &final_reference_line,std::vector<std::vector<PathPlanningUtilities::CoordinationPoint>> &candidate_reference_lines, const KDTree &kdtree);

    // navigation mode
    int navigation_mode_;

    // navigation planning front end failed recorder
    int front_end_failed_recorder_ = 0;


    
    // Parameters relate with ROS
    ros::NodeHandle nh_; // ROS handle 
    tf::TransformListener* tf_listener_ptr_;  // tf监听器
    tf::StampedTransform tf_transformer_;  // tf变换矩阵
    ros::Publisher visualization_global_reference_pub_; // Visualization publish
    ros::Publisher guided_path_pub_;  // Publish low speed navigation path
    ros::Subscriber occupancy_grid_sub_; // subscribe grid map
    ros::Subscriber odom_sub_; // Update position
    ros::Subscriber movement_sub_; // Update velocity
    ros::Subscriber curvature_sub_; // Update curvature
    ros::Subscriber obstacle_sub_;  // 障碍物信息更新节点
    ros::ServiceClient map_service_client_;  // 地图服务
    ros::ServiceClient obstacle_trajectory_prediction_service_client_;  // 障碍物轨迹预测服务
    

    // Mutex lock
    std::mutex current_vehicle_world_position_mutex_;  // Vehicle position in world coordination
    std::mutex current_vehicle_movement_mutex_;  // Vehicle movement information
    std::mutex current_vehicle_kappa_mutex_;  // Vehicle kappa
    std::mutex current_vehicle_world_position_ready_mutex_; // Vehicle position ready flag lock
    std::mutex current_vehicle_movement_ready_mutex_; // Vehicle movement ready flag lock
    std::mutex current_vehicle_kappa_ready_mutex_; // Vehicle kappa ready flag lock
    std::mutex obstacle_mutex_; // Obstacle lock
    std::mutex structure_road_obstacles_mutex_; // Structure road obstacle lock;
    std::mutex perception_object_mutex_; // Obstacle lock
    std::mutex occupancy_grid_mutex_; // occupancy grid lock
    

    // Vehicle information
    PathPlanningUtilities::VehicleState current_vehicle_world_position_;  // 车辆位置信息、朝向信息(世界坐标系下)
    PathPlanningUtilities::VehicleMovementState current_vehicle_movement_;  // 车辆的速度信息
    double current_vehicle_kappa_;  // 车辆曲率信息
    std::vector<PathPlanningUtilities::CoordinationPoint> history_reference_line_;  // 历史导航参考线
    std::vector<std::vector<PathPlanningUtilities::CoordinationPoint>> candidate_reference_lines_; // Candidate navigation line
    
    // Judge information upload state
    bool current_vehicle_world_position_ready_; // Vehicle information ready flag
    bool current_vehicle_movement_ready_; // Vehicle movement ready flag
    bool current_vehicle_kappa_ready_; // Vehicle kappa ready flag
    bool VEHICLE_OCCUPANCY_GRID_READY_FLAG_; // is occupancy map ready



    // Map related information
    Lane left_lane_;
    Lane right_lane_;
    Lane center_lane_;
    size_t guidance_type_;
    double expected_velocity_upper_bound_;
    bool is_avoidance_capable_;
    bool is_single_lane_;
    bool is_length_enough_;
    double remain_distance_;
    double distance_to_goal_;
    GridMap occupancy_grid_; 

    // Flag
    bool TRAFFIC_LIGHT_USAGE_FLAG_ = false;  // 交通灯是否使用
    bool NOT_PERMANENT_TRAFFIC_RULE_USAGE_FLAG_ = false;  // 临时空气墙是否使用
    bool IS_DANGEROUS_FLAG_ = false;  // 是否有障碍物导致状态不安全

    // Obstacle related information
    std::vector<ibeo_lidar_msgs::object_filter> perception_objects_;  // 感知的物体信息 
    std::vector<Obstacle> obstacles_;   // 障碍物列表
    std::vector<Obstacle> structure_road_obstacles_;    // 此障碍物用于进行结构化道路生成避障倒库，包含检测到的障碍物和道路两边点生成的虚拟障碍物
    std::vector<vec_map_cpp_msgs::VirtualObstacle> traffic_rule_obstacles_raw_;  // 由原始交通规则生成的障碍物
    std::vector<vec_map_cpp_msgs::VirtualObstacle> traffic_rule_obstacles_;  // 由上者得到的有效交通规则障碍物

};

