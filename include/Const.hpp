/*
    Copyright [2020] Jian ZhiQiang
*/

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// 常量整理
// 重要常量
#define PI 3.141592653589793238462643383279  // pi
#define EPS 1.0e-8  // 浮点数比较的精度
#define LESS_THAN_BOUNDARY_RATION 0.9 // 为了避免达到边界值的系数
#define MAX_VALUE 10000000.0  // 最大数
#define MIN_VALUE -100000000.0  // 最小数
#define DISTANCE_FROM_REAR_TO_CENTER 1.4249  // 后轴中心与几何中心之间的距离
#define VELOCITY_THRESHOLD 4.5  // 速度阈值，小于阈值为低速，大于为高速
#define LANE_GAP_DISTANCE 0.1  // 道路点之间的间距
#define AVOIDANCE_DEVIATION_GAP 0.3  // 待选路径之间的横向间距
#define AVOIDANCE_MIN_DISTANCE 6.0  // 避障经过的最小距离
#define CONSTANT_DISTANCE 12.5  // 规划最小距离
#define GAP_MAINTAIN 8.0  // 停车后与前车距离(gap)
#define MAX_BIAS 1.8  // 避障最大横向偏移量
#define MIN_SPEED 2.0  // 可观察到的最小速度(小于这个速度的障碍物将被视为静止)
#define SUBVEHICLE_COMMON_DECCELERATION 2.0  // 正常情况下汽车的加速度(适用于本车,用于计算规划期望距离)
#define MAX_DECCELERATION 5.0  // 根据RSS的文档，正常汽车刹车的减速度为3m/s^2 ~ 8m/s^2(用于安全距离计算和障碍物占用区域长度计算)
#define COMMON_DECCELERATION 3.2  // 设定的减速正常减速度(适用于本车,用于限制最大减速度)
#define MAX_NORMAL_ACCELERATION 1.8  // 根据文档，正常汽车的法向加速度不大于1.8m/s^2时人没有反应，不大于3.6m/s^2时人可以忍受(适用于本车,用于限制最大加速度和最大曲率)
#define MAX_NORMAL_JERK 1.0  // 根据文档，汽车法向加加速度在0.4m/s^3 ~ 1.0m/s^3时，人可以忍受(适用于本车,用于限制最大曲率)
#define OBSTACLE_MARGIN 1.0  // 障碍物预测路径点之间的间隔
#define EXTENDED_PATH_SMOOTH_INDEXES 20  // 延伸路径用于生成路径平滑的长度
#define SAMPLE_LENGTH 5.0  // 规划时的纵向采样间隔
#define AVOIDANCE_VELOCITY 1.9  // 避障状态下设定的目标速度
#define EXTENDED_DISTANCE_FOR_LOW_SPEED 17.5  // 低速情况下的拓展距离[m]
#define EXPECTED_DECCELERATION_FOR_LOW_SPEED -1.0  // 低速情况下的期望减速度
#define PRIORITY_INIT_VALUE_MAX 10.0  // 优先级初始化值
#define PRIORITY_INIT_VALUE_HIGH 7.5  // 优先级初始化值
#define PRIORITY_INIT_VALUE_MEDIAN 5.0  // 优先级初始化值
#define PRIORITY_INIT_VALUE_LOW 2.5  // 优先级初始化值
#define PRIORITY_INIT_VALUE_MIN 0.0  // 优先级初始化值
#define VELOCITY_MAX_INCREASE_VALUE 5.0  // 在一次规划过程中,速度能够增加的最大量


// DecisionMaking.cpp中的常量
#define STATE_CHOOSING_ACCELERATION_THRESHOLD 0.2  // 在状态选择过程中,判断状态的加速度是否小于阈值0.2,如果小于阈值,则判定为在进行减速
#define PRIORITY_INCREMENT_VALUE 25.0  // 在状态选择过程中,没有受任何影响下初始的优先级增量
#define PRIORITY_INCREMENT_VALUE_LOWER 24.5  // 在状态选择过程中,没有受任何影响下初始的优先级增量(只针对右换道,因为右换道是不推荐的)
#define SHORTEST_DISTANCE_TO_PUBLISH_WHEN_STOP 1.6  // 当距离小于1.6米的时候,且当前车为停止状态,不发布路径,该路径是无意义的
#define MAX_STOP_COUNT 5  // 连续规划出停车状态的最多次数,如果连续规划出停车状态的次数大于5,则进入特殊逻辑
#define STATE_MAINTAIN_FREQUENCY 2  // 状态保持过程中的判断频率
#define VELOCITY_UPDATE_FREQUENCY_IN_STATE_MAINTAIN 0.5  // 状态保持过程中,速度曲率更新的频率
#define STOP_STATE_FINISH_MIN_TRAVELED_DISTANCE 1.0  // 结束停车状态必须走过的最小距离
#define TO_EXPECT_STATE_MAX_HEAD_OFFSET 0.3  // 从停止状态切换到期望状态的最大车头偏差
#define TO_EXPECT_STATE_MAX_THETA_OFFSET 0.01  // 从停止状态切换到期望状态的最大角度偏差

// SubVehicle.cpp中的常量
#define ROS_UPDATE_FREQUENCY 50  // ros消息接收频率
#define REACH_DESTINATION_MAX_DISTANCE 1.8  // 离目标在1.8内则判断到达终点
#define MOTION_PLANNING_FREQUENCY 20  // 运动规划的频率

// MotionPlanning.cpp中的常量
#define NAVIGATION_LENGTH_ENOUGH_MIN_VALUE 100.0  // 如果全局导航小于100.0,则全局导航长度不足以进行超车
#define NAVIGATION_LENGTH_ENOUGH_MIN_COEF 20.0  // 如果全局导航小于速度上限*20.0,则全局导航长度不足以进行超车
#define LOCAL_PLANNING_LONGITUDE_SAMPLE_COEF 0.3  // 局部规划纵向采样系数
#define LOCAL_PLANNING_LONGITUDE_SAMPLE_MIN_LENGTH 5.0  // 局部规划纵向采样最短规划距离[m]
#define MIN_VELOCITY_FOR_ACCEPTABLE_CURVATURE_CALCULATION 1.0  // 用于计算可接受曲率的最小速度
#define LONGITUDE_PATH_SELECTION_YAW_WEIGHT 6.0  // 纵向最优路径选择的角度权重

// ObstacleAndRSS.cpp中的常量
#define OBSTACLE_OCCUPANCY_WIDTH_MAX_TO_LANE_WIDTH 0.8 // 障碍物占用区域宽度占道路宽度的最大比例
#define OBSTACLE_OCCUPANCY_WIDTH_EXPAND_CONST 0.4  // 障碍物占用区域宽度膨胀大小
#define MAX_DISTANCE_TO_NOT_PERMANENT_TRAFFIC_RULE_TO_MAKE_INVALID 10.8  // 离临时空气墙距离小于10.8米,则空气墙无效化
#define MAX_DISTANCE_TO_DETECT_TRAFFIC_LIGHT 60.0  // 大于60米则不进行红绿灯检测
#define CONSUME_TIME_THRESHOLD_FOR_YELLOW_LIGHT_JUDGEMENT 0.001  // consume_time大于0.001则判定为黄灯
#define VEHICLE_OUTOF_TRAFFIC_LINE_JUDGEMENT_COEF 0.2  // 如果车中心点到停止线距离小于车长的0.2,则认为不需要继续进行红绿灯检测,直接通过
#define YELLOW_LIGHT_DURATION 2.8  // 黄灯的持续时间
#define MAX_VELOCITY_TO_OVERTAKE_TRAFFIC_LINE_IN_YELLOW_LIGHT 3.5  // 黄灯时闯过停止线的最大速度
#define MAX_ACCELERATION_IN_YELLOW_LIGHT 6.0  // 黄灯时进行紧急刹车的最大加速度
#define MIN_VELOCITY_TO_OVERTAKE_TRAFFIC_LINE_IN_YELLOW_LIGHT_WITH_EMERGENCY 5.0  // 如果车速大于5米/秒,无法杀下来,则选择闯黄灯
#define MIN_VELOCITY_TO_OVERTAKE_TRAFFIC_LINE_IN_RED_LIGHT 2.0  // 红灯时闯过停止线的最低速度,如果小于2米/秒,则选择刹车
#define OBSTACLE_OCCUPANCY_AREA_SAMPLING_GAP_FOR_STATIC_OBSTACLE 1  // 静态障碍物障碍物占用区域采样间隔(不进行采样)
#define ALOWED_RESPONSE_TIME_WHEN_SAFE_DISTANCE_NOT_SATISFIED 2.0  // 在无法满足安全距离的时候,允许车辆进行反应的最长时间(车辆可以在这个时间内调整速度来满足安全模型)
#define ANOTHER_MAX_ACCELERATION_FOR_WAIT_MIN_VALUE -0.6  // 安全模型第二种情形下的加速度最小边界
#define SAFE_DISTANCE_CALCULATION_ACCELERATION_SAMPLING_GAP 0.1  // 安全模型计算过程中加速度采样间隔
#define EXPECTED_ACCELERATION_TO_MAX_ACCELERATION_RATIO 0.8 // 期望加速度不超过0.8乘最大速度
#define MAX_ACCELERATION_FOR_NOT_FOLLOW_GUIDANCE 0.5  // 非任务导向下的最大加速度
#define TIME_TO_EXPECTED_MAX_VELOCITY_FOLLOW_GUIDANCE 2.0  // 在任务导向下从当前速度到期望最大速度所花时间
#define TIME_TO_VELOCITY_FOR_NOT_FOLLOW_GUIDANCE 4.0  // 在非任务导向下从当前速度到非任务导向速度速度所花时间
#define VELOCITY_TO_EXPECTED_VELOCITY_RADIO_FOR_NOT_FOLLOW_GUIDANCE 0.5  // 非任务导向下的速度占期望速度比例
#define TIME_TO_EXPECTED_VELOCITY_WHEN_DECCELERATION 3.5  // 减速时达到期望速度时的时间
#define SMALL_VALUE_FOR_AVOID_REACH_BOUNDARY 0.01  // 一个小量,避免达到边界
#define DYNAMIC_OBSTACLE_EXPAND_LENGTH_AS_OCCUPANCY 0.4  // 动态障碍物作为占用区时的长度膨胀常量
#define DYNAMIC_OBSTACLE_EXPAND_LENGTH_NOT_AS_OCCUPANCY 0.4  // 动态障碍物不作为占用区时的长度膨胀常量
#define DYNAMIC_OBSTACLE_EXPAND_WIDTH_NOT_AS_OCCUPANCY 0.4  // 动态障碍物不作为占用区时的宽度膨胀常量
#define STATIC_OBSTACLE_EXPAND_LENGTH_AS_OCCUPANCY 1.4  // 动态障碍物作为占用区时的长度膨胀常量
#define STATIC_OBSTACLE_EXPAND_WIDTH_AS_OCCUPANCY 1.4  // 动态障碍物作为占用区时的宽度膨胀常量
#define STATIC_OBSTACLE_EXPAND_LENGTH_NOT_AS_OCCUPANCY 0.1  // 动态障碍物不作为占用区时的长度膨胀常量
#define STATIC_OBSTACLE_EXPAND_WIDTH_NOT_AS_OCCUPANCY 0.1  // 动态障碍物不作为占用区时的宽度膨胀常量
#define EXTRA_EXPAND_MIN_VELOCITY 2.0  // 本车大于该速度时,障碍物要进行额外的膨胀
#define EXTRA_EXPAND_VELOCITY_COEF 0.2  // 进行额外膨胀时的速度系数
#define MAX_EXTRA_EXPAND_FOR_VELOCITY 2.0  // 由障碍物导致的最大膨胀
#define VIRTUAL_OBSTACLE_WIDTH 0.15  // 空气墙障碍物的宽度
#define RECTANGLE_INTERACTION_EXPAND_RATION 1.0  // 判断距离碰撞时的膨胀系数
#define SUBVEHICLE_OCCUPANCY_SAMPLING_GAP_FOR_TRAFFIC_CHECK 30  // 本车占用区域采样间隔(交通检测时)
#define SUBVEHICLE_OCCUPANCY_WIDTH_EXPAND_RATIO_TRAFFIC_CHECK 1.0  // 本车占用区域宽度膨胀系数(交通检测时)
#define SUBVEHICLE_OCCUPANCY_LENGTH_EXPAND_RATIO_TRAFFIC_CHECK 1.0  // 本车占用区域长度膨胀系数(交通检测时)
#define SUBVEHICLE_OCCUPANCY_WIDTH_EXPAND_RATIO_OBSTACLE_CHECK 1.2  // 本车占用区域宽度膨胀系数(障碍物检测时)
#define SUBVEHICLE_OCCUPANCY_LENGTH_EXPAND_RATIO_OBSTACLE_CHECK 1.1  // 本车占用区域长度膨胀系数(障碍物检测时)
#define BIGGER_OBSTACLE_EXTRA_EXPAND_CONST 0.3  // 额外膨胀0.3米来判断障碍物与本车的距离是否大于0.3米
#define DYNAMIC_OBSTACLE_COLLISION_CHECK_SAMPLING_GAP 3  // 动态障碍物碰撞判断时的障碍物占用区域采样间隔
#define OBSTACLE_TO_PATH_MAX_DISTANCE 10.0  // 障碍物到路径最大距离

// SpecialMotionPlanning.cpp中的常量
#define MIN_LATERAL_OFFSET_TO_BE_CONSIDERED 0.3 // 小于这个横向偏移量的将被忽略不计
#define LONGITUDE_SAMPLING_RATIO_FOR_LOW_SPEED_STATE 0.3  // 低速状态下的纵向目标点采样系数
#define MIN_LONGITUDE_PLANNING_LENGTH_FOR_LOW_SPEED_STATE 5.0  // 低速状态下的最短规划距离
#define LONGITUDE_PATH_SELECTION_YAW_WEIGHT_FOR_LOW_SPEED_STATE 6.0  // 纵向最优路径选择的角度权重
#define AIM_ACCELERATION_SAMPLING_GAP_FOR_LOW_SPEED_STATE 0.45  // 低速状态下加速度的采样间隔
#define TO_CHOOSE_PATH_LONGER_DISTANCE  6.0  // 只有在一条路径比另一条路径长6米时,才认为存在长度差异
#define SINGLE_LANE_AND_TRAFFIC_FACTOR_STOP_DISTANCE_SHRINK_RATIO 0.6  // 在单车道,交通规则停车时的停车距离缩短系数
#define TRAFFIC_FACTOR_STOP_DISTANCE_SHRINK_RATIO 0.85  // 交通规则停车时的停车距离缩短系数
#define KAPPA_TO_PRIORITY_FACTOR 10.0  // 曲率转化为优先级系数
#define LANE_CENTER_ADDED_PRIORITY_CONST 1.0  // 中心线的优先级增量
#define THRESHOLD_DISTANCE_REGARD_BIG_AND_SMALL_OBSTACLE_SAME 1.5  // 在这个距离内则认为大的和小的障碍物不存在差异
#define MAX_THETA_TO_ALLOW_QUICK_PASS 0.2  // 允许快速通过的最大偏差角度
#define AVOIDANCE_STATE_MIN_VELOCITY 2.0  // 蔽障状态下的最小速度
#define MAX_CURVE_EXTEND_FACTOR 1.2  // 低速状态下最长纵向拓展倍率
#define EXTEND_FACTOR_GAP 0.3  // 低速状态下的纵向采样间隔
#define MAX_CURVATURE 0.2  // 低速状态下最大曲率
#define EXPAND_RATIO 1.0  // 碰撞判定时的膨胀系数
#define MAX_DISTANCE_RATIO 0.8  // 障碍物最远考虑距离与车辆宽度比值

// 世界坐标系到frenet坐标系的变换矩阵
struct TransMatrix {
    Eigen::Matrix2d rotation_;
    Eigen::Vector2d trans_;
};

// 单区间（开区间）
struct Section {
    double min_;
    double max_;
};

