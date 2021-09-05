/*
 * @Author: fjw 
 * @Date: 2021-04-23 12:48:37 
 * @Last Modified by: fjw
 * @Last Modified time: 2021-04-23 12:58:47
 */

#include "Common.hpp"
#include "Visualization.hpp"

// 删除可视化marker
visualization_msgs::Marker VisualizationMethods::visualizedeleteMarker(int id) {
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "world";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETE;
    delete_marker.id = id;
    return delete_marker;
}

// 删除从起始id开始的全部可视化marker
visualization_msgs::Marker VisualizationMethods::visualizedeleteAllMarker(int start_id) {
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "world";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_marker.id = start_id;
    return delete_marker;
}

// 构建颜色
std_msgs::ColorRGBA VisualizationMethods::color(double r, double g, double b, double a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

// 可视化低速状态下的全局导航
// 将路径转化为marker
visualization_msgs::Marker VisualizationMethods::visualizeCurvesToMarker(const std::vector<PathPlanningUtilities::CoordinationPoint> &curve, const std_msgs::ColorRGBA &color, int id, double fontsize) {
    visualization_msgs::Marker curve_marker;
    curve_marker.header.frame_id = "world";
    curve_marker.header.stamp = ros::Time::now();
    curve_marker.type = visualization_msgs::Marker().LINE_STRIP;
    curve_marker.color = color;
    curve_marker.id = id;
    geometry_msgs::Vector3 v3c;
    v3c.x = fontsize;
    curve_marker.scale = v3c;
    for (size_t i = 0; i < curve.size(); i++) {
        PathPlanningUtilities::CoordinationPoint curve_point = curve[i];
        geometry_msgs::Point point;
        point.x = curve_point.worldpos_.position_.x_;
        point.y = curve_point.worldpos_.position_.y_;
        curve_marker.points.push_back(point);
    }
    return curve_marker;
}

// 将路径转化为marker
visualization_msgs::Marker VisualizationMethods::visualizeCurvesToMarker(const PathPlanningUtilities::Path &curve, const std_msgs::ColorRGBA &color, int id, double fontsize) {
    visualization_msgs::Marker curve_marker;
    curve_marker.header.frame_id = "world";
    curve_marker.header.stamp = ros::Time::now();
    curve_marker.type = visualization_msgs::Marker().LINE_STRIP;
    curve_marker.color = color;
    curve_marker.id = id;
    geometry_msgs::Vector3 v3c;
    v3c.x = fontsize;
    curve_marker.scale = v3c;
    for (size_t i = 0; i < curve.size(); i++) {
        PathPlanningUtilities::Point2f curve_point = curve[i];
        geometry_msgs::Point point;
        point.x = curve_point.x_;
        point.y = curve_point.y_;
        curve_marker.points.push_back(point);
    }
    return curve_marker;
}

// 在特定位置生成圆球marker
visualization_msgs::Marker VisualizationMethods::visualizeSphere(double position_x, double position_y, double radius, std_msgs::ColorRGBA color, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.id = id;
    marker.color = color;
    marker.pose.position.x = position_x;
    marker.pose.position.y = position_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = 0.01;
    return marker;
}

// 将障碍物形状转化为maker，障碍物类型为obstacle
visualization_msgs::Marker VisualizationMethods::visualizeObstacleShape(const Obstacle &obstacle, int id) {
    visualization_msgs::Marker obstacle_marker;
    obstacle_marker.header.frame_id = "world";
    obstacle_marker.header.stamp = ros::Time::now();
    obstacle_marker.type = visualization_msgs::Marker().CUBE;
    obstacle_marker.id = id;
    obstacle_marker.scale.x = std::max(obstacle.getObstacleLength(), 0.1);
    obstacle_marker.scale.y = std::max(obstacle.getObstacleWidth(), 0.1);
    obstacle_marker.scale.z = 2.0;
    obstacle_marker.pose.position.x = obstacle.getObstaclePosition().x_;
    obstacle_marker.pose.position.y = obstacle.getObstaclePosition().y_;
    obstacle_marker.pose.position.z = 0.0;
    obstacle_marker.pose.orientation.x = 0.0;
    obstacle_marker.pose.orientation.y = 0.0;
    obstacle_marker.pose.orientation.z = sin(obstacle.getObstacleOrientation()/2.0);
    obstacle_marker.pose.orientation.w = cos(obstacle.getObstacleOrientation()/2.0);
    if (Tools::isZero(obstacle.getObstacleVelocity())) {
        // 这里表示障碍物是静止障碍物
        std_msgs::ColorRGBA color;
        color.r = 1;
        color.g = 0;
        color.b = 0;
        color.a = 0.8;
        obstacle_marker.color = color;
    } else {
        // 这里表示障碍物不是静止障碍物
        std_msgs::ColorRGBA color;
        color.r = 0;
        color.g = 1;
        color.b = 0;
        color.a = 0.8;
        obstacle_marker.color = color;
    }
    return obstacle_marker;
}

// 将障碍物可视化
void VisualizationMethods::visualizeObstacles(const std::vector<Obstacle> &obstacles, const ros::Publisher &publisher) {
    // 首先清空之前的可视化
    visualization_msgs::MarkerArray delete_marker_array, obstacle_marker_array;
    // delete_marker_array.markers.push_back(visualizedeleteAllMarker(0));
    // publisher.publish(delete_marker_array);
    // 开始可视化
    int count = 0;
    // 首先是障碍物本身
    for (size_t i = 0; i < obstacles.size(); i++) {
        obstacle_marker_array.markers.push_back(visualizeObstacleShape(obstacles[i], count));
        count++;
    }
    // // 然后是障碍物的速度
    // for (size_t i = 0; i < obstacles.size(); i++) {
    //     obstacle_marker_array.markers.push_back(visualizeObstacleVelocity(obstacles[i], count));
    //     count++;
    // }
    // // 最后是障碍物轨迹
    // for (size_t i = 0; i < obstacles.size(); i++) {
    //     std::vector<visualization_msgs::Marker> predicted_trajectory_markers = visualizeObstacleTrajectory(obstacles[i], count);
    //     count += predicted_trajectory_markers.size();
    //     obstacle_marker_array.markers.insert(obstacle_marker_array.markers.end(), predicted_trajectory_markers.begin(), predicted_trajectory_markers.end());
    // }
    publisher.publish(obstacle_marker_array);
}
