/*
 * @Author: fjw 
 * @Date: 2021-04-23 12:48:03 
 * @Last Modified by: fjw
 * @Last Modified time: 2021-04-23 12:58:39
 */

#include "Common.hpp"


namespace VisualizationMethods {
// 删除可视化marker
visualization_msgs::Marker visualizedeleteMarker(int id);

// 删除从起始id开始的全部可视化marker
visualization_msgs::Marker visualizedeleteAllMarker(int start_id);

// 构建颜色
std_msgs::ColorRGBA color(double r, double g, double b, double a);

visualization_msgs::Marker visualizeCurvesToMarker(const std::vector<PathPlanningUtilities::CoordinationPoint> &curve, const std_msgs::ColorRGBA &color, int id, double fontsize);

visualization_msgs::Marker visualizeCurvesToMarker(const PathPlanningUtilities::Path &curve, const std_msgs::ColorRGBA &color, int id, double fontsize);

// 在特定位置生成圆球marker
visualization_msgs::Marker visualizeSphere(double position_x, double position_y, double radius, std_msgs::ColorRGBA color, int id);

// 将障碍物形状转化为maker，障碍物类型为obstacle
visualization_msgs::Marker visualizeObstacleShape(const Obstacle &obstacle, int id);

// 将障碍物可视化
void visualizeObstacles(const std::vector<Obstacle> &obstacles, const ros::Publisher &publisher);


};  // namespace VisualizationMethods