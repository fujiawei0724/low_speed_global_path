/*
 * @Author: fjw 
 * @Date: 2021-04-17 11:57:00 
 * @Last Modified by: fjw
 * @Last Modified time: 2021-04-23 16:48:47
 */


#include "LowSpeedNavigation.hpp"


int main(int argc, char** argv) {
    // Initiate log file
    std::cout << "low speed navigation start" << std::endl;
    // Read project content
    std::string root_path = ros::package::getPath("low_speed_navigation_planning");
    std::cout << "访问系统路径成功" << std::endl;
    std::string log_file_path = "/log/";
    log_file_path = root_path + log_file_path;
    Tools::resetLogFile(log_file_path);
    std::cout << "创建日志文件夹成功" << log_file_path << std::endl;
    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::GLOG_INFO, log_file_path.c_str());
    std::cout << "创建日志流成功" << std::endl;
    // // Delete previous low speed navigation planning result
    // std::string record_file_path = "/navigation_data_record/";
    // record_file_path = root_path + record_file_path;
    // Tools::resetLogFile(record_file_path);
    // boost::filesystem::remove_all(record_file_path);
    // std::cout << "清空之前的导航路径记录" << std::endl;

    // For automatic test
    // 首先读取本工程的目录
    std::string file_path = ros::package::getPath("low_speed_navigation_planning");
    // 创建数据文件夹
    file_path += "/navigation_data_record/";
    Tools::resetLogFile(file_path);
    // 读取系统当前时钟作为文件名称
    file_path += Tools::returnCurrentTimeAndDate() + "flag.csv";
    std::ofstream file(file_path);
    if (file) {

        // record
        file << "flag" << "\n";
        
    }
    file.close();

    std::string unoptimized_record_file_path = "/unoptimized_path_data_record/";
    unoptimized_record_file_path = root_path + unoptimized_record_file_path;
    Tools::resetLogFile(unoptimized_record_file_path);
    boost::filesystem::remove_all(unoptimized_record_file_path);
    std::cout << "清空之前的未优化导航路径记录" << std::endl;
    // Initiate ros 
    ros::init(argc, argv, "low_speed_navigation_planning");
    ros::NodeHandle nh("~");
    std::unique_ptr<LowSpeedNavigation> low_speed_navigation_ptr(new LowSpeedNavigation(nh));
    low_speed_navigation_ptr->runLowSpeedNavigationPlanning();
    // Shut down log file 
    google::ShutdownGoogleLogging();
    return 0;
}