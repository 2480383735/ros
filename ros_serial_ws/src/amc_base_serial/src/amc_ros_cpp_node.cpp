
#include <iostream>
#include <cstdio>
#include <ros/ros.h>
#include <thread>
#include "amc_base_serial/amc_serial.hpp"
#include "amc_base_serial/amc_ros.hpp"

std::shared_ptr<CarBaseRos> car;

// 主函数
int main(int argc, char *argv[]) {
    // 执行 ros 节点初始化
    ros::init(argc,argv,"amc_ros_cpp_node");
    car = std::make_shared<CarBaseRos>();
    // 连接串口
    std::string serialNmaeParam = "ttyUSB*";
    if (argc > 1) { serialNmaeParam = argv[1]; }
    try {
        car->serialConnect(serialNmaeParam);
    } catch(const std::exception& e) {
        ROS_ERROR("Exception caught: %s", e.what());
        return 0;
    }
    // 小车使能
    car->machineEnable();
    // 创建线程
    std::thread t([](){ros::spin();});
    ros::Rate rate(50);
    while (ros::ok())
    {
        car->receivedFeedbackMessage();
        rate.sleep();
    }
    t.join();
    // 小车失能
    car->machineDisable();
    return 0;
}

