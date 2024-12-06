#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <signal.h>

#include "laser_mapping.h"


// 全局变量：rosbag 文件对象
rosbag::Bag bag;

// 信号处理函数，确保关闭 bag 文件
void SigHandle(int sig) {
    ROS_WARN("Received signal %d, closing rosbag...", sig);
    bag.close();
    ros::shutdown();
}

//
void CloudRegisteredCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    bag.write("/cloud_registered", ros::Time::now(), msg);
}

void PathCallback(const nav_msgs::Path::ConstPtr& msg) {
    bag.write("/path", ros::Time::now(), msg);
}

void TrunkArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    bag.write("/trunk_array", ros::Time::now(), msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosbag_recorder");
    ros::NodeHandle nh;

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>(10.0, 7.0);
    laser_mapping->InitROS(nh);

    // 打开 rosbag 文件
    bag.open("new_rosbag.bag", rosbag::bagmode::Write);

    // 注册信号处理函数
    signal(SIGINT, SigHandle);

    // 订阅需要录制的话题
    ros::Subscriber sub_cloud_registered = nh.subscribe("/cloud_registered", 1000, CloudRegisteredCallback);
    ros::Subscriber sub_path = nh.subscribe("/path", 1000, PathCallback);
    ros::Subscriber sub_trunk_array = nh.subscribe("/trunk_array", 1000, TrunkArrayCallback);

    // 循环等待消息
    ros::Rate rate(5000);
    while (ros::ok()) {
        ros::spinOnce();
        laser_mapping->Run();
        rate.sleep();
    }

    // 关闭 rosbag 文件
    bag.close();
    ROS_INFO("Rosbag recording completed.");
    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish();

    faster_lio::Timer::PrintAll();
    // LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    // laser_mapping->Savetrajectory(FLAGS_traj_log_file);
    // LOG(INFO) << "save trunk to: " << FLAGS_trunk_log_file;
    // laser_mapping->Savetrunk(FLAGS_trunk_log_file);

    return 0;
}
