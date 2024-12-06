#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkSmartPointer.h>
#include <vtkCylinderSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <glog/logging.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <thread>
#include <csignal>

#include "laser_mapping.h"
#include "utils.h"

DEFINE_string(config_file, std::string(ROOT_DIR) + "config/avia.yaml", "path to config file");
DEFINE_string(bag_file, "/home/mspace/data/102.bag", "path to the ros bag");
DEFINE_string(time_log_file, std::string(ROOT_DIR) + "Log/time.log", "path to time log file");
DEFINE_string(traj_log_file, std::string(ROOT_DIR) + "Log/traj.txt", "path to traj log file");
DEFINE_string(trunk_log_file, std::string(ROOT_DIR) + "Log/trunk.txt", "path to trunk log file");

// 全局路径和树干容器
std::vector<pcl::PointXYZ> path_points;
std::map<int, std::string> trunk_ids; // 防止重复添加树干模型

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

void monitorTrajFile(const std::string& traj_file, pcl::visualization::PCLVisualizer::Ptr viewer) {
    std::ifstream infile(traj_file, std::ios::ate); // 从文件末尾开始
    if (!infile.is_open()) {
        LOG(ERROR) << "Unable to open trajectory file: " << traj_file;
        return;
    }
    infile.seekg(0, std::ios::end);  // 从文件末尾读取

    while (!viewer->wasStopped()) {
        std::string line;

        while (std::getline(infile, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            double timestamp, x, y, z, qx, qy, qz, qw;
            iss >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw;

            pcl::PointXYZ new_point(x, y, z);
            path_points.push_back(new_point);

            if (path_points.size() > 1) {
                viewer->addLine(
                    path_points[path_points.size() - 2], path_points.back(),
                    1.0, 0.0, 0.0,
                    "line_" + std::to_string(path_points.size()));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 每100ms轮询一次
    }
}

void monitorTrunkFile(const std::string &trunk_file, pcl::visualization::PCLVisualizer::Ptr viewer) {
    std::ifstream infile(trunk_file);
    if (!infile.is_open()) {
        LOG(ERROR) << "Unable to open trunk file: " << trunk_file;
        return;
    }

    std::string line;
    while (!viewer->wasStopped()) {
        infile.clear(); // Reset flags for re-reading
        infile.seekg(0, std::ios::beg); // Start from the beginning
        while (std::getline(infile, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            int trunk_id;
            float x, y, z, dx, dy, dz, radius, height;
            char dummy;
            iss >> trunk_id
                >> dummy >> x >> dummy >> y >> dummy >> z >> dummy
                >> dummy >> dx >> dummy >> dy >> dummy >> dz >> dummy
                >> radius >> height;

            if (trunk_ids.find(trunk_id) == trunk_ids.end()) {
                // Create cylinder
                auto cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
                cylinderSource->SetRadius(radius);
                cylinderSource->SetHeight(height);
                cylinderSource->SetResolution(30);
                cylinderSource->Update();

                auto transform = vtkSmartPointer<vtkTransform>::New();
                transform->Translate(x, y, z);

                Eigen::Vector3f axis(dx, dy, dz);
                Eigen::Vector3f y_axis(0, 1.0f, 0);
                Eigen::Vector3f rotation_axis = y_axis.cross(axis);
                float dot_product = std::clamp(y_axis.dot(axis), -1.0f, 1.0f);
                float angle = acos(dot_product);

                if (rotation_axis.norm() > 1e-6) {
                    rotation_axis.normalize();
                    transform->RotateWXYZ(angle * 180.0f / M_PI, rotation_axis[0], rotation_axis[1], rotation_axis[2]);
                }

                auto transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
                transformFilter->SetTransform(transform);
                transformFilter->SetInputConnection(cylinderSource->GetOutputPort());
                transformFilter->Update();

                std::string cylinder_id = "trunk_" + std::to_string(trunk_id);
                viewer->addModelFromPolyData(transformFilter->GetOutput(), cylinder_id);
                trunk_ids[trunk_id] = cylinder_id;
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// 读取路径文件并绘制轨迹线
void addTrajectory(pcl::visualization::PCLVisualizer::Ptr viewer, const std::string& traj_file) {
    std::ifstream infile(traj_file);
    if (!infile.is_open()) {
        LOG(ERROR) << "Unable to open trajectory file: " << traj_file;
        return;
    }

    std::vector<pcl::PointXYZ> path_points;
    std::string line;

    while (std::getline(infile, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        double timestamp, x, y, z, qx, qy, qz, qw;
        iss >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw;
        path_points.emplace_back(x, y, z);
    }

    // 添加路径线
    for (size_t i = 1; i < path_points.size(); ++i) {
        viewer->addLine(path_points[i - 1], path_points[i], 1.0, 0.0, 0.0, "line_" + std::to_string(i));
    }
}

// 读取树干数据并绘制圆柱体
void addMarkers(pcl::visualization::PCLVisualizer::Ptr viewer, const std::string& trunk_file) {
    std::ifstream infile(trunk_file);
    if (!infile.is_open()) {
        LOG(ERROR) << "Error: Unable to open trunk file: " << trunk_file;
        return;
    }

    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        int trunk_id;
        float x, y, z, dx, dy, dz, radius, height;
        char dummy;
        iss >> trunk_id
            >> dummy >> x >> dummy >> y >> dummy >> z >> dummy
            >> dummy >> dx >> dummy >> dy >> dummy >> dz >> dummy
            >> radius >> height;

        // 创建圆柱体
        auto cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
        cylinderSource->SetRadius(radius);
        cylinderSource->SetHeight(height);
        cylinderSource->SetResolution(30);
        cylinderSource->Update();

        // 使用 VTK 的变换来旋转和移动圆柱体
        auto transform = vtkSmartPointer<vtkTransform>::New();
        transform->Translate(x, y, z);

        Eigen::Vector3f axis(dx, dy, dz);
        Eigen::Vector3f y_axis(0, 1.0f, 0);  // VTK 圆柱默认方向
        Eigen::Vector3f rotation_axis = y_axis.cross(axis);  // 旋转轴
        float dot_product = std::clamp(y_axis.dot(axis), -1.0f, 1.0f);
        float angle = acos(dot_product);  // 旋转角度

        if (rotation_axis.norm() > 1e-6) {  // 避免零向量
            rotation_axis.normalize();
            transform->RotateWXYZ(angle * 180.0f / M_PI, rotation_axis[0], rotation_axis[1], rotation_axis[2]);
        }

        auto transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        transformFilter->SetTransform(transform);
        transformFilter->SetInputConnection(cylinderSource->GetOutputPort());
        transformFilter->Update();

        std::string cylinder_id = "trunk_" + std::to_string(trunk_id);
        viewer->addModelFromPolyData(transformFilter->GetOutput(), cylinder_id);
    }
}


ros::Time last_save_time(0);  // 记录上次保存时间

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    const std::string bag_file = FLAGS_bag_file;
    const std::string config_file = FLAGS_config_file;

    double accumulation_time = 10.0, distance = 7.0;
    ros::Duration save_interval(accumulation_time); 
    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>(accumulation_time, distance);
    if (!laser_mapping->InitWithoutROS(FLAGS_config_file)) {
        LOG(ERROR) << "laser mapping init failed.";
        return -1;
    }

    /// handle ctrl-c
    signal(SIGINT, SigHandle);

    // just read the bag and send the data
    LOG(INFO) << "Opening rosbag, be patient";
    rosbag::Bag bag(FLAGS_bag_file, rosbag::bagmode::Read);
    rosbag::View view(bag);

    LOG(INFO) << "Go!";
    for (const rosbag::MessageInstance &m : view) {
        ros::Time current_time = m.getTime();  // 获取当前消息时间

        auto livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
        if (livox_msg) {
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &livox_msg]() {
                    laser_mapping->LivoxPCLCallBack(livox_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
            continue;
        }

        auto point_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (point_cloud_msg) {
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &point_cloud_msg]() {
                    laser_mapping->StandardPCLCallBack(point_cloud_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
            continue;
        }

        auto imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg) {
            laser_mapping->IMUCallBack(imu_msg);
            continue;
        }

        if (current_time - last_save_time > save_interval) {
            LOG(INFO) << "Saving trajectory to: " << FLAGS_traj_log_file;
            laser_mapping->Savetrajectory(FLAGS_traj_log_file);
            LOG(INFO) << "Saving trunk data to: " << FLAGS_trunk_log_file;
            laser_mapping->Savetrunk(FLAGS_trunk_log_file);

            last_save_time = current_time;  // 更新上次保存时间
        }

        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
    }

    // 初始化可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    // 加载点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // 添加点云
    // std::string pcd_file = std::string(ROOT_DIR) + "/data/scans.pcd";
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
    //     LOG(ERROR) << "Couldn't read PCD file: " << pcd_file;
    //     return -1;
    // }
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
    // pcl::VoxelGrid<pcl::PointXYZ> vg;
    // vg.setInputCloud(cloud);
    // vg.setLeafSize(1.0f, 1.0f, 1.0f);
    // vg.filter(*filtered_cloud);

    // viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud, "point cloud");

    // 添加路径
    std::string traj_file = std::string(ROOT_DIR) + "Log/traj.txt";
    //addTrajectory(viewer, traj_file);

    // 添加树干圆柱体
    std::string trunk_file = std::string(ROOT_DIR) + "Log/trunk.txt";
    //addMarkers(viewer, trunk_file);

    // 启动监控线程
    std::thread log_monitor(monitorTrajFile, traj_file, viewer);

    // 启动可视化
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100); // 每100ms更新一次
    }
    // viewer->spin();

    log_monitor.join(); // 确保主线程退出时关闭监控线程
    return 0;
}
