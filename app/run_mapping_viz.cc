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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <thread>
#include <mutex>
#include <csignal>
#include "laser_mapping.h"

// struct TrunkCylinderDesc {
//     int trunk_id;              
//     Eigen::Vector3f position;  
//     Eigen::Vector3f direction; 
//     float radius;              
//     float height;              
// }

std::mutex viewer_mutex;
std::string config_file = std::string(ROOT_DIR) + "config/avia.yaml";
std::string bag_file = "/home/mspace/data/102.bag";
std::string time_log_file = std::string(ROOT_DIR) + "Log/time.log";
std::string traj_log_file = std::string(ROOT_DIR) + "Log/traj.txt";
std::string trunk_log_file = std::string(ROOT_DIR) + "Log/trunk.txt";

std::vector<pcl::PointXYZ> path_points;

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    std::cout << "Catch signal: " << sig << std::endl;
}

void addTrajectory(pcl::PointXYZ new_point, pcl::visualization::PCLVisualizer::Ptr viewer) {
    std::lock_guard<std::mutex> lock(viewer_mutex); 
    path_points.push_back(new_point);

    if (path_points.size() > 1) {
        viewer->addLine(
            path_points[path_points.size() - 2], path_points.back(),
            1.0, 0.0, 0.0,
            "line_" + std::to_string(path_points.size()));
    }
}

void addTrunkModel(TrunkCylinderDesc t, pcl::visualization::PCLVisualizer::Ptr viewer) {
    auto cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
    cylinderSource->SetRadius(t.radius);
    cylinderSource->SetHeight(t.height);
    cylinderSource->SetResolution(30);
    cylinderSource->Update();

    auto transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(t.position.x(), t.position.y(), t.position.z());

    Eigen::Vector3f axis = t.direction.normalized();
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

    std::lock_guard<std::mutex> lock(viewer_mutex);
    std::string cylinder_id = "trunk_" + std::to_string(t.trunk_id);
    viewer->addModelFromPolyData(transformFilter->GetOutput(), cylinder_id);
}

int main(int argc, char **argv) {
    double accumulation_time = 10.0, distance = 7.0;
    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>(accumulation_time, distance);

    if (!laser_mapping->InitWithoutROS(config_file)) {
        std::cerr << "laser mapping init failed." << std::endl;
        return -1;
    }

    // handle ctrl-c
    signal(SIGINT, SigHandle);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    std::thread viewer_thread([&viewer]() {
        while (!viewer->wasStopped()) {
            {
                std::lock_guard<std::mutex> lock(viewer_mutex);
                viewer->spinOnce(10);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    });

    // just read the bag and send the data
    std::cout << "Opening rosbag, be patient" << std::endl;
    rosbag::Bag bag(bag_file, rosbag::bagmode::Read);
    rosbag::View view(bag);

    std::cout << "Go!" << std::endl;
    for (const rosbag::MessageInstance &m : view) {
        if (faster_lio::options::FLAG_EXIT) break;

        auto livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
        if (livox_msg) {
            laser_mapping->LivoxPCLCallBack(livox_msg);
            laser_mapping->Run();
            continue;
        }

        auto point_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (point_cloud_msg) {
            laser_mapping->StandardPCLCallBack(point_cloud_msg);
            laser_mapping->Run();
            continue;
        }

        auto imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg) {
            laser_mapping->IMUCallBack(imu_msg);
            continue;
        }  
    }
    viewer_thread.join();

    return 0;
}
