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

int main() {
    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // 初始化可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);

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
    std::string traj_file = std::string(ROOT_DIR) + "/data/traj.ptxt";
    addTrajectory(viewer, traj_file);

    // 添加树干圆柱体
    std::string trunk_file = std::string(ROOT_DIR) + "/data/trunk.txt";
    addMarkers(viewer, trunk_file);

    // 启动可视化
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100); // 每100ms更新一次
    }
    // viewer->spin();

    return 0;
}
