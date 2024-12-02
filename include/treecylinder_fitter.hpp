#include <visualization_msgs/MarkerArray.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>

// ADD zjl
struct TrunkCylinderDesc {
    int trunk_id;              // 树干圆柱的唯一标识符
    Eigen::Vector3f position;  // 树干圆柱中心点位置
    Eigen::Vector3f direction; // 树干圆柱的轴向
    float radius;              // 树干圆柱的半径
    float height;              // 树干圆柱的高度

    TrunkCylinderDesc()
        : trunk_id(0), position(Eigen::Vector3f::Zero()), direction(Eigen::Vector3f::UnitZ()), radius(0.0f), height(0.0f) {}

    TrunkCylinderDesc(int id, Eigen::Vector3f pos, Eigen::Vector3f dir, float r, float h)
        : trunk_id(id), position(pos), direction(dir), radius(r), height(h) {}
};

class TreeCylinderFitter {
public:
    TreeCylinderFitter() 
        : accumulation_time_(0), max_distance_(0), initialized_(false), current_id_(0) {
            trunk_cylinder_map_.clear();
        }

    TreeCylinderFitter(double accumulation_time, double max_distance)
        : accumulation_time_(accumulation_time), max_distance_(max_distance), initialized_(false), current_id_(0) {
            trunk_cylinder_map_.clear();
        }

    void addPointCloud(const PointCloudType::Ptr& cloud, const ros::Time& timestamp, const Eigen::Vector3f& curPos) {
        // 距离过滤，仅保留当前位姿附近 max_distance_ 范围内的点
        PointCloudType filtered_cloud;
        pcl::CropBox<PointType> box_filter;
        box_filter.setMin(Eigen::Vector4f(curPos(0) - max_distance_, curPos(1) - max_distance_, -std::numeric_limits<float>::max(), 1.0));
        box_filter.setMax(Eigen::Vector4f(curPos(0) + max_distance_, curPos(1) + max_distance_, std::numeric_limits<float>::max(), 1.0));
        box_filter.setInputCloud(cloud);
        box_filter.filter(filtered_cloud);

        // 累加点云
        //*accumulated_cloud_ += *filtered_cloud;
        accumulated_cloud_->points.insert(accumulated_cloud_->points.end(), 
            filtered_cloud.points.begin(), filtered_cloud.points.end());

        if (!initialized_) {
            start_time_ = timestamp;
            initialized_ = true;
        }

        // 检查是否到达积累时间
        if ((timestamp - start_time_).toSec() >= accumulation_time_) {
            updatePointCloud();
            start_time_ = timestamp;
            accumulated_cloud_->clear();
        }
    }

    // 获取所有树干描述符
    std::vector<TrunkCylinderDesc> getAllTrunkCylinderDesc() const {
        std::vector<TrunkCylinderDesc> result;
        for (const auto& [id, data] : trunk_cylinder_map_) {
            result.push_back(data.first);  // 提取 TrunkCylinderDesc
        }
        return result;
    }

private:
    double accumulation_time_;     // 累计时间（秒）
    double max_distance_;          // 点云的最大距离范围
    bool initialized_;             // 是否已经开始树干模型拟合
    ros::Time start_time_;         // 开始时间 
    int current_id_;               // 当前圆柱模型的ID
    PointCloudType::Ptr accumulated_cloud_ = boost::make_shared<PointCloudType>(); // 累积点云
    // 树干模型描述符和点云映射
    std::map<int, std::pair<TrunkCylinderDesc, PointCloudType::Ptr>> trunk_cylinder_map_;

    void updatePointCloud() {
        //PointCloudType::Ptr accumulated_cloud_vg_ = voxelGrid(accumulated_cloud_, 0.5f);
        PointCloudType::Ptr trunk_cloud = extractTrunk(accumulated_cloud_, 100, 0.8f, 0.2f);
        PointCloudType::Ptr trunk_sor = OutlierRemoval(trunk_cloud, 1.0);
        std::vector<PointCloudType::Ptr> trunk_clusters = clusterExtraction(trunk_sor, 0.5);

        for (const auto& cluster : trunk_clusters) {
            TrunkCylinderDesc new_model;
            if(!fitCylinderModel(cluster, new_model)) continue;

            bool matched = false;
            for (auto& [id, data] : trunk_cylinder_map_) {
                auto& [desc, cloud] = data;
                if (isSameCylinder(desc, new_model)) {
                    // 合并点云
                    cloud->points.insert(cloud->points.end(), cluster->points.begin(), cluster->points.end());
                    cloud->width = cloud->points.size();
                    cloud->height = 1;

                    // 重新拟合圆柱模型
                    TrunkCylinderDesc updated_model;
                    if (fitCylinderModel(cloud, updated_model)) {
                        //LOG(INFO) << "匹配成功--圆柱" << desc.trunk_id << "参数由 " << desc.position.transpose()  << " " << desc.direction.transpose() << " " << desc.radius << " " << desc.height  
                        //<< " 更新为" << updated_model.position.transpose() << " " << updated_model.direction.transpose() << " " << updated_model.radius << " " << updated_model.height << std::endl;
                        desc = updated_model;  // 更新圆柱参数
                        desc.trunk_id = id;
                    }
                    matched = true;
                    break;
                }
            }
            if (!matched) {
                new_model.trunk_id = ++current_id_;
                trunk_cylinder_map_[new_model.trunk_id] = {new_model, cluster};

                //LOG(INFO) << "添加新的" << new_model.trunk_id << "树干圆柱：" << new_model.position.transpose() << new_model.direction.transpose() << " " << new_model.radius << " " << new_model.height << std::endl;
            }
        }
    }

    bool isSameCylinder(const TrunkCylinderDesc& model1, const TrunkCylinderDesc& model2) {
        // 判断两个圆柱是否为同一树干模型（可以根据位置和方向的相似性）
        //float position_diff = (model1.position - model2.position).norm();
        // 计算投影在 xOy 平面上的距离
        float position_diff = (model1.position.head<2>() - model2.position.head<2>()).norm();
        //float direction_similarity = model1.direction.dot(model2.direction);

        float position_threshold = 0.2f;   // 位置差异阈值
        //float direction_threshold = 0.9f;  // 方向相似性阈值
        //std::cout<<"position_diff:"<<position_diff<<"  direction_similarity:"<<direction_similarity<<std::endl;

        float r1 = model1.radius;
        float r2 = model2.radius;
        //return position_diff < r1+r2 && direction_similarity > direction_threshold;
        return position_diff < r1 + r2 + position_threshold;
    }

    // TrunkCylinderDesc updateCylinderDesc(const TrunkCylinderDesc& old_model, const TrunkCylinderDesc& new_model) {
    //     float alpha = 0.9f;  // 新模型权重
    //     Eigen::Vector3f updated_position = new_model.position;
    //     Eigen::Vector3f updated_direction = (alpha * new_model.direction + (1.0f - alpha) * old_model.direction).normalized();
    //     float updated_radius = alpha * new_model.radius + (1.0f - alpha) * old_model.radius;
    //     float updated_height = std::max(old_model.height, new_model.height);
    //     return {old_model.trunk_id, updated_position, updated_direction, updated_radius, updated_height};
    // }

    PointCloudType::Ptr extractTrunk(PointCloudType::Ptr &cloud, int k_neighbors, float flatness_threshold, float zn_threshold)
    {
        PointCloudType::Ptr trunk_cloud = boost::make_shared<PointCloudType>();
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(cloud);

        for (const auto& point : cloud->points) {
            std::vector<int> indices;
            std::vector<float> distances;
            if (kdtree.nearestKSearch(point, k_neighbors, indices, distances) > 0) {
                Eigen::Matrix3f cov_matrix;
                pcl::computeCovarianceMatrix(*cloud, indices, cov_matrix);

                // 计算特征值和特征向量
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(cov_matrix);
                Eigen::Vector3f eigenvalues = eigensolver.eigenvalues();
                Eigen::Matrix3f eigenvectors = eigensolver.eigenvectors();
                // 计算平坦度Fn
                float Fn = 1.0f - eigenvalues(0) / eigenvalues.sum();
                // 法向量的 Zn 分量
                float Zn = eigenvectors.col(0).dot(Eigen::Vector3f(0, 0, 1.0f));

                // 判断是否满足树干的条件
                if (Fn > flatness_threshold && abs(Zn) < zn_threshold) {
                    trunk_cloud->points.emplace_back(point);
                }
            }
        }
        return trunk_cloud;
    }

    PointCloudType::Ptr OutlierRemoval(const PointCloudType::Ptr& cloud, double stddev_mult) {
        PointCloudType::Ptr filtered_cloud = boost::make_shared<PointCloudType>();
        pcl::StatisticalOutlierRemoval<PointType> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(stddev_mult);
        sor.filter(*filtered_cloud);
        return filtered_cloud;
    }

    std::vector<PointCloudType::Ptr> clusterExtraction(const PointCloudType::Ptr& cloud, double tolerance) {
        pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
        tree->setInputCloud(cloud);
        
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance(tolerance);  // 设置聚类的距离容忍度
        ec.setMinClusterSize(200);          // 每个聚类的最小点数
        ec.setMaxClusterSize(20000);        // 每个聚类的最大点数
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract(cluster_indices);

        std::vector<PointCloudType::Ptr> clusters;
        clusters.reserve(cluster_indices.size());
        for (const auto& indices : cluster_indices) {
            PointCloudType::Ptr cluster = boost::make_shared<PointCloudType>();
            cluster->points.reserve(indices.indices.size());

            for (int idx : indices.indices) {
                cluster->points.push_back(cloud->points[idx]);
            }
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;

            clusters.push_back(std::move(cluster));
        }
        return clusters;
    }

    // 拟合圆柱模型
    bool fitCylinderModel(const PointCloudType::Ptr& cloud, TrunkCylinderDesc & model) {
        // PointType min_pt, max_pt;
        // pcl::getMinMax3D(*cloud, min_pt, max_pt);
        // float height = max_pt.z - min_pt.z;

        // 法线估计
        pcl::NormalEstimation<PointType, pcl::Normal> ne;
        pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud);
        ne.setKSearch(50);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
        ne.compute(*cloud_normals);

        pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
        seg.setInputCloud(cloud);
        seg.setInputNormals(cloud_normals);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.2);
        seg.setRadiusLimits(0, 0.5);

        pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();
        pcl::ModelCoefficients::Ptr coefficients = boost::make_shared<pcl::ModelCoefficients>();
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < 100) {
            // LOG(WARNING) << "No sufficient inliers for cylinder model!";
            return false;
        }

        PointCloudType::Ptr cloud_filtered = boost::make_shared<PointCloudType>();
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_filtered);

        if (coefficients->values.size() >= 7) {
            Eigen::Vector3f axis_point(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            Eigen::Vector3f axis_direction(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
            
            float min_proj = std::numeric_limits<float>::max();
            float max_proj = std::numeric_limits<float>::lowest();
            Eigen::Vector3f min_point, max_point;

            for (size_t i = 0; i < cloud_filtered->points.size(); ++i) {
                const auto& pt = cloud_filtered->points[i];
                Eigen::Vector3f point(pt.x, pt.y, pt.z);
                float projection = (point - axis_point).dot(axis_direction);
                Eigen::Vector3f projected_point = axis_point + projection * axis_direction;

                if (projection < min_proj) {
                    min_proj = projection;
                    min_point = projected_point;
                }
                if (projection > max_proj) {
                    max_proj = projection;
                    max_point = projected_point;
                }
            }

            // 计算圆柱体中心点和高度
            Eigen::Vector3f center_point = (min_point + max_point) / 2.0;
            float height = max_proj - min_proj;

            //if(height <= 3.0f || min_point.z > 2.0f) return false;
            // 判断圆柱是否符合树干的标准
            if(height <= 3.0f || fabs(axis_direction[2]) < 0.9f) return false;
            
            model.position = center_point;
            model.direction = axis_direction[2] < 0 ? -axis_direction: axis_direction;
            model.radius = coefficients->values[6];
            model.height = height;

            return true;
        }
        return false;
    }

};
