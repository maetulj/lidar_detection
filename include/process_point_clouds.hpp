#pragma once

#include <memory>
#include <utility>
#include <exception>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include "logger.hpp"
#include "options.hpp"
#include "processors/ransac.hpp"
#include "processors/kd_tree.hpp"

namespace LidarDetection
{

template<typename PointT>
class ProcessPointClouds
{
public:
    using cloud_type = pcl::PointCloud<PointT>;
    using cloud_ptr_type = typename pcl::PointCloud<PointT>::Ptr;
    using tree_type = Eigen::Vector4f;

    /**
     * Get the number of points in the cloud.
     * 
     * @return The number of points in the given cloud.
     */
    auto numPoints(const cloud_ptr_type &cloud) const -> decltype(cloud->points.size())
    {
        return cloud->points.size();
    }

    /**
     * Load the point cloud from the given file.
     * 
     * @throw Throws runtime error if the file could not be read.
     */
    void loadPcd(const std::string file)
    {
        m_cloud_ptr = boost::make_shared<cloud_type>();

        if (pcl::io::loadPCDFile<PointT>(file, *m_cloud_ptr) == -1)
        {
            throw std::runtime_error("Could not read the given file!");
        }

        PRINT_STREAM("Loaded " << this->numPoints(m_cloud_ptr) << " points from " << file);
    }

    /**
     * Read the files in the directory and return them sorted.
     * 
     * @return Sorted vector with filenames.
     */
    std::vector<boost::filesystem::path> streamPcd(const std::string data_path)
    {
        std::vector<boost::filesystem::path> paths(
            boost::filesystem::directory_iterator{ data_path },
            boost::filesystem::directory_iterator{}
        );

        std::sort(paths.begin(), paths.end());

        return paths;
    }

    std::vector<cloud_ptr_type> process(
        const Options options = Options()
    )
    {
        // Save the options.
        m_options = options;

        // Reset the clusters.
        m_clusters.clear();
        
        // Filter the cloud.
        this->filter();

        // m_clusters.push_back(m_cloud_ptr);

        // Segment the plane from the input cloud.
        this->segment();

        // Cluster the objects.
        this->cluster(
            m_options()["clustering"]["tolerance"],
            m_options()["clustering"]["min"],
            m_options()["clustering"]["max"]
        );

        // Return the processed point clouds.
        return m_clusters;
    }

    /**
     * Return the extracted ground plane.
     */
    cloud_ptr_type groundPlane() const
    {
        return m_ground_ptr;
    }

private:
    cloud_ptr_type m_cloud_ptr;
    cloud_ptr_type m_ground_ptr;
    std::vector<cloud_ptr_type> m_clusters;
    Options m_options;

    /**
     * Filter the cloud.
     * - voxel grid downsampling
     * - extract the region of interest
     * - remove roof points
     */
    void filter()
    {
        const auto logger = Logger("Filtering took ");

        cloud_ptr_type cloud_downsampled = boost::make_shared<cloud_type>();
        cloud_ptr_type cloud_region = boost::make_shared<cloud_type>();
        cloud_ptr_type cloud_roof_removed = boost::make_shared<cloud_type>();

        // Voxel grid downsampling.
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(m_cloud_ptr);
        voxel_filter.setLeafSize(
            m_options()["filtering"]["voxel_size"],
            m_options()["filtering"]["voxel_size"],
            m_options()["filtering"]["voxel_size"]
        );
        voxel_filter.filter(*cloud_downsampled);

        // Region filtering.
        pcl::CropBox<PointT> crop_filter;
        crop_filter.setInputCloud(cloud_downsampled);
        crop_filter.setMin(m_options.region("min"));
        crop_filter.setMax(m_options.region("max"));
        crop_filter.filter(*cloud_region);

        // Extract the rooftop.
        std::vector<int> roof_points;
        pcl::CropBox<PointT> roof_filter;
        roof_filter.setInputCloud(cloud_region);

        const auto rooftop_min = m_options()["rooftop"]["min"];
        roof_filter.setMin(Eigen::Vector4f(rooftop_min["x"], rooftop_min["y"], rooftop_min["z"], 1));
        
        const auto rooftop_max = m_options()["rooftop"]["max"];
        roof_filter.setMax(Eigen::Vector4f(rooftop_max["x"], rooftop_max["y"], rooftop_max["z"], 1));
        roof_filter.filter(roof_points);

        pcl::PointIndices::Ptr roof_indices = boost::make_shared<pcl::PointIndices>();
        roof_indices->indices = roof_points;

        // Extract the roof indices.
        pcl::ExtractIndices<PointT> extract_filter(true);
        extract_filter.setInputCloud(cloud_region);
        extract_filter.setIndices(roof_indices);
        extract_filter.setNegative(true);
        extract_filter.filter(*cloud_roof_removed);

        // Save the filtered cloud.
        m_cloud_ptr = cloud_roof_removed;
    }

    /**
     * Segment the point cloud into obstacles and the ground plane using RANSAC. 
     */
    void segment()
    {
        const auto logger = Logger("Segmentation took ");

        // Extract the inliers.
        pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();

        inliers->indices = Processors::Ransac<PointT>(
            m_cloud_ptr,
            m_options()["segmentation"]["iterations"],
            m_options()["segmentation"]["distance_threshold"]
        );

        auto clouds = this->separateClouds(inliers);

        // Ground plane.
        m_ground_ptr = clouds.second;

        // Obstacles.
        m_cloud_ptr = clouds.first;
    }

    /**
     * Separate the cloud into two distinct clouds representing:
     * - obstacles
     * - ground
     * 
     * @return Pair consisting of obstacles clouds and ground plane.
     */
    std::pair<cloud_ptr_type, cloud_ptr_type> separateClouds(const pcl::PointIndices::Ptr &inliers) const
    {
        // Create the output clouds.
        cloud_ptr_type plane = boost::make_shared<cloud_type>();
        cloud_ptr_type obstacles = boost::make_shared<cloud_type>();

        // Fill the plane with inliers.
        for (auto inlier : inliers->indices)
        {
            plane->points.push_back(m_cloud_ptr->points[inlier]);
        }

        // Extract the obstacles.
        pcl::ExtractIndices<PointT> extract_filter;
        extract_filter.setInputCloud(m_cloud_ptr);
        extract_filter.setIndices(inliers);
        extract_filter.setNegative(true);
        extract_filter.filter(*obstacles);

        return std::pair<cloud_ptr_type, cloud_ptr_type>(obstacles, plane);
    }

    /**
     * Cluster the obstacles cloud into objects clouds.
     * 
     * @param tolerance Tolerance threshold.
     * @param min_size Min size of the clustered cloud.
     * @param max_size Max size of the clustered cloud.
     * 
     * @return A vector containing the clustered clouds.
     */
    void cluster(
        const double tolerance,
        const double min_size,
        const double max_size
    )
    {
        const auto logger = Logger("Clustering took ");

        // Create the KD-Tree.
        auto tree = std::make_shared<Processors::KdTree<tree_type, 3>>();

        for (int i = 0; i < m_cloud_ptr->points.size(); ++i)
        {
            tree->insert(m_cloud_ptr->points[i].getVector4fMap(), i);
        }

        this->euclideanCluster(
            tree,
            tolerance,
            min_size,
            max_size
        );

        PRINT_STREAM("Found " << m_clusters.size() << " clusters.");
    }

    /**
     * This function produces a vector of clusters (vectors of points).
     * All the points are processed and joined 
     * into clusters based on the given distance tolerance. 
     * 
     * @param tree <Processors::KdTree*> The KD-Tree of points.
     * @param distance_tolerance <float> Distance tolerance.
     * @param min_size <int> Minimal size of the cluster.
     * @param max_size <int> Maximal size of the cluster.
     */
    void euclideanCluster(
        std::shared_ptr<Processors::KdTree<tree_type, 3>> tree,
        const double distance_tolerance,
        const int min_size,
        const int max_size
    )
    {
        const auto logger = Logger("Euclidean clustering took ");

        // List of processed points.
        std::vector<bool> processed_points(m_cloud_ptr->points.size(), false);

        // Process all points.
        for (int i = 0; i < m_cloud_ptr->points.size(); ++i)
        {
            if (processed_points[i])
            {
                // Point has already been processed.
                continue;
            }

            // Create the cluster.
            cloud_ptr_type cluster = boost::make_shared<cloud_type>();

            // Check proximity.
            this->proximity(i, processed_points, cluster, tree, distance_tolerance);

            // Check for validity.
            if (cluster->points.size() < min_size || cluster->points.size() > max_size)
            {
                continue;
            }

            // Add cluster to clusters.
            PRINT_STREAM("Created cluster with " << cluster->points.size() << " points.");
            m_clusters.push_back(cluster);
        }
    }

    /**
     * Proximity function. It processes the current point,
     * finds its neighbours and creates a cluster from them.
     * 
     * @param processed_points <std::vector<bool>&> Already processed points.
     * @param cluster <std::vector<int>> Current cluster.
     * @param tree <Processors::KdTree*> Pointer to the KD-Tree of processed points.
     * @param distance_tol <float> Distance tolerance.
     */
    void proximity(
        const int i,
        std::vector<bool>& processed_points,
        cloud_ptr_type& cluster,
        std::shared_ptr<Processors::KdTree<tree_type, 3>> tree,
        const double distance_tolerance
    )
    {
        // Mark point as processed.
        processed_points[i] = true;

        // Add point to the cluster.
        cluster->points.push_back(m_cloud_ptr->points[i]);

        // Get nearby points.
        const std::vector<int> neighbours = tree->search(m_cloud_ptr->points[i].getVector4fMap(), distance_tolerance);

        // Go through each nearby point.
        for (auto point : neighbours)
        {
            // If point has not yet been processed it, process it.
            if ( ! processed_points[point])
            {
                this->proximity(
                    point,
                    processed_points,
                    cluster,
                    tree,
                    distance_tolerance
                );
            }
        }
    }
}; 

} // namespace