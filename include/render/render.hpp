#pragma once

#include <memory>
#include <functional>
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include "render/color.hpp"
#include "render/vect3.hpp"
#include "render/camera_angle.h"
#include "render/car.hpp"
#include "processors/box.hpp"
#include "processors/boxq.hpp"
#include "processors/bounding_box.hpp"
#include "logger.hpp"

namespace Render
{

class Renderer
{
public:
    Renderer(const CameraAngle camera_angle = TopDown)
    {
        m_visualizer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");

        initCamera(camera_angle);
    }

    void initCamera(const CameraAngle camera_angle)
    {
        m_visualizer->setBackgroundColor(0, 0, 0);

        // Set camera position and angle.
        m_visualizer->initCameraParameters();

        // Distance away in meters.
        const int distance = 16;

        switch (camera_angle)
        {
            case XY:
                m_visualizer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
                break;
            case TopDown:
                m_visualizer->setCameraPosition(0, 0, distance, 1, 0, 1);
                break;
            case Side:
                m_visualizer->setCameraPosition(0, -distance, 0, 0, 0, 1);
                break;
            case FPS:
                m_visualizer->setCameraPosition(-10, 0, 0, 0, 0, 1);
        }

        if (camera_angle != FPS)
        {
            m_visualizer->addCoordinateSystem(1.0);
        }
    }

    void renderPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string name,
        const Color color
    )
    {
        m_visualizer->addPointCloud<pcl::PointXYZ>(cloud, name);
        m_visualizer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name
        );
        m_visualizer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name
        );
    }

    void renderPointCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        const std::string name,
        const Color color
    )
    {
        if (color.r == -1)
        {
            // Select color based on input value.
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
            m_visualizer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
        }
        else
        {
            m_visualizer->addPointCloud<pcl::PointXYZI>(cloud, name);
            m_visualizer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR,
                color.r,
                color.g,
                color.b,
                name
            );
        }

        m_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    }

    void renderBox(const Processors::Box box, const int id, const Color color = Color(1, 0, 0), double opacity = 1.0)
    {
        if (opacity > 1.0)
        {
            opacity = 1.0;
        }
        
        if (opacity < 0.0)
        {
            opacity = 0.0;
        }

        const std::string cube = "box" + std::to_string(id);

        m_visualizer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

        const std::string cube_fill = "boxFill" + std::to_string(id);

        m_visualizer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube_fill);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cube_fill);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube_fill);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cube_fill);

    }

    void renderBox(const Processors::BoxQ box, const int id, const Color color = Color(1, 0, 0), double opacity = 1.0)
    {
        if (opacity > 1.0)
        {
            opacity = 1.0;
        }
        
        if (opacity < 0.0)
        {
            opacity = 0.0;
        }

        const std::string cube = "box" + std::to_string(id);

        m_visualizer->addCube(box.bbox_transform, box.bbox_quaternion, box.cube_length, box.cube_width, box.cube_height, cube);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

        const std::string cube_fill = "boxFill" + std::to_string(id);

        m_visualizer->addCube(box.bbox_transform, box.bbox_quaternion, box.cube_length, box.cube_width, box.cube_height, cube_fill);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cube_fill);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube_fill);
        m_visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cube_fill);
    }

    void run(std::function<void(void)> cb)
    {
        while ( ! m_visualizer->wasStopped())
        {
            const auto logger = Logger("Frame time: ");

            // Clear viewer.
            m_visualizer->removeAllPointClouds();
            m_visualizer->removeAllShapes();

            // Invoke the callback function.
            cb();

            m_visualizer->spinOnce();
        }
    }

    template<typename ClusterT>
    void displayClusters(const std::vector<ClusterT>& clusters)
    {
        const std::vector<Color> colors = {
            Color(1, 0, 0),
            Color(1.0, 1, 0),
            Color(0, 0, 1)
        };

        int cluster_id = 0;

        PRINT_STREAM("Clusters size: " << clusters.size());

        for (auto cluster : clusters)
        {
            PRINT_STREAM("Cluster " << cluster_id << " has " << cluster->points.size() << " points.");
        
            this->renderPointCloud(
                cluster,
                "cluster" + std::to_string(cluster_id),
                colors[cluster_id % 3]
            );

            // Render the bounding box.
            this->renderBox(Processors::BoundingBox::create(cluster), cluster_id, colors[cluster_id % 3]);

            ++cluster_id;
        }

        m_visualizer->addText("Clusters: " + std::to_string(cluster_id), 10, 20, "text");
    }

private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> m_visualizer;

    
};

} // namespace