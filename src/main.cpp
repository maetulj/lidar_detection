#include "render/render.hpp"
#include "logger.hpp"
#include "process_point_clouds.hpp"

using pcl_type = pcl::PointXYZI;

int main(int argc, char** argv)
{
    PRINT_STREAM("Starting LIDAR DETECTION");

    // Create the renderer.
    Render::Renderer renderer;

    // Create the point cloud processor.
    auto ppc = std::make_shared<LidarDetection::ProcessPointClouds<pcl_type>>();

    auto stream = ppc->streamPcd("../src/data/data_1");
    auto stream_it = stream.begin();

    renderer.run([&]() {
        // Load the point cloud.
        ppc->loadPcd((*stream_it).string());

        // Render the point clouds.
        renderer.displayClusters(ppc->process());

        // Display the ground plane.
        renderer.renderPointCloud(ppc->groundPlane(), "ground", Render::Color(0.0, 1.0, 0.0));

        ++stream_it;

        // Continuous loop.
        if (stream_it == stream.end())
        {
            stream_it = stream.begin();
        }
    });

    return 0;
}