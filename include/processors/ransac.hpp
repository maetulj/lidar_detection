#pragma once

#include <vector>
#include <unordered_set>
#include <random>
#include <cmath>
#include <pcl/common/common.h>
#include "../logger.hpp"

namespace Processors
{

/**
 * RANSAC for a plane.
 * 
 * @param plane Input set of points.
 * @param max_iterations Maximum number of iterations to do.
 * @param distance_tol Distance tolerance.
 * 
 * @return Set of points that fullfil this condition.
 */
template<typename PointT>
std::vector<int> Ransac(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    int max_iterations,
    const double distance_tol
)
{
    const auto logger = Logger("Ransac took ");

    // Initialize.
    std::vector<int> inliers_result;

    // Random generator.
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<> random(0, cloud->points.size() - 1);

    while (max_iterations--)
    {
        // Randomly select 3 points.
        const int index1 = random(mt);
        int index2;
        int index3;

        do {
            index2 = random(mt);
        } while (index1 == index2);

        do {
            index3 = random(mt);
        } while ((index1 == index3) || (index2 == index3));

        // Get the coordinates.
        const PointT p1 = cloud->points[index1];
        const PointT p2 = cloud->points[index2];
        const PointT p3 = cloud->points[index3];

        // Vectors that form the plane v1(p1->p2), v2(p1->p3).
		const Eigen::Vector3d v1(
			(p2.x - p1.x),
			(p2.y - p1.y),
			(p2.z - p1.z)
		);
		
		const Eigen::Vector3d v2(
			(p3.x - p1.x),
			(p3.y - p1.y),
			(p3.z - p1.z)
		);

        // Cross product (normal to the plane).
		auto v1_x_v2 = v1.cross(v2);

		const double A = v1_x_v2.x();
		const double B = v1_x_v2.y();
		const double C = v1_x_v2.z();
		const double D = -1.0 * (A * p1.x + B * p1.y + C * p1.z);

		const double root = std::sqrt(std::pow(A, 2.0) + std::pow(B, 2.0) + std::pow(C, 2.0));

		// Inliers for this plane.
		std::unordered_set<int> inliers_tmp;

		// Measure the distance between every point and the line.
		for (int i = 0; i < cloud->points.size(); ++i)
		{
			const double distance = std::abs(A * cloud->points[i].x + B * cloud->points[i].y + C * cloud->points[i].z + D) / root;

			// Add the inlier if the distance is less than the threshold.
			if (distance < distance_tol)
			{
				inliers_tmp.emplace(i);
			}
		}

		// Update the best fit.
		if (inliers_tmp.size() > inliers_result.size())
		{
			inliers_result = std::vector<int>(inliers_tmp.cbegin(), inliers_tmp.cend());
		}
    }

    return inliers_result;
}

} // namespace