#pragma once

#include <Eigen/Core>

namespace Processors
{

struct Box
{
    double x_min;
    double y_min;
    double z_min;

    double x_max;
    double y_max;
    double z_max;

    Box(const Eigen::Vector4f &min, const Eigen::Vector4f &max)
    {
        x_min = min[0];
        y_min = min[1];
        z_min = min[2];

        x_max = max[0];
        y_max = max[1];
        z_max = max[2];
    }
};

} // namespace