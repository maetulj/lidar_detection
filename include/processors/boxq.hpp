#pragma once

#include <Eigen/Geometry>

namespace Processors
{

struct BoxQ
{
    Eigen::Vector3f bbox_transform;
    Eigen::Quaternionf bbox_quaternion;
    double cube_length;
    double cube_width;
    double cube_height;
};

} // namespace