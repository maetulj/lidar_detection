#pragma once

#include "box.hpp"
#include <pcl/common/common.h>

namespace Processors
{

class BoundingBox
{
public:
    BoundingBox() = delete;

    template<typename PointT>
    static Box create(const typename boost::shared_ptr<pcl::PointCloud<PointT>> &cluster)
    {
        PointT min_point;
        PointT max_point;

        pcl::getMinMax3D(*cluster, min_point, max_point);

        return Box(min_point.getVector4fMap(), max_point.getVector4fMap());
    }
};

} // namespace