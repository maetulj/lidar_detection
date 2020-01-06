#pragma once

#include <memory>

namespace Processors
{

template<typename PointT>
struct Node
{
    PointT point;
    int id;

    std::shared_ptr<Node> left;
    std::shared_ptr<Node> right;

    Node(const PointT &arr, const int set_id)
    : point(arr)
    , id(set_id)
    {}
};

} // namespace