#pragma once

#include <memory>
#include <vector>
#include <cmath>
#include "node.hpp"

namespace Processors
{

template<typename PointT, size_t K = 2>
class KdTree
{
public:
    /**
	 * Insert the point into the KD-Tree.
	 * 
	 * @param point <PointT> Point coordinates.
	 * @param id <int> Index of the point.
	 */
    void insert(const PointT &point, const int id)
    {
        this->insert(m_root, 0, point, id);
    }

    /**
	 * Search the KD-Tree for points that are within the distance of the target.
	 * 
	 * @param target <PointT> Target point.
	 * @param distance_tolerance <float> Distance tolerance.
	 */
    std::vector<int> search(const PointT &target, const double distance_tolerance)
    {
        std::vector<int> ids;

        this->search(m_root, 0, target, distance_tolerance, ids);

        return ids;
    }

private:
    std::shared_ptr<Node<PointT>> m_root;

    /**
	 * Traverse the KD-Tree recursively.
	 * 
	 * @param node <Node* &>
	 * @param depth <uint>
	 * @param point <PointT>
	 * @param id <int> 
	 */
    void insert(
        std::shared_ptr<Node<PointT>> &node,
        const uint depth,
        const PointT &point,
        const int id
    )
    {
        // If the node is not yet defined.
        if (node == nullptr)
        {
            node = std::make_shared<Node<PointT>>(point, id);
            return;
        }

        // If current_depth % 2 => compare X, else compare Y.
        const uint current_depth = depth % K;

        if (point[current_depth] < node->point[current_depth])
        {
            // Insert to the left.
            this->insert(node->left, depth + 1, point, id);
        }
        else
        {
            // Insert to the right.
            this->insert(node->right, depth + 1, point, id);
        }
        
    }

    /**
	 * Recursively search the KD-Tree for the points that are within the distance tolerance of the target.
	 * 
	 * @param node <Node*&> Node to look from.
	 * @param depth <uint> Depth at which to look.
	 * @param target <PointT> Target coordinates.
	 * @param distance_tol <double> Distance tolerance.
	 * @param ids <std::vector<int>> Vector of points in distance tolerance of the target.
	 */
    void search(
        std::shared_ptr<Node<PointT>> &node,
        const uint depth,
        const PointT &target,
        const double distance_tol,
        std::vector<int> &ids
    )
    {
        // If node does not exist exit.
        if (node == nullptr)
        {
            return;
        }

        // Check if point is a viable candidate to calculate the distance.
        if (
			// Check if x and y coordinates of the node inside the box.
			(node->point[0] <= (target[0] + distance_tol) && node->point[0] >= (target[0] - distance_tol))
			&& (node->point[1] <= (target[1] + distance_tol) && node->point[1] >= (target[1] - distance_tol))
			&& (node->point[2] <= (target[2] + distance_tol) && node->point[2] >= (target[2] - distance_tol))
		)
		{
			double distance = std::sqrt(
				std::pow(node->point[0] - target[0], 2.0) 
				+ std::pow(node->point[1] - target[1], 2.0)
				+ std::pow(node->point[2] - target[2], 2.0)
				);

			if (distance < distance_tol)
			{
				ids.push_back(node->id);
			}
		}

		// Check children => only check appropriate leaves(coordinates).
		if ((target[depth % K] - distance_tol) < node->point[depth % K])
		{
			this->search(node->left, depth + 1, target, distance_tol, ids);
		}

		if ((target[depth % K] + distance_tol) > node->point[depth % K])
		{
			this->search(node->right, depth + 1, target, distance_tol, ids);
		}
    }
};

} // namespace
