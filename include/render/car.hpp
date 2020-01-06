#pragma once

#include <string>
#include "render/vect3.hpp"
#include "render/color.hpp"
#include <pcl/visualization/pcl_visualizer.h>

namespace Render
{

struct Car
{
    Vect3 position;
    Vect3 dimensions;

    std::string name;
    Color color;

    Car(
        const Vect3 setPosition,
        const Vect3 setDimensions,
        const Color setColor,
        const std::string setName
    )
    : position(setPosition)
    , dimensions(setDimensions)
    , color(setColor)
    , name(setName)
    {}

    /**
     * Render the car on the visualizer.
     * 
     * @param visualizer <pcl::visualization::PCLVisualizer::Ptr&> Visualizer to render on.
     */
    void render(const pcl::visualization::PCLVisualizer::Ptr &visualizer) const
    {
        // Render the bottom of the car.
        visualizer->addCube(
            position.x - dimensions.x / 2.0,
            position.x + dimensions.x / 2.0,
            position.y - dimensions.y / 2.0,
            position.y + dimensions.y / 2.0,
            position.z,
            position.z + dimensions.z * 2.0/3.0,
            color.r,
            color.g,
            color.b,
            name
        );

        visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
        visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
        visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name);

        // Render top of the car.
        const std::string top_name = name + "_top";
        visualizer->addCube(
            position.x - dimensions.x / 4.0,
            position.x + dimensions.x / 4.0,
            position.y - dimensions.y / 2.0,
            position.y + dimensions.y / 2.0,
            position.z + dimensions.z * 2.0/3.0,
            position.z + dimensions.z,
            color.r,
            color.g,
            color.b,
            top_name
        );

        visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, top_name);
        visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, top_name);
        visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, top_name);
    }

    /**
     * Helper function for collision detection.
     * 
     * @param point <double> Point to be checked.
     * @param center <double> Center of the object.
     * @param range <double> Range to check.
     * 
     * @return True if the point lies inside the range of the center.
     */
    inline bool inbetween(
        const double point,
        const double center,
        const double range
    ) const
    {
        return (center - range <= point) && (center + range >= point);
    }

    /**
     * Check for collision.
     * 
     * @param point <Vect3&> Point to check for.
     * 
     * @return True on collision, False if no collision.
     */
    bool checkCollision(const Vect3 &point)
    {
        return (
            inbetween(point.x, position.x, dimensions.x / 2.0) &&
            inbetween(point.y, position.y, dimensions.y / 2.0) &&
            inbetween(point.z, position.z + dimensions.z / 3.0, dimensions.z / 3.0)
        ) || (
            inbetween(point.x, position.x, dimensions.x / 4.0) &&
            inbetween(point.y, position.y, dimensions.y / 2.0) &&
            inbetween(point.z, position.z + dimensions.z * 5.0/ 6.0, dimensions.z / 6.0)
        );
    }
};

} // namespace