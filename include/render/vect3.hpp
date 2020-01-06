#pragma once

namespace Render
{

struct Vect3
{
    double x;
    double y;
    double z;

    Vect3(
        const double setX,
        const double setY,
        const double setZ
    )
    : x(setX)
    , y(setY)
    , z(setZ)
    {}

    Vect3 operator+(const Vect3 &vec) const
    {
        return Vect3(x + vec.x, y + vec.y, z + vec.z);
    }
};

} // namespace