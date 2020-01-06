#pragma once

namespace Render
{

struct Color
{
    double r;
    double g;
    double b;

    Color(
        const double setR,
        const double setG,
        const double setB
    )
    : r(setR)
    , g(setG)
    , b(setB)
    {}
};

} // namespace