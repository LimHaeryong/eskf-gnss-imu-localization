#ifndef _OPENGL_VIEWER_TYPE_H_
#define _OPENGL_VIEWER_TYPE_H_

enum class PointType
{
    GNSS,
    FILTERED
};

struct Point
{
    double x;
    double y;
    double z;
    PointType pointType;
    Point(double x, double y, double z, PointType pointType)
        : x(x), y(y), z(z), pointType(pointType)
    {}
};

#endif // _OPENGL_VIEWER_TYPE_H_