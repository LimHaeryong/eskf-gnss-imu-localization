#ifndef _OPENGL_VIEWER_TYPE_H_
#define _OPENGL_VIEWER_TYPE_H_

struct Point
{
    double x;
    double y;
    double z;

    Point(double x, double y, double z)
        : x(x), y(y), z(z)
    {}
};

#endif // _OPENGL_VIEWER_TYPE_H_