//
// Created by Jonas
//

#ifndef ENGINE_STRUCTS_H
#define ENGINE_STRUCTS_H
#include <list>
#include <algorithm>
#include <cmath>
#include "easy_image.h"
#include "vector3d.h"

using namespace std;
struct Color{
    double red;
    double green;
    double blue;

    Color() = default;
    Color(const Color& c);
    Color(double r, double g, double b);
};
struct Point2D{
    double x;
    double y;
};
struct Line2D{
    Point2D p1;
    Point2D p2;
    Color color;

    double z1;
    double z2;
};
struct Face{
    vector<int> pointIndexes;
    Face() = default;
    explicit Face(const vector<int> &pointIndexes);
};
struct Figure{
    std::vector<Vector3D> points;
    std::vector<Face> faces;
    Color drawColor;
};
using Figures3D = std::list<Figure>;

using Lines2D = std::list<Line2D>;

#endif //ENGINE_STRUCTS_H
