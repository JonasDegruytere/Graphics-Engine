//
// Created by Jonas on 15-5-2023.
//

#ifndef ENGINE_SYSTEM3DL_H
#define ENGINE_SYSTEM3DL_H
#include "Structs.h"
#include "ini_configuration.h"
#include "vector3d.h"
#include "l_parser.h"
#include <fstream>
#include <stack>
using namespace std;

namespace System3DL {
    Figure LSystem3D(const ini::Configuration &configuration, const std::string &figureName, Matrix &V, bool transform = true);
};
class Transformation{
public:
    static Matrix scaleFigure(double scale);

    static Matrix rotateX(double angle);

    static Matrix rotateY(double angle);

    static Matrix rotateZ(double angle);

    static Matrix translate(const Vector3D &vector);

    static Matrix eyePointTrans(const Vector3D &eyepoint);

    static void applyTransformation(Figure &fig, const Matrix &M);

    static void splitTriangles(Figure &figure);

    static void triangulate(Figure &fig);

    static void calculateValues(Lines2D &lines, double size, double &width, double &height, double &d, double &dx, double &dy);
};
namespace Transfor{
    void toPolar(const Vector3D &point, double &theta, double &phi, double &r);
}


#endif //ENGINE_SYSTEM3DL_H
