//
// Created by Jonas on 15-5-2023.
//

#ifndef ENGINE_WIREFRAMES_H
#define ENGINE_WIREFRAMES_H
#include <iostream>
#include <utility>
#include <vector>
#include "easy_image.h"
#include "ini_configuration.h"
#include "vector3d.h"
#include "System2DL.h"
#include "Structs.h"
#include "System3DL.h"
#include "Structs.h"
#include <cmath>
#include "Figures.h"
using namespace std;
Point2D doProjection(const Vector3D &point, double d);

Lines2D doProjection(const Figures3D &figs);

Lines2D doProjectionConst(const Figures3D &figs);

Figure createEyeFigure(const ini::Configuration &configuration, std::string &figureName, Matrix &V);

Figure createCube(const ini::Configuration &configuration, std::string &figureName, Matrix &V, bool light = false);

Figure createTetrahedron(const ini::Configuration &configuration, std::string &figureName, Matrix &V, bool light = false);

Figure createOctahedron(const ini::Configuration &configuration, std::string &figureName, Matrix &V, bool light = false);

Figure createIcosahedron(const ini::Configuration &configuration, std::string &figureName, Matrix &V, bool light = false);

Figure createDodecahedron(const ini::Configuration &configuration, std::string &figureName, Matrix &V, bool light = false);

Figure createSphere(const ini::Configuration &configuration, std::string &figureName, Matrix &V, bool light = false);

Figure createCone(const ini::Configuration &configuration, std::string &figureName, Matrix &V, bool light = false);

Figure createCylinder(const ini::Configuration &configuration, std::string &figureName, Matrix &V, bool light = false);

Figure createTorus(const ini::Configuration &configuration, std::string &figureName, Matrix &V, bool light = false);

Figure createBuckyBall(const ini::Configuration &configuration, std::string &figureName, Matrix &V, bool light = false);

namespace Lines3D {
    img::EasyImage wireframe(const ini::Configuration &configuration, bool zBuffer);
}

#endif //ENGINE_WIREFRAMES_H
