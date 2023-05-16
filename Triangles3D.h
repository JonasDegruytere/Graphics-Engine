//
// Created by Jonas on 15-5-2023.
//

#ifndef ENGINE_TRIANGLES3D_H
#define ENGINE_TRIANGLES3D_H
#include "easy_image.h"
#include "ini_configuration.h"
#include "vector3d.h"
#include <cmath>
#include "Figures.h"
#include "System3DL.h"
#include "Wireframes.h"
using namespace std;

namespace Triangles3D{
    img::EasyImage zBuffer(const ini::Configuration &configuration);
}


#endif //ENGINE_TRIANGLES3D_H
