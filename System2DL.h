//
// Created by Jonas on 15-5-2023.
//

#ifndef ENGINE_SYSTEM2DL_H
#define ENGINE_SYSTEM2DL_H
#include "easy_image.h"
#include "ini_configuration.h"
#include "l_parser.h"
#include <fstream>
#include <cmath>
#include <stack>
#include "Structs.h"
#include "ZBuffer.h"
using namespace std;

namespace System2DL {
    img::EasyImage LSystem2D(const ini::Configuration &configuration);
    img::EasyImage coordToPixel(Lines2D &lines, double size, img::Color backgroundColor, bool zBuffer = false);
};


#endif //ENGINE_SYSTEM2DL_H
