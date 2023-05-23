//
// Created by Jonas
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
    img::EasyImage DrawLines(Lines2D &lines, double size, img::Color backgroundColor, bool zBuffer = false);
    string generateLSystemString (string &currentString, map<char, std::string> &replacementRules, unsigned int depth);
};


#endif //ENGINE_SYSTEM2DL_H
