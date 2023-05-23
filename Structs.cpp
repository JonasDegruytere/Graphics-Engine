//
// Created by Jonas
//

#include "Structs.h"

Color::Color(const Color &c) {
    red = c.red;
    blue = c.blue;
    green = c.green;
}

Color::Color(double r, double g, double b) {
    red = r;
    green = g;
    blue = b;
}

Face::Face(const vector<int> &pointIndexes) : pointIndexes(pointIndexes) {}
