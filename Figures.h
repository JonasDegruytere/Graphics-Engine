//
// Created by Jonas on 15-5-2023.
//

#ifndef ENGINE_FIGURES_H
#define ENGINE_FIGURES_H
#include "Structs.h"
#include <cmath>
using namespace std;

class Figures {
public:
    Figures();
    static Figure getCubeFigure();

    static Figure getTetrahedronFigure();

    static Figure getOctahedronFigure();

    static Figure getIcosahedronFigure();

    static Figure getDodecahedronFigure();
};


#endif //ENGINE_FIGURES_H
