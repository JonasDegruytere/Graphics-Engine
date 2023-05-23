//
// Created by Jonas on 15-5-2023.
//

#include "Figures.h"

Figures::Figures() {}

Figure *Figures::cubeFigure = nullptr;
Figure *Figures::tetrahedronFigure = nullptr;
Figure *Figures::octahedronFigure = nullptr;
Figure *Figures::icosahedronFigure = nullptr;
Figure *Figures::dodecahedronFigure = nullptr;

Figure Figures::getCubeFigure() {
    if (cubeFigure == nullptr) {
        cubeFigure = new Figure;
        cubeFigure->points = {Vector3D::point(1, -1, -1), Vector3D::point(-1, 1, -1), Vector3D::point(1, 1, 1),
                              Vector3D::point(-1, -1, 1), Vector3D::point(1, 1, -1), Vector3D::point(-1, -1, -1),
                              Vector3D::point(1, -1, 1), Vector3D::point(-1, 1, 1)};
        Face face1({0, 4, 2, 6});
        Face face2({4, 1, 7, 2});
        Face face3({1, 5, 3, 7});
        Face face4({5, 0, 6, 3});
        Face face5({6, 2, 7, 3});
        Face face6({0, 5, 1, 4});
        cubeFigure->faces = {face1, face2, face3, face4, face5, face6};
    }
    return *cubeFigure;
}

Figure Figures::getTetrahedronFigure() {
    if (tetrahedronFigure == nullptr){
        tetrahedronFigure = new Figure;
        tetrahedronFigure->points = {Vector3D::point(1, -1, -1), Vector3D::point(-1, 1, -1),
                                     Vector3D::point(1, 1, 1), Vector3D::point(-1, -1, 1)};
        Face face1({0, 1, 2});
        Face face2({1, 3, 2});
        Face face3({0, 3, 1});
        Face face4({0, 2, 3});
        tetrahedronFigure->faces = {face1, face2, face3, face4};
    }
    return *tetrahedronFigure;
}

Figure Figures::getOctahedronFigure() {
    if (octahedronFigure == nullptr) {
        octahedronFigure = new Figure;
        octahedronFigure->points = {Vector3D::point(1, 0, 0), Vector3D::point(0, 1, 0), Vector3D::point(-1, 0, 0),
                                    Vector3D::point(0, -1, 0), Vector3D::point(0, 0, -1), Vector3D::point(0, 0, 1)};
        Face face1({0, 1, 5});
        Face face2({1, 2, 5});
        Face face3({2, 3, 5});
        Face face4({3, 0, 5});
        Face face5({1, 0, 4});
        Face face6({2, 1, 4});
        Face face7({3, 2, 4});
        Face face8({0, 3, 4});
        octahedronFigure->faces = {face1, face2, face3, face4, face5, face6, face7, face8};
    }
    return *octahedronFigure;
}

Figure Figures::getIcosahedronFigure() {
    if (icosahedronFigure == nullptr) {
        icosahedronFigure = new Figure;
        icosahedronFigure->points.emplace_back(Vector3D::point(0, 0, std::sqrt(5)/2));
        //punt 2 tot 6
        for (int i = 2; i < 7; ++i){
            icosahedronFigure->points.emplace_back(Vector3D::point(std::cos((i-2)*2*M_PI/5), std::sin((i-2)*2*M_PI/5), 0.5));
        }
        //punt 7 tot 11
        for (int i = 7; i < 12; ++i){
            icosahedronFigure->points.emplace_back(Vector3D::point(std::cos(M_PI/5 + (i - 7)*2*M_PI/5), std::sin(M_PI/5 + (i - 7)*2*M_PI/5), -0.5));
        }
        //punt 12
        icosahedronFigure->points.emplace_back(Vector3D::point(0, 0 , -std::sqrt(5)/2));

        Face face1({0, 1, 2});
        Face face2({0, 2, 3});
        Face face3({0, 3, 4});
        Face face4({0, 4, 5});
        Face face5({0, 5, 1});
        Face face6({1, 6, 2});
        Face face7({2, 6, 7});
        Face face8({2, 7, 3});
        Face face9({3, 7, 8});
        Face face10({3, 8, 4});
        Face face11({4, 8, 9});
        Face face12({4, 9, 5});
        Face face13({5, 9, 10});
        Face face14({5, 10, 1});
        Face face15({1, 10, 6});
        Face face16({11, 7, 6});
        Face face17({11, 8, 7});
        Face face18({11, 9, 8});
        Face face19({11, 10, 9});
        Face face20({11, 6, 10});

        icosahedronFigure->faces = {face1, face2, face3, face4, face5, face6, face7, face8, face9, face10, face11,
                                    face12, face13, face14, face15, face16, face17, face18, face19, face20};
    }
    return *icosahedronFigure;
}

Figure Figures::getDodecahedronFigure() {
    if (dodecahedronFigure == nullptr) {
        dodecahedronFigure = new Figure;
        Figure icosahedron = getIcosahedronFigure();
        double x, y, z;
        for (int i = 0; i < 20; ++i){
            auto& points = icosahedronFigure->faces[i].pointIndexes;
            x = (icosahedronFigure->points[points[0]].x + icosahedronFigure->points[points[1]].x + icosahedronFigure->points[points[2]].x)/3;
            y = (icosahedronFigure->points[points[0]].y + icosahedronFigure->points[points[1]].y + icosahedronFigure->points[points[2]].y)/3;
            z = (icosahedronFigure->points[points[0]].z + icosahedronFigure->points[points[1]].z + icosahedronFigure->points[points[2]].z)/3;
            dodecahedronFigure->points.emplace_back(Vector3D::point(x, y, z));
        }

        Face face1({0, 1, 2, 3, 4});
        Face face2({0, 5, 6, 7, 1});
        Face face3({1, 7, 8, 9, 2});
        Face face4({2, 9, 10, 11, 3});
        Face face5({3, 11, 12, 13, 4});
        Face face6({4, 13, 14, 5, 0});
        Face face7({19, 18, 17, 16, 15});
        Face face8({19, 14, 13, 12, 18});
        Face face9({18, 12, 11, 10, 17});
        Face face10({17, 10, 9, 8, 16});
        Face face11({16, 8, 7, 6, 15});
        Face face12({15, 6, 5, 14, 19});

        dodecahedronFigure->faces = {face1, face2, face3, face4, face5, face6, face7, face8, face9, face10, face11, face12};
    }
    return *dodecahedronFigure;
}