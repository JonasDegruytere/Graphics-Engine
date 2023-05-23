//
// Created by Jonas
//

#include "Triangles3D.h"

img::EasyImage Triangles3D::zBuffer(const ini::Configuration &configuration) {
    Figures3D figures;

    const unsigned int size = configuration["General"]["size"].as_int_or_die();
    vector<double> backgroundColor = configuration["General"]["backgroundcolor"].as_double_tuple_or_die();
    img::Color backgroundColorElement(backgroundColor[0] * 255, backgroundColor[1] * 255, backgroundColor[2] * 255);
    const unsigned int nrFigures = configuration["General"]["nrFigures"].as_int_or_die();
    vector<double> eye = configuration["General"]["eye"].as_double_tuple_or_die();
    Matrix V;
    V = Transformation::eyePointTrans(Vector3D::point(eye[0], eye[1], eye[2]));

    for (unsigned int i = 0; i < nrFigures; i++) {
        string figureName = "Figure" + to_string(i);
        string type = configuration[figureName]["type"].as_string_or_die();
        Figures3D currentFigs;
        if (type == "Cube") currentFigs = {createCube(configuration, figureName, V)};
        else if (type == "Tetrahedron") currentFigs = {createTetrahedron(configuration, figureName, V)};
        else if (type == "Octahedron") currentFigs = {createOctahedron(configuration, figureName, V)};
        else if (type == "Icosahedron") currentFigs = {createIcosahedron(configuration, figureName, V)};
        else if (type == "Dodecahedron") currentFigs = {createDodecahedron(configuration, figureName, V)};
        else if (type == "Sphere") currentFigs = {createSphere(configuration, figureName, V)};
        else if (type == "Cone") currentFigs = {createCone(configuration, figureName, V)};
        else if (type == "Cylinder") currentFigs = {createCylinder(configuration, figureName, V)};
        else if (type == "Torus") currentFigs = {(createTorus(configuration, figureName, V))};
        for (auto &currentFig : currentFigs) {
            Transformation::triangulate(currentFig);
            figures.push_back(currentFig);
        }
    }

    Lines2D projection = doProjectionConst(figures);
    double width, height, d, dx, dy;
    Transformation::calculateValues(projection, size, width, height, d, dx, dy);
    img::EasyImage image(lround(width), lround(height), backgroundColorElement);
    ZBuffer buffer(image.get_width(), image.get_height());
    for (auto &curFig : figures) {
        for (auto &curFace : curFig.faces) {
            img::Color drawcolor(lround(curFig.drawColor.red*255), lround(curFig.drawColor.green*255), lround(curFig.drawColor.blue*255));
            image.drawZbufTriangle(buffer, curFig.points[curFace.pointIndexes[0]], curFig.points[curFace.pointIndexes[1]],
                                   curFig.points[curFace.pointIndexes[2]], d, dx, dy, drawcolor);
        }
    }

    return image;
}