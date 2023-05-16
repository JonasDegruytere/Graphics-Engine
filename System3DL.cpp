//
// Created by Jonas on 15-5-2023.
//

#include "System3DL.h"

void Transfor::toPolar(const Vector3D &point, double &theta, double &phi, double &r) {
    r = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));
    theta = std::atan2(point.y, point.x);
    phi = std::acos(point.z/r);
}

void Transformation::splitTriangles(Figure &figure) {
    Figure newFig;
    newFig.drawColor = figure.drawColor;
    int pointCounter = 0;
    for (auto face : figure.faces) {
        Vector3D a = figure.points[face.pointIndexes[0]];
        Vector3D b = figure.points[face.pointIndexes[1]];
        Vector3D c = figure.points[face.pointIndexes[2]];

        Vector3D d = (a+b)/2;
        Vector3D e = (a+c)/2;
        Vector3D f = (b+c)/2;

        newFig.points.push_back(a);
        newFig.points.push_back(b);
        newFig.points.push_back(c);
        newFig.points.push_back(d);
        newFig.points.push_back(e);
        newFig.points.push_back(f);

        Face face1({pointCounter, pointCounter+3, pointCounter+4});
        Face face2({pointCounter+1, pointCounter+5, pointCounter+3});
        Face face3({pointCounter+2, pointCounter+4, pointCounter+5});
        Face face4({pointCounter+3, pointCounter+5, pointCounter+4});

        newFig.faces.push_back(face1);
        newFig.faces.push_back(face2);
        newFig.faces.push_back(face3);
        newFig.faces.push_back(face4);

        pointCounter += 6;
    }
    figure = newFig;
}

void Transformation::triangulate(Figure &fig) {
    std::vector<Face> newFaces;

    for (auto &f: fig.faces) {
        if (f.pointIndexes.size() > 3) {
            for (unsigned int i = 1; i < f.pointIndexes.size()-1; i++) {
                Face newFace({f.pointIndexes[0], f.pointIndexes[i], f.pointIndexes[i+1]});
                newFaces.push_back(newFace);
            }
        } else newFaces.push_back(f);
    }
    fig.faces = newFaces;
}

void Transformation::calculateValues(Lines2D &lines, double size, double &width, double &height, double &d, double &dx, double &dy) {
    std::list<Point2D> points;
    for (auto &i: lines) {
        points.push_back(i.p1);
        points.push_back(i.p2);
    }
    std::list<double> xPoints;
    std::list<double> yPoints;
    for (auto &i: points) {
        xPoints.push_back(i.x);
        yPoints.push_back(i.y);
    }

    double xMax = *std::max_element(xPoints.begin(), xPoints.end());
    double xMin = *std::min_element(xPoints.begin(), xPoints.end());
    double yMax = *std::max_element(yPoints.begin(), yPoints.end());
    double yMin = *std::min_element(yPoints.begin(), yPoints.end());

    double xRange = xMax - xMin;
    double yRange = yMax - yMin;

    width = size * (xRange / std::max(xRange, yRange));
    height = size * (yRange / std::max(xRange, yRange));

    d = 0.95 * (width / xRange);

    double DCx = d * ((xMax + xMin)/2.0);
    double DCy = d * ((yMax + yMin)/2.0);
    dx = (width/2.0) - DCx;
    dy = (height/2.0) - DCy;
}

Matrix Transformation::scaleFigure(double scale) {
    Matrix S;
    S(1, 1) = scale;
    S(2, 2) = scale;
    S(3, 3) = scale;

    return S;
}

Matrix Transformation::rotateX(double angle) {
    Matrix Mx;
    Mx(2, 2) = cos(angle);
    Mx(2, 3) = sin(angle);
    Mx(3, 2) = -sin(angle);
    Mx(3, 3) = cos(angle);

    return Mx;
}

Matrix Transformation::rotateY(double angle) {
    Matrix My;
    My(1, 1) = cos(angle);
    My(1, 3) = -sin(angle);
    My(3, 1) = sin(angle);
    My(3, 3) = cos(angle);

    return My;
}

Matrix Transformation::rotateZ(double angle) {
    Matrix Mz;
    Mz(1, 1) = cos(angle);
    Mz(1, 2) = sin(angle);
    Mz(2, 1) = -sin(angle);
    Mz(2, 2) = cos(angle);

    return Mz;
}

Matrix Transformation::translate(const Vector3D &vec) {
    Matrix T;
    T(4, 1) = vec.x;
    T(4, 2) = vec.y;
    T(4, 3) = vec.z;

    return T;
}

Matrix Transformation::eyePointTrans(const Vector3D &eyepoint) {
    double theta, phi, r;
    Transfor::toPolar(eyepoint, theta, phi, r);

    Matrix M;
    M(1, 1) = -std::sin(theta);
    M(1, 2) = -std::cos(theta) * std::cos(phi);
    M(1, 3) = std::cos(theta) * std::sin(phi);
    M(2, 1) = std::cos(theta);
    M(2, 2) = -std::sin(theta) * std::cos(phi);
    M(2, 3) = std::sin(theta) * std::sin(phi);
    M(3, 2) = std::sin(phi);
    M(3, 3) = std::cos(phi);
    M(4, 3) = -r;

    return M;
}

void Transformation::applyTransformation(Figure &fig, const Matrix &M) {
    for (auto &j : fig.points) {
        j *= M;
    }
}

Figure System3DL::LSystem3D(const ini::Configuration &configuration, const std::string &figureName, Matrix &V, bool transform, bool light) {
    const double rotateX = configuration[figureName]["rotateX"].as_double_or_die();
    const double rotateY = configuration[figureName]["rotateY"].as_double_or_die();
    const double rotateZ = configuration[figureName]["rotateZ"].as_double_or_die();
    const double scale = configuration[figureName]["scale"].as_double_or_die();
    std::vector<double> center = configuration[figureName]["center"].as_double_tuple_or_die();;
    const std::string inputfile = configuration[figureName]["inputfile"].as_string_or_die();
    std::vector<double> color;

    if (light) {
        color = configuration[figureName]["ambientReflection"].as_double_tuple_or_die();
    }
    else color = configuration[figureName]["color"].as_double_tuple_or_die();
    LParser::LSystem3D l_system;
    std::ifstream input_stream(inputfile);
    input_stream >> l_system;
    input_stream.close();

    const std::set<char> alphabet = l_system.get_alphabet();
    std::string initiator = l_system.get_initiator();
    double angle = l_system.get_angle();
    const unsigned int iterations = l_system.get_nr_iterations();

    std::map<char, std::string> replacements;
    std::map<char, bool> draw;
    for (auto i: alphabet) {
        draw[i] = l_system.draw(i);
        replacements[i] = l_system.get_replacement(i);
    }

    angle = (angle*M_PI)/180;

    Vector3D curPoint = Vector3D::point(0, 0, 0);
    Vector3D H = Vector3D::vector(1, 0, 0);
    Vector3D L = Vector3D::vector(0, 1, 0);
    Vector3D U = Vector3D::vector(0, 0, 1);
    std::stack<Vector3D> pointStack;
    std::stack<Vector3D> HStack;
    std::stack<Vector3D> LStack;
    std::stack<Vector3D> UStack;

    Figure fig;
    fig.drawColor = Color(color[0],color[1],color[2]);
    fig.points.push_back(curPoint);
    int indexCounter = 0;

    for (unsigned int i = 0; i < iterations; i++) {
        std::string replacement;
        for (auto j: initiator) {
            if (j == '+') replacement += "+";
            else if (j == '-') replacement += "-";
            else if (j == '(') replacement += "(";
            else if (j == ')') replacement += ")";
            else if (j == '^') replacement += "^";
            else if (j == '&') replacement += "&";
            else if (j == '\\') replacement += "\\";
            else if (j == '/') replacement += "/";
            else if (j == '|') replacement += "|";
            else replacement += replacements[j];
        }
        initiator = replacement;
    }

    std::vector<Vector3D> toAdd;
    for (unsigned int k = 0; k < initiator.length(); k++) {
        char i = initiator[k];
        if (i == '+') {
            if (!toAdd.empty()) {
                fig.points.push_back(toAdd[toAdd.size()-1]);
                indexCounter++;
                Face newFace({indexCounter, indexCounter-1});
                fig.faces.push_back(newFace);
                toAdd.clear();
            }
            Vector3D tempH(H);
            H = (H*cos(angle)) + (L*sin(angle));
            L = (L*cos(angle)) - (tempH*sin(angle));
            continue;
        }
        if (i == '-') {
            if (!toAdd.empty()) {
                fig.points.push_back(toAdd[toAdd.size()-1]);
                indexCounter++;
                Face newFace({indexCounter, indexCounter-1});
                fig.faces.push_back(newFace);
                toAdd.clear();
            }
            Vector3D tempH(H);
            H = (H*cos(-angle)) + (L*sin(-angle));
            L = (L*cos(-angle)) - (tempH*sin(-angle));
            continue;
        }
        if (i == '^') {
            if (!toAdd.empty()) {
                fig.points.push_back(toAdd[toAdd.size()-1]);
                indexCounter++;
                Face newFace({indexCounter, indexCounter-1});
                fig.faces.push_back(newFace);
                toAdd.clear();
            }
            Vector3D tempH(H);
            H = (H*cos(angle)) + (U*sin(angle));
            U = (U*cos(angle)) - (tempH*sin(angle));
            continue;
        }
        if (i == '&') {
            if (!toAdd.empty()) {
                fig.points.push_back(toAdd[toAdd.size()-1]);
                indexCounter++;
                Face newFace({indexCounter, indexCounter-1});
                fig.faces.push_back(newFace);
                toAdd.clear();
            }
            Vector3D tempH(H);
            H = (H*cos(-angle)) + (U*sin(-angle));
            U = (U*cos(-angle)) - (tempH*sin(-angle));
            continue;
        }
        if (i == '\\') {
            if (!toAdd.empty()) {
                fig.points.push_back(toAdd[toAdd.size()-1]);
                indexCounter++;
                Face newFace({indexCounter, indexCounter-1});
                fig.faces.push_back(newFace);
                toAdd.clear();
            }
            Vector3D tempL(L);
            L = (L*cos(angle)) - (U*sin(angle));
            U = (tempL*sin(angle)) + (U*cos(angle));
            continue;
        }
        if (i == '/') {
            if (!toAdd.empty()) {
                fig.points.push_back(toAdd[toAdd.size()-1]);
                indexCounter++;
                Face newFace({indexCounter, indexCounter-1});
                fig.faces.push_back(newFace);
                toAdd.clear();
            }
            Vector3D tempL(L);
            L = (L*cos(-angle)) - (U*sin(-angle));
            U = (tempL*sin(-angle)) + (U*cos(-angle));
            continue;
        }
        if (i == '|') {
            if (!toAdd.empty()) {
                fig.points.push_back(toAdd[toAdd.size()-1]);
                indexCounter++;
                Face newFace({indexCounter, indexCounter-1});
                fig.faces.push_back(newFace);
                toAdd.clear();
            }
            H = -H;
            L = -L;
            continue;
        }
        if (i == '(') {
            if (!toAdd.empty()) {
                fig.points.push_back(toAdd[toAdd.size()-1]);
                indexCounter++;
                Face newFace({indexCounter, indexCounter-1});
                fig.faces.push_back(newFace);
                toAdd.clear();
            }
            pointStack.push(curPoint);
            HStack.push(H);
            LStack.push(L);
            UStack.push(U);
            continue;
        }
        if (i == ')') {
            if (!toAdd.empty()) {
                fig.points.push_back(toAdd[toAdd.size()-1]);
                indexCounter++;
                Face newFace({indexCounter, indexCounter-1});
                fig.faces.push_back(newFace);
                toAdd.clear();
            }
            curPoint = pointStack.top();
            pointStack.pop();
            H = HStack.top();
            HStack.pop();
            L = LStack.top();
            LStack.pop();
            U = UStack.top();
            UStack.pop();

            fig.points.push_back(curPoint);
            indexCounter++;
            continue;
        }
        curPoint += H;
        if (draw[i]) toAdd.push_back(curPoint);
        else {
            if (!toAdd.empty()) {
                fig.points.push_back(toAdd[toAdd.size()-1]);
                indexCounter++;
                Face newFace({indexCounter, indexCounter-1});
                fig.faces.push_back(newFace);
                toAdd.clear();
            }
        }
    }

    Matrix S = Transformation::scaleFigure(scale);
    Matrix rX = Transformation::rotateX((rotateX*M_PI)/180);
    Matrix rY = Transformation::rotateY((rotateY*M_PI)/180);
    Matrix rZ = Transformation::rotateZ((rotateZ*M_PI)/180);
    Matrix T = Transformation::translate(Vector3D::point(center[0], center[1], center[2]));

    Matrix F = S * rX * rY * rZ * T * V;

    if (transform) Transformation::applyTransformation(fig, F);

    return fig;
}