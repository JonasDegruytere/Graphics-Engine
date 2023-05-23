//
// Created by Jonas on 15-5-2023.
//

#include "System2DL.h"
string System2DL::generateLSystemString(std::string &currentString, map<char, std::string> &replacementRules,
                                        unsigned int depth) {
    string nextString;
    if (depth == 0){
        return currentString;
    }
    for (const auto& step : currentString){
        switch(step){
            case '-': nextString.push_back('-'); break;
            case '+': nextString.push_back('+'); break;
            case '^': nextString.push_back('^'); break;
            case '&': nextString.push_back('&'); break;
            case '\\': nextString.push_back('\\'); break;
            case '/': nextString.push_back('/'); break;
            case '|': nextString.push_back('|'); break;
            case '(': nextString.push_back('('); break;
            case ')': nextString.push_back(')'); break;
            default:
                for (const auto& letter : replacementRules){
                    if (step == letter.first){
                        nextString += letter.second;
                    }
                }
        }
    }
    depth--;
    return generateLSystemString(nextString, replacementRules, depth);
}


img::EasyImage System2DL::LSystem2D(const ini::Configuration &configuration) {
    vector<double> lineColor = configuration["2DLSystem"]["color"].as_double_tuple_or_die();
    Color LineColorElement(lineColor[0], lineColor[1], lineColor[2]);
    string inputfile = configuration["2DLSystem"]["inputfile"].as_string_or_die();
    const unsigned int size = configuration["General"]["size"].as_int_or_die();
    vector<double> backgroundColor = configuration["General"]["backgroundcolor"].as_double_tuple_or_die();
    img::Color BackgroundColorElement(backgroundColor[0]*255, backgroundColor[1]*255, backgroundColor[2]*255);

    double starting_angle;
    map<char, bool> draw;
    string initiator;
    double angle;

    LParser::LSystem2D l_system;
    ifstream input_stream(inputfile);
    input_stream >> l_system;
    input_stream.close();
    
    angle = l_system.get_angle();
    starting_angle = l_system.get_starting_angle();
    const set<char> alphabet = l_system.get_alphabet();
    map<char, string> replacements;
    for (auto i: alphabet) {
        draw[i] = l_system.draw(i);
        replacements[i] = l_system.get_replacement(i);
    }
    initiator = l_system.get_initiator();
    const unsigned int iterations = l_system.get_nr_iterations();
    initiator = System2DL::generateLSystemString(initiator,replacements,iterations);

    double curDirection = starting_angle;
    Point2D currentPoint;
    currentPoint.x = 0;
    currentPoint.y = 0;
    Lines2D lines;
    Line2D newLine;
    stack<Point2D> pointsStack;
    stack<double> TempStack;
    for (auto i: initiator) {
        if (i == '+') {
            curDirection += angle;
        }
        else if (i == '-') {
            curDirection -= angle;
        }
        else if (i == '(') {
            pointsStack.push(currentPoint);
            TempStack.push(curDirection);
        }
        else if (i == ')') {
            Point2D temp_point = pointsStack.top();
            pointsStack.pop();
            currentPoint.x = temp_point.x;
            currentPoint.y = temp_point.y;
            curDirection = TempStack.top();
            TempStack.pop();
        }
        else {
            Point2D newPoint;
            newPoint.x = currentPoint.x + cos((curDirection * M_PI) / 180.0);
            newPoint.y = currentPoint.y + sin((curDirection * M_PI) / 180.0);
            if (draw[i]) {
                newLine.p1 = currentPoint;
                newLine.p2 = newPoint;
                newLine.color = LineColorElement;
                lines.push_back(newLine);
            }
            currentPoint = newPoint;
        }
    }
    return DrawLines(lines, size, BackgroundColorElement);
}

img::EasyImage System2DL::DrawLines(Lines2D &lines, double size, img::Color backgroundColor, bool zBuffer) {
    list<Point2D> points;
    list<double> xPoints;
    list<double> yPoints;
    for (auto &i: lines) {
        points.push_back(i.p1);
        points.push_back(i.p2);
    }
    for (auto &i: points) {
        xPoints.push_back(i.x);
        yPoints.push_back(i.y);
    }

    double xMax = *max_element(xPoints.begin(), xPoints.end());
    double xMin = *min_element(xPoints.begin(), xPoints.end());
    double yMax = *max_element(yPoints.begin(), yPoints.end());
    double yMin = *min_element(yPoints.begin(), yPoints.end());
    double xRange = xMax - xMin;
    double yRange = yMax - yMin;
    double imageX = size * (xRange / max(xRange, yRange));
    double imageY = size * (yRange / max(xRange, yRange));

    double d = 0.95 * (imageX / xRange);
    double DCx = d * ((xMax + xMin)/2.0);
    double DCy = d * ((yMax + yMin)/2.0);
    double dx = (imageX/2.0) - DCx;
    double dy = (imageY/2.0) - DCy;

    img::EasyImage image(lround(imageX), lround(imageY), backgroundColor);
    ZBuffer buffer(image.get_width(), image.get_height());
    for (auto &i : lines) {
        i.p1.x = i.p1.x * d + dx;
        i.p1.y = i.p1.y * d + dy;
        i.p2.x = i.p2.x * d + dx;
        i.p2.y = i.p2.y * d + dy;
        img::Color col(i.color.red*255,i.color.green*255,i.color.blue*255);
        if (zBuffer) {
            image.drawZbufLine(buffer, lround(i.p1.x), lround(i.p1.y), i.z1, lround(i.p2.x), lround(i.p2.y), i.z2, col);
        }
        else {
            image.draw_line(lround(i.p1.x), lround(i.p1.y), lround(i.p2.x), lround(i.p2.y), col);
        }
    }
    return image;
}
