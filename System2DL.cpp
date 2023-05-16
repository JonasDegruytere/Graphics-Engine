//
// Created by Jonas on 15-5-2023.
//

#include "System2DL.h"

img::EasyImage System2DL::LSystem2D(const ini::Configuration &configuration) {
    const unsigned int size = configuration["General"]["size"].as_int_or_die();
    std::vector<double> backgroundColor = configuration["General"]["backgroundcolor"].as_double_tuple_or_die();
    img::Color BackgroundColorElement(backgroundColor[0]*255, backgroundColor[1]*255, backgroundColor[2]*255);
    std::vector<double> lineColor = configuration["2DLSystem"]["color"].as_double_tuple_or_die();
    Color LineColorElement(lineColor[0], lineColor[1], lineColor[2]);
    std::string inputfile = configuration["2DLSystem"]["inputfile"].as_string_or_die();

    std::string initiator;
    double angle;
    double starting_angle;
    std::map<char, bool> draw;

    std::string stochastic;
    if (!configuration["2DLSystem"]["stochastic"].as_string_if_exists(stochastic)) {

        LParser::LSystem2D l_system;
        std::ifstream input_stream(inputfile);
        input_stream >> l_system;
        input_stream.close();

        const std::set<char> alphabet = l_system.get_alphabet();
        initiator = l_system.get_initiator();
        angle = l_system.get_angle();
        starting_angle = l_system.get_starting_angle();
        const unsigned int iterations = l_system.get_nr_iterations();
        std::map<char, std::string> replacements;
        for (auto i: alphabet) {
            draw[i] = l_system.draw(i);
            replacements[i] = l_system.get_replacement(i);
        }

        for (unsigned int i = 0; i < iterations; i++) {
            std::string replacement;
            for (auto j: initiator) {
                if (j == '+') replacement += "+";
                else if (j == '-') replacement += "-";
                else if (j == '(') replacement += "(";
                else if (j == ')') replacement += ")";
                else replacement += replacements[j];
            }
            initiator = replacement;
        }
    }

    Point2D cur_point;
    cur_point.x = 0;
    cur_point.y = 0;
    double cur_dir = starting_angle;
    Lines2D lines;
    Line2D new_line;
    std::stack<Point2D> points_stack;
    std::stack<double> dir_stack;
    for (auto i: initiator) {
        if (i == '+') cur_dir += angle;
        else if (i == '-') cur_dir -= angle;
        else if (i == '(') {
            points_stack.push(cur_point);
            dir_stack.push(cur_dir);
        } else if (i == ')') {
            Point2D temp_point = points_stack.top();
            points_stack.pop();
            cur_point.x = temp_point.x;
            cur_point.y = temp_point.y;

            cur_dir = dir_stack.top();
            dir_stack.pop();
        } else {
            Point2D new_point;
            new_point.x = cur_point.x + cos((cur_dir * M_PI) / 180.0);
            new_point.y = cur_point.y + sin((cur_dir * M_PI) / 180.0);
            if (draw[i]) {
                new_line.p1 = cur_point;
                new_line.p2 = new_point;
                new_line.color = LineColorElement;
                lines.push_back(new_line);
            }
            cur_point = new_point;
        }
    }
    return coordToPixel(lines, size, BackgroundColorElement);
}

img::EasyImage System2DL::coordToPixel(Lines2D &lines, double size, img::Color backgroundColor, bool zBuffer) {
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

    double imageX = size * (xRange / std::max(xRange, yRange));
    double imageY = size * (yRange / std::max(xRange, yRange));

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
            image.draw_zbuf_line(buffer, lround(i.p1.x), lround(i.p1.y), i.z1, lround(i.p2.x), lround(i.p2.y), i.z2, col);
        } else {
            image.draw_line(lround(i.p1.x), lround(i.p1.y), lround(i.p2.x), lround(i.p2.y), col);
        }
    }
    return image;
}
