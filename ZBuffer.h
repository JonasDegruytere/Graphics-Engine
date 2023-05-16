//
// Created by Jonas on 15-5-2023.
//

#ifndef ENGINE_ZBUFFER_H
#define ENGINE_ZBUFFER_H
#include <vector>
#include <limits>
using namespace std;

class ZBuffer: public std::vector<std::vector<double>> {
public:
    ZBuffer(const unsigned int width, const unsigned int height) :
    std::vector<std::vector<double>>(width, std::vector<double>(height, std::numeric_limits<double>::infinity())) {}
};


#endif //ENGINE_ZBUFFER_H
