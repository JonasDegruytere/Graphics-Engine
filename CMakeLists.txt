cmake_minimum_required(VERSION 3.22)
project(engine)
set(CMAKE_CXX_STANDARD 14)

add_executable(engine engine.cpp System2DL.cpp System2DL.h Structs.cpp Structs.h
        easy_image.h easy_image.cc l_parser.h l_parser.cc ini_configuration.h
        ini_configuration.cc vector3d.cc vector3d.h System3DL.cpp System3DL.h
        Wireframes.cpp Wireframes.h Figures.cpp Figures.h ZBuffer.cpp ZBuffer.h Triangles3D.cpp Triangles3D.h)
