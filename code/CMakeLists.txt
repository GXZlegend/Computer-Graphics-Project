CMAKE_MINIMUM_REQUIRED(VERSION 3.16)
PROJECT(PJ CXX)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

ADD_SUBDIRECTORY(deps/vecmath)

SET(PJ_SOURCES
        src/image.cpp
        src/kdtree.cpp
        src/main.cpp
        src/mesh.cpp
        src/ppm.cpp
        src/scene_parser.cpp)

SET(PJ_INCLUDES
        include/camera.hpp
        include/curve.hpp
        include/group.hpp
        include/hit.hpp
        include/image.hpp
        include/kdtree.hpp
        include/light.hpp
        include/material.hpp
        include/mesh.hpp
        include/object3d.hpp
        include/plane.hpp
        include/ppm.hpp
        include/ray.hpp
        include/revsurface.hpp
        include/scene_parser.hpp
        include/sphere.hpp
        include/tracer.hpp
        include/transform.hpp
        include/triangle.hpp
        )

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

ADD_EXECUTABLE(${PROJECT_NAME} ${PJ_SOURCES} ${PJ_INCLUDES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} vecmath)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE include)
