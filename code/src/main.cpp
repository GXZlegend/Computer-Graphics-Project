#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>

#include "kdtree.hpp"
#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "ppm.hpp"

#include <string>

int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 3) {
        std::cout << "Usage: ./bin/PJ <input scene file> <output bmp file>" << std::endl;
        return 1;
    }
    std::string inputFile = argv[1];
    std::string outputFile = argv[2];  // only bmp is allowed.

    // First, parse the scene using SceneParser.
    // Then loop over each pixel in the image, shooting a ray
    // through that pixel and finding its intersection with
    // the scene.  Write the color at the intersection to that
    // pixel in your output image.
    SceneParser sceneparser(inputFile.c_str());
    Camera* camera = sceneparser.getCamera();
    Group* baseGroup = sceneparser.getGroup();

    // Build KDTree
    // KDTree *KDTreeRoot = new KDTree;
    // std::vector<Box> groupBox;
    // baseGroup->getBoxes(groupBox);
    // KDTreeRoot->buildKDTree(groupBox, (int) (8 + 1.3 * log((float) groupBox.size())), 0);

    // Save all lights
    std::vector<Light *> lights;
    for (int li = 0; li < sceneparser.getNumLights(); ++li) {
        lights.push_back(sceneparser.getLight(li));
    }

    // Set image
    Image img(camera->getWidth(), camera->getHeight());

    // Initialize viewpoints
    std::cout << "Processing backward pass of PPM" << std::endl;
    std::vector<std::vector<viewPoint>> imgView;
    ppmBackward(baseGroup, camera, 10, imgView);

    // Processive Photon Mapping

    for (int passId = 1; passId <= 100; ++passId) {
        std::cout << "Processing forward pass " << passId << std::endl;
        ppmForward(baseGroup, lights, 100000, imgView);
        for (int x = 0; x < camera->getWidth(); ++x) {
            for (int y = 0; y < camera->getHeight(); ++y) {
                int offset = x * camera->getHeight() + y;
                img.SetPixel(x, y, getRadiance(imgView[offset]));
            }
        }
        img.SaveImage((outputFile + std::to_string(passId) + ".bmp").c_str());
    }
    return 0;
}

