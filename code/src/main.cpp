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
#include "triangle.hpp"

#include <string>

using namespace std;

int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 3) {
        cout << "Usage: ./bin/PJ <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2];  // only bmp is allowed.

    // TODO: Main RayCasting Logic
    // First, parse the scene using SceneParser.
    // Then loop over each pixel in the image, shooting a ray
    // through that pixel and finding its intersection with
    // the scene.  Write the color at the intersection to that
    // pixel in your output image.
    SceneParser sceneparser(inputFile.c_str());
    Camera* camera = sceneparser.getCamera();
    Group* baseGroup = sceneparser.getGroup();
    KDTree *KDTreeRoot = new KDTree;
    std::vector<Box> groupBox;
    baseGroup->getBoxes(groupBox);
    KDTreeRoot->buildKDTree(groupBox, (int) (8 + 1.3 * log((float) groupBox.size())), 0);
    // KDTreeRoot->buildKDTree(groupBox, 0, 0);
    Image img(camera->getWidth(), camera->getHeight());
    for (int x = 0; x < camera->getWidth(); ++x) {
        for (int y = 0; y < camera->getHeight(); ++y) {
            Ray camRay = camera->generateRay(Vector2f(1.0 * x, 1.0 * y));
            Hit hit;
            bool intersect = baseGroup->intersect(camRay, hit, 0);
            if (intersect) {
                Vector3f finalColor = Vector3f::ZERO;
                for (int li = 0; li < sceneparser.getNumLights(); ++li) {
                    Light* light = sceneparser.getLight(li);
                    Vector3f L, lightColor;
                    light->getIllumination(camRay.pointAtParameter(hit.getT()), L, lightColor);
                    finalColor += hit.getMaterial()->Shade(camRay, hit, L, lightColor);
                }
                img.SetPixel(x, y, finalColor);
            }
            else
                img.SetPixel(x, y, sceneparser.getBackgroundColor());
        }
    }
    img.SaveImage(outputFile.c_str());
    cout << "Hello! Computer Graphics!" << endl;
    return 0;
}

