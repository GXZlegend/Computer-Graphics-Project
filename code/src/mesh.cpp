#include "mesh.hpp"
#include "triangle.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>

struct TriangleIndex {
    TriangleIndex() {
        x[0] = 0; x[1] = 0; x[2] = 0;
    }
    int &operator[](const int i) { return x[i]; }
    int x[3]{};
};

bool Mesh::intersect(const Ray &r, Hit &h, float tmin) {
    bool flag = false;
    for (int triId = 0; triId < (int) t.size(); ++triId)
        flag |= t[triId]->intersect(r, h, tmin);
    return flag;
}

void Mesh::getBoxes(std::vector<Box> &data) {
    for (int triId = 0; triId < (int) t.size(); ++triId)
        t[triId]->getBoxes(data);
}

Mesh::Mesh(const char *filename, Material *material) : Object3D(material) {
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    std::string line;
    std::string vTok("v");
    std::string fTok("f");
    std::string texTok("vt");
    char bslash = '/', space = ' ';
    std::string tok;
    int texID;
    while (true) {
        std::getline(f, line);
        if (f.eof()) {
            break;
        }
        if (line.size() < 3) {
            continue;
        }
        if (line.at(0) == '#') {
            continue;
        }
        std::stringstream ss(line);
        ss >> tok;
        if (tok == vTok) {
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            v.push_back(vec);
        } else if (tok == fTok) {
            if (line.find(bslash) != std::string::npos) {
                std::replace(line.begin(), line.end(), bslash, space);
                std::stringstream facess(line);
                TriangleIndex trig;
                facess >> tok;
                for (int ii = 0; ii < 3; ii++) {
                    facess >> trig[ii] >> texID;
                    trig[ii]--;
                }
                t.push_back(new Triangle(v[trig[0]], v[trig[1]], v[trig[2]], material));
            } else {
                TriangleIndex trig;
                for (int ii = 0; ii < 3; ii++) {
                    ss >> trig[ii];
                    trig[ii]--;
                }
                t.push_back(new Triangle(v[trig[0]], v[trig[1]], v[trig[2]], material));
            }
        } else if (tok == texTok) {
            Vector2f texcoord;
            ss >> texcoord[0];
            ss >> texcoord[1];
        }
    }
    computeBox();
    f.close();
}

void Mesh::computeBox() {
    box = t[0]->box;
    box.obj = this;
    for (int triId = 1; triId < (int) t.size(); ++triId) {
        for (int dim = 0; dim < 3; ++dim) {
            box.v[0][dim] = std::min(box.v[0][dim], t[triId]->box.v[0][dim]);
            box.v[1][dim] = std::max(box.v[1][dim], t[triId]->box.v[1][dim]);
        }
    }
}
