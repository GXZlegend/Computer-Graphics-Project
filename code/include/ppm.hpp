#ifndef PPM_H
#define PPM_H

#include <vector>
#include <algorithm>
#include <vecmath.h>
#include "camera.hpp"
#include "light.hpp"
#include "photon.hpp"
#include "tracer.hpp"

struct viewPoint {
    Trace trace;
    float alpha;
    float radius;
};

class PhotonKDTree {
public:
    PhotonKDTree() {
        left = nullptr;
        right = nullptr;
        box[0] = Vector3f(1e38);
        box[1] = Vector3f(-1e38);
    }
    
    ~PhotonKDTree() {
        delete left;
        delete right;
    }

    static struct cmpPhoton {
        cmpPhoton(int _d) {d = _d;}
        bool operator() (Photon a, Photon b) {
            return a.pos[d] < b.pos[d];
        }
        int d;
    };

    void build(std::vector<Photon>::iterator begin, std::vector<Photon>::iterator end, int splitDim) {
        int length = end - begin;
        int mid = length / 2;
        cmpPhoton func(splitDim);
        std::nth_element(begin, begin + mid, end, func);
        photon = *(begin + mid);
        box[0] = box[1] = photon.pos;
        if (mid > 0) {
            left->build(begin, begin + mid, (splitDim + 1) % 3);
            for (int dim = 0; dim < 3; ++dim) {
                box[0][dim] = std::min(box[0][dim], left->box[0][dim]);
                box[1][dim] = std::max(box[1][dim], left->box[1][dim]);
            }
        }
        if (mid < length - 1) {
            right->build(begin + mid + 1, end, (splitDim + 1) % 3);
            for (int dim = 0; dim < 3; ++dim) {
                box[0][dim] = std::min(box[0][dim], right->box[0][dim]);
                box[1][dim] = std::max(box[1][dim], right->box[1][dim]);
            }
        }
    }

    void collect(Vector3f p, float r, std::vector<Photon> &data) {
        float dis = 0;
        for (int dim = 0; dim < 3; ++dim) {
            if (p[dim] < box[0][dim]) {
                dis += (box[0][dim] - p[dim]) * (box[0][dim] - p[dim]);
            }
            else if(p[dim] > box[1][dim]) {
                dis += (p[dim] - box[1][dim]) * (p[dim] - box[1][dim]);
            }
        }
        if (dis <= r * r) {
            if ((p - photon.pos).squaredLength() <= r * r) {
                data.emplace_back(photon);
            }
            if (left != nullptr) {
                left->collect(p, r, data);
            }
            if (right != nullptr) {
                right->collect(p, r, data);
            }
        }
    }

private:
    PhotonKDTree *left, *right;
    Vector3f box[2];
    Photon photon;
};

void ppmBackward(Object3D *o, Camera *camera, int spp, std::vector<std::vector<Trace>> &data) {
    for (int x = 0; x < camera->getWidth(); ++x) {
        for (int y = 0; y < camera->getHeight(); ++y) {
            std::vector<Trace> trace;
            for (int sppId = 0; sppId < spp; ++spp) {
                Ray r = camera->generateRay(Vector2f(x, y));
                traceRay(o, r, Vector3f(1), 5, trace);
            }
            data.push_back(trace);
        }
    }
}

void ppmForward(Object3D *o, std::vector<Light*> lights, int rayNum) {
    std::vector<Photon> photons;
    for (Light *&l: lights) {
        for (int rayId = 0; rayId < rayNum; ++rayId) {
            std::pair<Ray, Vector3f> generation = l->generate();
            Ray r = generation.first;
            Vector3f col = generation.second;
            std::vector<Trace> trace;
            traceRay(o, r, col, 5, trace);
            for (Trace &t: trace) {
                photons.push_back(t.photon);
            }
        }
    }
    PhotonKDTree *root = new PhotonKDTree;
    root->build(photons.begin(), photons.end(), 0);
}

#endif // PPM_H
