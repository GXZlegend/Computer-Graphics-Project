#ifndef SPPM_H
#define SPPM_H

#include <cmath>
#include <vector>
#include <algorithm>
#include <vecmath.h>
#include "camera.hpp"
#include "light.hpp"
#include "photon.hpp"
#include "tracer.hpp"

const float NUM = 50;
const float ALPHA = 0.7;
const float RADIUS = 0.5;

struct viewPoint {
    viewPoint() {
        power = Vector3f::ZERO;
        num = NUM;
        alpha = ALPHA;
        radius = RADIUS;
    }
    Vector3f radiance() {
        return power / (acos(-1.0) * radius * radius * num);
    }
    Vector3f power;
    float num;
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

    void build(std::vector<Photon>::iterator begin, std::vector<Photon>::iterator end, int splitDim) {
        int length = end - begin;
        int mid = length / 2;
        cmpPhoton func(splitDim);
        std::nth_element(begin, begin + mid, end, func);
        photon = *(begin + mid);
        box[0] = box[1] = photon.pos;
        if (mid > 0) {
            left = new PhotonKDTree;
            left->build(begin, begin + mid, (splitDim + 1) % 3);
            for (int dim = 0; dim < 3; ++dim) {
                box[0][dim] = std::min(box[0][dim], left->box[0][dim]);
                box[1][dim] = std::max(box[1][dim], left->box[1][dim]);
            }
        }
        if (mid < length - 1) {
            right = new PhotonKDTree;
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

    struct cmpPhoton {
        cmpPhoton(int _d) {d = _d;}
        bool operator() (Photon a, Photon b) {
            return a.pos[d] < b.pos[d];
        }
        int d;
    };

    PhotonKDTree *left, *right;
    Vector3f box[2];
    Photon photon;
};

void sppmForward(Object3D *o, std::vector<Light*> lights, int rayNum, std::vector<Photon> &photons) {
    for (Light *&l: lights) {
        for (int rayId = 0; rayId < rayNum; ++rayId) {
            if (rayId % (rayNum / 10) == 0) {
                std::cout << "rayId " << rayId << std::endl;
            }
            std::pair<Ray, Vector3f> generation = l->generate();
            Ray r = generation.first;
            Vector3f col = generation.second;
            Photon origin;
            origin.pos = r.getOrigin();
            origin.dir = -r.getDirection();
            origin.power = col * 10;
            photons.push_back(origin);
            std::vector<Trace> trace;
            traceRay(o, r, col, 20, trace, true);
            for (Trace &t: trace) {
                photons.push_back(t.photon);
            }
        }
    }
    std::cout << photons.size() << " photons in total." << std::endl;
}

void sppmBackward(Object3D *o, Camera *camera, int spp, std::vector<Photon> &photons, std::vector<viewPoint> &imgView) {
    PhotonKDTree *root = new PhotonKDTree;
    root->build(photons.begin(), photons.end(), 0);
    bool firstPass = imgView.empty();
    for (int x = 0; x < camera->getWidth(); ++x) {
        for (int y = 0; y < camera->getHeight(); ++y) {
            int offset = x * camera->getHeight() + y;
            if (offset % (camera->getWidth() * camera->getHeight() / 10) == 0) {
                std::cout << "viewId " << offset << std::endl;
            }
            int addNum = 0;
            Vector3f addPower = Vector3f::ZERO;
            if (firstPass) {
                imgView.push_back(viewPoint());
            }
            std::vector<Trace> trace;
            for (int sppId = 0; sppId < spp; ++sppId) {
                Ray r = camera->generateRay(Vector2f(x, y));
                traceRay(o, r, Vector3f(1.0 / spp), 5, trace, false);
            }
            for (Trace &t: trace) {
                std::vector<Photon> collected;
                root->collect(t.photon.pos, imgView[offset].radius, collected);
                addNum += collected.size();
                for (Photon &p: collected) {
                    addPower += t.photon.power * t.material->Shade(t.photon.dir, p.dir, t.normal, p.power);
                }
            }
            float n_prime = imgView[offset].num + imgView[offset].alpha * addNum;
            float r_prime = imgView[offset].radius;
            Vector3f power_prime = imgView[offset].power + addPower;
            if (addNum > 0) {
                r_prime *= sqrt(n_prime / (imgView[offset].num + addNum));
                power_prime *= n_prime / (imgView[offset].num + addNum);
            }
            imgView[offset].num = n_prime;
            imgView[offset].radius = r_prime;
            imgView[offset].power = power_prime;
        }
    }
    delete root;
}

void sppmPass(Object3D *o, std::vector<Light*> lights, int rayNum, 
              Camera *camera, int spp, std::vector<viewPoint> &imgView) {
    std::vector<Photon> photons;
    sppmForward(o, lights, rayNum, photons);
    sppmBackward(o, camera, spp, photons, imgView);
}

#endif // SPPM_H
