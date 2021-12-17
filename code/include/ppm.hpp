#ifndef PPM_H
#define PPM_H

#include <cmath>
#include <vector>
#include <algorithm>
#include <vecmath.h>
#include "camera.hpp"
#include "light.hpp"
#include "photon.hpp"
#include "tracer.hpp"

const float ALPHA = 0.7;
const float RADIUS = 1;

struct viewPoint {
    Vector3f radiance() {
        return power / (acos(-1.0) * radius * radius * num);
    }
    Trace trace;
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

void ppmBackward(Object3D *o, Camera *camera, int spp, std::vector<std::vector<viewPoint>> &imgView) {
    for (int x = 0; x < camera->getWidth(); ++x) {
        std::cout << "Line " << x << std::endl;
        for (int y = 0; y < camera->getHeight(); ++y) {
            std::vector<Trace> trace;
            std::vector<viewPoint> view;
            for (int sppId = 0; sppId < spp; ++sppId) {
                Ray r = camera->generateRay(Vector2f(x, y));
                traceRay(o, r, Vector3f(1.0 / spp), 20, trace);
            }
            for (Trace &t: trace) {
                viewPoint point;
                point.trace = t;
                point.power = Vector3f::ZERO;
                point.num = 0;
                point.alpha = ALPHA;
                point.radius = RADIUS;
                view.push_back(point);
            }
            imgView.push_back(view);
        }
    }
}

void ppmForward(Object3D *o, std::vector<Light*> lights, int rayNum, std::vector<std::vector<viewPoint>> &imgView) {
    std::vector<Photon> photons;
    for (Light *&l: lights) {
        for (int rayId = 0; rayId < rayNum; ++rayId) {
            if (rayId % 10000 == 0) {
                std::cout << "rayId " << rayId << std::endl;
            }
            std::pair<Ray, Vector3f> generation = l->generate();
            Ray r = generation.first;
            Vector3f col = generation.second;
            Photon origin;
            origin.pos = r.getOrigin();
            origin.dir = -r.getDirection();
            origin.power = col;
            photons.push_back(origin);
            std::vector<Trace> trace;
            traceRay(o, r, col, 20, trace);
            for (Trace &t: trace) {
                photons.push_back(t.photon);
            }
        }
    }
    PhotonKDTree *root = new PhotonKDTree;
    root->build(photons.begin(), photons.end(), 0);
    for (std::vector<viewPoint> &view: imgView) {
        for (viewPoint &point: view) {
            std::vector<Photon> photon;
            root->collect(point.trace.photon.pos, point.radius, photon);
            int m = (int) photon.size();
            Vector3f power = Vector3f::ZERO;
            for (Photon &p: photon) {
                power += point.trace.photon.power
                       * point.trace.material->Shade(point.trace.photon.dir, p.dir, point.trace.normal, p.power);
                if (power[0] < 0 || power[1] < 0 || power[2] < 0) {
                    std::cout << "fuck" << std::endl;
                }
            }
            float n_prime = point.num + point.alpha * m;
            float r_prime = point.radius;
            Vector3f power_prime = point.power + power;
            if (point.num + m > 0) {
                r_prime *= sqrt(n_prime / (point.num + m));
                power_prime *= n_prime / (point.num + m);
            }
            point.num = n_prime;
            point.radius = r_prime;
            point.power = power_prime;
        }
    }
    delete root;
}

Vector3f getRadiance(std::vector<viewPoint> view) {
    Vector3f radiance = Vector3f::ZERO;
    for (viewPoint &point: view) {
        radiance += point.radiance();
    }
    return radiance;
}

#endif // PPM_H
