#include "ppm.hpp"
#include <vector>
#include <algorithm>

struct cmpPhoton {
    cmpPhoton(int _d) {d = _d;}
    bool operator() (Photon a, Photon b) {
        return a.pos[d] < b.pos[d];
    }
    int d;
};

void PhotonKDTree::build(std::vector<Photon>::iterator begin, std::vector<Photon>::iterator end, int splitDim) {
    if (begin == end) return;
    int length = end - begin;
    int mid = length / 2;
    cmpPhoton func(splitDim);
    std::nth_element(begin, begin + mid, end, func);
    photon = *(begin + mid);
    box[0] = box[1] = photon.pos;
    left->build(begin, begin + mid, (splitDim + 1) % 3);
    right->build(begin + mid + 1, end, (splitDim + 1) % 3);
    for (int dim = 0; dim < 3; ++dim) {
        if (left != nullptr) {
            box[0][dim] = std::min(box[0][dim], left->box[0][dim]);
            box[1][dim] = std::max(box[1][dim], left->box[1][dim]);
        }
        if (right != nullptr) {
            box[0][dim] = std::min(box[0][dim], right->box[0][dim]);
            box[1][dim] = std::max(box[1][dim], right->box[1][dim]);
        }
    }
}

void PhotonKDTree::collect(Vector3f p, float r, std::vector<Photon> &data) {
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

