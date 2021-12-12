#ifndef PPM_H
#define PPM_H

#include <vector>
#include <vecmath.h>
#include "ray.hpp"
#include "hit.hpp"
#include "material.hpp"
#include "photon.hpp"

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

    void build(std::vector<Photon>::iterator begin, std::vector<Photon>::iterator end, int dim);

    void collect(Vector3f p, float r, std::vector<Photon> &data);

private:
    PhotonKDTree *left, *right;
    Vector3f box[2];
    Photon photon;
};

#endif // PPM_H
