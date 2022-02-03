#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <algorithm>
#include <vecmath.h>
#include <cmath>
#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include "mesh.hpp"

class Mesh;

class KDTree {
public:

    KDTree(Mesh *m) {
        mesh = m;
        left = nullptr;
        right = nullptr;
    }

    ~KDTree() {
        delete left;
        delete right;
    }

    void build(std::vector<int> &triId, int depth, int d);
    bool intersect(const Ray &r, Hit &h, float tmin);
    void getBox(std::vector<int> &triId);

    Mesh *mesh;
    KDTree *left, *right;
    Vector3f box[2];
    int dim;
    std::vector<int> leafTriId;

private:

    bool intersectBox(const Ray &r, float tmin);
    bool innerBox(Vector3f &p);
};

#endif // KDTREE_H
