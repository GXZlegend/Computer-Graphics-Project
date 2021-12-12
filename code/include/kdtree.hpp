#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <vecmath.h>
#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"


class KDTree: public Object3D {

public:

    KDTree() {
        left = nullptr;
        right = nullptr;
        box.obj = this;
    }

    ~KDTree() override {
        delete left;
        delete right;
    }

    void buildKDTree(std::vector<Box> &rootBox, int depth, int dim);
    bool intersect(const Ray &r, Hit &h, float tmin) override;
    void getBoxes(std::vector<Box> &data) override;

private:
    KDTree *left, *right;
    std::vector<Box> leafbox;
    int dim;
};

#endif // KDTREE_H
