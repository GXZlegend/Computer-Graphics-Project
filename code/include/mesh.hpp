#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector3f.h"


class Box {

public:

    bool crossBox(const Ray &r, float tmin) {
        float t1 = -1e38, t2 = 1e38;
        for (int dim = 0; dim < 3; ++dim) {
            if (r.getDirection()[dim] > 0) {
                t1 = std::max(t1, (v[0][dim] - r.getOrigin()[dim]) / r.getDirection()[dim]);
                t2 = std::min(t2, (v[1][dim] - r.getOrigin()[dim]) / r.getDirection()[dim]);
            }
            else if (r.getDirection()[dim] < 0) {
                t1 = std::max(t1, (v[1][dim] - r.getOrigin()[dim]) / r.getDirection()[dim]);
                t2 = std::min(t2, (v[0][dim] - r.getOrigin()[dim]) / r.getDirection()[dim]);
            }
            else if (v[0][dim] > r.getOrigin()[dim] || v[1][dim] < r.getOrigin()[dim])
                return false;
        }
        return (t1 <= t2 && t2 > tmin);
    }
    
    Vector3f v[2];

};


class Mesh : public Object3D {

public:
    Mesh(const char *filename, Material *m);

    struct TriangleIndex {
        TriangleIndex() {
            x[0] = 0; x[1] = 0; x[2] = 0;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3]{};
    };

    std::vector<Vector3f> v;
    std::vector<TriangleIndex> t;
    bool intersect(const Ray &r, Hit &h, float tmin) override;
    void buildKDTree();

private:


};

#endif
