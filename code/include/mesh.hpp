#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector3f.h"
#include "kdtree.hpp"

class KDTree;

class Mesh : public Object3D {

public:
    Mesh(Material *m) : Object3D(m) {

    }
    
    Mesh(const char *filename, Material *m);

    struct TriangleIndex {
        TriangleIndex() {
            x[0] = 0; x[1] = 0; x[2] = 0;
        }
        TriangleIndex(int a, int b, int c) {
            x[0] = a; x[1] = b; x[2] = c;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3]{};
    };
    
    
    std::vector<Vector3f> v;
    std::vector<TriangleIndex> t;
    KDTree *root;
    bool intersect(const Ray &r, Hit &h, float tmin) override;
    void buildKDTree();

};

#endif
