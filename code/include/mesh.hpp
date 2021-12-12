#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"


class Mesh : public Object3D {

public:
    Mesh(const char *filename, Material *m);

    std::vector<Vector3f> v;
    std::vector<Triangle*> t;

    bool intersect(const Ray &r, Hit &h, float tmin) override;
    void getBoxes(std::vector<Box> &data) override;

private:
    void computeBox();

};

#endif
