#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    Plane() {

    }

    Plane(const Vector3f &_n, float _d, Material *m) : Object3D(m) {
        normal = _n;
        d = _d;
    }

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        float c = Vector3f::dot(normal, r.getDirection());
        if (c == 0)
            return false;
        float t = (d - Vector3f::dot(normal, r.getOrigin())) / c;
        if (t > tmin && t < h.getT()) {
            h.set(t, material, normal);
            return true;
        }
        return false;
    }
    
    Vector3f normal;
    float d;

protected:

};

#endif //PLANE_H
		

