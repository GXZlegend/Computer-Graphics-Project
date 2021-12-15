#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>


class Sphere : public Object3D {
public:
    Sphere() {
        center = Vector3f::ZERO;
        radius = 1;
        box.v[0] = Vector3f(-1);
        box.v[1] = Vector3f(1);
    }

    Sphere(const Vector3f &c, float r, Material *m) : Object3D(m) {
        center = c;
        radius = r;
        box.v[0] = c - r;
        box.v[1] = c + r;
    }

    ~Sphere() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        Vector3f l = center - r.getOrigin(), rd = r.getDirection().normalized();
        float length = r.getDirection().length();
        float tp = Vector3f::dot(l, rd);
        float d2 = l.squaredLength() - tp * tp;
        if (d2 > radius * radius)
            return false;
        float t2 = sqrt(radius * radius - d2), t;
        if ((tp - t2) / length > tmin)
            t = (tp - t2) / length;
        else if ((tp + t2) / length > tmin)
            t = (tp + t2) / length;
        else
            return false;
        if (t < h.getT()) {
            h.set(t, material, (r.pointAtParameter(t) - center).normalized());
            return true;
        }
        return false;
    }

    Vector3f center;
    float radius;

protected:

};


#endif //SPHERE_H
