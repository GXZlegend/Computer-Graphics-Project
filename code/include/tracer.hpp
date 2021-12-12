#ifndef TRACER_H
#define TRACER_H

#include "hit.hpp"
#include "ray.hpp"
#include "material.hpp"
#include "object3d.hpp"

void traceRay(Object3D *o, const Ray &r, int depth) {
    Hit h;
    bool flag = o->intersect(r, h, 0);
    if (flag) {
        
    }
}

#endif // TRACER_H
