#ifndef OBJECT3D_H
#define OBJECT3D_H

#include <vector>
#include <vecmath.h>
#include "ray.hpp"
#include "hit.hpp"
#include "material.hpp"

class Object3D;

struct Box {
    Vector3f v[2];
    Object3D *obj;
    bool intersect(const Ray &r, float tmin) {
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
};

// Base class for all 3d entities.
class Object3D {
public:
    Object3D() : material(nullptr) {}

    virtual ~Object3D() = default;

    explicit Object3D(Material *material) {
        this->material = material;
        box.v[0] = Vector3f(1e38);
        box.v[1] = Vector3f(-1e38);
        box.obj = this;
    }

    // Intersect Ray with this object. If hit, store information in hit structure.
    virtual bool intersect(const Ray &r, Hit &h, float tmin) = 0;

    virtual void getBoxes(std::vector<Box> &data) {
        data.push_back(box);
    }

    Box box;
    Material *material;
    
protected:

};

#endif // OBJECT3D_H
