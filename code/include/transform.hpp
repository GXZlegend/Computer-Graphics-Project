#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <vecmath.h>
#include "object3d.hpp"

// transforms a 3D point using a matrix, returning a 3D point
static Vector3f transformPoint(const Matrix4f &mat, const Vector3f &point) {
    return (mat * Vector4f(point, 1)).xyz();
}

// transform a 3D directino using a matrix, returning a direction
static Vector3f transformDirection(const Matrix4f &mat, const Vector3f &dir) {
    return (mat * Vector4f(dir, 0)).xyz();
}

static Box transformBox(const Matrix4f &mat, const Box &rawBox) {
    Box newBox(rawBox);
    newBox.v[0] = Vector3f(1e38);
    newBox.v[1] = Vector3f(1e-38);
    for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 2; ++y) {
            for (int z = 0; z < 2; ++z) {
                Vector3f v = transformPoint(mat, Vector3f(rawBox.v[x][0], 
                                                          rawBox.v[y][1],
                                                          rawBox.v[z][2]));
                for (int dim = 0; dim < 3; ++dim) {
                    newBox.v[0][dim] = std::min(newBox.v[0][dim], v[dim]);
                    newBox.v[1][dim] = std::max(newBox.v[1][dim], v[dim]);
                }
            }
        }
    }
    return newBox;
}

class Transform : public Object3D {
public:
    Transform() {}

    Transform(const Matrix4f &m, Object3D *obj) : o(obj) {
        transform = m;
        transformInverse = m.inverse();
        box = transformBox(m, obj->box);
    }

    ~Transform() {
    }

    virtual bool intersect(const Ray &r, Hit &h, float tmin) {
        Vector3f trSource = transformPoint(transformInverse, r.getOrigin());
        Vector3f trDirection = transformDirection(transformInverse, r.getDirection());
        Ray tr(trSource, trDirection);
        bool inter = o->intersect(tr, h, tmin);
        if (inter) {
            h.set(h.getT(), h.getMaterial(), transformDirection(transformInverse.transposed(), h.getNormal()).normalized());
        }
        return inter;
    }

    virtual void getBoxes(std::vector<Box> &data) override {
        std::vector<Box> rawData;
        o->getBoxes(rawData);
        for (int boxId = 0; boxId < (int) rawData.size(); ++boxId)
            data.push_back(transformBox(transform, rawData[boxId]));
    }

protected:
    Object3D *o; //un-transformed object
    Matrix4f transform;
    Matrix4f transformInverse;
};

#endif //TRANSFORM_H
