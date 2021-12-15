#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>


class Group : public Object3D {

public:

    Group() {

    }

    explicit Group (int num_objects) {
        for (int objId = 0; objId < num_objects; ++objId)
            o.push_back(nullptr);
        box.v[0] = Vector3f(1e38);
        box.v[1] = Vector3f(-1e38);
        box.obj = this;
    }

    ~Group() override {
        for (int objId = 0; objId < (int) o.size(); ++objId)
            delete o[objId];
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        bool flag = false;
        for (int objId = 0; objId < (int) o.size(); ++objId)
            flag |= o[objId]->intersect(r, h, tmin);
        return flag;
    }

    void getBoxes(std::vector<Box> &data) override {
        for (int objId = 0; objId < (int) o.size(); ++objId)
            o[objId]->getBoxes(data);
    }

    void addObject(int index, Object3D *obj) {
        o[index] = obj;
        for (int dim = 0; dim < 3; ++dim) {
            box.v[0][dim] = std::min(box.v[0][dim], obj->box.v[0][dim]);
            box.v[1][dim] = std::max(box.v[1][dim], obj->box.v[1][dim]);
        }
    }

    int getGroupSize() {
        return (int) o.size();
    }

private:
    std::vector<Object3D*> o;
};

#endif
