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

    void addObject(int index, Object3D *obj) {
        o[index] = obj;
    }

    int getGroupSize() {
        return (int) o.size();
    }

private:
    std::vector<Object3D*> o;
};

#endif
