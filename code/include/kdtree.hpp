#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <algorithm>
#include <vecmath.h>
#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"


class KDTree: public Object3D {

public:

    KDTree() {
        left = nullptr;
        right = nullptr;
        box.obj = this;
    }

    ~KDTree() override {
        delete left;
        delete right;
    }

    void buildKDTree(std::vector<Box> &rootBox, int depth, int splitDim) {
        box.v[0] = Vector3f(1e38);
        box.v[1] = Vector3f(-1e38);
        for (int boxId = 0; boxId < (int) rootBox.size(); ++boxId) {
            for (int dim = 0; dim < 3; ++dim) {
                box.v[0][dim] = std::min(box.v[0][dim], rootBox[boxId].v[0][dim]);
                box.v[1][dim] = std::max(box.v[1][dim], rootBox[boxId].v[1][dim]);
            }
        }
        dim = splitDim;
        if (!depth)
            leafbox = rootBox;
        else {
            bool flag;
            float split = findSplitPlane(rootBox, box.v[0], box.v[1], splitDim, flag);
            if (!flag)
                leafbox = rootBox;
            else {
                std::vector<Box> leftBox, rightBox;
                for (int boxId = 0; boxId < (int) rootBox.size(); ++boxId) {
                    if (rootBox[boxId].v[1][splitDim] <= split)
                        leftBox.push_back(rootBox[boxId]);
                    else if (rootBox[boxId].v[0][splitDim] >= split)
                        rightBox.push_back(rootBox[boxId]);
                    else {
                        leftBox.push_back(rootBox[boxId]);
                        rightBox.push_back(rootBox[boxId]);
                        leftBox.back().v[1][splitDim] = split;
                        rightBox.back().v[0][splitDim] = split;
                    }
                }
                left = new KDTree;
                right = new KDTree;
                left->buildKDTree(leftBox, depth - 1, (splitDim + 1) % 3);
                right->buildKDTree(rightBox, depth - 1, (splitDim + 1) % 3);
            }
        }
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        if (!box.intersect(r, tmin))
            return false;
        if (!leafbox.empty()) {
            bool flag = 0;
            for (int boxId = 0; boxId < (int) leafbox.size(); ++boxId)
                flag |= leafbox[boxId].intersect(r, tmin) && leafbox[boxId].obj->intersect(r, h, tmin);
            return flag;
        }
        else if (r.getDirection()[dim] > 0)
            return left->intersect(r, h, tmin) || right->intersect(r, h, tmin);
        else
            return right->intersect(r, h, tmin) || left->intersect(r, h, tmin);
    }

    void getBoxes(std::vector<Box> &data) override {
        if (!leafbox.empty())
            data.insert(data.end(), leafbox.begin(), leafbox.end());
        else {
            left->getBoxes(data);
            right->getBoxes(data);
        }
    }

private:
    struct KDTreePlane {
        KDTreePlane(Box b, int _dim, int _flag) {
            v[0] = b.v[0];
            v[1] = b.v[1];
            dim = _dim;
            flag = _flag;
        }
        bool operator<(const KDTreePlane &a) const {
            if (v[flag][dim] == a.v[a.flag][a.dim])
                return flag > a.flag;
            return v[flag][dim] < a.v[a.flag][a.dim];
        }
        Vector3f v[2];
        int dim, flag;
    };

    static float getArea(Vector3f v0, Vector3f v1) {
        Vector3f b = v1 - v0;
        return b[0] * b[1] + b[0] * b[2] + b[1] * b[2];
    }

    static float splitArea(Vector3f v0, Vector3f v1, int splitDim, float splitPlane, bool left) {
        if (left && v1[splitDim] > splitPlane)
            v1[splitDim] = splitPlane;
        if (!left && v0[splitDim] < splitPlane)
            v0[splitDim] = splitPlane;
        return getArea(v0, v1);
    }

    static float findSplitPlane(std::vector<Box> &box, Vector3f v0, Vector3f v1, int splitDim, bool &flag) {
        std::vector<KDTreePlane> plane;
        for (int boxId = 0; boxId < (int) box.size(); ++boxId) {
            plane.emplace_back(box[boxId], splitDim, 0);
            plane.emplace_back(box[boxId], splitDim, 1);
        }
        std::sort(plane.begin(), plane.end());

        std::vector<float> areaLeft, areaRight;
        std::vector<int> numLeft, numRight;
        int nPlane = (int) plane.size(), nLeft = 1, nRight = 1;
        Vector3f vLeft[2], vRight[2];
        vLeft[0] = plane[0].v[0];
        vLeft[1] = plane[0].v[1];
        vRight[0] = plane[nPlane - 1].v[0];
        vRight[1] = plane[nPlane - 1].v[1];

        for (int planeId = 1; planeId < nPlane - 1; ++planeId) {
            float splitPlane = plane[planeId].v[plane[planeId].flag][splitDim];
            numLeft.push_back(nLeft);
            if (plane[planeId].flag == 0) {
                areaLeft.push_back(splitArea(vLeft[0], vLeft[1], splitDim, splitPlane, true));
                ++nLeft;
            }
            for (int dim = 0; dim < 3; dim ++ ) {
                vLeft[0][dim] = std::min(vLeft[0][dim], plane[planeId].v[0][dim]);
                vLeft[1][dim] = std::max(vLeft[1][dim], plane[planeId].v[1][dim]);
            }
            if (plane[planeId].flag == 1)
                areaLeft.push_back(splitArea(vLeft[0], vLeft[1], splitDim, splitPlane, true));
        }

        for (int planeId = nPlane - 2; planeId; --planeId) {
            float splitPlane = plane[planeId].v[plane[planeId].flag][splitDim];
            numRight.push_back(nRight);
            if (plane[planeId].flag == 1) {
                areaRight.push_back(splitArea(vRight[0], vRight[1], splitDim, splitPlane, false));
                ++nRight;
            }
            for (int dim = 0; dim < 3; dim ++ ) {
                vRight[0][dim] = std::min(vRight[0][dim], plane[planeId].v[0][dim]);
                vRight[1][dim] = std::max(vRight[1][dim], plane[planeId].v[1][dim]);
            }
            if (plane[planeId].flag == 0)
                areaRight.push_back(splitArea(vRight[0], vRight[1], splitDim, splitPlane, false));
        }

        float minCost = 80 * (int) box.size();
        float sumArea = getArea(v0, v1);
        int splitPlane = 0;
        for (int planeId = 1; planeId < nPlane - 1; ++planeId) {
            float cost = 80 * (areaLeft[planeId - 1] * numLeft[planeId - 1]
                            + areaRight[nPlane - 2 - planeId] * numRight[nPlane - 2 - planeId]) / sumArea + 1;
            if (cost < minCost) {
                minCost = cost;
                splitPlane = planeId;
            }
        }

        if (splitPlane) {
            flag = true;
            return plane[splitPlane].v[plane[splitPlane].flag][splitDim];
        }
        else {
            flag = false;
            return 0;
        }
    }

    KDTree *left, *right;
    std::vector<Box> leafbox;
    int dim;
};

#endif // KDTREE_H
