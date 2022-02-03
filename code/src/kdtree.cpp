#include "kdtree.hpp"
#include <algorithm>

void KDTree::build(std::vector<int> &triId, int depth, int d) {
    dim = d;
    if (!depth) {
        leafTriId = triId;
        return;
    }
    std::vector<float> center;
    for (int &i: triId) {
        center.push_back((mesh->v[mesh->t[i][0]][d]
                        + mesh->v[mesh->t[i][1]][d]
                        + mesh->v[mesh->t[i][2]][d]) / 3);
    }
    std::nth_element(center.begin(), center.begin() + center.size() / 2, center.end());
    float split = *(center.begin() + center.size() / 2);
    std::vector<int> leftId, rightId;
    if (split < box[0][d] || split > box[1][d]) {
        leafTriId = triId;
        return;
    }
    for (int &i: triId) {
        if (mesh->v[mesh->t[i][0]][d] < split
         && mesh->v[mesh->t[i][1]][d] < split
         && mesh->v[mesh->t[i][2]][d] < split) {
            leftId.push_back(i);
        }
        else if (mesh->v[mesh->t[i][0]][d] > split
              && mesh->v[mesh->t[i][1]][d] > split
              && mesh->v[mesh->t[i][2]][d] > split) {
            rightId.push_back(i);
        }
        else {
            leftId.push_back(i);
            rightId.push_back(i);
        }
    }
    
    if (leftId.size() == triId.size() || rightId.size() == triId.size()) {
        leafTriId = triId;
        return;
    }
    if (leftId.size() > 0) {
        left = new KDTree(mesh);
        left->box[0] = box[0];
        left->box[1] = box[1];
        left->box[1][d] = split;
        left->build(leftId, depth - 1, (d + 1) % 3);
    }
    if (rightId.size() > 0) {
        right = new KDTree(mesh);
        right->box[0] = box[0];
        right->box[1] = box[1];
        right->box[0][d] = split;
        right->build(rightId, depth - 1, (d + 1) % 3);
    }
}

bool KDTree::intersect(const Ray &r, Hit &h, float tmin) {
    if (!intersectBox(r, tmin)) {
        return false;
    }
    if (!leafTriId.empty()) {
        bool flag = false;
        for (int &i: leafTriId) {
            Triangle triangle(mesh->v[mesh->t[i][0]], 
                              mesh->v[mesh->t[i][1]], 
                              mesh->v[mesh->t[i][2]], mesh->material);
            flag |= triangle.intersect(r, h, tmin);
        }
        return flag;
    }
    else if (r.getDirection()[dim] > 0) {
        bool flag = left->intersect(r, h, tmin);
        Vector3f p = r.pointAtParameter(h.getT());
        if (flag && left->innerBox(p)) {
            return true;
        }
        return right->intersect(r, h, tmin) || flag;
    }
    else {
        bool flag = right->intersect(r, h, tmin);
        Vector3f p = r.pointAtParameter(h.getT());
        if (flag && right->innerBox(p)) {
            return true;
        }
        return left->intersect(r, h, tmin) || flag;
    }
}

void KDTree::getBox(std::vector<int> &triId) {
    box[0] = Vector3f(1e38);
    box[1] = Vector3f(1e-38);
    for (int &i: triId) {
        for (int d = 0; d < 3; ++d) {
            box[0][d] = std::min(box[0][d], mesh->v[mesh->t[i][0]][d]);
            box[0][d] = std::min(box[0][d], mesh->v[mesh->t[i][1]][d]);
            box[0][d] = std::min(box[0][d], mesh->v[mesh->t[i][2]][d]);
            box[1][d] = std::max(box[1][d], mesh->v[mesh->t[i][0]][d]);
            box[1][d] = std::max(box[1][d], mesh->v[mesh->t[i][1]][d]);
            box[1][d] = std::max(box[1][d], mesh->v[mesh->t[i][2]][d]);
        }
    }
}

bool KDTree::intersectBox(const Ray &r, float tmin) {
    float t1 = -1e38, t2 = 1e38;
    for (int d = 0; d < 3; ++d) {
        if (r.getDirection()[d] > 0) {
            t1 = std::max(t1, (box[0][d] - r.getOrigin()[d]) / r.getDirection()[d]);
            t2 = std::min(t2, (box[1][d] - r.getOrigin()[d]) / r.getDirection()[d]);
        }
        else if (r.getDirection()[d] < 0) {
            t1 = std::max(t1, (box[1][d] - r.getOrigin()[d]) / r.getDirection()[d]);
            t2 = std::min(t2, (box[0][d] - r.getOrigin()[d]) / r.getDirection()[d]);
        }
        else if (box[0][d] > r.getOrigin()[d] || box[1][d] < r.getOrigin()[d])
            return false;
    }
    return (t1 <= t2 && t2 > tmin);
}

bool KDTree::innerBox(Vector3f &p) {
    for (int d = 0; d < 3; ++d) {
        if (p[d] < box[0][d] || p[d] > box[1][d]) {
            return false;
        }
    }
    return true;
}
