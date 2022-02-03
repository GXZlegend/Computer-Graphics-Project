# 大作业报告

> 郭星灼  智90  2019012383

## 图像结果

![result](/result.bmp)

## 运行方式

```plain
cd code
bash run_all.sh
```

## 实现简述

在本次大作业中，我实现了**渐进式光子投射（PPM）**，支持反射、折射、三角网格等，包括但不限于小作业出现的全部结构，**全部**代码在小作业的基础上由本人**独立**完成。

所实现的附加功能包括：

- 复杂三角网格的KDTree包围盒加速；
- OpenMP多线程加速；
- 景深效果（景深对准了霜之哀伤，花瓶和球较为模糊）；
- 抗锯齿；
- 软阴影（PPM自带效果）；
- 参数曲线旋转体解析求交；
- 复杂网格模型（46000+面片）；

## 代码细节

### 渐进式光子投射

包括两个pass，分别为ray tracing和photon tracing，方便起见，代码不区分两种tracing的实现，而是用struct Trace存储结果，然后进行后处理。

```c++
struct Trace {
    Photon photon;
    Vector3f normal;
    Material *material;
};
```

tracing的过程与光线追踪并没有什么差异，在镜面上进行反射折射，在漫反射表面上sample光线方向（仅发生在photon tracing）或存储光子。反射折射比例由无偏光折射公式计算得到，并设置最大深度为5。

对每一个视点开一个struct ViewPoint存储半径、能量等信息：

```c++
struct viewPoint {
    Vector3f radiance() {
        return power / (acos(-1.0) * radius * radius * num);
    }
    Trace trace;
    Vector3f power;
    float num;
    float alpha;
    float radius;
};
```

其中alpha取0.7，radius初值为0.3米。

基于这一思路实现两个pass，过程中使用OpenMP加速：

```c++
void ppmBackward(Object3D *o, Camera *camera, int spp, std::vector<std::vector<viewPoint>> &imgView) {
    for (int x = 0; x < camera->getWidth(); ++x) {
        std::cout << "Line " << x << std::endl;
        #pragma omp parallel for schedule(dynamic, 128), num_threads(8)
        for (int y = 0; y < camera->getHeight(); ++y) {
            std::vector<Trace> trace;
            std::vector<viewPoint> view;
            for (int sppId = 0; sppId < spp; ++sppId) {
                Ray r = camera->generateRay(Vector2f(x, y));
                traceRay(o, r, Vector3f(1.0 / spp), 5, trace, false);
            }
            for (Trace &t: trace) {
                viewPoint point;
                point.trace = t;
                point.power = Vector3f::ZERO;
                point.num = 0;
                point.alpha = ALPHA;
                point.radius = RADIUS;
                view.push_back(point);
            }
            imgView.push_back(view);
        }
    }
}

void ppmForward(Object3D *o, std::vector<Light*> lights, int rayNum, std::vector<std::vector<viewPoint>> &imgView) {
    std::vector<Photon> photons;
    #pragma omp parallel for schedule(dynamic, 60), num_threads(8)
    for (Light *&l: lights) {
        for (int rayId = 0; rayId < rayNum; ++rayId) {
            std::pair<Ray, Vector3f> generation = l->generate();
            Ray r = generation.first;
            Vector3f col = generation.second;
            Photon origin;
            origin.pos = r.getOrigin();
            origin.dir = -r.getDirection();
            origin.power = col * 10;
            photons.push_back(origin);
            std::vector<Trace> trace;
            traceRay(o, r, col, 5, trace, true);
            for (Trace &t: trace) {
                photons.push_back(t.photon);
            }
        }
    }
    std::cout << photons.size() << " photons in total." << std::endl;
    PhotonKDTree *root = new PhotonKDTree;
    root->build(photons.begin(), photons.end(), 0);    
    #pragma omp parallel for schedule(dynamic, 128), num_threads(8)
    for (int viewId = 0; viewId < (int) imgView.size(); ++viewId) {
        if (viewId % (imgView.size() / 100) == 0) {
            std::cout << "View " << viewId << std::endl;
        }
        for (viewPoint &point: imgView[viewId]) {
            std::vector<Photon> photon;
            root->collect(point.trace.photon.pos, point.radius, photon);
            int m = (int) photon.size();
            Vector3f power = Vector3f::ZERO;
            for (Photon &p: photon) {
                power += point.trace.photon.power
                       * point.trace.material->Shade(point.trace.photon.dir, p.dir, point.trace.normal, p.power);
            }
            float n_prime = point.num + point.alpha * m;
            float r_prime = point.radius;
            Vector3f power_prime = point.power + power;
            if (point.num + m > 0) {
                r_prime *= sqrt(n_prime / (point.num + m));
                power_prime *= n_prime / (point.num + m);
            }
            point.num = n_prime;
            point.radius = r_prime;
            point.power = power_prime;
        }
    }
    delete root;
}

Vector3f getRadiance(std::vector<viewPoint> view) {
    Vector3f radiance = Vector3f::ZERO;
    for (viewPoint &point: view) {
        radiance += point.radiance();
    }
    return radiance;
}
```

实验中每个pass生成200000个光子，总共渲染2500个pass。

### KDTree包围盒加速

对mesh建立KDTree包围盒，从而进行求交加速。每一个KDTree节点包含：mesh指针、左右子节点指针、包围盒大小位置，和节点对应的三角面片的编号（仅叶子节点）。

```c++
class KDTree {
public:

    KDTree(Mesh *m) {
        mesh = m;
        left = nullptr;
        right = nullptr;
    }

    ~KDTree() {
        delete left;
        delete right;
    }

    void build(std::vector<int> &triId, int depth, int d);
    bool intersect(const Ray &r, Hit &h, float tmin);
    void getBox(std::vector<int> &triId);

    Mesh *mesh;
    KDTree *left, *right;
    Vector3f box[2];
    int dim;
    std::vector<int> leafTriId;

private:

    bool intersectBox(const Ray &r, float tmin);
    bool innerBox(Vector3f &p);
};
```

建树的过程：每次根据面片重心的中位数位置进行划分，跨过划分平面的面片会同时分给两个子节点，当深度较大或是某个子节点包含当前节点的全部面片时停止。

```c++
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
```

查询过程较为朴素，首先判断是否与包围盒相交，然后根据光线方向，对两个子节点进行 “短路” 处理。

### OpenMP多线程加速

参见PPM部分的代码实现。

### 景深效果

仅需实现透镜相机即可，需要定义光圈半径（米）和景深深度（米）。

```c++
class LensCamera : public PerspectiveCamera {

public:
    LensCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, float angle,
            float r, float d) : PerspectiveCamera(center, direction, up, imgW, imgH, angle) {
        radius = r;
        depth = d;
    }

    virtual Ray generateRay(const Vector2f &point) override {
        Vector2f newPoint(point[0] + 1.0 * rand() / RAND_MAX - 0.5,
                          point[1] + 1.0 * rand() / RAND_MAX - 0.5);
        Ray r = PerspectiveCamera::generateRay(newPoint);
        Vector3f objPoint = r.pointAtParameter(depth / Vector3f::dot(r.getDirection(), direction));
        Vector3f newCenter = randomCenter();
        return Ray(newCenter, (objPoint - newCenter).normalized());
    }

protected:
    float radius;
    float depth;

private:
    Vector2f randomPoint() {
        float x, y;
        do {
            x = 2.0 * rand() / RAND_MAX - 1;
            y = 2.0 * rand() / RAND_MAX - 1;
        } while(x * x + y * y > 1);
        return Vector2f(x, y);
    }

    Vector3f randomCenter() {
        Vector3f dx = Vector3f::cross(direction, up).normalized();
        Vector3f dy = Vector3f::cross(direction, dx).normalized();
        Vector2f point = radius * randomPoint();
        return center + point[0] * dx + point[1] + dy;
    }
};
```

### 抗锯齿

只需要对每个像素sample多条光线，每一条光线加以微扰，参见相机实现。实验中spp取8。

### 软阴影

PPM自带的feature。

### 参数曲线旋转体解析法求交

根据习题课上所讲的公式，使用牛顿迭代解方程，解方程过程略。

此时由于初值影响很大，因此对旋转体先建立一个（略大一点的）mesh进行估计，根据光线与mesh相交的位置确定迭代初值。

```c++
RevSurface(Curve *pCurve, Material* material) : pCurve(pCurve), Object3D(material) {
    // Check flat.
    for (const auto &cp : pCurve->getControls()) {
        if (cp.z() != 0.0) {
            printf("Profile of revSurface must be flat on xy plane.\n");
            exit(0);
        }
    }
    mesh = new Mesh(material);
    
    pCurve->discretize(20, curvePoints);
    const int steps = 30;
    for (int ci = 0; ci < (int) curvePoints.size(); ++ci) {
        const CurvePoint &cp = curvePoints[ci];
        for (int i = 0; i < steps; ++i) {
            float t = (float) i / steps;
            Quat4f rot;
            rot.setAxisAngle(t * 2 * 3.14159, Vector3f::UP);
            Vector3f pnew = (Matrix3f::rotation(rot) * cp.V) / cos(t * 3.14159);
            mesh->v.push_back(pnew);
            int i1 = (i + 1 == steps) ? 0 : i + 1;
            if (ci != curvePoints.size() - 1) {
                mesh->t.emplace_back((ci + 1) * steps + i, ci * steps + i1, ci * steps + i);
                mesh->t.emplace_back((ci + 1) * steps + i, (ci + 1) * steps + i1, ci * steps + i1);
            }
        }
    }
    mesh->buildKDTree();
}
```

### 复杂网格模型

图片导入了霜之哀伤的obj模型，共46000+三角面片，并实现了KDTree进行求交优化，参见KDTree部分。
