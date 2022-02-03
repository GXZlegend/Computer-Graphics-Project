#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include "object3d.hpp"
#include "curve.hpp"
#include <cmath>
#include <tuple>

const int N_ITER = 10;

class RevSurface : public Object3D {

    Curve *pCurve;

public:
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
        std::cout << mesh->t.size() << std::endl;
        mesh->buildKDTree();
    }

    ~RevSurface() override {
        delete pCurve;
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        Hit temp(h);
        bool its = mesh->intersect(r, temp, tmin);
        if (!its) return false;
        Vector3f p = r.pointAtParameter(temp.getT());
        float radius = sqrt(p[0] * p[0] + p[2] * p[2]);

        float t_curve;
        float min_dis = 1e38;
        std::pair<float, float> range = pCurve->getRange();

        for (int ci = 0; ci < (int) curvePoints.size(); ++ci) {
            const CurvePoint &cp = curvePoints[ci];
            float dis = (cp.V[0] - radius) * (cp.V[0] - radius) + cp.V[1] * cp.V[1];
            if (dis < min_dis) {
                min_dis = dis;
                t_curve = (range.second - range.first) / (curvePoints.size() - 1) * ci;
            }
        }

        int iter = N_ITER;
        bool clamp_first = false, clamp_second = false;
        bool flag = false;

        while (iter--) {
            CurvePoint point = pCurve->getCurvePoint(t_curve);
            float t_ray, f, df;
            bool end_iter = false, t_flag = false;
            if (std::abs(r.getDirection()[1]) < 1e-7) {
                if (std::abs(point.V[1] - r.getOrigin()[1]) < 1e-2) {
                    float term_1 = r.getDirection()[0] * r.getDirection()[0] + r.getDirection()[2] * r.getDirection()[2];
                    float term_2 = r.getDirection()[0] * r.getOrigin()[0] + r.getDirection()[2] * r.getOrigin()[2];
                    float term_3 = r.getOrigin()[0] * r.getOrigin()[0] + r.getOrigin()[2] * r.getOrigin()[2] - point.V[0] * point.V[0];
                    if (term_2 * term_2 >= term_1 * term_3) {
                        float delta = std::sqrt(term_2 * term_2 - term_1 * term_3);
                        if (tmin < (-term_2 - delta) / term_1 && (-term_2 - delta) / term_1 < h.getT()) {
                            t_ray = (-term_2 - delta) / term_1;
                            t_flag = true;
                        }
                        else if (tmin < (-term_2 + delta) / term_1 && (-term_2 + delta) / term_1 < h.getT()) {
                            t_ray = (-term_2 + delta) / term_1;
                            t_flag = true;
                        }
                    }
                    end_iter = true;
                }
                f = point.V[1] - r.getOrigin()[1];
                df = point.T[1];
            }
            else {
                if (std::abs((r.pointAtParameter((point.V[1] - r.getOrigin()[1]) / r.getDirection()[1])
                            - Vector3f(0, point.V[1], 0)).length() - std::abs(point.V[0])) < 1e-2) {
                    if (tmin < (point.V[1] - r.getOrigin()[1]) / r.getDirection()[1] && (point.V[1] - r.getOrigin()[1]) / r.getDirection()[1] < h.getT()) {
                        t_ray = (point.V[1] - r.getOrigin()[1]) / r.getDirection()[1];
                        t_flag = true;
                    }
                    end_iter = true;
                }
                float term_1 = r.getDirection()[1] * point.V[0];
                float term_2 = r.getDirection()[1] * r.getOrigin()[0] + r.getDirection()[0] * (point.V[1] - r.getOrigin()[1]);
                float term_3 = r.getDirection()[1] * r.getOrigin()[2] + r.getDirection()[2] * (point.V[1] - r.getOrigin()[1]);
                float term_4 = r.getDirection()[1] * point.T[0];
                float term_5 = r.getDirection()[0] * point.T[1];
                float term_6 = r.getDirection()[2] * point.T[1];
                f = term_1 * term_1 - term_2 * term_2 - term_3 * term_3;
                df = 2 * (term_1 * term_4 - term_2 * term_5 - term_3 * term_6);
            }
            if (end_iter) {
                if (t_flag) {
                    flag = true;
                    Vector3f dt = r.pointAtParameter(t_ray);
                    Vector3f ds(-dt[2], 0, dt[0]);
                    dt[0] *= point.T[0] / point.V[0];
                    dt[1] = point.T[1];
                    dt[2] *= point.T[0] / point.V[0];
                    Vector3f normal = -Vector3f::cross(dt, ds).normalized();
                    h.set(t_ray, material, normal);
                }
                break;
            }
            if ((t_curve == range.first && clamp_first && f / df < 0)
             || (t_curve == range.second && clamp_second) && f / df > 0)
                break;
            t_curve -= f / df;
            if (t_curve <= range.first) {
                t_curve = range.first;
                clamp_first = true;
            }
            if (t_curve >= range.second) {
                t_curve = range.second;
                clamp_second = true;
            }
        }
        return flag;
    }

    std::vector<CurvePoint> curvePoints;
    Mesh *mesh;
};

#endif //REVSURFACE_HPP
