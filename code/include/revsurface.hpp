#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include "object3d.hpp"
#include "curve.hpp"
#include <tuple>

const int N_SAMPLE = 20;
const int N_ITER = 20;

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
    }

    ~RevSurface() override {
        delete pCurve;
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        bool flag = false;
        std::pair<float, float> range = pCurve->getRange();
        for (int sampleId = 0; sampleId <= N_SAMPLE; ++sampleId) {
            float t_curve = (range.second - range.first) * sampleId / N_SAMPLE + range.first;
            int iter = N_ITER;
            bool clamp_first = false, clamp_second = false;
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
        }
        return flag;
    }
};

#endif //REVSURFACE_HPP
