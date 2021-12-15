#ifndef TRACER_H
#define TRACER_H

#include <cmath>
#include <vector>
#include <vecmath.h>
#include "hit.hpp"
#include "ray.hpp"
#include "material.hpp"
#include "object3d.hpp"
#include "photon.hpp"

float minTime = 1e-2;
float minPower = 1e-5;

struct Trace {
    Photon photon;
    Vector3f normal;
    Material *material;
};

void traceRay(Object3D *o, const Ray &r, Vector3f power, int depth, std::vector<Trace> &data) {
    if (depth > 0) {
        Hit h;
        bool flag = o->intersect(r, h, minTime);
        if (flag) {
            Vector3f Ori = r.pointAtParameter(h.getT());
            Vector3f specularPower = power * h.getMaterial()->specularRatio;
            Vector3f diffusePower = power - specularPower;
            if (diffusePower.length() > minPower) {
                Trace t;
                t.photon.pos = Ori;
                t.photon.dir = r.getDirection();
                t.photon.power = diffusePower;
                t.normal = h.getNormal();
                t.material = h.getMaterial();
                data.push_back(t);
            }
            if (specularPower.length() > minPower) {
                float cosI = Vector3f::dot(r.getDirection(), h.getNormal());
                float sinI = sqrt(1 - cosI * cosI);
                float sinR;
                Vector3f proj = cosI * h.getNormal();
                Vector3f reflectionDir = r.getDirection() - 2 * proj;
                if (cosI < 0) {
                    // In-going ray
                    sinR = sinI / h.getMaterial()->refraction;
                }
                else {
                    // Out-going ray
                    sinR = sinI * h.getMaterial()->refraction;
                    if (sinR > 1) {
                        // Total reflection
                        traceRay(o, Ray(Ori, reflectionDir), specularPower, depth - 1, data);
                    }
                }
                float cosR = sqrt(1 - sinR * sinR);
                cosI = std::abs(cosI);
                Vector3f refractionDir = (r.getDirection() - proj) * sinR / sinI + proj * cosR / cosI;
                float sqrtRs = (cosI * sinR - sinI * cosR) / (cosI * sinR + sinI * cosR);
                float sqrtRp = (cosI * cosR - sinI * sinR) / (cosI * cosR + sinI * sinR);
                float R = (sqrtRs * sqrtRs + sqrtRp * sqrtRp) / 2, T = 1 - R;
                if (R < 0 || T < 0) {
                    std::cout << "fuck" << std::endl;
                    std::cout << cosI << std::endl;
                    std::cout << sinI << std::endl;
                    std::cout << cosR << std::endl;
                    std::cout << sinR << std::endl;
                    std::cout << sqrtRs << std::endl;
                    std::cout << sqrtRp << std::endl;
                }
                traceRay(o, Ray(Ori, reflectionDir), specularPower * R, depth - 1, data);
                traceRay(o, Ray(Ori, refractionDir), specularPower * T, depth - 1, data);
            }
        }
    }
}

#endif // TRACER_H
