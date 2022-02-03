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

Vector3f randomDiffuse(const Vector3f &normal) {
    Vector3f dir;
    do {
        dir[0] = 2.0 * rand() / RAND_MAX - 1;
        dir[1] = 2.0 * rand() / RAND_MAX - 1;
        dir[2] = 2.0 * rand() / RAND_MAX - 1;
    } while (dir.squaredLength() > 1 || Vector3f::dot(dir, normal) <= 0);
    return dir.normalized();
}

void traceRay(Object3D *o, const Ray &r, const Vector3f &power, int depth, std::vector<Trace> &data, bool sampleDiffuse) {
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
                if (sampleDiffuse && rand() < RAND_MAX * 0.2) {
                    // Diffuse
                    Vector3f dir = randomDiffuse(h.getNormal());
                    Ray diffuseRay(Ori, dir);
                    traceRay(o, diffuseRay, diffusePower, depth - 1, data, sampleDiffuse);
                }
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
                        traceRay(o, Ray(Ori, reflectionDir), specularPower, depth - 1, data, sampleDiffuse);
                    }
                }
                float cosR = sqrt(1 - sinR * sinR);
                cosI = std::abs(cosI);
                Vector3f refractionDir = (r.getDirection() - proj) * sinR / sinI + proj * cosR / cosI;
                float sqrtRs = (cosI * sinR - sinI * cosR) / (cosI * sinR + sinI * cosR);
                float sqrtRp = (cosI * cosR - sinI * sinR) / (cosI * cosR + sinI * sinR);
                float R = (sqrtRs * sqrtRs + sqrtRp * sqrtRp) / 2, T = 1 - R;
                traceRay(o, Ray(Ori, reflectionDir), specularPower * R, depth - 1, data, sampleDiffuse);
                traceRay(o, Ray(Ori, refractionDir), specularPower * T, depth - 1, data, sampleDiffuse);
            }
        }
    }
}

#endif // TRACER_H
