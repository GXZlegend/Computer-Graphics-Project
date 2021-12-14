#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <cstdlib>
#include <vecmath.h>
#include <float.h>
#include <cmath>


class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up).normalized();
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};


class PerspectiveCamera : public Camera {

public:
    PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, float angle) : Camera(center, direction, up, imgW, imgH) {
        focal = 0.5 * height / tan(0.5 * angle);
    }

    virtual Ray generateRay(const Vector2f &point) override {
        Vector3f dRc((point[0] - 0.5 * width) / focal, (0.5 * height - point[1]) / focal, 1);
        dRc.normalize();
        Vector3f dRw = Matrix3f(horizontal, -up, direction, true) * dRc;
        return Ray(center, dRw);
    }

protected:
    float focal;
};


class LensCamera : public PerspectiveCamera {

public:
    LensCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, float angle,
            float r, float d) : PerspectiveCamera(center, direction, up, imgW, imgH, angle) {
        radius = r;
        depth = d;
    }

    virtual Ray generateRay(const Vector2f &point) override {
        Ray r = PerspectiveCamera::generateRay(point);
        Vector3f objPoint = depth / Vector3f::dot(r.getDirection(), direction) * r.getDirection();
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
        } while(x * x + y * y <= 1);
        return Vector2f(x, y);
    }

    Vector3f randomCenter() {
        Vector3f dx = Vector3f::cross(direction, up).normalized();
        Vector3f dy = Vector3f::cross(direction, dx).normalized();
        Vector2f point = radius * randomPoint();
        return point[0] * dx + point[1] + dy;
    }
};

#endif //CAMERA_H
