#ifndef LIGHT_H
#define LIGHT_H

#include <utility>
#include <Vector3f.h>
#include "object3d.hpp"
#include "ray.hpp"


class Light {
public:
    Light() = default;

    virtual ~Light() = default;

    virtual void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const = 0;

    virtual std::pair<Ray, Vector3f> generate() const = 0;

};


class DirectionalLight : public Light {
public:
    DirectionalLight() = delete;

    DirectionalLight(const Vector3f &d, const Vector3f &c) {
        direction = d.normalized();
        color = c;
    }

    ~DirectionalLight() override = default;

    ///@param p unsed in this function
    ///@param distanceToLight not well defined because it's not a point light
    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = -direction;
        col = color;
    }

    virtual std::pair<Ray, Vector3f> generate() const override {
        printf("Cannot generate a ray from a directional light!");
        exit(0);
    }

private:

    Vector3f direction;
    Vector3f color;

};

class PointLight : public Light {
public:
    PointLight() = delete;

    PointLight(const Vector3f &p, const Vector3f &c) {
        position = p;
        color = c;
    }

    ~PointLight() override = default;

    virtual void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = (position - p);
        dir = dir / dir.length();
        col = color;
    }

    virtual std::pair<Ray, Vector3f> generate() const override {
        float x, y, z;
        do {
            x = 2.0 * rand() / RAND_MAX - 1;
            y = 2.0 * rand() / RAND_MAX - 1;
            z = 2.0 * rand() / RAND_MAX - 1;
        } while (x * x + y * y + z * z <= 1);
        return std::make_pair(Ray(position, Vector3f(x, y, z).normalized()), color);
    }

private:

    Vector3f position;
    Vector3f color;

};

class AreaLight : public Light {
public:
    AreaLight() = delete;

    AreaLight(const Vector3f &p, const Vector3f &z, const Vector3f &x, 
            const Vector3f &y, float wx, float wy, Vector3f &c) {
        center = p;
        axisZ = z;
        axisX = x;
        axisY = y;
        widthX = wx;
        widthY = wy;
        color = c;
    }

    static AreaLight XY(const Vector3f p, float wx, float wy, Vector3f &c) {
        return AreaLight(p, Vector3f(0, 0, 1), Vector3f(1, 0, 0), Vector3f(0, 1, 0),
                wx, wy, c);
    }

    static AreaLight XZ(const Vector3f p, float wx, float wz, Vector3f &c) {
        return AreaLight(p, Vector3f(0, 1, 0), Vector3f(1, 0, 0), Vector3f(0, 0, 1),
                wx, wz, c);
    }

    static AreaLight YZ(const Vector3f p, float wy, float wz, Vector3f &c) {
        return AreaLight(p, Vector3f(1, 0, 0), Vector3f(0, 1, 0), Vector3f(0, 0, 1),
                wy, wz, c);
    }

    virtual void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        Vector3f o = RandomOrigin();
        dir = (p - o).normalized();
        col = color;
    }

    virtual std::pair<Ray, Vector3f> generate() const override {
        return std::make_pair(Ray(RandomOrigin(), RandomDirection()), color);
    }

private:

    Vector3f RandomOrigin() const {
        float x = widthX * rand() / RAND_MAX - widthX / 2;
        float y = widthY * rand() / RAND_MAX - widthY / 2;
        return x * axisX + y * axisY;
    }

    Vector3f RandomDirection() const {
        float x, y, z;
        do {
            x = 2.0 * rand() / RAND_MAX - 1;
            y = 2.0 * rand() / RAND_MAX - 1;
            z = 2.0 * rand() / RAND_MAX - 1;
        } while (x * x + y * y + z * z <= 1 && !(x == 0 && y == 0 && z == 0));
        if (z < 0) {
            z = -z;
        }
        return (x * axisX + y * axisY + z * axisZ) / Vector3f(x, y, z).length();
    }

    Vector3f center;
    Vector3f axisZ;
    Vector3f axisX;
    Vector3f axisY;
    float widthX, widthY;
    Vector3f color;
};

#endif // LIGHT_H
