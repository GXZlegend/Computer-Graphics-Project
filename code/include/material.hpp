#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include <iostream>

class Material {
public:

    explicit Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO,
                      float s = 0, float ratio = 0, float ref = 1) :
            diffuseColor(d_color), specularColor(s_color), shininess(s), specularRatio(ratio), refraction(ref) {

    }

    virtual ~Material() = default;

    virtual Vector3f getDiffuseColor() const {
        return diffuseColor;
    }

    Vector3f Shade(const Vector3f &rayView, const Vector3f &rayLight, 
                   const Vector3f &normal, const Vector3f &lightColor) {
        Vector3f shaded = Vector3f::ZERO;
        float dot_diffuse = Vector3f::dot(-rayLight, normal);
        float dot_specular = Vector3f::dot(2 * dot_diffuse * normal + rayLight, -rayView);
        if (dot_diffuse > 0)
            shaded += diffuseColor * dot_diffuse * lightColor;
        if (dot_specular > 0)
            shaded += specularColor * pow(dot_specular, shininess) * lightColor;
        return shaded;
    }

    Vector3f Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor) {
        Vector3f shaded = Vector3f::ZERO;
        float dot_diffuse = Vector3f::dot(dirToLight, hit.getNormal());
        float dot_specular = Vector3f::dot(2 * dot_diffuse * hit.getNormal() - dirToLight, -ray.getDirection());
        if (dot_diffuse > 0)
            shaded += diffuseColor * dot_diffuse * lightColor;
        if (dot_specular > 0)
            shaded += specularColor * pow(dot_specular, shininess) * lightColor;
        return shaded;
    }

    Vector3f diffuseColor;
    Vector3f specularColor;
    float shininess;
    float specularRatio;
    float refraction;
};


#endif // MATERIAL_H
