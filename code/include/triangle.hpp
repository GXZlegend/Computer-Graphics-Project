#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
		vertices[0] = a;
		vertices[1] = b;
		vertices[2] = c;
        normal = Vector3f::cross(b - a, c - a).normalized();
	}

	bool intersect( const Ray& ray,  Hit& hit , float tmin) override {
		Vector3f Rd = ray.getDirection();
		Vector3f E1 = vertices[0] - vertices[1];
		Vector3f E2 = vertices[0] - vertices[2];
		Vector3f S = vertices[0] - ray.getOrigin();
		float D0 = Matrix3f(Rd, E1, E2, true).determinant();
		if (D0 == 0)
			return false;
		float D1 = Matrix3f(S, E1, E2, true).determinant();
		float D2 = Matrix3f(Rd, S, E2, true).determinant();
		float D3 = Matrix3f(Rd, E1, S, true).determinant();
		float t = D1 / D0, beta = D2 / D0, gamma = D3 / D0;
		if (t > tmin && beta >= 0 && gamma >= 0 && beta + gamma <= 1 && t < hit.getT()) {
			hit.set(t, material, normal);
			return true;
		}
		return false;
	}
	
	Vector3f normal;
	Vector3f vertices[3];
	
protected:

};

#endif //TRIANGLE_H
