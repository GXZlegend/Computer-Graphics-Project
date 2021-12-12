#ifndef CURVE_HPP
#define CURVE_HPP

#include "object3d.hpp"
#include <vecmath.h>
#include <vector>
#include <utility>

#include <algorithm>

// TODO (PA3): Implement Bernstein class to compute spline basis function.
//       You may refer to the python-script for implementation.

// The CurvePoint object stores information about a point on a curve
// after it has been tesselated: the vertex (V) and the tangent (T)
// It is the responsiblility of functions that create these objects to fill in all the data.
struct CurvePoint {
    Vector3f V; // Vertex
    Vector3f T; // Tangent  (unit)
};

class Curve : public Object3D {
protected:
    std::vector<Vector3f> controls;
public:
    explicit Curve(std::vector<Vector3f> points) : controls(std::move(points)) {
        for (int controlId = 0; controlId < (int) controls.size(); ++controlId) {
            for (int dim = 0; dim < 3; ++dim) {
                box.v[0][dim] = std::min(box.v[0][dim], controls[controlId][dim]);
                box.v[1][dim] = std::max(box.v[1][dim], controls[controlId][dim]);
            }
        }
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        return false;
    }

    std::vector<Vector3f> &getControls() {
        return controls;
    }

    virtual std::pair<float, float> getRange() = 0;
    virtual CurvePoint getCurvePoint(float t) = 0;
    virtual void discretize(int resolution, std::vector<CurvePoint>& data) = 0;
};

class BezierCurve : public Curve {
public:
    explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4 || points.size() % 3 != 1) {
            printf("Number of control points of BezierCurve must be 3n+1!\n");
            exit(0);
        }
    }

    std::pair<float, float> getRange() override { return std::make_pair(0.0, 1.0); }

    CurvePoint getCurvePoint(float t) override {
        int n = (int) controls.size();
        std::vector<Vector3f> vertex(controls);
        for (int degree = 1; degree < n - 1; ++degree)
            for (int pointId = 0; pointId < n - degree; pointId ++ )
                vertex[pointId] = (1 - t) * vertex[pointId] + t * vertex[pointId + 1];
        CurvePoint point;
        point.V = (1 - t) * vertex[0] + t * vertex[1];
        point.T = n * (vertex[1] - vertex[0]);
        return point;
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();
        // TODO (PA3): fill in data vector
        for (int sampleId = 0; sampleId < resolution; ++sampleId) {
            float t = 1.0 / resolution * sampleId;
            CurvePoint sample = getCurvePoint(t);
            sample.T.normalize();
            data.push_back(sample);
        }
    }

protected:

};

class BsplineCurve : public Curve {
public:
    BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4) {
            printf("Number of control points of BspineCurve must be more than 4!\n");
            exit(0);
        }
        k = 3;
        for (int knotId = 0; knotId <= (int) controls.size() + k; ++knotId)
            knots.push_back(1.0 * knotId / ((int) controls.size() + k));
    }

    std::pair<float, float> getRange() override { return std::make_pair(knots[k], knots[controls.size()]); }

    CurvePoint getCurvePoint(float t) override {
        int intervalId = getInterval(t);
        std::vector<float> B;
        B.push_back(1);
        for (int degree = 1; degree < k; ++degree) {
            for (int funId = intervalId - degree; funId <= intervalId; ++funId) {
                if (funId == intervalId - degree)
                    B.push_back((knots[funId + degree + 1] - t) / (knots[funId + degree + 1] - knots[funId + 1]) * B[degree - 1]);
                else if (funId == intervalId)
                    B[0] = (t - knots[funId]) / (knots[funId + degree] - knots[funId]) * B[0];
                else
                    B[intervalId - funId] = (t - knots[funId]) / (knots[funId + degree] - knots[funId]) * B[intervalId - funId]
                                          + (knots[funId + degree + 1] - t) / (knots[funId + degree + 1] - knots[funId + 1]) * B[intervalId - funId - 1];
            }
        }
        CurvePoint point;
        point.V = Vector3f::ZERO;
        point.T = Vector3f::ZERO;
        for (int funId = intervalId - k; funId <= intervalId; ++funId) {
            if (funId == intervalId - k) {
                point.V += controls[funId] * ((knots[funId + k + 1] - t) / (knots[funId + k + 1] - knots[funId + 1]) * B[2]);
                point.T -= controls[funId] * (k / (knots[funId + k + 1] - knots[funId + 1]) * B[2]);
            }
            else if (funId == intervalId) {
                point.V += controls[funId] * ((t - knots[funId]) / (knots[funId + k] - knots[funId]) * B[0]);
                point.T += controls[funId] * (k / (knots[funId + k] - knots[funId]) * B[0]);
            }
            else {
                point.V += controls[funId] * ((t - knots[funId]) / (knots[funId + k] - knots[funId]) * B[intervalId - funId]
                          + (knots[funId + k + 1] - t) / (knots[funId + k + 1] - knots[funId + 1]) * B[intervalId - funId - 1]);
                point.T += controls[funId] * (k / (knots[funId + k] - knots[funId]) * B[intervalId - funId]
                          - k / (knots[funId + k + 1] - knots[funId + 1]) * B[intervalId - funId - 1]);
            }
        }
        return point;
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();
        // TODO (PA3): fill in data vector
        int n = (int) controls.size();
        for (int intervalId = k; intervalId < n; ++intervalId) {
            for (int sampleId = 0; sampleId < resolution; ++sampleId) {
                float t = knots[intervalId] + 1.0 / resolution * sampleId / (n + k);
                CurvePoint sample = getCurvePoint(t);
                sample.T.normalize();
                data.push_back(sample);
            }
        }
    }

protected:
    std::vector<float> knots;
    int k;

private:
    int getInterval(float t) {
        int intervalId = std::upper_bound(knots.begin(), knots.end(), t) - knots.begin() - 1;
        if (intervalId == (int) controls.size())
            --intervalId;
        return intervalId;
    }
};

#endif // CURVE_HPP
