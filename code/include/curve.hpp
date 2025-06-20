#ifndef CURVE_HPP
#define CURVE_HPP

#include "object3d.hpp"
#include <vecmath.h>
#include <vector>
#include <utility>

#include <algorithm>

// TODO (PA2): Implement Bernstein class to compute spline basis function.
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
    explicit Curve(std::vector<Vector3f> points) : controls(std::move(points)) {}

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        return false;
    }

    std::vector<Vector3f> &getControls() {
        return controls;
    }

    virtual void discretize(int resolution, std::vector<CurvePoint>& data) = 0;

    void drawGL() override {
        Object3D::drawGL();
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glDisable(GL_LIGHTING);
        glColor3f(1, 1, 0);
        glBegin(GL_LINE_STRIP);
        for (auto & control : controls) { glVertex3fv(control); }
        glEnd();
        glPointSize(4);
        glBegin(GL_POINTS);
        for (auto & control : controls) { glVertex3fv(control); }
        glEnd();
        std::vector<CurvePoint> sampledPoints;
        discretize(30, sampledPoints);
        glColor3f(1, 1, 1);
        glBegin(GL_LINE_STRIP);
        for (auto & cp : sampledPoints) { glVertex3fv(cp.V); }
        glEnd();
        glPopAttrib();
    }
};

class BezierCurve : public Curve {
public:
    explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4 || points.size() % 3 != 1) {
            printf("Number of control points of BezierCurve must be 3n+1!\n");
            exit(0);
        }
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();
        int segments = (controls.size() - 1) / 3;
        for(int seg = 0; seg < segments; seg++){
            Vector3f P0 = controls[3 * seg];
            Vector3f P1 = controls[3 * seg + 1];
            Vector3f P2 = controls[3 * seg + 2];
            Vector3f P3 = controls[3 * seg + 3];

            for(int i = 0; i <= resolution; i++){
                float t = (float) i / resolution;
                float u = 1.0f - t;
                Vector3f V = u * u * u * P0 + 3 * u * u * t * P1 + 3 * u * t * t * P2 + t * t * t * P3;
                Vector3f T = 3 * u * u * (P1 - P0) + 6 * u * t * (P2 - P1) + 3 * t* t * (P3 - P2);
                
                T.normalize();
                data.push_back({V, T});
            }
        }
    }
};

class BsplineCurve : public Curve {
public:
    BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4) {
            printf("Number of control points of BspineCurve must be more than 4!\n");
            exit(0);
        }
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override {
        data.clear();
        int n = controls.size();
        int k = 3; 
        int numKnots = n + k + 1;
        vector<float> t(numKnots);

        for(int i = 0; i < numKnots; i++){
            t[i] = float(i) / (n + k);
        }

        float t_start = t[k];
        float t_end = t[n];

        for(int seg = k; seg <= n-1; seg++){
            for(int j = 0; j<=resolution; j++){
                float u = (float)j / resolution;
                float mu = (1.0f - u) * t[seg] + u * t[seg + 1];

                float denom = t[seg + 1] - t[seg];
                float localT = denom > 0 ? (mu - t[seg]) / denom : 0;

                float t2 = localT * localT;
                float t3 = t2 * localT;

                float B0 = (1 - localT) * (1 - localT) * (1 - localT) / 6.0f;
                float B1 = (3 * t3 - 6 * t2 + 4) / 6.0f;
                float B2 = (-3 * t3 + 3 * t2 + 3 * localT + 1) / 6.0f;
                float B3 = t3 / 6.0f;

                Vector3f V = B0 * controls[seg - 3] + B1 * controls[seg - 2] + B2 * controls[seg - 1] + B3 * controls[seg];

                float dB0 = -0.5f * (1 - localT) * (1 - localT);
                float dB1 = 1.5f * t2 - 2 * localT;
                float dB2 = -1.5f * t2 + localT + 0.5f;
                float dB3 = 0.5f * t2;

                Vector3f T = dB0 * controls[seg - 3] + dB1 * controls[seg - 2] + dB2 * controls[seg - 1] + dB3 * controls[seg];

                T.normalize();
                data.push_back({ V, T });
            }
        }

    }

protected:

};

#endif // CURVE_HPP
