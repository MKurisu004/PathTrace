#pragma once
#include "object3d.hpp"
#include <omp.h>
#include <vecmath.h>
#include "random_utils.hpp" // for rnd()

class Rectangle : public Object3D {
public:
    Rectangle(const Vector3f &o, const Vector3f &u, float lu, const Vector3f &v, float lv, Material *mat)
        : Object3D(mat), origin(o), U(u.normalized()), V(v.normalized()), LU(lu), LV(lv) {

        normal = Vector3f::cross(U, V).normalized();
        area = (2*LU) * (2*LV) * Vector3f::cross(U, V).normalized().length();
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        float denom = Vector3f::dot(r.getDirection(), normal);
        if (fabs(denom) < 1e-6f) return false; // parallel to plane
        float t = Vector3f::dot(origin - r.getOrigin(), normal) / denom;
        if (t < tmin || t > h.getT()) return false;

        // Compute hit point
        Vector3f P = r.pointAtParameter(t);
        // Vector from center to hit point
        Vector3f d = P - origin;
        // Project onto local axes
        float uDist = Vector3f::dot(d, U);
        float vDist = Vector3f::dot(d, V);
        // Check bounds
        if (uDist < -LU || uDist > LU || vDist < -LV || vDist > LV)
            return false;

        // Valid hit: choose normal facing the ray
        Vector3f nHit = denom < 0.0f ? normal : -normal;
        h.set(t, material, nHit, this);
        return true;
    }

    int getType() const override {
        return 2;
    }

    // Sample a point on rectangle for NEE
    Vector3f sampleDirect(const Vector3f &p, Vector3f &outDir, float &pdfA, Vector3f &xNormal) const override {
        float ru = (rnd() * 2.0f - 1.0f) * LU;
        float rv = (rnd() * 2.0f - 1.0f) * LV;
        Vector3f x = origin + U * ru + V * rv;

        
        outDir = (x - p).normalized();
        xNormal = normal;
        if (Vector3f::dot(xNormal, outDir) > 0.0f)
            xNormal = -xNormal;
        
        // Area PDF
        pdfA = 1.0f / area;

        return x;
    }

private:
    Vector3f origin;
    Vector3f U, V;
    Vector3f normal;
    float LU, LV, area;
};


