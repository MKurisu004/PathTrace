#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include "random_utils.hpp" // for rnd()

class Triangle : public Object3D {
public:
    // a, b, c are the triangle vertices
    Triangle(const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m)
        : Object3D(m) {
        vertices[0] = a;
        vertices[1] = b;
        vertices[2] = c;
        // Precompute normal and area
        edge1 = b - a;
        edge2 = c - a;
        normal = Vector3f::cross(edge1, edge2).normalized();
        area = 0.5f * Vector3f::cross(edge1, edge2).length();
    }

    ~Triangle() override = default;

    bool intersect(const Ray& ray, Hit& hit, float tmin) override {
        const Vector3f& v0 = vertices[0];
        // Moller-Trumbore
        Vector3f pvec = Vector3f::cross(ray.getDirection(), edge2);
        float det = Vector3f::dot(edge1, pvec);
        if (fabs(det) < 1e-6f) return false;
        float inv_det = 1.0f / det;

        Vector3f tvec = ray.getOrigin() - v0;
        float u = Vector3f::dot(tvec, pvec) * inv_det;
        if (u < 0.0f || u > 1.0f) return false;

        Vector3f qvec = Vector3f::cross(tvec, edge1);
        float v = Vector3f::dot(ray.getDirection(), qvec) * inv_det;
        if (v < 0.0f || u + v > 1.0f) return false;

        float t_candidate = Vector3f::dot(edge2, qvec) * inv_det;
        if (t_candidate < tmin || t_candidate > hit.getT()) return false;

        hit.set(t_candidate, material, normal, this);
        return true;
    }

    // Uniform area sampling on triangle for NEE
    Vector3f sampleDirect(const Vector3f &p,
                          Vector3f &outDir,
                          float &pdfA,
                          Vector3f &xNormal) const override {
        // Sample barycentric coords
        float u1 = rnd();
        float u2 = rnd();
        if (u1 + u2 > 1.0f) {
            u1 = 1.0f - u1;
            u2 = 1.0f - u2;
        }
        // Point on triangle
        Vector3f x = vertices[0] + edge1 * u1 + edge2 * u2;
        // Compute direction and normal
        outDir = (x - p).normalized();
        xNormal = normal;
        if (Vector3f::dot(xNormal, outDir) > 0.0f)
            xNormal = -xNormal;
        // PDF w.r.t. area
        pdfA = 1.0f / area;
        return x;
    }

    Vector3f vertices[3];
    Vector3f edge1, edge2;
    Vector3f normal;
    float area;
};

#endif // TRIANGLE_H
