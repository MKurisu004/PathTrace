#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement functions and add more fields as necessary

class Sphere : public Object3D {
public:
    Sphere() : radius(0), center(0, 0, 0)  {
        // unit ball at the center
    }

    Sphere(const Vector3f &center, float radius, Material *material) 
        : Object3D(material), center(center), radius(radius) {}

    ~Sphere() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        Vector3f OC = r.getOrigin() - center;

        float a = Vector3f::dot(r.getDirection(), r.getDirection());
        float b = 2 * Vector3f::dot(OC, r.getDirection());
        float c = Vector3f::dot(OC, OC) - radius * radius;

        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0) return false;

        float sqrt_discriminant = std::sqrt(discriminant);
        
        float t_candidate = (-b - sqrt_discriminant) / (2 * a);

        if(t_candidate < tmin || t_candidate > h.getT()){
            t_candidate = (-b + sqrt_discriminant) / (2 * a);
            if(t_candidate < tmin || t_candidate > h.getT()){
                return false;
            }
        }

        Vector3f hitPoint = r.pointAtParameter(t_candidate);
        Vector3f normal = (hitPoint - center) / radius;
        h.set(t_candidate, material, normal, this);
        return true;
    }
    
    int getType() const override {
        return 1;
    }

    Vector3f sampleDirect(const Vector3f &p, Vector3f &outDir, float &pdfA, Vector3f &xNormal) const override {
        // 1) 在单位球面上做面积采样
        float u1 = rnd();             // ∈ [0,1)
        float u2 = rnd();             // ∈ [0,1)
        float z  = 1.0f - 2.0f * u1;   // cosθ, 均匀面积分布
        float r  = std::sqrt(std::max(0.0f, 1.0f - z*z));
        float phi = 2.0f * M_PI * u2;
        // 球面方向向量
        Vector3f dir(r * std::cos(phi),
                    r * std::sin(phi),
                    z);

        // 2) 在世界坐标系下的采样点
        Vector3f x = center + dir * radius;

        // 3) 输出方向 / 法线
        outDir = (x - p).normalized();
        xNormal = dir;  // 采样点处的表面法线（已单位化）

        // 如果采样点法线和出射方向同向，则翻转
        if (Vector3f::dot(xNormal, outDir) > 0.0f)
            xNormal = -xNormal;

        // 4) 面积 PDF
        pdfA = 1.0f / (4.0f * M_PI * radius * radius);

        return x;
    }

protected:
    Vector3f center;
    float radius;
};


#endif
