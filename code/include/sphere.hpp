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
        h.set(t_candidate, material, normal);
        return true;
    }

protected:
    Vector3f center;
    float radius;
};


#endif
