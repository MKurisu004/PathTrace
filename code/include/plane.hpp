#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    Plane() : normal(Vector3f::UP), d(0){

    }

    Plane(const Vector3f &n, float d, Material *m) : Object3D(m), d(d){
        normal = n.normalized();
    }

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        float denom = Vector3f::dot(normal, r.getDirection());

        if (std::fabs(denom) < 1e-6) return false;

        float t_candidate = (d - Vector3f::dot(normal, r.getOrigin())) / denom;

        if(t_candidate < tmin || t_candidate > h.getT()){
            return false;
        }
        h.set(t_candidate, material, normal);
        return true;
    }

protected:
    Vector3f normal;
    float d;
};

#endif //PLANE_H
		

