#ifndef OBJECT3D_H
#define OBJECT3D_H

#include "ray.hpp"
#include "hit.hpp"
#include "material.hpp"

// Base class for all 3d entities.
class Object3D {
public:
    Object3D() : material(nullptr) {}

    virtual ~Object3D() = default;

    Material *getMaterial() const {
        return material;
    }

    explicit Object3D(Material *material) {
        this->material = material;
    }

    Vector3f getEmission() const{
        if (material) {
            return material->getEmission();
        }
        return Vector3f::ZERO;
    }

    virtual Vector3f sampleDirect(const Vector3f &p, Vector3f &outDir, float &pdfA, Vector3f &xNormal) const = 0;

    // Intersect Ray with this object. If hit, store information in hit structure.
    virtual bool intersect(const Ray &r, Hit &h, float tmin) = 0;
protected:
    Material *material;
};

#endif

