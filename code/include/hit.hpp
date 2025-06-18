#ifndef HIT_H
#define HIT_H

#include <vecmath.h>
#include "ray.hpp"
#include "object3d.hpp"

class Material;

class Object3D;

class Hit { // Hit 类用于记录光线与物体相交时的详细信息，主要包括交点距离、法向量以及与该交点关联的材质。
public:

    // constructors
    Hit() {
        material = nullptr;
        t = 1e38;
    }

    Hit(float _t, Material *m, const Vector3f &n) {
        t = _t;
        material = m;
        normal = n;
    }

    Hit(const Hit &h) {
        t = h.t;
        material = h.material;
        normal = h.normal;
    }

    // destructor
    ~Hit() = default;

    float getT() const {
        return t;
    }

    Material *getMaterial() const {
        return material;
    }

    const Vector3f &getNormal() const {
        return normal;
    }

    Object3D* getObject() const     { return object; }

    void set(float _t, Material *m, const Vector3f &n, Object3D *obj) {
        t = _t;
        material = m;
        normal = n;
        object = obj;
    }

private:
    float t;    // 光线参数 t
    Material *material; // 材质
    Vector3f normal;    // 交点处的法向量
    Object3D *object;

};

inline std::ostream &operator<<(std::ostream &os, const Hit &h) {
    os << "Hit <" << h.getT() << ", " << h.getNormal() << ">";
    return os;
}

#endif // HIT_H
