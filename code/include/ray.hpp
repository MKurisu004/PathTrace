#ifndef RAY_H
#define RAY_H

#include <cassert>
#include <iostream>
#include <Vector3f.h>


// Ray class mostly copied from Peter Shirley and Keith Morley
class Ray {
public:

    Ray() = delete;
    Ray(const Vector3f &orig, const Vector3f &dir) {
        origin = orig;
        direction = dir;
    }

    Ray(const Ray &r) {
        origin = r.origin;
        direction = r.direction;
    }

    const Vector3f &getOrigin() const { // 获取光线的起点
        return origin;
    }

    const Vector3f &getDirection() const {  // 获取方向向量
        return direction;
    }

    Vector3f pointAtParameter(float t) const {  // 计算光线在参数 t (光线长度) 处的点
        return origin + direction * t;
    }

    void setDirection(const Vector3f &dir) {  // 设置光线的方向向量
        direction = dir.normalized();
    }

private:

    Vector3f origin;
    Vector3f direction;

};

inline std::ostream &operator<<(std::ostream &os, const Ray &r) {
    os << "Ray <" << r.getOrigin() << ", " << r.getDirection() << ">";
    return os;
}

#endif // RAY_H
