#pragma once
#include <vecmath.h>
#include "ray.hpp"
#include "hit.hpp"
#include "scene_parser.hpp"
#include "group.hpp" 
#include "material.hpp"
#include "light.hpp" 
#include "utils.hpp"  
#include <cmath>

inline Vector3f RayTrace(const SceneParser &scene, const Ray &ray, int depth, float weight = 1.0f) {
    // 超过最大递归深度终止
    if (depth > MAX_DEPTH || weight < 0.0001f) 
        return Vector3f(0,0,0);

    // 求最近交点
    Hit hit;
    bool isect = scene.getGroup()->intersect(ray, hit, kEpsilon);
    if (!isect)
        return scene.getBackgroundColor();

    //  获取材质和交点信息
    Material *m = hit.getMaterial();
    Vector3f N = hit.getNormal().normalized();
    Vector3f I = ray.getDirection().normalized();
    Vector3f P = ray.pointAtParameter(hit.getT());
    Vector3f matType = m->getType();

    Vector3f result(0,0,0);

    // 局部光照 
    if (matType.x() > 0.5f) {
        // 直接光：Phong + 阴影
        for (int i = 0; i < scene.getNumLights(); ++i) {
            Vector3f L, lightCol;
            scene.getLight(i)->getIllumination(P, L, lightCol);

            // 阴影检测
            Ray shadowRay(P + N * kEpsilon, L.normalized());
            Hit shadowHit;
            if (scene.getGroup()->intersect(shadowRay, shadowHit, kEpsilon))
                continue;

            result += m->phongShade(ray, hit, L, lightCol);
        }
    }

    // 完美镜面反射 
    if (matType.y() > 0.5f) {
        Vector3f R = (I - 2 * Vector3f::dot(I, N) * N).normalized();
        Ray reflRay(P, R);
        result += weight * RayTrace(scene, reflRay, depth + 1, 0.2 * weight);
    }

    // 折射
    if (matType.z() > 0.5f) {
        float ior = m->getRefractiveIndex();
        float cosI = -Vector3f::dot(N, I);

        // 判断折射方向
        float eta = (cosI > 0 ? 1.0f / ior : ior);
        Vector3f n   = (cosI > 0 ? N : -N);
        float k      = 1 - eta * eta * (1 - cosI * cosI);

        if (k < 0) {
            // 全反射
            Vector3f R = (I - 2 * Vector3f::dot(I, n) * n).normalized();
            Ray reflRay(P + n * kEpsilon, R);
            result += weight * RayTrace(scene, reflRay, depth + 1, 0.2 * weight);
        } else {
            // 折射
            Vector3f T = (eta * I + (eta * cosI - sqrtf(k)) * n).normalized();
            Ray refrRay(P - n * kEpsilon, T);
            result += weight * RayTrace(scene, refrRay, depth + 1, 0.2 * weight);
        }
    }

    return result;
}