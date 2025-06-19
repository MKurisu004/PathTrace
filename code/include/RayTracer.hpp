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

    // 直接光：Phong + 阴影
    if (matType.x() > 0.5){
        for (int i = 0; i < scene.getNumLights(); ++i) {
            Vector3f L, lightCol;
            scene.getLight(i)->getIllumination(P, L, lightCol);

            float lightDist = (scene.getLight(i)->getPosition() - P).length();
            Ray shadowRay(P + N*kEpsilon, L);
            Hit shadowHit;

            if(scene.getLight(i)->getType() == Vector3f(0, 1, 0)){
                if (scene.getGroup()->intersect(shadowRay, shadowHit, kEpsilon) && shadowHit.getT() < lightDist)
                    continue; 
            } else if (scene.getLight(i)->getType() == Vector3f(1, 0 , 0)){
                
            }
            
            // 否则不算阴影，继续累加光照
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
        Vector3f n = (cosI > 0 ? N : -N);
        float eta = (cosI > 0 ? 1.0f/ior : ior);
        cosI = std::fabs(cosI);
        float k = 1 - eta*eta*(1 - cosI*cosI);

        // Fresnel 计算
        float F = fresnelSchlick(cosI, m->getF0()).x(); // 银行fresnelSchlick返回vec3，这里取其 x
        // 1. 全反射（F=1）或内全反射时
        if (k < 0.0f) {
            Vector3f R = reflect(I, n);
            Ray reflRay(P + n * kEpsilon, R);
            result += RayTrace(scene, reflRay, depth+1, weight * F);
        } else {
            // 2. 同时有反射和透射
            Vector3f R = reflect(I, n);
            Vector3f T = (eta * I + (eta * cosI - std::sqrt(k)) * n).normalized();

            Ray reflRay(P + n * kEpsilon, R);
            Ray refrRay(P - n * kEpsilon, T);

            // 混合两者
            result += F     * RayTrace(scene, reflRay, depth+1, weight * F)
                    + (1-F) * RayTrace(scene, refrRay, depth+1, weight * (1-F));
        }
}

    return result;
}