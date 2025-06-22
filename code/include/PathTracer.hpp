#pragma once
#include <vecmath.h>
#include "ray.hpp"
#include "hit.hpp"
#include "scene_parser.hpp"
#include "group.hpp" 
#include "material.hpp"
#include "light.hpp" 
#include "utils.hpp"
#include "ray_utils.hpp"
#include "random_utils.hpp"
#include "sphere.hpp"
#include <cmath>
#include <iostream>

// Path Trace with cos weighted
inline Vector3f PathTrace(const SceneParser &scene, Ray ray, int depth){ 

    Hit hit;
    bool isect = scene.getGroup()->intersect(ray, hit, kEpsilon);
    if (!isect) {
        return scene.getBackgroundColor();
    } 

    // get hit information
    Material *m = hit.getMaterial();
    Vector3f N = hit.getNormal().normalized();
    Vector3f I = ray.getDirection().normalized();
    Vector3f P = ray.pointAtParameter(hit.getT());
    Vector3f matType = m->getType();
    Vector3f emission = m->getEmission();
    Vector3f color = m->getColor();
    float alpha = m->getAlpha();
    Vector3f F0 = m->getF0();

    if(emission != Vector3f::ZERO) {
        if(Vector3f::dot(I, N) < 0) return emission;
        else return Vector3f::ZERO;
    }

    if (Vector3f::dot(N, I) > 0 && matType.x() >= 0.5f) return Vector3f::ZERO;    // 确保法向量和入射光在同一半空间内


    // Russian Roulette
    double rrFactor = 1.0;
    if(depth > MAX_DEPTH){
        if (rnd() <= rrStopProb){
            return Vector3f(0, 0, 0); 
        }
        rrFactor = 1.0 / (1.0 - rrStopProb);
    }

    Vector3f newDir;
    Vector3f newOrigin = P + N * kEpsilon;

    if (matType.x() == 1){
        // Diffuse material
        newDir = diffuse(I, N);
        Ray newRay(newOrigin, newDir);
        return color * PathTrace(scene, newRay, depth + 1) * rrFactor;
    } else if (matType.y() == 1){
        // Specular material
        newDir = reflect(I, N);
        Ray newRay(newOrigin, newDir);
        return color * PathTrace(scene,newRay, depth + 1) * rrFactor;
    } else if (matType.z() == 1){
        // Refractive material
        float refractiveIndex = m->getRefractiveIndex();
        newDir = refract(I, N, refractiveIndex);
        Ray newRay(newOrigin, newDir);
        return color * PathTrace(scene, newRay, depth + 1) * rrFactor;
    } else if (std::fabs(matType.z()) < 1e-6f){
        // Cook - Torrance material
        float kd = matType.x();
        float ks = matType.y();

        // randomly sample a direction in the hemisphere
        float r = RND1;
        Vector3f newDir;
        float pdf;
        Vector3f wo = -I;
        Vector3f wi;
        Vector3f h;
        float D = 0;
        float G = 0;
        Vector3f F = Vector3f::ZERO;

        if (r < kd) {
            newDir = diffuse(I, N);
            wi = newDir;
            h = (wi + wo).normalized();
            D = D_GGX(h, N, alpha);
            pdf = kd * std::max(0.0f, Vector3f::dot(N, newDir)) / M_PI + ks * (D * std::abs(Vector3f::dot(h, N))) / (4 * std::max(1e-6f, Vector3f::dot(wi, h)));
        } else {
            // Sample GGX 
            newDir = sampleGGX(I, N, alpha);
            wi = newDir;
            h = (wi + wo).normalized();
            D = D_GGX(h, N, alpha);
            pdf = kd * std::max(0.0f, Vector3f::dot(N, newDir)) / M_PI + ks * (D * std::abs(Vector3f::dot(h, N))) / (4 * std::max(1e-6f, Vector3f::dot(wi, h)));
            
        }

        // 采样光线
        Ray newRay (P + N * kEpsilon, newDir);

        // 计算 fr
        float cos_i = std::max(0.0f, Vector3f::dot(N, wi));
        float cos_o = std::max(0.0f, Vector3f::dot(N, wo));

        Vector3f diffuseBRDF = color / M_PI;
        F = Vector3f(1, 1, 1);
        // F = fresnelSchlick(std::max(0.0f, Vector3f::dot(wi, h)), F0);
        G = G_Smith(wo, wi, N, alpha);
        Vector3f specularBRDF = (D * F * G) / (4 * cos_i * cos_o + 1e-6f);


        Vector3f fr = ( kd * diffuseBRDF + ks * specularBRDF);

        // 递归结果
        Vector3f Li = PathTrace(scene, newRay, depth + 1);

    
        // 最终返回
        return fr * Li * rrFactor * cos_i / pdf;
    } else {
        return Vector3f(0,0,0);
    }
}

inline Vector3f PathTraceNEE(const SceneParser &scene, Ray ray, int depth){

    // 获得交点信息
    Hit hit;
    bool isect = scene.getGroup()->intersect(ray, hit, kEpsilon);
    if (!isect) {
        return scene.getBackgroundColor();
    } 
    Material *m = hit.getMaterial();
    Vector3f N = hit.getNormal().normalized();
    Vector3f I = ray.getDirection().normalized();
    Vector3f P = ray.pointAtParameter(hit.getT());
    Vector3f matType = m->getType();
    Vector3f emission = m->getEmission();    
    Vector3f color = m->getColor();
    float alpha = m->getAlpha();
    Vector3f F0 = m->getF0();
    Object3D *hitObj = hit.getObject();
    int objType = hitObj->getType();

    if (Vector3f::dot(N, I) > 0 && matType.x() >= 0.5f) return Vector3f::ZERO;    // 确保法向量和入射光在同一半空间内

    if(emission != Vector3f::ZERO) {
        if(Vector3f::dot(I, N) < 0) return emission;
        else return Vector3f::ZERO;
    }

    // 计算光源直接光照项 
    Vector3f L_dir(0, 0, 0);

    for (int ei = 0; ei < scene.getNumEmissiveObjects(); ++ei) {
        if (m->getType().x() < 0.5f) continue;   
        Object3D *obj = scene.getEmissiveObject(ei);
        Vector3f wi, xN;
        float pdfA = 0.0f;

        // 对光源采样，得到光源上的采样点 x 
        Vector3f x = obj->sampleDirect(P, wi, pdfA, xN);


        Vector3f offset = x - P;
        float dist2 = offset.squaredLength();
        float dist  = std::sqrt(dist2);

        if(Vector3f::dot(offset, xN) >= 0) continue;

        if (pdfA <= FLOAT_EPSILON) continue;

        Ray shadow(P, wi);

        Hit shadowHit;
        shadowHit.set(dist, nullptr, Vector3f::ZERO, nullptr);
        if (scene.getGroup()->intersect(shadow, shadowHit, kEpsilon)){
            Object3D *shadowHitObject = shadowHit.getObject();
            Material *HitObjectMaterial =  shadowHitObject->getMaterial();
            Vector3f HitEmission = HitObjectMaterial->getEmission();
            if(shadowHit.getT() < dist) continue;
            if(objType == 5 && Vector3f::dot(wi, xN) >= 0) continue;
        }


        float cosP = std::max(0.0f, Vector3f::dot(N, wi));
        float cosX = std::max(0.0f, Vector3f::dot(xN, -wi));
        Vector3f Li = obj->getEmission();
        Vector3f wo = -I;
        Vector3f fr = m->evalNeeBRDF(wo, wi, N);

        // 计算直接光照
        L_dir += Li * fr * cosP * cosX / (dist2 * pdfA);
    }

    // Russian Roulette
    double rrFactor = 1.0;
    if(depth > MAX_DEPTH){
        if (rnd() <= rrStopProb){
            return  emission + L_dir; 
        }
        rrFactor = 1.0 / (1.0 - rrStopProb);
    }

    // 计算间接光照
    Vector3f L_indir = Vector3f(0,0,0);
    Vector3f newDir;
    Vector3f newOrigin = P + N * kEpsilon;

    if (matType.x() == 1) {
        // 漫反射
        newDir = diffuse(I, N);
        Ray    newRay(newOrigin, newDir);
        Hit    h2;
        if (scene.getGroup()->intersect(newRay, h2, kEpsilon)) {
            Object3D *o2 = h2.getObject();

            if (o2->getEmission() != Vector3f::ZERO) {
                // 间接光照命中光源，舍弃
                return L_dir;
            } else {
                L_indir = color * PathTraceNEE(scene, newRay, depth+1);
            }
        }
        
    } else if (matType.y() == 1) {
        // 镜面反射
        newDir = reflect(I, N);
        Ray newRay(newOrigin, newDir);
        Vector3f fr = color;
        L_indir = fr * PathTraceNEE(scene, newRay, depth + 1);
    } else if (matType.z() == 1) {
        // 折射
        newDir = refract(I, N, m->getRefractiveIndex());
        Ray newRay(newOrigin, newDir);
        Vector3f fr = color;
        L_indir = fr * PathTraceNEE(scene, newRay, depth + 1);
    } else {
        // 其他类型，直接跳过
        L_indir = 0;
    }

    return L_dir + L_indir * rrFactor;
}


inline Vector3f PathTraceMIS(const SceneParser &scene, Ray ray, int depth){

    // 获取交点信息
    Hit hit;
    bool isect = scene.getGroup()->intersect(ray, hit, kEpsilon);
    if (!isect) {
        return scene.getBackgroundColor();
    } 
    Material *m = hit.getMaterial();
    Vector3f N = hit.getNormal().normalized();
    Vector3f I = ray.getDirection().normalized();
    if (Vector3f::dot(N, I) > 0) N = -N;    // 确保法向量和入射光在同一半空间内
    Vector3f P = ray.pointAtParameter(hit.getT());
    Vector3f matType = m->getType();
    Vector3f emission = m->getEmission();
    Vector3f color = m->getColor();
    float alpha = m->getAlpha();
    Vector3f F0 = m->getF0();
    Vector3f wo = -I;

    if(emission != Vector3f::ZERO) return emission;

    // 计算光源直接光照项
    Vector3f L_dir(0, 0, 0);
    for (int ei = 0; ei < scene.getNumEmissiveObjects(); ++ei) {
        if (m->getType().x() < 0.5f) continue;   

        // 策略一，对光源采样
        Object3D *obj = scene.getEmissiveObject(ei);
        Vector3f wi, xN;
        float pdfA_li = 0.0f;

        // 对光源采样，得到光源上的采样点 x 
        Vector3f x = obj->sampleDirect(P, wi, pdfA_li, xN);
        if (pdfA_li < FLOAT_EPSILON) continue;

        Vector3f offset = x - P;
        float dist2 = offset.squaredLength();
        float dist  = std::sqrt(dist2);

        if (pdfA_li <= FLOAT_EPSILON) continue;

        // 检测阴影
        Ray shadow(P + N * kEpsilon, wi);
        Hit shadowHit;
        if (scene.getGroup()->intersect(shadow, shadowHit, kEpsilon)){
            Object3D *shadowHitObject = shadowHit.getObject();
            Material *HitObjectMaterial =  shadowHitObject->getMaterial();
            Vector3f HitEmission = HitObjectMaterial->getEmission();
            if(HitEmission == Vector3f::ZERO) continue;
        }
        float cosP = std::max(0.0f, Vector3f::dot(N, wi));
        float cosX = std::max(0.0f, Vector3f::dot(xN, -wi));
        Vector3f Li = obj->getEmission();
        Vector3f wo = -I;
        Vector3f fr = m->evalNeeBRDF(wo, wi, N);

        // 该策略下的能量和pdf
        Vector3f L_i =  Li * fr * cosP * cosX / (dist2 * pdfA_li);
        float pdfA = pdfA_li * dist2 / std::max(1e-6f, cosX);

        // 策略二，均匀球面采样（只考虑命中光源）
        float pdfB;
        Vector3f wi_b = uniformHemisphere(N, pdfB);
        Vector3f fr_b   = m->evalNeeBRDF(wo, wi_b, N);
        float   cosP_b = std::max(0.0f, Vector3f::dot(N, wi_b));
        Vector3f L_b = Vector3f::ZERO;

        // 检测是否命中光源
        if (pdfB > FLOAT_EPSILON && fr_b.length() > 0.0f && cosP_b > 0.0f) {
            Ray   shadowB(P + N * kEpsilon, wi_b);
            Hit   hitB;
            if (scene.getGroup()->intersect(shadowB, hitB, kEpsilon)) {
                Object3D *hitObj = hitB.getObject();
                Vector3f Le_b = hitObj->getEmission();
                
                // 命中光源，则本次有效，计算L_b
                if (Le_b != Vector3f::ZERO) {
                    L_b = Le_b * fr_b * cosP_b / pdfB; 
                }

                // 未命中光源，无效，权重赋零
                else {
                    L_b = Vector3f::ZERO;
                    pdfB = 0.0;
                }
            }
        }

        // 计算两种策略的权重，确保能量无偏
        float wA = pdfA / (pdfA + pdfB + 1e-12f);
        float wB = pdfB / (pdfA + pdfB + 1e-12f);

        // 计算直接光照项
        L_dir += wA * L_i + wB * L_b;
    }

    // Russian Roulette
    double rrFactor = 1.0;
    if(depth > MAX_DEPTH){
        if (rnd() <= rrStopProb){
            return  emission + L_dir; 
        }
        rrFactor = 1.0 / (1.0 - rrStopProb);
    }

    // 计算间接光照
    Vector3f L_indir = Vector3f(0,0,0);
    Vector3f newDir;
    Vector3f newOrigin = P + N * kEpsilon;

    if (matType.x() == 1) {
        // 漫反射
        newDir = diffuse(I, N);
        Ray    newRay(newOrigin, newDir);
        Hit    h2;
        if (scene.getGroup()->intersect(newRay, h2, kEpsilon)) {
            Object3D *o2 = h2.getObject();
            if (o2->getEmission() != Vector3f::ZERO) {
                // 间接光照命中光源，舍弃
                return L_dir;
            } else {
                // 否则正常递归，把间接贡献累加上去
                L_indir = color * PathTraceNEE(scene, newRay, depth+1);
                
            }
        }
    } else if (matType.y() == 1) {
        // 镜面反射
        newDir = reflect(I, N);
        Ray newRay(newOrigin, newDir);
        L_indir = color * PathTraceMIS(scene, newRay, depth+1) ;
    } else if (matType.z() == 1) {
        // 折射
        newDir = refract(I, N, m->getRefractiveIndex());
        Ray newRay(newOrigin, newDir);
        L_indir = color * PathTraceMIS(scene, newRay, depth+1) ;
    } else {
        // 其他类型，直接跳过
        L_indir = 0;
    }
    return L_dir + L_indir * rrFactor;
}


// 平凡PT实现

inline Vector3f PathTraceBasic(const SceneParser &scene, Ray ray, int depth){ 
    
    // Russian Roulette
    double rrFactor = 1.0;
    if(depth > MAX_DEPTH){
        if (rnd() <= rrStopProb){
            return Vector3f(0, 0, 0); 
        }
        rrFactor = 1.0 / (1.0 - rrStopProb);
    }

    Hit hit;
    bool isect = scene.getGroup()->intersect(ray, hit, kEpsilon);
    if (!isect) {
        return scene.getBackgroundColor();
    } 

    // get hit information
    Material *m = hit.getMaterial();
    Vector3f N = hit.getNormal().normalized();
    Vector3f I = ray.getDirection().normalized();
    Vector3f P = ray.pointAtParameter(hit.getT());
    Vector3f matType = m->getType();
    Vector3f emission = m->getEmission();
    Vector3f color = m->getColor();
    float alpha = m->getAlpha();
    Vector3f F0 = m->getF0();

    if(emission != Vector3f::ZERO) return emission;

    if (matType.x() == 1){
        // Diffuse material
        float pdf;
        Vector3f newDir = uniformHemisphere(N, pdf);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        Vector3f brdf = color * (1.0f / M_PI);
        float cosTheta = std::max(0.0f, Vector3f::dot(N, newDir));
        Vector3f Li = PathTraceBasic(scene, newRay, depth + 1);
        return (brdf * Li * cosTheta / pdf) * rrFactor;
    } else if (matType.y() == 1){
        // Specular material
        Vector3f newDir = reflect(I, N);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        return color * PathTraceBasic(scene,newRay, depth + 1) * rrFactor;
    } else if (matType.z() == 1){
        // Refractive material
        float refractiveIndex = m->getRefractiveIndex();
        Vector3f newDir = refract(I, N, refractiveIndex);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        return color * PathTraceBasic(scene, newRay, depth + 1) * rrFactor;
    } else {
        return Vector3f(0,0,0);
    }
}

inline Vector3f PathTraceFre(const SceneParser &scene, Ray ray, int depth){ 
    Hit hit;
    bool isect = scene.getGroup()->intersect(ray, hit, kEpsilon);
    if (!isect) {
        return scene.getBackgroundColor();
    } 

    // get hit information
    Material *m = hit.getMaterial();
    Vector3f N = hit.getNormal().normalized();
    Vector3f I = ray.getDirection().normalized();
    Vector3f P = ray.pointAtParameter(hit.getT());
    Vector3f matType = m->getType();
    Vector3f emission = m->getEmission();
    Vector3f color = m->getColor();
    float alpha = m->getAlpha();
    Vector3f F0 = m->getF0();

    if(emission != Vector3f::ZERO) {
        if(Vector3f::dot(I, N) < 0) return emission;
        else return Vector3f::ZERO;
    }

    if (Vector3f::dot(N, I) > 0 && matType.x() >= 0.5f) return Vector3f::ZERO;

    // Russian Roulette
    double rrFactor = 1.0;
    if(depth > MAX_DEPTH){
        if (rnd() <= rrStopProb){
            return Vector3f(0, 0, 0); 
        }
        rrFactor = 1.0 / (1.0 - rrStopProb);
    }

    Vector3f newDir;
    Vector3f newOrigin = P + N * kEpsilon;

    if (matType.x() == 1){
        // Diffuse material
        newDir = diffuse(I, N);
        Ray newRay(newOrigin, newDir);
        return color * PathTraceFre(scene, newRay, depth + 1) * rrFactor;
    } else if (matType.y() == 1){
        // Specular material
        newDir = reflect(I, N);
        Ray newRay(newOrigin, newDir);
        return color * PathTraceFre(scene,newRay, depth + 1) * rrFactor;
    } else if (matType.z() == 1){
        // Refractive material with Fresnel
        float ior = m->getRefractiveIndex();  // 折射率
        // 计算入射角余弦
        float cosi = (std::clamp(Vector3f::dot(I, N), -1.0f, 1.0f));
        bool MatToAir = (cosi > 0.0f);
        float eta_i = 1.0f, eta_t = ior;
        Vector3f n = N;
        if (MatToAir) {
            // 射线从介质里面射到空气，翻转法线
            std::swap(eta_i, eta_t);
            n = -N;
            cosi = -cosi;  // 保持负值
        }

        float cosTheta = (cosi<0) ? -cosi : cosi;


        float R0 = (eta_i - eta_t) / (eta_i + eta_t);
        R0 = R0 * R0;
        float Fr       = R0 + (1.0f - R0) * std::pow(1.0f - cosTheta, 5.0f);


        // 检测全反射
        float eta = eta_i / eta_t;
        float k   = 1.0f - eta * eta * (1.0f - cosi * cosi);
        bool  tir = (k < 0.0f);

        // 随机选择反射或折射
        float u = rnd();
        bool doReflect = tir || (u < Fr);


        Vector3f newDir = doReflect ? reflect(I, n) : refract(I, n, ior);
        Vector3f bias   = (doReflect ? n : -n) * kEpsilon;
        Vector3f newOrigin = P + bias;
        Ray     newRay(newOrigin, newDir);

        // 保持能量守恒
        float pdf = tir ? 1.0f : (doReflect ? Fr : (1.0f - Fr));

        return color * PathTraceFre(scene, newRay, depth + 1) * rrFactor / pdf;
    } else {
        return Vector3f(0,0,0);
    }
}