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


inline Vector3f PathTrace(const SceneParser &scene, Ray ray, int depth){ 
    
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
        Vector3f newDir = diffuse(I, N);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        return color * PathTrace(scene, newRay, depth + 1) * rrFactor;
    } else if (matType.y() == 1){
        // Specular material
        Vector3f newDir = reflect(I, N);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        return color * PathTrace(scene,newRay, depth + 1) * rrFactor;
    } else if (matType.z() == 1){
        // Refractive material
        float refractiveIndex = m->getRefractiveIndex();
        Vector3f newDir = refract(I, N, refractiveIndex);
        Vector3f newOrigin = P + N * kEpsilon;
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
    // 光能守恒权重
    float weight = (depth == 0) ? 0.5f : 1.0f;

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

    if(emission != Vector3f::ZERO) return emission;

    // 计算光源直接光照项 
    Vector3f L_dir(0, 0, 0);
    for (int ei = 0; ei < scene.getNumEmissiveObjects(); ++ei) {
        if(matType != Vector3f(1, 0, 0)) continue;
        Object3D *obj = scene.getEmissiveObject(ei);
        Material *mLe = obj->getMaterial();
        Vector3f wi, xN;
        float pdfA = 0.0f;

        // 对光源采样，得到光源上的采样点 x 
        Vector3f x = obj->sampleDirect(P, wi, pdfA, xN);

        Vector3f offset = x - P;
        float dist2 = offset.squaredLength();
        float dist  = std::sqrt(dist2);

        if (pdfA <= FLOAT_EPSILON) continue;

        // 检测阴影
        Ray shadow(P + N * kEpsilon, wi);
        Hit shadowHit;
        if (scene.getGroup()->intersect(shadow, shadowHit, kEpsilon) && shadowHit.getT() < dist)
            continue;


        float cosP = std::max(0.0f, Vector3f::dot(N, wi));
        float cosX = std::max(0.0f, Vector3f::dot(xN, -wi));
        
        Vector3f Li = obj->getEmission();
        Vector3f wo = -I;
        Vector3f fr = m->evalNeeBRDF(wo, wi, N);

        Vector3f res = Li * fr * cosP * cosX / (dist2 * pdfA);

        // 计算直接光照
        L_dir += res;
        Object3D *shaded = hit.getObject();  
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
        L_indir = weight * color * PathTraceNEE(scene, newRay, depth+1) ;
    } else if (matType.y() == 1) {
        // 镜面反射
        newDir = reflect(I, N);
        Ray newRay(newOrigin, newDir);
        L_indir = weight * color * PathTraceNEE(scene, newRay, depth+1) ;
    } else if (matType.z() == 1) {
        // 折射
        newDir = refract(I, N, m->getRefractiveIndex());
        Ray newRay(newOrigin, newDir);
        L_indir = weight * color * PathTraceNEE(scene, newRay, depth+1) ;
    } else {
        // 其他类型，直接跳过
        L_indir = 0;
    }
    return L_dir + L_indir * rrFactor;
}


inline Vector3f PathTraceMIS(const SceneParser &scene, Ray ray, int depth){

    // 光能守恒权重
    float weight = (depth == 0) ? 0.5f : 1.0f;

    // 获取交点信息
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
    Vector3f wo = -I;

    if(emission != Vector3f::ZERO) return emission;

    // 计算光源直接光照项
    Vector3f L_dir(0, 0, 0);
    for (int ei = 0; ei < scene.getNumEmissiveObjects(); ++ei) {
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
        if (scene.getGroup()->intersect(shadow, shadowHit, kEpsilon) && shadowHit.getT() < dist)
            continue;

        float cosP = std::max(0.0f, Vector3f::dot(N, wi));
        float cosX = std::max(0.0f, Vector3f::dot(xN, -wi));
        Vector3f Le = obj->getEmission();
        Vector3f wo = -I;
        Vector3f fr = m->evalNeeBRDF(wo, wi, N);

        float pdfA = pdfA_li * dist2 / std::max(1e-6f, cosX);

        float pdfB = m->pdfBsdf(wo, wi, N);

        float wA = pdfA / (pdfA + pdfB + 1e-12f);



        // Monte Carlo 方法计算直接光照, 存储在向量中
        Vector3f L_i =  Le * fr * cosP * cosX / (dist2 * pdfA_li);
        L_dir += weight * wA * L_i;
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
        L_indir = weight * color * PathTraceNEE(scene, newRay, depth+1) ;
    } else if (matType.y() == 1) {
        // 镜面反射
        newDir = reflect(I, N);
        Ray newRay(newOrigin, newDir);
        L_indir = weight * color * PathTraceNEE(scene, newRay, depth+1) ;
    } else if (matType.z() == 1) {
        // 折射
        newDir = refract(I, N, m->getRefractiveIndex());
        Ray newRay(newOrigin, newDir);
        L_indir = weight * color * PathTraceNEE(scene, newRay, depth+1) ;
    } else {
        // 其他类型，直接跳过
        L_indir = 0;
    }
    return L_dir + L_indir * rrFactor;
}