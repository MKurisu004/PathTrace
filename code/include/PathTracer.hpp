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

    if (matType.x() == 1){
        // Diffuse material
        Vector3f newDir = diffuse(I, N);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        return emission + color * PathTrace(scene, newRay, depth + 1) * rrFactor;
    } else if (matType.y() == 1){
        // Specular material
        Vector3f newDir = reflect(I, N);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        return emission + color * PathTrace(scene,newRay, depth + 1) * rrFactor;
    } else if (matType.z() == 1){
        // Refractive material
        float refractiveIndex = m->getRefractiveIndex();
        Vector3f newDir = refract(I, N, refractiveIndex);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        return emission + color * PathTrace(scene, newRay, depth + 1) * rrFactor;
    } else if (matType.z() == 0){
        // Cook - Torrance material
        float kd = matType.x();
        float ks = matType.y();

        if (std::fabs(kd + ks - 1.0f) > FLOAT_EPSILON) {
        float sum = kd + ks;
        if (sum > FLOAT_EPSILON) {
            kd /= sum;
            ks /= sum;
        } else {
            kd = 1.0f;
            ks = 0.0f;
            Vector3f newDir = diffuse(I, N);
            Vector3f newOrigin = P + N * kEpsilon;
            Ray newRay(newOrigin, newDir);
            return emission + color * PathTrace(scene, newRay, depth + 1) * rrFactor;
        }
    }

        // randomly sample a direction in the hemisphere
        float r = RND1;
        Vector3f newDir;
        float pdf;
        if (r < kd) {
            newDir = diffuse(I, N);
            pdf = kd * std::max(0.0f, Vector3f::dot(N, newDir)) / M_PI; 
        } else {
            // Sample GGX 
            newDir = sampleGGX(I, N, alpha);
            Vector3f h = (newDir + I).normalized();
            float D = D_GGX(h, N, alpha);
            pdf = ks * (D * std::max(0.0f, Vector3f::dot(h, N))) / (4 * std::max(1e-6f, Vector3f::dot(-I, h)));
        }

        Ray newRay (P + N * kEpsilon, newDir);

        Vector3f h = (newDir + I).normalized();
        float cos_i = std::max(0.0f, Vector3f::dot(N, newDir));
        float cos_o = std::max(0.0f, Vector3f::dot(N, -I));

        Vector3f diffuseBRDF = color / M_PI;

        float D = D_GGX(h, N, alpha);
        Vector3f F = fresnelSchlick(std::max(0.0f, Vector3f::dot(-I, h)), F0);
        float G = G_Smith(-I, newDir, N, alpha);
        Vector3f specularBRDF = (D * F * G) / (4 * cos_i * cos_o + 1e-6f);

        Vector3f fr = kd * diffuseBRDF + ks * specularBRDF;
        Vector3f Li = PathTrace(scene, newRay, depth + 1);
        return emission + fr * Li * rrFactor * cos_i / pdf;
    } else {
        return Vector3f(0, 0, 0); 
    }
}


inline Vector3f PathTraceNEE(const SceneParser &scene, Ray ray, int depth){
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



    // NEE calculation
    Vector3f L_dir(0, 0, 0);
    for (int ei = 0; ei < scene.getNumEmissiveObjects(); ++ei) {
        Object3D *obj = scene.getEmissiveObject(ei);
        Material *mLe = obj->getMaterial();
        Vector3f wi, xN;
        float pdfA = 0.0f;

        // get sample point on the light source
        Vector3f x = obj->sampleDirect(P, wi, pdfA, xN);

        Vector3f offset = x - P;
        float dist2 = offset.squaredLength();
        float dist  = std::sqrt(dist2);

        if (pdfA <= FLOAT_EPSILON) continue;

        // shadow detection
        Ray shadow(P + N * kEpsilon, wi);
        Hit shadowHit;
        if (scene.getGroup()->intersect(shadow, shadowHit, kEpsilon) && shadowHit.getT() < dist)
            continue;

        float cosP = std::max(0.0f, Vector3f::dot(N, wi));
        float cosX = std::max(0.0f, Vector3f::dot(xN, -wi));
        Vector3f Le = obj->getEmission();
        Vector3f wo = -I;
        Vector3f fr = m->evalNeeBRDF(wo, wi, N);

        // Monte Carlo
        L_dir += Le * fr * cosP * cosX / (dist2 * pdfA);
    }

    // Russian Roulette
    double rrFactor = 1.0;
    if(depth > MAX_DEPTH){
        if (rnd() <= rrStopProb){
            return  emission + L_dir; 
        }
        rrFactor = 1.0 / (1.0 - rrStopProb);
    }


    // Indirect illumination
    Vector3f L_indir(0, 0, 0);
    if (matType.x() == 1){
        // Diffuse material
        Vector3f newDir = diffuse(I, N);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        L_indir += color * PathTraceNEE(scene, newRay, depth + 1) ;
    } else if (matType.y() == 1){
        // Specular material
        Vector3f newDir = reflect(I, N);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        L_indir += color * PathTraceNEE(scene,newRay, depth + 1) ;
    } else if (matType.z() == 1){
        // Refractive material
        float refractiveIndex = m->getRefractiveIndex();
        Vector3f newDir = refract(I, N, refractiveIndex);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        L_indir += color * PathTraceNEE(scene, newRay, depth + 1);
    } else if (matType.z() == 0){
        // Cook - Torrance material
        float kd = matType.x();
        float ks = matType.y();

        if (std::fabs(kd + ks - 1.0f) > FLOAT_EPSILON) {
            float sum = kd + ks;
            if (sum > FLOAT_EPSILON) {
                kd /= sum;
                ks /= sum;
            } else {
                kd = 1.0f;
                ks = 0.0f;
                Vector3f newDir = diffuse(I, N);
                Vector3f newOrigin = P + N * kEpsilon;
                Ray newRay(newOrigin, newDir);
                L_indir += color * PathTraceNEE(scene, newRay, depth + 1);
            }
        }

        // randomly sample a direction in the hemisphere
        float r = rnd();
        Vector3f newDir;
        float pdf;
        if (r < kd) {
            newDir = diffuse(I, N);
            pdf = kd * std::max(0.0f, Vector3f::dot(N, newDir)) / M_PI; 
        } else {
            // Sample GGX 
            newDir = sampleGGX(I, N, alpha);
            Vector3f h = (newDir + I).normalized();
            float D = D_GGX(h, N, alpha);
            pdf = ks * (D * std::max(0.0f, Vector3f::dot(h, N))) / (4 * std::max(1e-6f, Vector3f::dot(-I, h)));
        }

        Ray newRay (P + N * kEpsilon, newDir);

        Vector3f h = (newDir + I).normalized();
        float cos_i = std::max(0.0f, Vector3f::dot(N, newDir));
        float cos_o = std::max(0.0f, Vector3f::dot(N, -I));

        Vector3f diffuseBRDF = color / M_PI;

        float D = D_GGX(h, N, alpha);
        Vector3f F = fresnelSchlick(std::max(0.0f, Vector3f::dot(-I, h)), F0);
        float G = G_Smith(-I, newDir, N, alpha);
        Vector3f specularBRDF = (D * F * G) / (4 * cos_i * cos_o + 1e-6f);

        Vector3f fr = kd * diffuseBRDF + ks * specularBRDF;
        Vector3f Li = PathTraceNEE(scene, newRay, depth + 1);
        L_indir += Li * fr * cos_i / pdf;
    } else {
        L_indir += 0;
    }

    return emission + L_dir + L_indir * rrFactor;
}


inline Vector3f PathTraceMIS(const SceneParser &scene, Ray ray, int depth){
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


    // NEE calculation
    Vector3f L_dir(0, 0, 0);

    //  策略 A：直接从发光物体采样
    for (int ei = 0; ei < scene.getNumEmissiveObjects(); ++ei) {
        Object3D *obj = scene.getEmissiveObject(ei);
        Vector3f wi, xN;
        float pdfA = 0.0f;

        // 从发光面面积采样一个点 x
        Vector3f x = obj->sampleDirect(P, wi, pdfA, xN);
        if (pdfA < FLOAT_EPSILON) continue;

        Vector3f offset = x - P;
        float dist2 = offset.squaredLength();
        float dist  = std::sqrt(dist2);

        // 阴影检测
        Ray shadow(P + N * kEpsilon, wi);
        Hit shadowHit;
        if (scene.getGroup()->intersect(shadow, shadowHit, kEpsilon)
            && shadowHit.getT() < dist) continue;

        float cosP = std::max(0.0f, Vector3f::dot(N, wi));
        float cosX = std::max(0.0f, Vector3f::dot(xN, -wi));
        Vector3f Le  = obj->getEmission();

        // 计算如果用 BSDF 采样到这个同一方向 wi 时的 pdf_B
        float pdfB = m->pdfBsdf(-I, wi, N);  

        // 计算相同测度下的权重
        float pdfA_dir = pdfA * dist2 / std::max(1e-6f, cosX);
        float wA = (pdfA_dir) / (pdfA_dir + pdfB + 1e-12f);

        // NEE 原始估计 * 权重
        Vector3f fr  = m->evalNeeBRDF(-I, wi, N);
        L_dir += Le * fr * cosP * cosX / (dist2 * pdfA) * wA;
    }

    // 策略 B：BSDF 采样
    // 从当前材质的采样器里抽一个 newDir
    Vector3f newDir;
    float pdfB = m->sampleBsdf(-I, N, newDir);  // BSDF 采样完成
    if (pdfB > FLOAT_EPSILON) {
        // 构造射线，看它是否打到了某个发光物体
        Ray bsdfRay(P + N * kEpsilon, newDir);
        Hit bsdfHit;
        if (scene.getGroup()->intersect(bsdfRay, bsdfHit, kEpsilon)) {
            Object3D *hitObj = bsdfHit.getObject();      
            Vector3f Le = hitObj->getEmission();
            if (Le.length() > 0.0f) {
                float cosP = std::max(0.0f, Vector3f::dot(N, newDir));

                // 再次对光源采样
                Vector3f wi_p, xN_p;
                float pdfA_area_p;
                Vector3f x_p = hitObj->sampleDirect(P, wi_p, pdfA_area_p, xN_p);
                float cosX_p    = std::max(0.0f, Vector3f::dot(xN_p, -wi_p));
                float dist2_p   = (x_p - P).squaredLength();

                // 转换到相同测度，计算权重
                float pdfA_dir_p = pdfA_area_p * dist2_p / std::max(1e-6f, cosX_p);
                float wB = (pdfB) / (pdfA_dir_p + pdfB + 1e-12f);

                // 材质 BRDF
                Vector3f fr = m->evalNeeBRDF(-I, newDir, N);

                // BSDF 采样估计：Le * fr * cosθ / pdfB * wB
                L_dir += Le * fr * cosP / pdfB * wB;
            } else L_dir += 0;
        }
    }

    // Russian Roulette
    double rrFactor = 1.0;
    if(depth > MAX_DEPTH){
        if (rnd() <= rrStopProb){
            return  emission + L_dir; 
        }
        rrFactor = 1.0 / (1.0 - rrStopProb);
    }


    // Indirect illumination
    Vector3f L_indir(0, 0, 0);
    if (matType.x() == 1){
        // Diffuse material
        Vector3f newDir = diffuse(I, N);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        L_indir += color * PathTraceMIS(scene, newRay, depth + 1) ;
    } else if (matType.y() == 1){
        // Specular material
        Vector3f newDir = reflect(I, N);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        L_indir += color * PathTraceMIS(scene,newRay, depth + 1) ;
    } else if (matType.z() == 1){
        // Refractive material
        float refractiveIndex = m->getRefractiveIndex();
        Vector3f newDir = refract(I, N, refractiveIndex);
        Vector3f newOrigin = P + N * kEpsilon;
        Ray newRay(newOrigin, newDir);
        L_indir += color * PathTraceMIS(scene, newRay, depth + 1);
    } else if (matType.z() == 0){
        // Cook - Torrance material
        float kd = matType.x();
        float ks = matType.y();

        if (std::fabs(kd + ks - 1.0f) > FLOAT_EPSILON) {
            float sum = kd + ks;
            if (sum > FLOAT_EPSILON) {
                kd /= sum;
                ks /= sum;
            } else {
                kd = 1.0f;
                ks = 0.0f;
                Vector3f newDir = diffuse(I, N);
                Vector3f newOrigin = P + N * kEpsilon;
                Ray newRay(newOrigin, newDir);
                L_indir += color * PathTraceMIS(scene, newRay, depth + 1);
            }
        }

        // randomly sample a direction in the hemisphere
        float r = rnd();
        Vector3f newDir;
        float pdf;
        if (r < kd) {
            newDir = diffuse(I, N);
            pdf = kd * std::max(0.0f, Vector3f::dot(N, newDir)) / M_PI; 
        } else {
            // Sample GGX 
            newDir = sampleGGX(I, N, alpha);
            Vector3f h = (newDir + I).normalized();
            float D = D_GGX(h, N, alpha);
            pdf = ks * (D * std::max(0.0f, Vector3f::dot(h, N))) / (4 * std::max(1e-6f, Vector3f::dot(-I, h)));
        }

        Ray newRay (P + N * kEpsilon, newDir);

        Vector3f h = (newDir + I).normalized();
        float cos_i = std::max(0.0f, Vector3f::dot(N, newDir));
        float cos_o = std::max(0.0f, Vector3f::dot(N, -I));

        Vector3f diffuseBRDF = color / M_PI;

        float D = D_GGX(h, N, alpha);
        Vector3f F = fresnelSchlick(std::max(0.0f, Vector3f::dot(-I, h)), F0);
        float G = G_Smith(-I, newDir, N, alpha);
        Vector3f specularBRDF = (D * F * G) / (4 * cos_i * cos_o + 1e-6f);

        Vector3f fr = kd * diffuseBRDF + ks * specularBRDF;
        Vector3f Li = PathTraceMIS(scene, newRay, depth + 1);
        L_indir += Li * fr * cos_i / pdf;
    } else {
        L_indir += 0;
    }

    return emission + L_dir + L_indir * rrFactor;
}