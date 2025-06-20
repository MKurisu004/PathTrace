#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include "random_utils.hpp"
#include "ray_utils.hpp"
#include "texture.hpp"

#include <iostream>

// : Implement Shade function that computes Phong introduced in class.
class Material {
public:

    explicit Material(const Vector3f &c, const Vector3f &s_color = Vector3f::ZERO, float s = 0, 
        const Vector3f &emission = Vector3f::ZERO, float alpha = 0, const Vector3f &type_ = Vector3f(1, 0, 0), 
        float refractiveIndex_ = 1.0f, float R0 = 0.0f, const char *filename = "") : 
            color(c), specularColor(s_color), shininess(s), emission(emission), alpha(alpha), type(type_), 
            refractiveIndex(refractiveIndex_), R0(R0), texture(filename){
                if(std::strcmp(filename, "") != 0) hasTexture = true;
                else hasTexture = true;
    }

    virtual ~Material() = default;

    virtual Vector3f getDiffuseColor() const {
        return color;
    }

    // set type
    void setType(const Vector3f &t){
        type = t;
    }

    // 设置折射率
    void setRefractiveIndex(float ri) {
        refractiveIndex = ri;
    }

    // 获取类型
    const Vector3f &getType() const {
        return type;
    }

    // 获取折射率
    float getRefractiveIndex() const {
        return refractiveIndex;
    }

    // 获取颜色
    Vector3f getColor() const {
        return color;
    }

    // 获取发光系数
    Vector3f getEmission() const {
        return emission;
    }

    // get roughness for Cook-Torrance model
    float getAlpha() const {
        if(alpha <= 1e-4f) {
            return std::sqrt(2.0f / (shininess + 2.0f));
        }
        return alpha;
    }

    // get F0 for Fresnel equation
    Vector3f getF0() const {
        if(type.x() == 1.0f){
            float F0_scalar = (1.0f - refractiveIndex) / (1.0f + refractiveIndex);
            F0_scalar = F0_scalar * F0_scalar;
            return Vector3f(F0_scalar, F0_scalar, F0_scalar);
        } else {
            return R0;
        }
    }

    // check if the material is a light source
    bool isLight() const{
        return emission != Vector3f::ZERO;
    }

    Vector3f phongShade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor) {
        Vector3f N = hit.getNormal();
        Vector3f V = -ray.getDirection().normalized();
        Vector3f L = dirToLight.normalized();
        Vector3f R = (2 * Vector3f::dot(L, N) * N - L).normalized();
       
        Vector3f shaded = lightColor * (color * relu(Vector3f::dot(L, N)) + specularColor * (pow(relu(Vector3f::dot(V, R)), shininess)));
        return shaded;
    }

    inline Vector3f evalNeeBRDF(const Vector3f &wo,
                            const Vector3f &wi,
                            const Vector3f &N) const {
    // 漫反射：任意 wi 都有反射
    if (type.x() == 1) {
        return color * (1.0f / M_PI);
    }
    // 镜面
    if (type.y() == 1) {
        return Vector3f::ZERO;
    }
    // 折射
    if (type.z() == 1) {
        return Vector3f::ZERO;
    }
    return Vector3f::ZERO;
}

    inline float pdfBsdf(const Vector3f &wo, const Vector3f &wi, const Vector3f &N) const {
        if (type.x() == 1.0f) {
            return std::max(0.0f, Vector3f::dot(wi, N)) / M_PI;
        } else if (type.y() == 1.0f || type.z() == 1.0f){
            return 1.0f;
        } else {
            float kd = type.x();
            float ks = type.y();

            float pdf_diff = std::max(0.0f, Vector3f::dot(wi, N)) / M_PI;

            Vector3f h = (wo + wi).normalized();

            float D  = D_GGX(h,   N, alpha);
            float cos_i = std::max(0.0f, Vector3f::dot(h, N));
            float odoth = std::abs(Vector3f::dot(wo, h));
            float pdf_spec = D  * cos_i / (4.0f * std::max(1e-6f, odoth));

            return kd * pdf_diff + ks * pdf_spec;
        }
    }

    inline float sampleBsdf(const Vector3f &wo, const Vector3f &N, Vector3f &outDir){
        if (type.x() == 1.0f) {
            outDir = diffuse(wo, N);
            return std::max(0.0f, Vector3f::dot(N, outDir)) / M_PI;
        } else if (type.y() == 1.0f) {
            outDir = reflect(wo, N);
            return 1.0f;
        } else if (type.z() == 1.0f){
            outDir = refract(wo, N, refractiveIndex);
            return 1.0f;
        } else {
            float kd = type.x();
            float ks = type.y();
            float r = rnd();
            if (r < kd) {
                outDir = diffuse(wo, N);
                return kd * (std::max(0.0f, Vector3f::dot(N, outDir)) / M_PI);
            } else {
                outDir = sampleGGX(-wo, N, alpha);
                Vector3f h = (wo + outDir).normalized();

                float D  = D_GGX(h,   N, alpha);
                float cos_i = std::max(0.0f, Vector3f::dot(h, N));
                float odoth = std::abs(Vector3f::dot(wo, h));
                float pdf_spec = D  * cos_i / (4.0f * std::max(1e-6f, odoth));

                return ks * pdf_spec;
            }
        }
    }

protected:
    Vector3f color;             // 自身颜色
    Vector3f specularColor;     // 镜面反射颜色
    float shininess;            // 高光指数
    float refractiveIndex;      // 折射率
    Vector3f emission;          // 发光系数
    float alpha;                // 用于 Cook-Torrance 模型的粗糙度
    Vector3f type;              // 材质类型
    float R0;                   // 正入射反射系数
    Texture texture;
    bool hasTexture;

    float relu(float x){
        return std::max((float)0, x);
    }
};


#endif // MATERIAL_H
