#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include "random_utils.hpp"
#include "ray_utils.hpp"

#include <iostream>

// : Implement Shade function that computes Phong introduced in class.
class Material {
public:

    explicit Material(const Vector3f &c, const Vector3f &s_color = Vector3f::ZERO, float s = 0, 
        const Vector3f &emission = Vector3f::ZERO, float alpha = 0, const Vector3f &type_ = Vector3f(1, 0, 0), float refractiveIndex_ = 1.0f) : 
            color(c), specularColor(s_color), shininess(s), emission(emission), alpha(alpha), type(type_), refractiveIndex(refractiveIndex_) {

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
        float F0_scalar = (1.0f - refractiveIndex) / (1.0f + refractiveIndex);
        F0_scalar = F0_scalar * F0_scalar;
        return Vector3f(F0_scalar, F0_scalar, F0_scalar);
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

    inline Vector3f evalNeeBRDF(const Vector3f &wo, const Vector3f &wi, const Vector3f &N) const {
        if (type.x() == 1){         // Diffuse material
            return color * (1.0f / M_PI);
        } else if (type.y() == 1){  // Specular material
            return Vector3f::ZERO;
        } else if (type.z() == 1){  // Refractive material
             return Vector3f::ZERO;
        } else if (type.z() == 0){  // Cook-Torrance material
            float kd = type.x();
            float ks = type.y();

            Vector3f h = (wi + wo).normalized();
            float cos_i = std::max(0.0f, Vector3f::dot(N, wi));
            float cos_o = std::max(0.0f, Vector3f::dot(N, wo));

            Vector3f diffuseBRDF = color / M_PI;

            float D = D_GGX(h, N, alpha);
            Vector3f F = fresnelSchlick(std::max(0.0f, Vector3f::dot(wo, h)), this->getF0());
            float G = G_Smith(wo, wi, N, alpha);
            Vector3f specularBRDF = (D * F * G) / (4 * cos_i * cos_o + 1e-6f);

            Vector3f fr = kd * diffuseBRDF + ks * specularBRDF;
            return fr;
        }
        return Vector3f::ZERO; // 如果没有匹配的类型，返回零向量
    }

protected:
    Vector3f color;             // 自身颜色
    Vector3f specularColor;     // 镜面反射颜色
    float shininess;            // 高光指数
    float refractiveIndex;      // 折射率
    Vector3f emission;        // 发光系数
    float alpha;              // 用于 Cook-Torrance 模型的粗糙度
    Vector3f type;              // 材质类型

    float relu(float x){
        return std::max((float)0, x);
    }
};


#endif // MATERIAL_H
