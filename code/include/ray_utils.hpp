#pragma once
#include "material.hpp"
#include "random_utils.hpp"

inline float clamp(float low, float high, float x) {
    return std::max(low, std::min(high, x));
}

// 从局部坐标 (x, y, z) 构建世界向量
inline static Vector3f toWorld(const Vector3f &local, const Vector3f &N) {
    Vector3f T, B;
    if (std::fabs(N.x()) > std::fabs(N.y()))
        T = Vector3f(N.z(), 0, -N.x()).normalized();
    else
        T = Vector3f(0, N.z(), -N.y()).normalized();
    B = Vector3f::cross(T, N);
    return (local.x() * T + local.y() * B + local.z() * N).normalized();
}

inline Vector3f diffuse(const Vector3f &I, const Vector3f &N){
    // 随机采样两个均匀分布的随机数
    float r1 = rnd();  // ∈ [0, 1]
    float r2 = rnd();  // ∈ [0, 1]

    // 将它们变换成余弦加权半球分布
    float phi = 2.0f * M_PI * r1;
    float x = cosf(phi) * sqrtf(1 - r2);
    float y = sinf(phi) * sqrtf(1 - r2);
    float z = sqrtf(r2); // 越接近法线方向越多

    // 构建局部坐标系
    Vector3f tangent, bitangent;
    if (fabs(N.x()) > fabs(N.y()))
        tangent = Vector3f(N.z(), 0, -N.x()).normalized();
    else
        tangent = Vector3f(0, N.z(), -N.y()).normalized();
    bitangent = Vector3f::cross(tangent, N);

    // 从局部向量 (x, y, z) 变换到世界坐标系
    return (x * tangent + y * bitangent + z * N).normalized();
};

inline Vector3f reflect(const Vector3f &I, const Vector3f &N){
    return (I - 2 * Vector3f::dot(I, N) * N).normalized();
};

inline Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) {
    float cosi = clamp(-1.0f, 1.0f, Vector3f::dot(I, N));  // 余弦入射角
    float etai = 1.0f;  // 默认入射介质为空气
    float etat = ior;   // 折射率

    Vector3f n = N;  // 默认法线方向

    // 如果 cosi > 0，说明 I 和 N 同向，说明是从物体内部出来的，需要交换介质
    if (cosi > 0.0f) {
        std::swap(etai, etat);
        n = -N;
        cosi = -cosi;
    }

    float eta = etai / etat;
    float k = 1.0f - eta * eta * (1.0f - cosi * cosi);

    // 如果 k < 0，说明发生全反射，返回一个反射方向（或ZERO）
    if (k < 0.0f)
        return reflect(I, N);

    return (eta * I + (eta * cosi - std::sqrt(k)) * n).normalized();
};

inline Vector3f sampleGGX(const Vector3f &I, const Vector3f &N, float alpha) {
    // 1) 两个均匀随机数
    float u1 = rnd();
    float u2 = rnd();

    // 2) GGX α^2
    float a2 = alpha * alpha;

    // 3) 采样半向量的极角 θ_h（根据 GGX 分布的逆 CDF）
    //    CDF^{-1}(u2) 推导可得:
    float phi    = 2.0f * M_PI * u1;
    float cos2   = (1.0f - u2) / (1.0f + (a2 - 1.0f) * u2);
    float cosh   = std::sqrt(cos2);
    float sinh   = std::sqrt(1.0f - cos2);

    // 4) 局部半向量 h_local，再转换到世界
    Vector3f h_local(sinh * std::cos(phi), sinh * std::sin(phi), cosh);
    Vector3f h = toWorld(h_local, N);

    // 5) 针对视向量 I 做镜面反射，得到采样的入射方向 ω_i
    Vector3f wi = I - 2.0f * Vector3f::dot(I, h) * h;
    return wi.normalized();
};

inline float D_GGX(const Vector3f &h, const Vector3f &N, float alpha) {
    float a2 = alpha * alpha;
    float cosTheta = std::max(0.0f, Vector3f::dot(h, N));
    float cosTheta2 = cosTheta * cosTheta;
    float denom = (cosTheta2 * (a2 - 1.0f) + 1.0f);
    denom = M_PI * denom * denom;
    return a2 / (denom + 1e-6f); 
};

inline float GSchlickGGX(float cosTheta, float alpha) {
    float nom = cosTheta;
    float demon = cosTheta * (1.0f - alpha) + alpha;
    return nom / (demon + 1e-6f);
};

inline float G_Smith(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, float alpha){
    float cos_wi = std::max(0.0f, Vector3f::dot(wi, N));
    float cos_wo = std::max(0.0f, Vector3f::dot(wo, N));
    float G1_wi = GSchlickGGX(cos_wi, alpha);
    float G1_wo = GSchlickGGX(cos_wo, alpha);
    return G1_wi * G1_wo;
};

inline Vector3f fresnelSchlick(float cosTheta, const Vector3f &F0){
    return F0 + (Vector3f(1.0f, 1.0f, 1.0f) - F0) * std::pow(1.0f - cosTheta, 5.0f);
};

