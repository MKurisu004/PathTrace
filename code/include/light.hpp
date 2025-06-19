#ifndef LIGHT_H
#define LIGHT_H

#include <Vector3f.h>
#include <iostream>
#include "object3d.hpp"

class Light {
public:
    Light() = default;

    virtual ~Light() = default;

    virtual void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const = 0;
    virtual Vector3f sampleDirect(const Vector3f &p, Vector3f &outDir, float &pdfA, Vector3f &xNormal) const = 0;
    virtual Vector3f getEmission() const = 0;
    virtual Vector3f getPosition() const = 0;

    Vector3f getType(){
        return type;
    }

protected:
    Vector3f type;
};


class DirectionalLight : public Light {
public:
    DirectionalLight() = delete;

    DirectionalLight(const Vector3f &d, const Vector3f &c) {
        direction = d.normalized();
        color = c;
        type = Vector3f(1, 0, 0);
    }

    ~DirectionalLight() override = default;

    ///@param p unsed in this function
    ///@param distanceToLight not well defined because it's not a point light
    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = -direction;
        col = color;
    }

    Vector3f getPosition() const override{
        return -direction * 1e30f;
    }

    Vector3f sampleDirect(const Vector3f &p, Vector3f &outDir, float &pdfA, Vector3f &xNormal) const override {
        outDir = -direction;  
        pdfA = 1.0f;
        xNormal = direction;  
        return direction;
    }

    Vector3f getEmission() const override {
        return color;
    }

private:

    Vector3f direction;
    Vector3f color;

};

class PointLight : public Light {
public:
    PointLight() = delete;

    PointLight(const Vector3f &p, const Vector3f &c) {
        position = p;
        color = c;
        type = Vector3f(0, 1, 0);
    }

    ~PointLight() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = (position - p);
        dir = dir / dir.length();
        col = color;
    }

    Vector3f getPosition() const override{
        return position;
    }

    
    Vector3f sampleDirect(const Vector3f &p, Vector3f &outDir, float &pdfA, Vector3f &xNormal) const override {
        outDir = (position - p).normalized();
        pdfA = 1.0f; 
        xNormal = Vector3f::UP; 
        return position;
    }

    Vector3f getEmission() const override {
        return color; 
    }

private:

    Vector3f position;
    Vector3f color;

};

class AreaLight : public Light {
public:
    // o = center, u,v span the rectangle, lu,lv 是半长
    AreaLight(const Vector3f &o,
              const Vector3f &u, float lu,
              const Vector3f &v, float lv,
              const Vector3f &emission)
      :  origin(o),
         U(u.normalized()), LU(lu),
         V(v.normalized()), LV(lv),
         emission(emission)
    {
        // 预计算面积
        type = Vector3f(0, 0, 1);
        area = (2*LU) * (2*LV) * Vector3f::cross(U, V).normalized().length();
    }

    // Whitted-Style 还是要保留旧接口
    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // 这里简单返回看向中心的方向 + 发光强度 / area
        dir = (origin - p).normalized();
        col = emission * area;  
    }

    Vector3f getEmission() const override {
        return emission;
    }

    Vector3f getPosition() const override{
        return origin;
    }


    // NEE 专用：均匀采样一个面上的点 x
    Vector3f sampleDirect(const Vector3f &p,
                          Vector3f &outDir,
                          float &pdfA,
                          Vector3f &xNormal) const override
    {
        // 在 [-LU,LU]×[-LV,LV] 上均匀采 u',v'
        float ru = rnd()*2 - 1; // ∈ [-1,1]
        float rv = rnd()*2 - 1;
        Vector3f x = origin + U*(ru*LU) + V*(rv*LV);

        // 向量和法线
        outDir = (x - p).normalized();
        xNormal = Vector3f::cross(U, V).normalized();  // 矩形平面法线

        // 面积 pdf = 1 / area
        pdfA = 1.0f / area;

        return x;
    }

private:
    Vector3f origin, U, V, emission;
    float LU, LV, area;
};

#endif // LIGHT_H
