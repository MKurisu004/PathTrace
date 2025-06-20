#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"


class Mesh : public Object3D {

public:
    Mesh(const char *filename, Material *m);

    struct TriangleIndex {
        TriangleIndex() {
            x[0] = 0; 
            x[1] = 0; 
            x[2] = 0;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3]{};
    };

    std::vector<TriangleIndex> vIndex;  // 顶点索引
    std::vector<TriangleIndex> vtIndex; // 纹理索引
    std::vector<TriangleIndex> vnIndex; // 顶点法线索引
    std::vector<Object3D*> triangles;   // 三角形面片
    std::vector<Vector3f> v;            // 顶点坐标
    std::vector<Vector2f> vt;           // 顶点纹理
    std::vector<Vector3f> vn;           // 顶点法线
    std::vector<Vector3f> n;            // 三角形法线

    bool intersect(const Ray &r, Hit &h, float tmin) override;

    Vector3f sampleDirect(const Vector3f &p, Vector3f &outDir, float &pdfA, Vector3f &xNormal) const override {
        pdfA = 0.0f;
        return Vector3f::ZERO;
    }

private:

    // Normal can be used for light estimation
    void computeNormal();
    void setTexture();
    void setNormal();
};

#endif
