#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// TODO: implement this class and add more fields as necessary,
class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
		vertices[0] = a;
		vertices[1] = b;
		vertices[2] = c;
		normal = Vector3f::cross(b-a, c-a).normalized();
	}
	~Triangle() override = default;

	bool intersect( const Ray& ray,  Hit& hit , float tmin) override {
		const Vector3f &v0 = vertices[0];
		const Vector3f &v1 = vertices[1];
		const Vector3f &v2 = vertices[2];

		Vector3f edge1 = v1 - v0;
		Vector3f edge2 = v2 - v0;

		Vector3f pvec = Vector3f::cross(ray.getDirection(), edge2);
		float det = Vector3f::dot(edge1, pvec);

		if(std::fabs(det) <1e-6){
			return false;
		}

		float inv_det = 1.0f / det;

		Vector3f tvec = ray.getOrigin() - v0;
		float u = Vector3f::dot(tvec, pvec) * inv_det; 

		if (u < 0.0f || u> 1.0f){
			return false;
		}

		Vector3f qvec = Vector3f::cross(tvec, edge1);
		float v = Vector3f::dot(ray.getDirection(), qvec) * inv_det;
		if (v < 0.0f || (u + v) > 1.0f){
			return false;
		}

		float t_candidate = Vector3f::dot(edge2, qvec) * inv_det;

		if(t_candidate < tmin || t_candidate > hit.getT()){
            return false;
        }
        hit.set(t_candidate, material, normal);
        return true;
	}

	Vector3f normal;
	Vector3f vertices[3];
};

#endif //TRIANGLE_H
