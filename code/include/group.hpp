#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>


// TODO: Implement Group - add data structure to store a list of Object*
class Group : public Object3D {

public:

    Group() {

    }

    explicit Group (int num_objects) : objList(num_objects){}

    ~Group() override {

    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        bool hitObj = false;
        for(auto obj : objList){
            if (obj){
                if(obj->intersect(r, h, tmin)){
                    hitObj = true;
                }
            }
        }
        return hitObj;
    }

    int getType() const override {
        return 7;
    }

    void addObject(int index, Object3D *obj) {
        assert(index >= 0 && index < (int)objList.size());
        objList[index] = obj;
    }  

    int getGroupSize() {
        return objList.size();
    }

    Object3D *getObject(int index) {
        if (index < 0 || index >= objList.size()) {
            std::cerr << "Index out of bounds in Group::getObject" << std::endl;
            return nullptr;
        }
        return objList[index];
    }
    
    Vector3f sampleDirect(const Vector3f &p, Vector3f &outDir, float &pdfA, Vector3f &xNormal) const override {
        pdfA = 0.0f;
        return Vector3f::ZERO;
    }

private:
    std::vector<Object3D *> objList;
};

#endif
	
