#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <omp.h>
#include <random>
#include <atomic>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "RayTracer.hpp"
#include "PathTracer.hpp"
#include "random_utils.hpp"

#include <string>

using namespace std;

static constexpr int PROG_STEPS = 100;


int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc < 3 || argc > 4) {
        cout << "Usage: ./PA1 <scene.txt> <out.bmp> [RT|PT|NEE|MIS|CT]\n";
        return 1;
    }
    string inputFile  = argv[1];
    string outputFile = argv[2];
    string mode       = (argc==4 ? argv[3] : "PT");

    SceneParser sceneParser(inputFile.c_str());
    Camera *camera = sceneParser.getCamera();
    int width = camera->getWidth();
    int height = camera->getHeight();

    Image image(width, height);

    // TODO: Main RayCasting Logic
    // First, parse the scene using SceneParser.
    // Then loop over each pixel in the image, shooting a ray
    // through that pixel and finding its intersection with
    // the scene.  Write the color at the intersection to that
    // pixel in your output image.
    int64_t totalPixels = int64_t(width) * height;
    int64_t step = totalPixels / PROG_STEPS;  // 每 1% 打印一次
    int64_t counted = 0;

    #pragma omp parallel
    {   
        initRNG(omp_get_thread_num());

        #pragma omp for schedule(dynamic,1)
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {

                Vector3f finalColor(0,0,0);
                int iterations = (mode == "RT") ? RT_ITERATIONS : PT_ITERATIONS;

                
                for (int i = 0; i < iterations; ++i) {
                    Ray camRay = camera->generateRay(Vector2f(x + rnd(), y + rnd()));
                    if (mode == "RT")       finalColor += RayTrace(sceneParser, camRay, 0, 1.0f);
                    else if (mode == "PT")  finalColor += PathTrace(sceneParser, camRay, 0);
                    else if (mode == "NEE") {
                        finalColor += PathTraceNEE(sceneParser, camRay, 0);
                        
                    }
                    else if (mode == "MIS") finalColor += PathTraceMIS(sceneParser, camRay, 0);
                    else if (mode == "CT")  finalColor += PathTrace(sceneParser, camRay, 0);
                    else if (mode == "BSC") finalColor += PathTraceBasic(sceneParser, camRay, 0);
                    else if (mode == "FRE") finalColor += PathTraceFre(sceneParser, camRay, 0);
                }
                image.SetPixel(x, y, finalColor / iterations);
            
            }

            #pragma omp critical
            {
                static int rowsDone = 0;
                ++rowsDone;
                int perc = rowsDone * 100 / height;
                std::cout << "\rProgress: " << perc << "% " << std::flush;
            }
        }
    }
    
    
    std::cout << "\rProgress: 100%   " << std::endl;

    image.SaveBMP(outputFile.c_str());
    cout << "Rendered image saved to " << outputFile << endl;
    return 0;
}

