#pragma once
#include <random>
#include <cstdlib>
#include <cmath>

static std::default_random_engine rnd_engine(std::random_device{}());



#define MAX_DEPTH 5
#define PT_ITERATIONS 2000  // Number of iterations for Path Tracing
#define RT_ITERATIONS 10 // Number of iterations for Ray Tracing
const float EPLISON = 1e-5f;
const float kEpsilon = 1e-4f;
const float FLOAT_EPSILON = 1e-4f; // Floating point epsilon for numerical stability
const float RussianRoulette = 0.8f; // Russian Roulette threshold
const double rrStopProb = 0.1; // Russian Roulette stop probability
const int NewtonDepth = 20; // Maximum iterations for Newton's method

#define RND0 (std::uniform_real_distribution<float>(-0.5f, 0.5f)(rnd_engine))   // [-0.5, 0.5]
#define RND1 (std::uniform_real_distribution<float>(0.0f, 1.0f)(rnd_engine))    // [0.0, 1.0]
#define RND2 (std::uniform_real_distribution<float>(-1.0f, 1.0f)(rnd_engine))   // [-1.0, 1.0]
