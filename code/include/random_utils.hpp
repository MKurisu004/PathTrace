// random_utils.hpp
#pragma once
#include <random>

// 线程私有的 RNG
inline thread_local std::mt19937 rng;
// 均匀分布 [0,1)
inline thread_local std::uniform_real_distribution<float> uniDist(0.0f, 1.0f);

// 由 main.cpp 在并行区调用一次
inline void initRNG(int seedOffset) {
    int tid = seedOffset;  // 你可以传 omp_get_thread_num()
    rng.seed(static_cast<unsigned>(time(nullptr)) + tid);
}

// 全局可见的 rnd()
inline float rnd() {
    return uniDist(rng);
}