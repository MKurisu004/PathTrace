#!/usr/bin/env bash

# 第一个参数：模式 RT|PT|NEE|MIS|CT
MODE=${1:-PT}

# CMake 构建
CC=clang
CXX=clang++
cmake -B build
cmake --build build

mkdir -p output

case "$MODE" in
  RT)
    SCENE=testcases/RT_BASIC.txt
    OUT=output/RT_BASIC.bmp
    ;;
  PT)
    SCENE=testcases/PT_FRE.txt
    OUT=output/PT_COS.bmp
    ;;
  NEE)
    SCENE=testcases/PT_NEE.txt
    OUT=output/PT_NEE.bmp
    ;;
  MIS)
    SCENE=testcases/PT_BASIC.txt
    OUT=output/PT_MIS.bmp
    ;;
  CT)
    SCENE=testcases/PT_CTMat.txt
    OUT=output/PT_CTMat.bmp
    ;;
  BSC)
    SCENE=testcases/PT_NEE.txt
    OUT=output/PT_BASIC.bmp
    ;;
  FRE)
    SCENE=testcases/PT_FRE.txt
    OUT=output/PT_FRE.bmp
    ;;
  *)
    echo "Unknown mode '$MODE'. Valid are RT, PT, NEE, MIS, CT, BSC, FRE."
    exit 1
    ;;
esac

echo "Running mode $MODE → $SCENE → $OUT"
build/PA1 "$SCENE" "$OUT" "$MODE"