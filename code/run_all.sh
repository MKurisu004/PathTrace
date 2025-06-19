#!/usr/bin/env bash

# 第一个参数：模式 RT|PT|NEE|MIS|CT
MODE=${1:-PT}

# CMake 构建
CC=/opt/homebrew/opt/llvm/bin/clang \
CXX=/opt/homebrew/opt/llvm/bin/clang++ \
cmake -B build
cmake --build build

mkdir -p output

case "$MODE" in
  RT)
    SCENE=testcases/RT_BASIC.txt
    OUT=output/RT_BASIC.bmp
    ;;
  PT)
    SCENE=testcases/PT_BASIC.txt
    OUT=output/PT_BASIC.bmp
    ;;
  NEE)
    SCENE=testcases/PT_BASIC.txt
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
  *)
    echo "Unknown mode '$MODE'. Valid are RT, PT, NEE, MIS, CT."
    exit 1
    ;;
esac

echo "Running mode $MODE → $SCENE → $OUT"
build/PA1 "$SCENE" "$OUT" "$MODE"