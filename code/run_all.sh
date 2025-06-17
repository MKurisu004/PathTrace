#!/usr/bin/env bash
CC=/opt/homebrew/opt/llvm/bin/clang \
CXX=/opt/homebrew/opt/llvm/bin/clang++ \
cmake -B build
cmake --build build

# 创建输出目录
mkdir -p output

# 如果传入参数 "lldb"，则只使用 lldb 调试第一个测试用例（你也可以按照需要修改调试目标）
if [ "$1" == "lldb" ]; then
    echo "Running under lldb: testcases/scene01_basic.txt"
    # 使用 lldb 启动可执行文件，注意 -- 确保后面的参数传给被调试程序
    lldb -- build/PA1 -- testcases/scene01_basic.txt output/scene01.bmp
else
    # 依次运行所有测试用例
    build/PA1 testcases/scene01_basic.txt output/scene01.bmp
fi