#!/bin/bash
set -ex

echo "========================================================================="
echo "Generating cmake project"
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=${BUILD_MODE}

echo "========================================================================="
echo "Building project"
cmake --build .
