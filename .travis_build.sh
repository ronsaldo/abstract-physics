#!/bin/bash

echo "========================================================================="
echo "Generating cmake project"
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=${BUILD_MODE} || exit 1

echo "========================================================================="
echo "Building project"
make || exit 1
