#!/bin/bash

echo "Building the Lidar Detection Program"
mkdir build || true
cd build

cmake -DCMAKE_BUILD_TYPE=Release ..
make 

cd ..
