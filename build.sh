#!/bin/bash

echo "Building the Lidar Detection Program"
mkdir build || true
cd build

cmake ..
make 

cd ..
