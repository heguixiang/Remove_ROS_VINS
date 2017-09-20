#/bin/bash
cd camera_model
mkdir -p build
cd build
cmake ..
make 

cd ../../vins_estimator
mkdir -p build
cd build 
cmake ..
make

