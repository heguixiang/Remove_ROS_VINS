# This  repositories is a version of HKUST-Aerial-Robotics/VINS-Mono without ROS mechanism. The project can be passed on the euroc dataset so far. [EuRoc](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) ASL data format, this version can run both on 32bit OS and 64 bit OS platform.

# thanks to the reference program： 
# [HKUST-Aerial-Robotics/VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
# What need to be installed
```
  1. Ceres Solver
  2. Opencv 3.1
  3. Eigen 3.2.0
  4. boost
  5. Pangolin
```
# How to build:
```
  1. mkdir VINS_Workspace
  2. cd VINS_Workspace
  3. git clone git@github.com:heguixiang/Remove_ROS_VINS.git
  4. mv Remove_ROS_VINS src
  5. ./generate.sh
``` 
# How to run:
```
  1. cd VINS_Workspace
  2. mkdir -p data/image/MH_01_easy
  3. mkdir -p data/imu/
  4. cd VINS_Workspace/data & git clone git@github.com:heguixiang/EuRoc-Timestamps.git
  5. download the EuRoc dataset(e.g. Machine Hall 01)
  6. cp the MH_01_easy/mav0/cam0/data to data/image/MH_01_easy
  7. cp the MH_01_easy/mav0/imu0/data.csv to data/imu/
  8. cd VINS_Workspace
  9. ./src/vins_estimator/build/vins_estimator ./src/config/euroc/euroc_config.yaml ./data/image/MH_01_easy/data/ ./data/EuRoc-Timestamps/MH01.txt ./data/imu/data.csv
```  
