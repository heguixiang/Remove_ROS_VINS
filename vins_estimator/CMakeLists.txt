cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread -fopenmp")

include_directories(${CMAKE_SOURCE_DIR}/../camera_model/include)
#add_library( lib_camera_model STATIC IMPORTED )
#set_target_properties(lib_camera_model PROPERTIES IMPORTED_LOCATION  ${CMAKE_SOURCE_DIR}/../camera_model/build/libcamera_model.a)

include_directories(
					/usr/local/include/
					/usr/local/include/opencv
					)
set(OPENCV_LIB_DIR "/usr/local/lib")
add_library(opencv_calib3d SHARED IMPORTED)
set_target_properties(opencv_calib3d PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_calib3d.so)

add_library(opencv_core SHARED IMPORTED)
set_target_properties(opencv_core PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_core.so)

add_library(opencv_features2d SHARED IMPORTED)
set_target_properties(opencv_features2d PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_features2d.so)

add_library(opencv_flann SHARED IMPORTED)
set_target_properties(opencv_flann PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_flann.so)

add_library(opencv_highgui SHARED IMPORTED)
set_target_properties(opencv_highgui PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_highgui.so)

add_library(opencv_imgcodecs SHARED IMPORTED)
set_target_properties(opencv_imgcodecs PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_imgcodecs.so)

add_library(opencv_imgproc SHARED IMPORTED)
set_target_properties(opencv_imgproc PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_imgproc.so)

add_library(opencv_ml SHARED IMPORTED)
set_target_properties(opencv_ml PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_ml.so)

add_library(opencv_objdetect SHARED IMPORTED)
set_target_properties(opencv_objdetect PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_objdetect.so)

add_library(opencv_photo SHARED IMPORTED)
set_target_properties(opencv_photo PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_photo.so)

add_library(opencv_shape SHARED IMPORTED)
set_target_properties(opencv_shape PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_shape.so)

add_library(opencv_stitching SHARED IMPORTED)
set_target_properties(opencv_stitching PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_stitching.so)

add_library(opencv_superres SHARED IMPORTED)
set_target_properties(opencv_superres PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_superres.so)

add_library(opencv_video SHARED IMPORTED)
set_target_properties(opencv_video PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_video.so)

add_library(opencv_videoio SHARED IMPORTED)
set_target_properties(opencv_videoio PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_videoio.so)

add_library(opencv_videostab SHARED IMPORTED)
set_target_properties(opencv_videostab PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_videostab.so)

add_library(opencv_viz SHARED IMPORTED)
set_target_properties(opencv_viz PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/libopencv_viz.so)

find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})



#message("CERES_INCLUDE_DIR:" ${CERES_INCLUDE_DIR}  " CERES_LIBS: "  ${CERES_LIBRARIES} )
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#message("CMAKE MOUDLE PATH:", ${CMAKE_MODULE_PATH})
#find_package(Eigen3  REQUIRED)
include_directories(
# ${EIGEN3_INCLUDE_DIR}
		/usr/include/eigen3
)
#message("EIGEN3_INCLUDE_DIR:" ${EIGEN3_INCLUDE_DIR}  " EIGEN3_LIBS: "  ${Eigen3_LIBRARIES} )

#add_library(ceres STATIC IMPORTED)
#set_target_properties(ceres PROPERTIES IMPORTED_LOCATION /usr/local/lib/libceres.a)
#include_directories(/usr/local/include)

#add_library(glog STATIC IMPORTED)
#set_target_properties(glog PROPERTIES IMPORTED_LOCATION /usr/lib/x86_64-linux-gnu/libglog.a)
#include_directories(/usr/include/glog)

#add_library(suitesparse STATIC IMPORTED)
#set_target_properties(suitesparse PROPERTIES IMPORTED_LOCATION /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.a)
#include_directories(/usr/include/suitesparse) #cholmod

#add_library(cxsparse STATIC IMPORTED)
#set_target_properties(cxsparse PROPERTIES IMPORTED_LOCATION /usr/lib/x86_64-linux-gnu/libcxsparse.a)

#add_library(cholmod SHARED IMPORTED)
#set_target_properties(cholmod PROPERTIES IMPORTED_LOCATION /usr/lib/x86_64-linux-gnu/libcholmod.so)

#add_library(atlas SHARED IMPORTED)
#set_target_properties(atlas PROPERTIES IMPORTED_LOCATION /usr/lib/libatlas.so)
#add_library(lapack_atlas SHARED IMPORTED)
#set_target_properties(lapack_atlas PROPERTIES IMPORTED_LOCATION /usr/lib/liblapack_atlas.so)
#include_directories(/usr/include/atlas)
	
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)


include_directories(${Boost_INCLUDE_DIRS})
add_executable(vins_estimator
    src/estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
#    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/loop-closure/ThirdParty/DBoW/BowVector.cpp
    src/loop-closure/ThirdParty/DBoW/FBrief.cpp
    src/loop-closure/ThirdParty/DBoW/FeatureVector.cpp
    src/loop-closure/ThirdParty/DBoW/QueryResults.cpp
    src/loop-closure/ThirdParty/DBoW/ScoringObject.cpp
    src/loop-closure/ThirdParty/DUtils/Random.cpp
    src/loop-closure/ThirdParty/DUtils/Timestamp.cpp
    src/loop-closure/ThirdParty/DVision/BRIEF.cpp
    src/loop-closure/ThirdParty/VocabularyBinary.cpp
    src/loop-closure/loop_closure.cpp
    src/loop-closure/keyframe.cpp
    src/loop-closure/keyframe_database.cpp
    src/feature_tracker/feature_tracker.cpp 
  #  src/feature_tracker/feature_tracker_parameters.cpp
    ../camera_model/src/chessboard/Chessboard.cc
    ../camera_model/src/calib/CameraCalibration.cc
    ../camera_model/src/camera_models/Camera.cc
    ../camera_model/src/camera_models/CameraFactory.cc
    ../camera_model/src/camera_models/CostFunctionFactory.cc
    ../camera_model/src/camera_models/PinholeCamera.cc
    ../camera_model/src/camera_models/CataCamera.cc
    ../camera_model/src/camera_models/EquidistantCamera.cc
    ../camera_model/src/camera_models/ScaramuzzaCamera.cc
    ../camera_model/src/sparse_graph/Transform.cc
    ../camera_model/src/gpl/gpl.cc
    ../camera_model/src/gpl/EigenQuaternionParameterization.cc
    )


target_link_libraries(vins_estimator #lib_camera_model 
					opencv_calib3d opencv_core opencv_features2d opencv_flann opencv_highgui opencv_imgcodecs opencv_imgproc opencv_ml opencv_objdetect opencv_photo opencv_shape opencv_stitching opencv_superres opencv_video opencv_videoio opencv_videostab opencv_viz   
					${CERES_LIBRARIES}
#					suitesparse cxsparse cholmod atlas lapack_atlas
#					glog
#					ceres
		) 


