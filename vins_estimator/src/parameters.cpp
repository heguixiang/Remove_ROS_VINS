#include "parameters.h"
#include <string>
#include <unistd.h>
#include <iostream>

#define FILENAMEPATH_MAX 80
using namespace std;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
int LOOP_CLOSURE = 0;
int MIN_LOOP_NUM;
std::string CAM_NAMES_ESTIMATOR;   //add
std::string PATTERN_FILE;
std::string VOC_FILE;
std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
int IMAGE_ROW, IMAGE_COL;
std::string VINS_FOLDER_PATH;
int MAX_KEYFRAME_NUM;

//feature tracker section
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE_FEATURE_TRACKER;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
double FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

void printConfigData( void)
{
   ROS_INFO_STREAM(" INIT_DEPTH:" << INIT_DEPTH);
   ROS_INFO_STREAM(" MIN_PARALLAX:" << MIN_PARALLAX);
   ROS_INFO_STREAM("ACC_N:" << ACC_N  << "ACC_W:" << ACC_W );
   ROS_INFO_STREAM("GYR_N:" << GYR_N  << "GYR_W:" << GYR_W );
   ROS_INFO_STREAM("RIC.front():" << RIC.front());
   ROS_INFO_STREAM("TIC.front():" << TIC.front());
  
   ROS_INFO_STREAM("BIAS_ACC_THRESHOLD" <<BIAS_ACC_THRESHOLD<< "BIAS_GYR_THRESHOLD" << BIAS_GYR_THRESHOLD);
   ROS_INFO_STREAM("SOLVER_TIME" <<SOLVER_TIME);
   ROS_INFO_STREAM("NUM_ITERATIONS" << NUM_ITERATIONS);
   
   ROS_INFO_STREAM("ESTIMATE_EXTRINSIC" <<ESTIMATE_EXTRINSIC);
   ROS_INFO_STREAM("EX_CALIB_RESULT_PATH" << EX_CALIB_RESULT_PATH);
   ROS_INFO_STREAM("VINS_RESULT_PATH" <<VINS_RESULT_PATH);
   ROS_INFO_STREAM("MIN_LOOP_NUM" << MIN_LOOP_NUM);
   ROS_INFO_STREAM("CAM_NAMES_ESTIMATOR" <<CAM_NAMES_ESTIMATOR<< "PATTERN_FILE" << PATTERN_FILE);
  
   ROS_INFO_STREAM("VOC_FILE" <<VOC_FILE );
   ROS_INFO_STREAM("VINS_FOLDER_PATH" << VINS_FOLDER_PATH);
   ROS_INFO_STREAM("IMAGE_ROW" << IMAGE_ROW << "IMAGE_COL" <<IMAGE_COL);
   ROS_INFO_STREAM("MAX_KEYFRAME_NUM" <<MAX_KEYFRAME_NUM);
   ROS_INFO_STREAM( "CAM_NAMES.front()" << CAM_NAMES.front());  
   ROS_INFO_STREAM("FISHEYE_MASK" <<FISHEYE_MASK);
   ROS_INFO_STREAM( "MAX_CNT" << MAX_CNT);
   ROS_INFO_STREAM("MIN_DIST" <<MIN_DIST<< "WINDOW_SIZE_FEATURE_TRACKER" << WINDOW_SIZE_FEATURE_TRACKER);
 
   ROS_INFO_STREAM("FREQ" <<FREQ);
   ROS_INFO_STREAM( "F_THRESHOLD" << F_THRESHOLD);
   ROS_INFO_STREAM("SHOW_TRACK" <<SHOW_TRACK<< "STEREO_TRACK" << STEREO_TRACK);
   ROS_INFO_STREAM("EQUALIZE" <<EQUALIZE<< "ROW" << ROW);
   ROS_INFO_STREAM("COL" <<COL<< "FOCAL_LENGTH" << FOCAL_LENGTH);
   ROS_INFO_STREAM("FISHEYE" <<FISHEYE);
   ROS_INFO_STREAM("PUB_THIS_FRAME" << PUB_THIS_FRAME);
   
  
}
void readParameters(const string & config_file)
{


    cv::FileStorage fsSettings(config_file.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings " << config_file << std::endl;
    }


    VINS_FOLDER_PATH = getcwd(NULL,FILENAMEPATH_MAX);

   
//    fsSettings["image_topic"] >> IMAGE_TOPIC;
//    fsSettings["imu_topic"] >> IMU_TOPIC;

    IMAGE_COL = fsSettings["image_width"];
    IMAGE_ROW = fsSettings["image_height"];

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];


    fsSettings["output_path"] >> VINS_RESULT_PATH;
    VINS_RESULT_PATH = VINS_FOLDER_PATH + VINS_RESULT_PATH;
    cout << VINS_RESULT_PATH << endl;
    std::ofstream foutC(VINS_RESULT_PATH, std::ios::out);
    foutC.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        fsSettings["ex_calib_result_path"] >> EX_CALIB_RESULT_PATH;
        EX_CALIB_RESULT_PATH = VINS_FOLDER_PATH + EX_CALIB_RESULT_PATH;

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            fsSettings["ex_calib_result_path"] >> EX_CALIB_RESULT_PATH;
            EX_CALIB_RESULT_PATH = VINS_FOLDER_PATH + EX_CALIB_RESULT_PATH;
	    ROS_INFO_STREAM("EX_CALIB_RESULT_PATH" << EX_CALIB_RESULT_PATH);
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 
    LOOP_CLOSURE = fsSettings["loop_closure"];
    if (LOOP_CLOSURE == 1)
    {
        fsSettings["voc_file"] >> VOC_FILE;;
        fsSettings["pattern_file"] >> PATTERN_FILE;
        VOC_FILE = VINS_FOLDER_PATH + VOC_FILE;
        PATTERN_FILE = VINS_FOLDER_PATH + PATTERN_FILE;
        MIN_LOOP_NUM = fsSettings["min_loop_num"];
        CAM_NAMES_ESTIMATOR = VINS_FOLDER_PATH + "/"+ config_file;   //add
	//CAM_NAMES_ESTIMATOR = "/home/solomon/merge_vins_version/src/config/euroc/euroc_config.yaml";
    }
  

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;
    MAX_KEYFRAME_NUM = 1000;
    
    // feature tracker
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "/src/config/fisheye_mask.jpg";
    CAM_NAMES.push_back(VINS_FOLDER_PATH + "/" + config_file);
    //CAM_NAMES.push_back("/home/solomon/merge_vins_version/src/config/euroc/euroc_config.yaml");

    WINDOW_SIZE_FEATURE_TRACKER = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460.0;
    PUB_THIS_FRAME = false; 
    
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;
    if (FREQ == 0)
        FREQ = 100;
   
    fsSettings.release();
    printConfigData();
}
