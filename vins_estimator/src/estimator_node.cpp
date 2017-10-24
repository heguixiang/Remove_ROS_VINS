#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
//#include <ros/ros.h>
//#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"
#include "loop-closure/loop_closure.h"
#include "loop-closure/keyframe.h"
#include "loop-closure/keyframe_database.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
/****************** load image section ***********************/
#include <iostream>
#include <fstream>
#include <chrono>
/****************** load image section ***********************/

/****************** feature tracker section ***********************/
#include "../../include/PointCloud.h"
#include "../../include/Imu.h"
#include "feature_tracker/feature_tracker.h"

/************************* visualization ***********************/
#include <pangolin/pangolin.h>

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;

//ros::Publisher pub_img,pub_match;
//ros::Publisher pub_match;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
bool running_flag = true;
bool view_done = false;
/****************** feature tracker section ***********************/

/****************** load image section ***********************/
using namespace std;
/****************** load image section ***********************/

Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
std::mutex m_posegraph_buf;
queue<int> optimize_posegraph_buf;
queue<KeyFrame*> keyframe_buf;
queue<RetriveData> retrive_data_buf;

int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_loop_drift;
std::mutex m_keyframedatabase_resample;
std::mutex m_update_visualization;
std::mutex m_keyframe_buf;
std::mutex m_retrive_data_buf;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;

queue<pair<cv::Mat, double>> image_buf;
LoopClosure *loop_closure;
KeyFrameDatabase keyframe_database;

int global_frame_cnt = 0;
//camera param
camodocal::CameraPtr m_camera;
vector<int> erase_index;
std_msgs::Header cur_header;
Eigen::Vector3d relocalize_t{Eigen::Vector3d(0, 0, 0)};
Eigen::Matrix3d relocalize_r{Eigen::Matrix3d::Identity()};

nav_msgs::Path  loop_path;
void updateLoopPath(nav_msgs::Path _loop_path)
{
    loop_path = _loop_path;
}
void ViewCameraPose(Eigen::Vector3d loop_correct_t, Eigen::Matrix3d loop_correct_r, pangolin::OpenGlMatrix &M)
{
	int idx2 = WINDOW_SIZE - 1;
	if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
	{
		int i = idx2;
		Vector3d P = (loop_correct_r * estimator.Ps[i] + loop_correct_t) + (loop_correct_r * estimator.Rs[i]) * estimator.tic[0];
		//Quaterniond R = Quaterniond((loop_correct_r * estimator.Rs[i]) * estimator.ric[0]);
		Eigen::Matrix3d R = (loop_correct_r * estimator.Rs[i]) * estimator.ric[0];

		M.m[0] = R(0,0); 
		M.m[1] = R(1,0); 
		M.m[2] = R(2,0); 
		M.m[3] = 0.0; 

		M.m[4] = R(0,1); 
		M.m[5] = R(1,1); 
		M.m[6] = R(2,1); 
		M.m[7] = 0.0; 

		M.m[8] = R(0,2); 
		M.m[9] = R(1,2); 
		M.m[10] = R(2,2); 
		M.m[11] = 0.0; 
		
		
		M.m[12] = P.x(); 
		M.m[13] = P.y(); 
		M.m[14] = P.z(); 
		M.m[15] = 1.0;	
	//    cout << "M.m[0]:" <<M.m[0] << "M.m[1]:" << M.m[1] << "M.m[2]" << M.m[2] << endl; 	
	//    cout << "M.m[4]:" <<M.m[4] << "M.m[5]:" << M.m[5] << "M.m[6]" << M.m[6] << endl; 	
	//    cout << "M.m[8]:" <<M.m[8] << "M.m[9]:" << M.m[9] << "M.m[10]" << M.m[10] << endl; 	
	  //  cout << "M.m[12]:" <<M.m[12] << "M.m[13]:" << M.m[13] << "M.m[14]" << M.m[14] << endl; 	

	}
	else
		 M.SetIdentity();
}
void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
	const float &w = 0.08f; //mCameraSize;
	const float h = w*0.75;
	const float z = w*0.6;

	glPushMatrix();

#ifdef HAVE_GLES
	glMultMatrixf(Twc.m);
#else
	glMultMatrixd(Twc.m);
#endif

	glLineWidth(2);  //set line width
	glColor3f(0.0f,0.0f,1.0f);   //blue
	glBegin(GL_LINES);           //draw camera 
	glVertex3f(0,0,0);
	glVertex3f(w,h,z);
	glVertex3f(0,0,0);
	glVertex3f(w,-h,z);
	glVertex3f(0,0,0);
	glVertex3f(-w,-h,z);
	glVertex3f(0,0,0);
	glVertex3f(-w,h,z);

	glVertex3f(w,h,z);
	glVertex3f(w,-h,z);
	glVertex3f(-w,h,z);
	glVertex3f(-w,-h,z);
	glVertex3f(-w,h,z);
	glVertex3f(w,h,z);
	glVertex3f(-w,-h,z);
	glVertex3f(w,-h,z);
	glEnd();
	glPopMatrix();
}
void visualization()
{
	float mViewpointX = -0;
	float mViewpointY = -5;
	float mViewpointZ = -10;
	float mViewpointF = 500;
	pangolin::CreateWindowAndBind("VINS: Map Visualization",1024,768); //create a display window
	glEnable(GL_DEPTH_TEST); //launch depth test
	glEnable(GL_BLEND);      //use blend function
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA); //set blend alpha value
	pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175)); //new button and menu
 	pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
 	pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
 	pangolin::Var<bool> menuShowPath("menu.Show Path", true, true);
   	// Define Camera Render Object (for view / scene browsing)
	pangolin::OpenGlRenderState s_cam(
				    pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
				    pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,VISUALLOOKATX, VISUALLOOKATY, VISUALLOOKATZ)
					);

// Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
		    .SetHandler(new pangolin::Handler3D(s_cam));
	
	pangolin::OpenGlMatrix Twc;
	Twc.SetIdentity();
	while(!pangolin::ShouldQuit() & running_flag)
	{
	  
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		ViewCameraPose(relocalize_t, relocalize_r, Twc);
	        if(menuFollowCamera)
		   s_cam.Follow(Twc);
		d_cam.Activate(s_cam);
		glClearColor(0.0f,0.0f,0.0f,0.0f); //背景色设置为白色
		DrawCurrentCamera(Twc);		
		if(menuShowPoints)
			keyframe_database.viewPointClouds();
		if(menuShowPath)
			keyframe_database.viewPath();
		pangolin::FinishFrame();

	}
	cout << "pangolin thread end" << endl;
    view_done = true;	
	//exit(1);
}


void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse() * estimator.g);

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba - tmp_Q.inverse() * estimator.g);

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = relocalize_r * estimator.Ps[WINDOW_SIZE] + relocalize_t;
    tmp_Q = relocalize_r * estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
      
        if (imu_buf.empty() || feature_buf.empty())	   
            return measurements;
        if (!(imu_buf.back()->header.stamp > feature_buf.front()->header.stamp))
        {
       //     ROS_WARN("wait for imu, only should happen at the beginning");
	    cout << "WARN: wait for imu, only should happen at the beginning" << endl;
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp < feature_buf.front()->header.stamp))
        {
        //    ROS_WARN("throw img, only should happen at the beginning");
	    cout << "WARN: throw img, only should happen at the beginning" << endl;
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp <= img_msg->header.stamp)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
// ROS_INFO_STREAM("IMUs end data timestamp: " << IMUs.back()->header.stamp << "IMUs size: "<< IMUs.size() << "img_msg timestamp" << img_msg->header.stamp );
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    m_buf.lock();
    imu_buf.push(imu_msg);
   // ROS_INFO("----------IMU DATA. timestamp %f------------",imu_msg->header.stamp.toSec());
    m_buf.unlock();
  //  con.notify_one();   //remove by solomon
    

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
   //     std_msgs::Header header = imu_msg->header;
    //    header.frame_id = "world";
     //   if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
      //      pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    m_buf.lock();
    
    feature_buf.push(feature_msg);
    //ROS_INFO("----------feature timestamp %f------------",feature_msg->header.stamp.toSec());
    m_buf.unlock();
    con.notify_one();
}

void send_imu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (current_time < 0)
        current_time = t;
    double dt = t - current_time;
    current_time = t;

    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};

    double dx = imu_msg->linear_acceleration.x - ba[0];
    double dy = imu_msg->linear_acceleration.y - ba[1];
    double dz = imu_msg->linear_acceleration.z - ba[2];

    double rx = imu_msg->angular_velocity.x - bg[0];
    double ry = imu_msg->angular_velocity.y - bg[1];
    double rz = imu_msg->angular_velocity.z - bg[2];
    //ROS_DEBUG("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);

    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

//thread:loop detection
void process_loop_detection()
{
    if(loop_closure == NULL)
    {
        const char *voc_file = VOC_FILE.c_str();
        TicToc t_load_voc;
       // ROS_DEBUG("loop start loop");
	cout << "loop start loop" << endl;
        cout << "voc file: " << voc_file << endl;
        loop_closure = new LoopClosure(voc_file, IMAGE_COL, IMAGE_ROW);
     //   ROS_DEBUG("loop load vocbulary %lf", t_load_voc.toc());
	cout << "loop load vocbulary:"<< t_load_voc.toc() << endl;
        loop_closure->initCameraModel(CAM_NAMES_ESTIMATOR);  //add
    }

    while(LOOP_CLOSURE)   
    {
        KeyFrame* cur_kf = NULL; 
        m_keyframe_buf.lock();
        while(!keyframe_buf.empty())
        {
            if(cur_kf!=NULL)
                delete cur_kf;
            cur_kf = keyframe_buf.front();
            keyframe_buf.pop();
        }
        m_keyframe_buf.unlock();
        if (cur_kf != NULL)
        {
            cur_kf->global_index = global_frame_cnt;
            m_keyframedatabase_resample.lock();
            keyframe_database.add(cur_kf);
            m_keyframedatabase_resample.unlock();

            cv::Mat current_image;
            current_image = cur_kf->image;   

            bool loop_succ = false;
            int old_index = -1;
            vector<cv::Point2f> cur_pts;
            vector<cv::Point2f> old_pts;
            TicToc t_brief;
            cur_kf->extractBrief(current_image);
            //printf("loop extract %d feature using %lf\n", cur_kf->keypoints.size(), t_brief.toc());
            TicToc t_loopdetect;
            loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);
            double t_loop = t_loopdetect.toc();
            //ROS_DEBUG("t_loopdetect %f ms", t_loop);
	    // cout << "t_loopdetect %f ms" << t_loop << endl;
            if(loop_succ)
            {
                KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);
                if (old_kf == NULL)
                {
                   // ROS_WARN("NO such frame in keyframe_database");
		  cout << "WARN: NO such frame in keyframe_database" << endl;
                   // ROS_BREAK();
		  //break;
		  continue;
                }
           //     ROS_DEBUG("loop succ %d with %drd image", global_frame_cnt, old_index);
		//cout << "loop succ " <<global_frame_cnt <<  " with " << old_index << "rd image" << endl;
                assert(old_index!=-1);
                
                Vector3d T_w_i_old, PnP_T_old;
                Matrix3d R_w_i_old, PnP_R_old;

                old_kf->getPose(T_w_i_old, R_w_i_old);
                std::vector<cv::Point2f> measurements_old;
                std::vector<cv::Point2f> measurements_old_norm;
                std::vector<cv::Point2f> measurements_cur;
                std::vector<int> features_id_matched;  
                cur_kf->findConnectionWithOldFrame(old_kf, measurements_old, measurements_old_norm, PnP_T_old, PnP_R_old, m_camera);
                measurements_cur = cur_kf->measurements_matched;
                features_id_matched = cur_kf->features_id_matched;
                // send loop info to VINS relocalization
                int loop_fusion = 0;
                if( (int)measurements_old_norm.size() > MIN_LOOP_NUM && global_frame_cnt - old_index > 35 && old_index > 30)
                {

                    Quaterniond PnP_Q_old(PnP_R_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.R_old = R_w_i_old;
                    retrive_data.relative_pose = false;
                    retrive_data.relocalized = false;
                    retrive_data.measurements = measurements_old_norm;
                    retrive_data.features_ids = features_id_matched;
                    retrive_data.loop_pose[0] = PnP_T_old.x();
                    retrive_data.loop_pose[1] = PnP_T_old.y();
                    retrive_data.loop_pose[2] = PnP_T_old.z();
                    retrive_data.loop_pose[3] = PnP_Q_old.x();
                    retrive_data.loop_pose[4] = PnP_Q_old.y();
                    retrive_data.loop_pose[5] = PnP_Q_old.z();
                    retrive_data.loop_pose[6] = PnP_Q_old.w();
                    m_retrive_data_buf.lock();
                    retrive_data_buf.push(retrive_data);
                    m_retrive_data_buf.unlock();
                    cur_kf->detectLoop(old_index);
                    old_kf->is_looped = 1;
                    loop_fusion = 1;

                    m_update_visualization.lock();
                    keyframe_database.addLoop(old_index);
                 //   CameraPoseVisualization* posegraph_visualization = keyframe_database.getPosegraphVisualization();
                //    pubPoseGraph(posegraph_visualization, cur_header);  
                    m_update_visualization.unlock();
                }


                // visualization loop info
                if(0 && loop_fusion)
                {
                    int COL = current_image.cols;
                    //int ROW = current_image.rows;
                    cv::Mat gray_img, loop_match_img;
                    cv::Mat old_img = old_kf->image;
                    cv::hconcat(old_img, current_image, gray_img);
                    cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
                    cv::Mat loop_match_img2;
                    loop_match_img2 = loop_match_img.clone();
                    /*
                    for(int i = 0; i< (int)cur_pts.size(); i++)
                    {
                        cv::Point2f cur_pt = cur_pts[i];
                        cur_pt.x += COL;
                        cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
                    }
                    for(int i = 0; i< (int)old_pts.size(); i++)
                    {
                        cv::circle(loop_match_img, old_pts[i], 5, cv::Scalar(0, 255, 0));
                    }
                    for (int i = 0; i< (int)old_pts.size(); i++)
                    {
                        cv::Point2f cur_pt = cur_pts[i];
                        cur_pt.x += COL ;
                        cv::line(loop_match_img, old_pts[i], cur_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
                    }
                    ostringstream convert;
                    convert << "/home/tony-ws/raw_data/loop_image/"
                            << cur_kf->global_index << "-" 
                            << old_index << "-" << loop_fusion <<".jpg";
                    cv::imwrite( convert.str().c_str(), loop_match_img);
                    */
                    for(int i = 0; i< (int)measurements_cur.size(); i++)
                    {
                        cv::Point2f cur_pt = measurements_cur[i];
                        cur_pt.x += COL;
                        cv::circle(loop_match_img2, cur_pt, 5, cv::Scalar(0, 255, 0));
                    }
                    for(int i = 0; i< (int)measurements_old.size(); i++)
                    {
                        cv::circle(loop_match_img2, measurements_old[i], 5, cv::Scalar(0, 255, 0));
                    }
                    for (int i = 0; i< (int)measurements_old.size(); i++)
                    {
                        cv::Point2f cur_pt = measurements_cur[i];
                        cur_pt.x += COL ;
                        cv::line(loop_match_img2, measurements_old[i], cur_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
                    }

                    ostringstream convert2;
                    convert2 << "/home/tony-ws/raw_data/loop_image/"
                            << cur_kf->global_index << "-" 
                            << old_index << "-" << loop_fusion <<"-2.jpg";
                    cv::imwrite( convert2.str().c_str(), loop_match_img2);
                }
                  
            }
            //release memory
            cur_kf->image.release();
            global_frame_cnt++;
	 //   cout << "---------keyframe_database.size():" << keyframe_database.size() << endl;
            if (t_loop > 1000 || keyframe_database.size() > MAX_KEYFRAME_NUM)
            {
                m_keyframedatabase_resample.lock();
                erase_index.clear();
                keyframe_database.downsample(erase_index);
                m_keyframedatabase_resample.unlock();
                if(!erase_index.empty())
                    loop_closure->eraseIndex(erase_index);
            }
        }
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

//thread: pose_graph optimization
void process_pose_graph()
{
    while(true)            
    {
        m_posegraph_buf.lock();
        int index = -1;
        while (!optimize_posegraph_buf.empty())
        {
            index = optimize_posegraph_buf.front();
            optimize_posegraph_buf.pop();
        }
        m_posegraph_buf.unlock();
        if(index != -1)
        {
            Vector3d correct_t = Vector3d::Zero();
            Matrix3d correct_r = Matrix3d::Identity();
            TicToc t_posegraph;
            keyframe_database.optimize4DoFLoopPoseGraph(index,
                                                    correct_t,
                                                    correct_r);
	    cout << "t_posegraph " <<  t_posegraph.toc() << " ms"<<endl; 
            m_loop_drift.lock();
            relocalize_r = correct_r;
            relocalize_t = correct_t;
            m_loop_drift.unlock();
            m_update_visualization.lock();
            keyframe_database.updateVisualization();
         //   CameraPoseVisualization* posegraph_visualization = keyframe_database.getPosegraphVisualization();
            m_update_visualization.unlock();
            pubOdometry(estimator, cur_header, relocalize_t, relocalize_r);
        //    pubPoseGraph(posegraph_visualization, cur_header); 
            nav_msgs::Path refine_path = keyframe_database.getPath();
            updateLoopPath(refine_path);
        }

        std::chrono::milliseconds dura(5000);
        std::this_thread::sleep_for(dura);
    }
}

// thread: visual-inertial odometry
void process()
{
    while (true) 
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();

        for (auto &measurement : measurements)
        {
            for (auto &imu_msg : measurement.first)
                send_imu(imu_msg);

            auto img_msg = measurement.second;
      //      ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());
	//    cout << "processing vision data with stamp "<<  img_msg->header.stamp.toSec() << endl;

            TicToc t_s;
            map<int, vector<pair<int, Vector3d>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                //ROS_ASSERT(z == 1);
		assert(z == 1);
                image[feature_id].emplace_back(camera_id, Vector3d(x, y, z));
            }
            estimator.processImage(image, img_msg->header);
            /**
            *** start build keyframe database for loop closure
            **/
            if(LOOP_CLOSURE)
            {
                // remove previous loop
                vector<RetriveData>::iterator it = estimator.retrive_data_vector.begin();
                for(; it != estimator.retrive_data_vector.end(); )
                {
                    if ((*it).header < estimator.Headers[0].stamp.toSec())
                    {
                        it = estimator.retrive_data_vector.erase(it);
                    }
                    else
                        it++;
                }
                m_retrive_data_buf.lock();
                while(!retrive_data_buf.empty())
                {
                    RetriveData tmp_retrive_data = retrive_data_buf.front();
                    retrive_data_buf.pop();
                    estimator.retrive_data_vector.push_back(tmp_retrive_data);
                }
                m_retrive_data_buf.unlock();
                //WINDOW_SIZE - 2 is key frame
                if(estimator.marginalization_flag == 0 && estimator.solver_flag == estimator.NON_LINEAR)
                {   
                    Vector3d vio_T_w_i = estimator.Ps[WINDOW_SIZE - 2];
                    Matrix3d vio_R_w_i = estimator.Rs[WINDOW_SIZE - 2];
                    i_buf.lock();
                    while(!image_buf.empty() && image_buf.front().second < estimator.Headers[WINDOW_SIZE - 2].stamp.toSec())
                    {
                        image_buf.pop();
                    }
                    i_buf.unlock();
                    //assert(estimator.Headers[WINDOW_SIZE - 1].stamp.toSec() == image_buf.front().second);
                    // relative_T   i-1_T_i relative_R  i-1_R_i
                    cv::Mat KeyFrame_image;
                    KeyFrame_image = image_buf.front().first;
                    
                    const char *pattern_file = PATTERN_FILE.c_str();
                    Vector3d cur_T;
                    Matrix3d cur_R;
                    cur_T = relocalize_r * vio_T_w_i + relocalize_t;
                    cur_R = relocalize_r * vio_R_w_i;
                    KeyFrame* keyframe = new KeyFrame(estimator.Headers[WINDOW_SIZE - 2].stamp.toSec(), vio_T_w_i, vio_R_w_i, cur_T, cur_R, image_buf.front().first, pattern_file, relocalize_t, relocalize_r);
                    keyframe->setExtrinsic(estimator.tic[0], estimator.ric[0]);
                    keyframe->buildKeyFrameFeatures(estimator, m_camera);
                    m_keyframe_buf.lock();
                    keyframe_buf.push(keyframe);
                    m_keyframe_buf.unlock();
                    // update loop info
                    if (!estimator.retrive_data_vector.empty() && estimator.retrive_data_vector[0].relative_pose)
                    {
                        if(estimator.Headers[0].stamp.toSec() == estimator.retrive_data_vector[0].header)
                        {
                            KeyFrame* cur_kf = keyframe_database.getKeyframe(estimator.retrive_data_vector[0].cur_index);                            
                            if (abs(estimator.retrive_data_vector[0].relative_yaw) > 30.0 || estimator.retrive_data_vector[0].relative_t.norm() > 20.0)
                            {
                           //     ROS_DEBUG("Wrong loop");
			      cout << "Wrong loop" <<endl;
                                cur_kf->removeLoop();
                            }
                            else 
                            {
                                cur_kf->updateLoopConnection( estimator.retrive_data_vector[0].relative_t, 
                                                              estimator.retrive_data_vector[0].relative_q, 
                                                              estimator.retrive_data_vector[0].relative_yaw);
                                m_posegraph_buf.lock();
                                optimize_posegraph_buf.push(estimator.retrive_data_vector[0].cur_index);
                                m_posegraph_buf.unlock();
                            }
                        }
                    }
                }
            }
         //   double whole_t = t_s.toc();
        //    printStatistics(estimator, whole_t);
	    cout << "position: " << estimator.Ps[WINDOW_SIZE].transpose() << endl;
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";
            cur_header = header;
            m_loop_drift.lock();
            if (estimator.relocalize)
            {
                relocalize_t = estimator.relocalize_t;
                relocalize_r = estimator.relocalize_r;
            }
            pubOdometry(estimator, header, relocalize_t, relocalize_r);
        //    pubKeyPoses(estimator, header, relocalize_t, relocalize_r);
        //    pubCameraPose(estimator, header, relocalize_t, relocalize_r);
        //    pubPointCloud(estimator, header, relocalize_t, relocalize_r);
       //     pubTF(estimator, header, relocalize_t, relocalize_r);
            m_loop_drift.unlock();
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}


void img_callback(const cv::Mat &show_img, const ros::Time &timestamp)
{
    if(LOOP_CLOSURE)
    {
        i_buf.lock();
        image_buf.push(make_pair(show_img, timestamp.toSec()));
        i_buf.unlock();
    }
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = timestamp.toSec();
    }

    // frequency control
    if (round(1.0 * pub_count / (timestamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (timestamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = timestamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

  //  cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  //  cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
       // ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            //trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)));
	  trackerData[i].readImage(show_img.rowRange(ROW * i, ROW * (i + 1)));
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
               // clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
		clahe->apply(show_img.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
              //  trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
	      trackerData[i].cur_img = show_img.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    if ( PUB_THIS_FRAME && STEREO_TRACK && trackerData[0].cur_pts.size() > 0)
    {
        pub_count++;
        r_status.clear();
        r_err.clear();
        TicToc t_o;
        cv::calcOpticalFlowPyrLK(trackerData[0].cur_img, trackerData[1].cur_img, trackerData[0].cur_pts, trackerData[1].cur_pts, r_status, r_err, cv::Size(21, 21), 3);
   //     ROS_DEBUG("spatial optical flow costs: %fms", t_o.toc());
        vector<cv::Point2f> ll, rr;
        vector<int> idx;
        for (unsigned int i = 0; i < r_status.size(); i++)
        {
            if (!inBorder(trackerData[1].cur_pts[i]))
                r_status[i] = 0;

            if (r_status[i])
            {
                idx.push_back(i);

                Eigen::Vector3d tmp_p;
                trackerData[0].m_camera->liftProjective(Eigen::Vector2d(trackerData[0].cur_pts[i].x, trackerData[0].cur_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                ll.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));

                trackerData[1].m_camera->liftProjective(Eigen::Vector2d(trackerData[1].cur_pts[i].x, trackerData[1].cur_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                rr.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
            }
        }
        if (ll.size() >= 8)
        {
            vector<uchar> status;
            TicToc t_f;
            cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 1.0, 0.5, status);
         //   ROS_DEBUG("find f cost: %f", t_f.toc());
            int r_cnt = 0;
            for (unsigned int i = 0; i < status.size(); i++)
            {
                if (status[i] == 0)
                    r_status[idx[i]] = 0;
                r_cnt += r_status[idx[i]];
            }
        }
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;

      //  feature_points->header = img_msg->header;
	feature_points->header.stamp = timestamp; //here need to double check,because of missing seq variable assignment
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            if (i != 1 || !STEREO_TRACK)
            {
                auto un_pts = trackerData[i].undistortedPoints();
                auto &cur_pts = trackerData[i].cur_pts;
                auto &ids = trackerData[i].ids;
                for (unsigned int j = 0; j < ids.size(); j++)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                 //   ROS_ASSERT(inBorder(cur_pts[j]));
		    assert(inBorder(cur_pts[j]));
                }
            }
            else if (STEREO_TRACK)
            {
                auto r_un_pts = trackerData[1].undistortedPoints();
                auto &ids = trackerData[0].ids;
                for (unsigned int j = 0; j < ids.size(); j++)
                {
                    if (r_status[j])
                    {
                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        geometry_msgs::Point32 p;
                        p.x = r_un_pts[j].x;
                        p.y = r_un_pts[j].y;
                        p.z = 1;

                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    }
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_callback(feature_points);          //add

		/*----------------add ui ---------------------*/
        cv::Mat tmp_img = show_img.rowRange(0, ROW);
        cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);
        for (unsigned int j = 0; j < trackerData[0].cur_pts.size(); j++)
        {
            double len = std::min(1.0, 1.0 * trackerData[0].track_cnt[j] / WINDOW_SIZE_FEATURE_TRACKER);
            cv::circle(tmp_img, trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
		}
		cv::namedWindow("vins", CV_WINDOW_NORMAL); 
        cv::imshow("vins", tmp_img);
        cv::waitKey(5);
		/*----------------add ui ---------------------*/
    }
  //  ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
   
}
/******************* load image begin ***********************/
void LoadImages(const string &strImagePath, const string &strTimesStampsPath,
		vector<string> &strImagesFileNames, vector<double> &timeStamps)
{
    ifstream fTimes;
    fTimes.open(strTimesStampsPath.c_str());
    timeStamps.reserve(5000); //reserve vector space
    strImagesFileNames.reserve(5000); 
    while(!fTimes.eof())
    {
	string s;
	getline(fTimes,s);
	if(!s.empty())
	{
	    stringstream ss;
	    ss << s;
	    strImagesFileNames.push_back(strImagePath + "/" + ss.str() + ".png");
	    double t;
	    ss >> t;
	    timeStamps.push_back(t/1e9);
	}
    }
}
/******************* load image end ***********************/

/******************* load IMU begin ***********************/

void LoadImus(ifstream & fImus, const ros::Time &imageTimestamp)
{

    while(!fImus.eof())
    {
	string s;
	getline(fImus,s);
	if(!s.empty())
	{
	   char c = s.at(0);
 	   if(c<'0' || c>'9')      //remove first line in data.csv
		       continue;       
	    stringstream ss;
	    ss << s;
	    double tmpd;
	    int cnt=0;
	    double data[7];
	    while(ss >> tmpd)
	    {
		data[cnt] = tmpd;
		cnt++;
		if(cnt ==7)
		  break;
		if(ss.peek() == ',' || ss.peek() == ' ')
		  ss.ignore();
	    }
	    data[0] *=1e-9; //convert to second unit
	    sensor_msgs::ImuPtr imudata(new sensor_msgs::Imu);
	    imudata->angular_velocity.x = data[1];
	    imudata->angular_velocity.y = data[2];
	    imudata->angular_velocity.z = data[3];
	    imudata->linear_acceleration.x = data[4];
	    imudata->linear_acceleration.y = data[5];
	    imudata->linear_acceleration.z = data[6];
		uint32_t  sec = data[0];
		uint32_t nsec = (data[0]-sec)*1e9;
		nsec = (nsec/1000)*1000+500;
	    imudata->header.stamp = ros::Time(sec,nsec);
	    imu_callback(imudata);
	    if (imudata->header.stamp > imageTimestamp)       //load all imu data produced in interval time between two consecutive frams 
	      break;
	}
    }
}
/******************* load IMU end ***********************/

int main(int argc, char **argv)
{
  /******************* load image begin ***********************/
    if(argc != 5)
    {
	cerr << endl << "Usage: ./vins_estimator path_to_setting_file path_to_image_folder path_to_times_file path_to_imu_data_file" <<endl;
	return 1;
    }
    
    //imu data file 
    ifstream fImus;
    fImus.open(argv[4]);
    
    cv::Mat image;
    int ni;//num image
    
    //read parameters section
    readParameters(argv[1]);
    
    estimator.setParameter();
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]); //add
    
	vector<string> vStrImagesFileNames;
    vector<double> vTimeStamps;
    LoadImages(string(argv[2]),string(argv[3]),vStrImagesFileNames,vTimeStamps);
    
    int imageNum = vStrImagesFileNames.size();
    
    if(imageNum<=0)
    {
	cerr << "ERROR: Failed to load images" << endl;
	return 1;
    }
    
    std::thread measurement_process{process};
    
     measurement_process.detach();
   
    std::thread loop_detection, pose_graph;
    if (LOOP_CLOSURE)
     {     
	loop_detection = std::thread(process_loop_detection);   
	pose_graph = std::thread(process_pose_graph);
	std::thread visualization_thread{visualization}; //visualization thread
	loop_detection.detach();
	pose_graph.detach();
        visualization_thread.detach();
	//loop_detection.join();
	//pose_graph.join();
	m_camera = CameraFactory::instance()->generateCameraFromYamlFile(CAM_NAMES_ESTIMATOR);
     }
    for(ni=0; ni<imageNum; ni++)
    {
      
      double  tframe = vTimeStamps[ni];   //timestamp
	  uint32_t  sec = tframe;
      uint32_t nsec = (tframe-sec)*1e9;
      nsec = (nsec/1000)*1000+500;
      ros::Time image_timestamp = ros::Time(sec, nsec);
       // read imu data
      LoadImus(fImus,image_timestamp);
       
	//read image from file
      image = cv::imread(vStrImagesFileNames[ni],CV_LOAD_IMAGE_UNCHANGED);
      
      if(image.empty())
      {
	  cerr << endl << "Failed to load image: " << vStrImagesFileNames[ni] <<endl;
	  return 1;
      }
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
      img_callback(image, image_timestamp);
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
      double timeSpent =std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();
      
      //wait to load the next frame image
      double T=0;
      if(ni < imageNum-1)
	T = vTimeStamps[ni+1]-tframe; //interval time between two consecutive frames,unit:second
      else if(ni>0)    //lastest frame
	T = tframe-vTimeStamps[ni-1];
      
      if(timeSpent < T)
	usleep((T-timeSpent)*1e6); //sec->us:1e6
      else
	cerr << endl << "process image speed too slow, larger than interval time between two consecutive frames" << endl;
      
    }
	running_flag = false;
	while(!view_done)     //main thread wait view thread used its data structure
		usleep(5000);
/******************* load image end ***********************/
    return 0;
}
