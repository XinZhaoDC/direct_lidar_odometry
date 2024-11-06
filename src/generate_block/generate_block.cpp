#include <ros/ros.h>
#include <Eigen/Dense>
#include <mutex>
#include <queue>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <boost/filesystem.hpp>
//#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

std::string lidar_topic, key_frame_lidar_topic, odometry_topic,ground_truth_odom_topic,dataFolder,keyFrameFolder, groundTruthName;
bool pcd_save, odom_save, use_key_frame;
//bool transform;

std::queue<nav_msgs::Odometry::ConstPtr> odom_buf;
std::queue<sensor_msgs::PointCloud2::ConstPtr> pcl_buf, key_frame_pcl_buf;
std::queue<nav_msgs::Odometry::ConstPtr> ground_truth_odom_buf;
std::mutex bufMutex;
int frame_merge_num;

void key_frame_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg){
    bufMutex.lock();
    // ROS_INFO("receive pcl");
    key_frame_pcl_buf.push(msg);
    bufMutex.unlock();
}

void pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    bufMutex.lock();
    // ROS_INFO("receive pcl");
    pcl_buf.push(msg);
    bufMutex.unlock();
}

void odom_cbk(const nav_msgs::Odometry::ConstPtr &msg) 
{
    bufMutex.lock();
    // ROS_INFO("receive odometry");
    odom_buf.push(msg);
    bufMutex.unlock();
}

void ground_truth_odom_cbk(const nav_msgs::Odometry::ConstPtr &msg){

    //double currentTime = msg->header.stamp.toSec();
    //char odomFileName[256];

    //uint sec = currentTime;
    //uint nsec = (currentTime-sec)*1e9;

    // must use the format of sec_nsec
    //sprintf(odomFileName,"%s/%d_%d.odom",groundTruthFolder.c_str(),sec,nsec);

    Eigen::Quaterniond ref_q;
    Eigen::Vector3d ref_t(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);

    ref_q.x() = msg->pose.pose.orientation.x;
    ref_q.y() = msg->pose.pose.orientation.y;
    ref_q.z() = msg->pose.pose.orientation.z;
    ref_q.w() = msg->pose.pose.orientation.w;
  
    Eigen::Matrix3d rot = ref_q.toRotationMatrix();

    // save pose
    if(pcd_save){
        FILE *fp = fopen(groundTruthName.c_str(),"a+");
        /*fprintf(fp,"%f %f %f %f\n %f %f %f %f\n %f %f %f %f\n 0 0 0 1",
                rot(0,0),rot(0,1),rot(0,2),ref_t(0),
                rot(1,0),rot(1,1),rot(1,2),ref_t(1),
                rot(2,0),rot(2,1),rot(2,2),ref_t(2));*/
        fprintf(fp,"%f %f %f %f %f %f %f %f %f %f %f %f\n",
                ref_t(0),ref_t(1),ref_t(2),ref_q.x(),ref_q.y(),ref_q.z(),ref_q.w());
        fflush(fp);
        fclose(fp);
    }

}


void transformAndOutput(std::vector<nav_msgs::Odometry> currentOdoMsg,
 std::vector<sensor_msgs::PointCloud2> currentPclMsg)
{
    if(currentPclMsg.size() > 0 && currentOdoMsg.size() > 0 && currentPclMsg.size() == currentOdoMsg.size())
    {
        double currentTime = currentOdoMsg[0].header.stamp.toSec();
        char posFileName[256];
        char pclFileName[256];

        uint sec = currentTime;
        uint nsec = (currentTime-sec)*1e9;

        // must use the format of sec_nsec
        sprintf(posFileName,"%s/%d_%d.odom",dataFolder.c_str(),sec,nsec);
        sprintf(pclFileName,"%s/%d_%d.pcd",dataFolder.c_str(),sec,nsec);

        int id_ref = 0;

        Eigen::Quaterniond ref_q;
        Eigen::Vector3d ref_t(currentOdoMsg[id_ref].pose.pose.position.x,currentOdoMsg[id_ref].pose.pose.position.y,currentOdoMsg[id_ref].pose.pose.position.z);

        ref_q.x() = currentOdoMsg[id_ref].pose.pose.orientation.x;
        ref_q.y() = currentOdoMsg[id_ref].pose.pose.orientation.y;
        ref_q.z() = currentOdoMsg[id_ref].pose.pose.orientation.z;
        ref_q.w() = currentOdoMsg[id_ref].pose.pose.orientation.w;
  
        Eigen::Matrix3d rot = ref_q.toRotationMatrix();

        // save pose
        if(pcd_save){
            FILE *fp = fopen(posFileName,"w");
            fprintf(fp,"%f %f %f %f\n %f %f %f %f\n %f %f %f %f\n 0 0 0 1",
                    rot(0,0),rot(0,1),rot(0,2),ref_t(0),
                    rot(1,0),rot(1,1),rot(1,2),ref_t(1),
                    rot(2,0),rot(2,1),rot(2,2),ref_t(2));
            fflush(fp);
            fclose(fp);
        }
        

        // save pcl
        pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        for (size_t i = 0; i < currentPclMsg.size(); i++)
        {
            Eigen::Quaterniond q;
            q.x() = currentOdoMsg[i].pose.pose.orientation.x;
            q.y() = currentOdoMsg[i].pose.pose.orientation.y;
            q.z() = currentOdoMsg[i].pose.pose.orientation.z;
            q.w() = currentOdoMsg[i].pose.pose.orientation.w;
            q.normalize();
            Eigen::Vector3d t(currentOdoMsg[i].pose.pose.position.x,currentOdoMsg[i].pose.pose.position.y,currentOdoMsg[i].pose.pose.position.z);

            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(currentPclMsg[i],*temp_cloud);

            Eigen::Matrix4f transform; transform.setIdentity();
            transform.block<3,3>(0,0) = (ref_q.inverse()*q).toRotationMatrix().cast<float>();
            transform.block<3,1>(0,3) = (ref_q.inverse()*(-ref_t+t)).cast<float>();
            
            pcl::transformPointCloud(*temp_cloud,*temp_cloud_trans,transform);
            //transform to ref
            *all_cloud += *temp_cloud_trans;
        }
        if(pcd_save){
            pcl::io::savePCDFileBinary<pcl::PointXYZI>(pclFileName,*all_cloud);
        }
        
        //std::cout<<"cloud's size:   "<<all_cloud->points.size()<<std::endl;
    }
}

void process()
{
    ros::Rate rate(100);
    std::vector<nav_msgs::Odometry> currentOdoMsg;
    std::vector<sensor_msgs::PointCloud2> currentPclMsg;
    std::vector<sensor_msgs::PointCloud2> currentKeyFramePclMsg;
    while (ros::ok()) {
        bufMutex.lock();///lock before access Buf

        if (!odom_buf.empty() && !pcl_buf.empty()){  

            currentOdoMsg.push_back(*odom_buf.front());
            odom_buf.pop();
            currentPclMsg.push_back(*pcl_buf.front());
            pcl_buf.pop();

            // next image
            if(currentOdoMsg.size() >= frame_merge_num)
            {
            // save process
            transformAndOutput(currentOdoMsg,currentPclMsg);
            currentOdoMsg.clear();
            currentPclMsg.clear();
            }           
        }

        if(!key_frame_pcl_buf.empty()){
            currentKeyFramePclMsg.push_back(*key_frame_pcl_buf.front());
            key_frame_pcl_buf.pop();

            if(currentKeyFramePclMsg.size()>=frame_merge_num){
                if(pcd_save){
                    double currentTime = currentKeyFramePclMsg[0].header.stamp.toSec();
                    char keyFramePclFileName[256];

                    uint sec = currentTime;
                    uint nsec = (currentTime-sec)*1e9;

                    // must use the format of sec_nsec
                    sprintf(keyFramePclFileName,"%s/%d_%d.pcd",keyFrameFolder.c_str(),sec,nsec);
                    pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                    for (size_t i = 0; i < currentKeyFramePclMsg.size(); i++){
                        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                        pcl::fromROSMsg(currentKeyFramePclMsg[i],*temp_cloud);
                        *all_cloud += *temp_cloud;
                    }
                    pcl::io::savePCDFileBinary<pcl::PointXYZI>(keyFramePclFileName,*all_cloud);
                    currentKeyFramePclMsg.clear();
                }
            }
        }

        bufMutex.unlock();
        rate.sleep();
            
    }
    if(currentOdoMsg.size() != 0 )
    {
        // save process
        transformAndOutput(currentOdoMsg,currentPclMsg);
        currentOdoMsg.clear();
        currentPclMsg.clear();
    }
    if(!currentKeyFramePclMsg.empty()){
        if(pcd_save){
            double currentTime = currentKeyFramePclMsg[0].header.stamp.toSec();
            char keyFramePclFileName[256];

            uint sec = currentTime;
            uint nsec = (currentTime-sec)*1e9;

            // must use the format of sec_nsec
            sprintf(keyFramePclFileName,"%s/%d_%d.pcd",keyFrameFolder.c_str(),sec,nsec);
            pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            for (size_t i = 0; i < currentKeyFramePclMsg.size(); i++){
                pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::fromROSMsg(currentKeyFramePclMsg[i],*temp_cloud);
                *all_cloud += *temp_cloud;
            }
            pcl::io::savePCDFileBinary<pcl::PointXYZI>(keyFramePclFileName,*all_cloud);
            currentKeyFramePclMsg.clear();
        }
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_block");
    ros::NodeHandle nh;

    nh.param<std::string>("lidar_msg_name",lidar_topic, "/cloud_registered_body");
    nh.param<std::string>("key_frame_lidar_msg_name",key_frame_lidar_topic, "/key_frame_cloud_registered_body");
    nh.param<std::string>("odometry_msg_name",odometry_topic, "/Odometry"); 
    nh.param<std::string>("dataFolder",dataFolder, "/home/workspace/data/frames");
    nh.param<std::string>("keyFrameFolder",keyFrameFolder, "/home/workspace/data/key_frames");
    nh.param<std::string>("ground_truth_odom_msg_name",ground_truth_odom_topic, "/lidar_slam/odom"); 
    nh.param<std::string>("groundTruthName",groundTruthName, "/home/workspace/data/ground_truth.odom");
    //nh.param<bool>("dlo/pcd_save",pcd_save,false);
    nh.param<bool>("pcd_save",pcd_save,true);
    nh.param<bool>("odom_save",odom_save,true);
    //nh.param<bool>("use_key_frame",use_key_frame,false);
    //nh.param<bool>("transform",transform,false);
    nh.param<int>("frame_merge_num",frame_merge_num,10);

    if(!boost::filesystem::exists(dataFolder))
    {
        boost::filesystem::create_directories(dataFolder);
    }

    if(!boost::filesystem::exists(keyFrameFolder))
    {
        boost::filesystem::create_directories(keyFrameFolder);
    }

    /*if(!boost::filesystem::exists(groundTruthFolder))
    {
        boost::filesystem::create_directories(groundTruthFolder);
    }*/

    ros::Subscriber sub_key_frame_pcl = nh.subscribe(key_frame_lidar_topic, 200000, key_frame_pcl_cbk);
    ros::Subscriber sub_pcl = nh.subscribe(lidar_topic, 200000, pcl_cbk);
    ros::Subscriber sub_odom = nh.subscribe(odometry_topic, 200000, odom_cbk);
    ros::Subscriber sub_ground_truth_odom=nh.subscribe(ground_truth_odom_topic, 200000, ground_truth_odom_cbk);

    std::thread thread_process{process};
    ros::spin();

}