#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>
#include <istream>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace Eigen;

const int DEPTH_BOUNDARY = 0;
// const int DEPTH_DIST = 4;

pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>* octree_cloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

int LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    if(!fAssociation.is_open())
    {
        printf("Failed to open association file!\n");
        return -1;
    }
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
    return 0;
}

int find_nearest_timestamp_index(double timestamp, vector<double> timestamp_list)
{
    double min_diff = 1e10;
    int min_index = 0;
    for(int i=0; i<timestamp_list.size(); i++)
    {
        double diff = abs(timestamp_list[i] - timestamp);
        if(diff < min_diff)
        {
            min_diff = diff;
            min_index = i;
        }
    }
    return min_index;
}

void build_octree()
{
    octree_cloud = new pcl::octree::OctreePointCloudDensity<pcl::PointXYZRGB>(0.04);
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->is_dense = true;
    octree_cloud->setInputCloud(cloud);
    octree_cloud->addPointsFromInputCloud();
    octree_cloud->defineBoundingBox(-100, -100, -100, 100, 100, 100);
}

int main(int argc, char **argv)
{
    if (argc != 5) {
        cout << "Usage: ./RGBD_Mapping trajectory_path data_dir pcd_save_path depth_sample_interval" << endl;
        return -1;
    }
    // string trajectory_path="/home/lg/mapping/CameraTrajectory.txt";
    // string data_dir="/home/lg/rosbags/map-1230-room_converted";
    // string save_path="map.pcd";

    string trajectory_path=argv[1];
    string data_dir=argv[2];
    string save_path=argv[3];
    int depth_sample_interval = atoi(argv[4]);

    double fx=837.7212741256517, fy=837.9832241106268, cx=373.10712027511505, cy=288.6532662167279;
    double k1=0.1450250121142708, k2=-0.6241891990756884, p1=-0.001003591349543722, p2=5.110535636253432e-05;
    float depth_map_factor = 1. / 1000.0; // 深度值转换到真实深度的因子，深度图像中的深度值为毫米，转换到米需要除以1000

    // Transformation from rgb to body
    Eigen::MatrixXd T_body_rgb(4,4); 
    T_body_rgb << -7.44865037e-03,  9.99942840e-01, -7.67037823e-03,  1.65648915e-03,
                  -9.99972104e-01, -7.45268749e-03, -4.97877147e-04, -5.28298498e-02,
                  -5.55013620e-04,  7.66645575e-03,  9.99970458e-01,  1.84414142e-02,
                   0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00;
    
    cv::Mat K = cv::Mat::eye(3,3,CV_64F);
    K.at<double>(0,0) = fx;
    K.at<double>(1,1) = fy;
    K.at<double>(0,2) = cx;
    K.at<double>(1,2) = cy;
    cv::Mat d = cv::Mat::zeros(5,1,CV_64F);
    d.at<double>(0,0) = k1;
    d.at<double>(1,0) = k2;
    d.at<double>(2,0) = p1;
    d.at<double>(3,0) = p2;

    /***** load image name list *****/
    string association_path = data_dir + "/associations.txt";
    vector<string> rgb_image_name_list, depth_image_name_list;
    vector<double> timestamp_list;
    if(LoadImages(association_path, rgb_image_name_list, depth_image_name_list, timestamp_list)==-1)
    {
        return -1;
    }

    build_octree();

    ifstream trajectory_file;
    trajectory_file.open(trajectory_path);
    if(!trajectory_file.is_open())
    {
        printf("Failed to open trajectory file!\n");
        return -1;
    }

    /***** main loop *****/
    string line;
    while(getline(trajectory_file, line))
    {
        double timestamp, x, y, z, qx, qy, qz, qw;
        sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &timestamp, &x, &y, &z, &qx, &qy, &qz, &qw);
        Vector3d P(x, y, z);
        Quaterniond Q(qw, qx, qy, qz);
        Matrix3d R = Q.toRotationMatrix();
        int image_index = find_nearest_timestamp_index(timestamp, timestamp_list);
        string rgb_image_path = data_dir + "/" + rgb_image_name_list[image_index];
        string depth_image_path = data_dir + "/" + depth_image_name_list[image_index];
        printf("%.6f\n", timestamp);
        cv::Mat rgb_image = cv::imread(rgb_image_path, cv::IMREAD_UNCHANGED);
        cv::Mat depth_image = cv::imread(depth_image_path, cv::IMREAD_UNCHANGED);

        if((fabs(depth_map_factor-1.0f)>1e-5) && depth_image.type()!=CV_32F)
            depth_image.convertTo(depth_image, CV_32F, depth_map_factor);
        
        for(int v=DEPTH_BOUNDARY; v < depth_image.rows-DEPTH_BOUNDARY; v+=depth_sample_interval)
        {
            for(int u=DEPTH_BOUNDARY; u < depth_image.cols-DEPTH_BOUNDARY; u+=depth_sample_interval)
            {
                float depth = depth_image.at<float>(v,u);
                if(depth < 0.3 || depth > 3.5)
                    continue;
                cv::Point2f point_2d = cv::Point2f(u, v);
                
                // point 2d to point 3d
                std::vector<cv::Point2f> point_2d_vec, un_point_2d_vec;
                point_2d_vec.push_back(point_2d);
                cv::undistortPoints(point_2d_vec, un_point_2d_vec, K, d);
                cv::Point3f point_3d(un_point_2d_vec[0].x, un_point_2d_vec[0].y, 1.);
                point_3d.x = point_3d.x * depth;
                point_3d.y = point_3d.y * depth;
                point_3d.z = point_3d.z * depth;
                
                // point 3d in camera to point 3d in world
                Vector4d point_3d_camera(point_3d.x, point_3d.y, point_3d.z, 1.);
                Vector4d point_3d_body = T_body_rgb * point_3d_camera;
                Vector3d point_3d_world = R * point_3d_body.head(3) + P;
                Vector3d point_3d_show(point_3d_world(0), point_3d_world(1), point_3d_world(2));
                
                // add point to cloud
                pcl::PointXYZRGB search_point;
                search_point.x = point_3d_show(0);
                search_point.y = point_3d_show(1);
                search_point.z = point_3d_show(2);
                search_point.r = rgb_image.at<cv::Vec3b>(v,u)[2];
                search_point.g = rgb_image.at<cv::Vec3b>(v,u)[1];
                search_point.b = rgb_image.at<cv::Vec3b>(v,u)[0];
                if(octree_cloud->getVoxelDensityAtPoint(search_point) > 1)
                    continue;  // 利用octree忽略重复点
                octree_cloud->addPointToCloud(search_point, cloud);
            }
        }
    }

    // save cloud
    pcl::io::savePCDFileASCII(save_path, *cloud);

    // visualization
    pcl::visualization::CloudViewer viewer("cloud viewer");
    // cloud.make_NormalEstimationOMP();
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
    return 0;
}