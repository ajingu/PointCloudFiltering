#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>

using namespace std;
using namespace cv;

typedef struct _intrinsics
{
	float cx;
	float cy;
	float fx;
	float fy;
}Intrinsics;

class PointCloudVisualizer
{
private:
	pcl::visualization::PCLVisualizer viewer;
	pcl::PointCloud<pcl::PointXYZRGB> master_cloud;
	Intrinsics camera_intrinsics;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float leaf_length);

public:
	PointCloudVisualizer(string name, const Intrinsics& _camera_intrinsics);
	bool wasStopped();
	void addCameraCoordinate(Eigen::Affine3f& camera_mat, string name);
	void addPointCloud(Mat& depth_mat, Mat& color_mat, Eigen::Matrix4f& transform_mat);
	void spinOnce();
};