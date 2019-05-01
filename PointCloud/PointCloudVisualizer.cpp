#include "PointCloudVisualizer.h"

PointCloudVisualizer::PointCloudVisualizer(string name, const Intrinsics& _camera_intrinsics)
{
	viewer.setWindowName(name);
	master_cloud = pcl::PointCloud<pcl::PointXYZRGB>();
	camera_intrinsics = _camera_intrinsics;
}

bool PointCloudVisualizer::wasStopped()
{
	return viewer.wasStopped();
}


void PointCloudVisualizer::addPointCloud(Mat& depth_mat, Mat& color_mat, Eigen::Matrix4f& transform_mat)
{
	pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
	int mat_cols = depth_mat.cols;
	int mat_rows = depth_mat.rows;

	new_cloud.width = mat_cols;
	new_cloud.height = mat_rows;
	new_cloud.is_dense = false;
	new_cloud.points.resize(new_cloud.width * new_cloud.height);

	for (int row = 0; row < mat_rows; row++)
	{
		for (int col = 0; col < mat_cols; col++)
		{
			float depth = depth_mat.at<float>(row, col);
			Vec3b color = color_mat.at<Vec3b>(row, col);

			int idx = row * mat_cols + col;

			new_cloud[idx].x = (col - camera_intrinsics.cx) / camera_intrinsics.fx * depth;
			new_cloud[idx].y = (row - camera_intrinsics.cy) / camera_intrinsics.fy * depth;
			new_cloud[idx].z = depth;
			new_cloud[idx].b = (uint8_t)color.val[0];
			new_cloud[idx].g = (uint8_t)color.val[1];
			new_cloud[idx].r = (uint8_t)color.val[2];
		}
	}

	pcl::transformPointCloud(new_cloud, new_cloud, transform_mat);

	master_cloud += new_cloud;

	cout << "new: " << new_cloud.points.size() << endl;
	cout << "master: " << master_cloud.points.size() << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = voxelGridFilter(master_cloud.makeShared(), 0.01f);
	master_cloud.points = cloud_filtered->points;

	viewer.removeAllPointClouds();
	cout << "filtered: " << cloud_filtered->points.size() << endl;
	viewer.addPointCloud(cloud_filtered, "cloud");
}

void PointCloudVisualizer::addCameraCoordinate(Eigen::Affine3f& camera_mat, string name)
{
	viewer.addCoordinateSystem(0.2, camera_mat, name, 0);
}

void PointCloudVisualizer::spinOnce()
{
	viewer.spinOnce();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudVisualizer::voxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float leaf_length)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(leaf_length, leaf_length, leaf_length);
	sor.filter(*cloud_filtered);

	return cloud_filtered;
}