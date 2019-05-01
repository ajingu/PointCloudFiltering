#include "PointCloudVisualizer.h"

#include <chrono>

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
	std::chrono::system_clock::time_point start, end;
	double elapsed;

	start = std::chrono::system_clock::now();
	pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
	int mat_cols = depth_mat.cols;
	int mat_rows = depth_mat.rows;

	new_cloud.width = mat_cols;
	new_cloud.height = mat_rows;
	new_cloud.is_dense = false;
	new_cloud.points.resize(new_cloud.width * new_cloud.height);
	end = std::chrono::system_clock::now();
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	cout << "Time_intial: " << elapsed << endl;
	
	start = std::chrono::system_clock::now();

	depth_mat.forEach<float>([&new_cloud, &color_mat, &mat_cols, this](float &depth, const int position[2]) -> void {
		int idx = position[0] * mat_cols + position[1];
		new_cloud[idx].x = (position[1] - camera_intrinsics.cx) / camera_intrinsics.fx * depth;
		new_cloud[idx].y = (position[0] - camera_intrinsics.cy) / camera_intrinsics.fy * depth;
		new_cloud[idx].z = depth;

		Vec3b color = color_mat.at<Vec3b>(position[0], position[1]);
		new_cloud[idx].b = (uint8_t)color.val[0];
		new_cloud[idx].g = (uint8_t)color.val[1];
		new_cloud[idx].r = (uint8_t)color.val[2];
	});

	end = std::chrono::system_clock::now();
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	cout << "Time_set: " << elapsed << endl;

	pcl::transformPointCloud(new_cloud, new_cloud, transform_mat);

	master_cloud += new_cloud;

	cout << "new: " << new_cloud.points.size() << endl;
	cout << "master: " << master_cloud.points.size() << endl;

	start = std::chrono::system_clock::now();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = voxelGridFilter(master_cloud.makeShared(), 0.05f);
	end = std::chrono::system_clock::now();
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	cout << "Time_voxel: " << elapsed << endl;
	master_cloud.points = cloud_filtered->points;

	start = std::chrono::system_clock::now();
	viewer.removeAllPointClouds();
	cout << "filtered: " << cloud_filtered->points.size() << endl;
	viewer.addPointCloud(cloud_filtered, "cloud");
	end = std::chrono::system_clock::now();
	elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	cout << "Time_add: " << elapsed << endl;
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