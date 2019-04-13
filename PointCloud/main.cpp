#include <iostream> 

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

using namespace cv;
using namespace std;

#define MAX_NUM 3
#define DELTA_TRANSLATION_X 0.05


pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_length)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(leaf_length, leaf_length, leaf_length);
	sor.filter(*cloud_filtered);

	return cloud_filtered;
}

int main(int argc, char** argv)
{
	float fx = 363, fy = 364, cx = 256.4, cy = 202;

	pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");

	pcl::PointCloud<pcl::PointXYZ> masterCloud;

	for (int i = 1; i < MAX_NUM + 1; i++)
	{
		string src_name = "./data/depth";
		if (i < 10)
		{
			src_name += "0";
		}
		src_name += to_string(i) + ".png";

		Mat src, dst;
		src = imread(src_name, IMREAD_UNCHANGED);
		src.convertTo(dst, CV_32FC1, 1 / 5000.0f);

		cout << dst.cols << ", " << dst.rows << endl;

		pcl::PointCloud<pcl::PointXYZ> newCloud;
		newCloud.width = dst.cols;
		newCloud.height = dst.rows;
		newCloud.is_dense = false;
		newCloud.points.resize(newCloud.width * newCloud.height);

		for (int row = 0; row < dst.rows; row++)
		{
			for (int col = 0; col < dst.cols; col++)
			{
				float depth = dst.at<float>(row, col);

				int idx = row * dst.cols + col;
				newCloud[idx].x = (col - cx) / fx * depth;
				newCloud[idx].y = (row - cy) / fy * depth;
				newCloud[idx].z = depth;
			}
		}

		masterCloud += newCloud;

		cout << "new: " <<  newCloud.points.size() << endl;
		cout << "master: " << masterCloud.points.size() << endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = voxel_grid_filter(masterCloud.makeShared(), 0.05f);
		masterCloud.points = cloud_filtered->points;

		viewer.removeAllPointClouds();
		cout << "filtered: " << cloud_filtered->points.size() << endl;
		viewer.addPointCloud(cloud_filtered, "cloud");
		
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();

			if (GetKeyState(VK_ESCAPE) < 0) {
				break;
			}
		}
	}

	return (0);
}