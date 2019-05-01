#include <iostream> 

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>

#include "PointCloudVisualizer.h"

using namespace cv;
using namespace std;

#define MAX_NUM 17
#define DELTA_TRANSLATION_X 0.05


int main(int argc, char** argv)
{
	try
	{
		Intrinsics camera_intrinsics = {256.4, 202, 363, 364};
		PointCloudVisualizer visualizer = PointCloudVisualizer("Simple Viewer", camera_intrinsics);


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

			Mat color_mat(dst.rows, dst.cols, CV_8UC3, Vec3b(255, 0, 0));


			Eigen::Translation<float, 3> translation = Eigen::Translation<float, 3>(-DELTA_TRANSLATION_X * (i - 1), 0.0f, 0.0f);
			Eigen::DiagonalMatrix<float, 3> scaling = Eigen::Scaling(-1.0f, -1.0f, 1.0f);
			Eigen::Quaternionf rotate = Eigen::Quaternionf::Identity();
			Eigen::Affine3f cameraMatrix = translation * scaling * rotate;
			Eigen::Matrix4f transformMatrix = cameraMatrix.matrix();


			visualizer.addPointCloud(dst, color_mat, transformMatrix);
			visualizer.addCameraCoordinate(cameraMatrix, src_name);

			while (!visualizer.wasStopped())
			{
				visualizer.spinOnce();

				if (GetKeyState(VK_ESCAPE) < 0) {
					break;
				}
			}
		}
	}
	catch (exception& ex)
	{
		cout << ex.what() << endl;
	}

	return 0;
}