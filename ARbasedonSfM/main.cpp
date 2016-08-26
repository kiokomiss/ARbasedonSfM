#include <iostream>
#include <string.h>

#include "Distance.h"
#include "MultiCameraPnP.h"
#include "Visualization.h"

#include <opencv2/core.hpp>
#include <Windows.h>
#include <fstream>
////////////////////////////////////////////////////////////////////
// Standard includes:
#include <opencv2/opencv.hpp>
#include <gl/gl.h>
#include <gl/glu.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;
/**
 * Processes a recorded video or live view from web-camera and allows you to adjust homography refinement and 
 * reprojection threshold in runtime.
 */
void processVideo(cv::VideoCapture& capture, double downscale_factor);
void getFrameBatch(cv::VideoCapture&  capture, std::vector<cv::Mat>& images, int imgBatchCount = 3, double downscale_factor = 0.5, bool shouldQuit = false);

class VisualizerListener : public SfMUpdateListener {
public:
	void update(std::vector<cv::Point3d> pcld,
		std::vector<cv::Vec3b> pcldrgb,
		std::vector<cv::Point3d> pcld_alternate,
		std::vector<cv::Vec3b> pcldrgb_alternate,
		std::vector<cv::Matx34d> cameras) {
		ShowClouds(pcld, pcldrgb, pcld_alternate, pcldrgb_alternate);

		vector<cv::Matx34d> v = cameras;
		for (unsigned int i = 0; i<v.size(); i++) {
			stringstream ss; ss << "camera" << i;
			cv::Matx33f R;
			R(0, 0) = v[i](0, 0); R(0, 1) = v[i](0, 1); R(0, 2) = v[i](0, 2);
			R(1, 0) = v[i](1, 0); R(1, 1) = v[i](1, 1); R(1, 2) = v[i](1, 2);
			R(2, 0) = v[i](2, 0); R(2, 1) = v[i](2, 1); R(2, 2) = v[i](2, 2);
			visualizerShowCamera(R, cv::Vec3f(v[i](0, 3), v[i](1, 3), v[i](2, 3)), 255, 0, 0, 0.2, ss.str());
		}
	}
};

std::vector<cv::Mat> images;

int main(int argc, const char * argv[])
{
	
	double downscale_factor = 0.5;
    if (argc < 2)
    {
        std::cout << "Input video not specified" << std::endl;
        std::cout << "Usage: markerless_sfm_demo [filepath to recorded video or image]" << std::endl;
        return 1;
    }

	std::string input = argv[1];
  
     cv::VideoCapture cap(input);
     if (cap.open(input))
            {
				std::cout << "ProcessVideo" << std::endl;
                processVideo(cap, downscale_factor);
            }
			else std::cout << "!cap.open(input)" << std::endl;
   
    return 0;
}

void processVideo(cv::VideoCapture& capture, double downscale_factor)
{
	int imgBatchCount = 5;
	cv::Mat currentFrame;
	bool shouldQuit = false;

	vector<cv::Point3d> cld;
	vector<cv::Matx34d> cmrs;
	std::ofstream  cloudlog;
	cloudlog.open("C:\\Users\\evelina.boytsova\\Documents\\Visual Studio 2015\\Projects\\ARbasedonSfM\\Cloud.txt");
	capture >> currentFrame;

	cv::Size frameSize(currentFrame.cols*downscale_factor, currentFrame.rows*downscale_factor);

	cv::Ptr<VisualizerListener> visualizerListener = new VisualizerListener;

	getFrameBatch(capture, images, imgBatchCount, downscale_factor, shouldQuit);

	cv::Ptr<MultiCameraPnP> distance = new MultiCameraPnP(images);
	distance->use_rich_features = false;

	distance->attach(visualizerListener);
	RunVisualizationThread();

	distance->RecoverDepthFromImages();
	
	images.clear();
	//bool finish = false;

	while ((!currentFrame.empty()) ) {
		    
		    getFrameBatch(capture, images, imgBatchCount, downscale_factor, shouldQuit);
			std::cout << "======================================================================\n";
			std::cout << "========================   Next Batch Start  ========================\n";
			std::cout << "======================================================================\n";
			distance->attach(visualizerListener);
		//	RunVisualizationThread();

			distance->RecoverDepthFromImages();
		    images.clear();
			//get the scale of the result cloud using PCA
			double scale_cameras_down = 1.0;
			
				std::vector<cv::Point3d> add_to_cloud = distance->getPointCloud();
				pcl::PointCloud<pcl::PointXYZ> cloud;

				// Fill in the cloud data
				cloud.width = add_to_cloud.size();
				cloud.height = 1;
				cloud.is_dense = false;
				cloud.points.resize(cloud.width * cloud.height);

				for (unsigned int i = 0; i<add_to_cloud.size(); i++) {
					cld.push_back(add_to_cloud[i]);
					cloudlog << add_to_cloud[i].x << " " << add_to_cloud[i].y << " " << add_to_cloud[i].z << endl;
					cloud.points[i].x = add_to_cloud[i].x;
					cloud.points[i].y = add_to_cloud[i].y;
					cloud.points[i].z = add_to_cloud[i].z;
				}

				pcl::io::savePCDFileASCII("C:\\Users\\evelina.boytsova\\Documents\\Visual Studio 2015\\Projects\\ARbasedonSfM\\test_pcd.pcd", cloud);

				if (cld.size() == 0) cld = distance->getPointCloudBeforeBA();
				cv::Mat_<double> cldm(cld.size(), 3);
				for (unsigned int i = 0; i < cld.size(); i++) {
					cldm.row(i)(0) = cld[i].x;
					cldm.row(i)(1) = cld[i].y;
					cldm.row(i)(2) = cld[i].z;
				}
				cv::Mat_<double> mean;
				cv::PCA pca(cldm, mean, CV_PCA_DATA_AS_ROW);
				scale_cameras_down = pca.eigenvalues.at<double>(0) / 5.0;
			
			std::vector<cv::Matx34d>  add_to_cmrs = distance->getCameras();
			for (unsigned int i = 0; i<add_to_cmrs.size(); i++) {
				cmrs.push_back(add_to_cmrs[i]);
			}
			visualizerListener->update(cld,
				distance->getPointCloudRGB(),
				distance->getPointCloudBeforeBA(),
				distance->getPointCloudRGBBeforeBA(),
				cmrs);
			
			
			/*
			WaitForVisualizationThread();
			*/
		}
	cloudlog.close();
	
   }


void getFrameBatch(cv::VideoCapture&  capture, std::vector<cv::Mat>& images, int imgBatchCount, double downscale_factor, bool shouldQuit) {

    cv::Mat currentFrame;
	for (unsigned int i = 0; i < (imgBatchCount); i++) { //images.size()<imgBatchCount  ??
		capture >> currentFrame;
	
			cout << " I=" << i;
			if (currentFrame.empty())
			{
				std::cout << "Cannot open video capture device" << std::endl;
				shouldQuit = true;
				return;
			}

			if (downscale_factor != 1.0)
				cv::resize(currentFrame, currentFrame, cv::Size(), downscale_factor, downscale_factor, cv::INTER_AREA);

			images.push_back(currentFrame);
	
	}
}

