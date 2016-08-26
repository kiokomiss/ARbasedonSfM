

#include <iostream>
#include <string.h>

#include "Distance.h"
#include "MultiCameraPnP.h"
#include "Visualization.h"

using namespace std;

#include <opencv2/core.hpp>
//#include <opencv2/gpu/gpu.hpp>

/*std::vector<cv::Mat> images;
std::vector<std::string> images_names;

int main(int argc, char** argv) {
	if (argc < 2) {
		cerr << "USAGE: " << argv[0] << " <path_to_images>  [downscale factor = 1.0]" << endl;
		return 0;
	}
	
	double downscale_factor = 1.0;
	if(argc >= 3)
		downscale_factor = atof(argv[3]);

	open_imgs_dir(argv[1],images,images_names,downscale_factor);
	if(images.size() == 0) { 
		cerr << "can't get image files" << endl;
		return 1;
	}

	cv::Ptr<MultiCameraPnP> distance = new MultiCameraPnP(images,images_names,string(argv[1]));
		distance->use_rich_features = true;
	    distance->use_gpu = false;
	
	cv::Ptr<VisualizerListener> visualizerListener = new VisualizerListener; //with ref-count
	distance->attach(visualizerListener);
	RunVisualizationThread();

	distance->RecoverDepthFromImages();

	//get the scale of the result cloud using PCA
	double scale_cameras_down = 1.0;
	{
		vector<cv::Point3d> cld = distance->getPointCloud();
		if (cld.size()==0) cld = distance->getPointCloudBeforeBA();
		cv::Mat_<double> cldm(cld.size(),3);
		for(unsigned int i=0;i<cld.size();i++) {
			cldm.row(i)(0) = cld[i].x;
			cldm.row(i)(1) = cld[i].y;
			cldm.row(i)(2) = cld[i].z;
		}
		cv::Mat_<double> mean;
		cv::PCA pca(cldm,mean,CV_PCA_DATA_AS_ROW);
		scale_cameras_down = pca.eigenvalues.at<double>(0) / 5.0;
		//if (scale_cameras_down > 1.0) {
		//	scale_cameras_down = 1.0/scale_cameras_down;
		//}
	}
	
	visualizerListener->update(distance->getPointCloud(),
							   distance->getPointCloudRGB(),
							   distance->getPointCloudBeforeBA(),
							   distance->getPointCloudRGBBeforeBA(),
							   distance->getCameras());
							   

	//ShowCloud(distance->getPointCloud(), 
	//		   distance->getPointCloudRGB(),
	//		   "baseline_only");
	//WaitForVisualizationThread();
	//return 1;
	
//	ShowClouds(distance->getPointCloud(), 
//			   distance->getPointCloudRGB(),
//			   distance->getPointCloudBeforeBA(),
//			   distance->getPointCloudRGBBeforeBA()
//			   );
	WaitForVisualizationThread();
}
*/
