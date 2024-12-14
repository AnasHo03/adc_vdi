#include <sl/Camera.hpp>
#include <opencv2.opencv.hpp>

int main (int argc, char **argv){
	sl::Camera zed;
	
	sl::InitParameters init_params;
	init_params.camera_resolution = sl::RESOLUTION::HD720;
	init_params.camera_fps = 30;
	
	sl::ERROR_CODE err=zed.open(init_params);
	if (err != sl::ERROR_CODE::SUCCESS){
		std::cerr<< "ERROR"<< sl::toString(err)<<endl;
		return 1;
	}
	
	sl::Mat zed_image;
	cv::Mat cv_image;
	
	cv::namedWindow("ZED 2i Camera", cv::WINDOW_AUTOSIZE);
	
	while (True){
		if (zed.grab()==sl::ERRORR_CODE::SUCCESS){
			zed.retrieveImage(zed_image, sl::VIEW::LEFT);
			cv_image = cv::Mat(zed_imge.getHeight(),zed_image.getWidth(),CV_8UC4, zed_image.getPttr<sl::uchar1>(sl::MEM::CPU));
			cv::imshow("ZED 2i Camera", cv_image);
		
		
		}
		
		if  (ccv::waitKey(30) >= 0){
			break;
		
		}
		
		
	
	}
	zed.close();
	return 0;



}

