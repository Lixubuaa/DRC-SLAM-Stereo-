#ifndef DYNAMICPROCESS_H
#define DYNAMICPROCESS_H

#include "Map.h"
#include "Frame.h"
#include "KeyFrame.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>


namespace ORB_SLAM2
{
	class DynamicProcess
	{
	public:
		DynamicProcess();
		std::vector<cv::Rect> Process(cv::Mat,cv::Mat,cv::Mat,cv::Mat& dynamic_mask);
		ORB_SLAM2::Map* sWorld;

		KeyFrame* mCurrentKeyFrame;
		cv::Mat im, im_r;
		
	};
}
#endif