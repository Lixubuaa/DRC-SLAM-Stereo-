#ifndef INITIALPARAMETER_H
#define INITIALPARAMETER_H

#include<opencv2/core/core.hpp>
#include<boost/thread.hpp>
#include "so3.h"

namespace ORB_SLAM2
{
	class InitialParameter
	{
	public:
		Sophus::SO3 GetInitialRot();
		void SetInitialRot(const cv::Mat &InitialRot);
		void SetInitialRot(const Sophus::SO3 &InitialRot);
	protected:
		boost::mutex mMutexInitialParameter;
	private:
		Sophus::SO3 _InitialRot;
	};
}

#endif INITIALPARAMETER_H