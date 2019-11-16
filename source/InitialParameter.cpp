#include "stdafx.h"
#include "InitialParameter.h"
#include "Converter.h"

namespace ORB_SLAM2
{

	Sophus::SO3 InitialParameter::GetInitialRot()
	{
		boost::mutex::scoped_lock lock(mMutexInitialParameter);
		return _InitialRot;
	}
	void InitialParameter::SetInitialRot(const cv::Mat &InitialRot)
	{
		boost::mutex::scoped_lock lock(mMutexInitialParameter);
		_InitialRot = Sophus::SO3(Converter::toMatrix3d(InitialRot));
	}

	void InitialParameter::SetInitialRot(const Sophus::SO3 &InitialRot)
	{
		boost::mutex::scoped_lock lock(mMutexInitialParameter);
		_InitialRot = InitialRot;
	}
}