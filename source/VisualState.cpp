#include "VisualState.h"


namespace ORB_SLAM2
{
	VisualState::VisualState()
	{
		//_R.setIdentity();
		//_R = Sophus::SO3();
		//_t.setZero();
	}

	VisualState::VisualState(const VisualState &_vs):
		_t(_vs._t), _R(_vs._R)
	{
		//normalizeRotation();
	}

	//VisualState::VisualState(const cv::Mat& T)
	//{
	//	cv::Mat R_Mat = T.rowRange(0, 3).colRange(0, 3);
	//	cv::Mat t_Mat = T.rowRange(0, 3).col(3);
	//	Matrix3d R = Converter::toMatrix3d(R_Mat);
	//	_t = Converter::toVector3d(t_Mat);
	//	_R = Sophus::SO3(R);
	//}

	void VisualState::updateVS(const Vector6d& update)
	{
		Vector3d upd_R = update.segment<3>(0);
		Vector3d upd_t = update.segment<3>(3);

		Matrix3d R = Get_RotMatrix();

		_t = _t + R*upd_t;
		_R = _R*Sophus::SO3::exp(upd_R);
	}
}