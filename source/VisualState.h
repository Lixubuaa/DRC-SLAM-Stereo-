#ifndef VISUALSTATE_H
#define VISUALSTATE_H

#include "Eigen/Geometry"
#include "so3.h"

namespace ORB_SLAM2
{

	using namespace Eigen;

	typedef Matrix<double, 6, 1> Vector6d;

	class VisualState
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		VisualState();
		VisualState(const VisualState& _ns);

		//Quaterniond Get_qR(){return _qR;}     // rotation
		Sophus::SO3 Get_R() const{ return _R; }
		//Matrix3d Get_RotMatrix(){return _qR.toRotationMatrix();}   // rotation matrix
		Matrix3d Get_RotMatrix() const{ return _R.matrix(); }
		Vector3d Get_t() const{ return _t; } // transportation
		void Set_Tra(const Vector3d &tra){ _t = tra; }
		void Set_Rot(const Matrix3d &rot){ _R = Sophus::SO3(rot); }
		void Set_Rot(const Sophus::SO3 &rot){ _R = rot; }


		void updateVS(const Vector6d& update);

	public:
		/*
		* Note:
		* don't add pointer as member variable.
		* operator = is used in g2o
		*/

		Vector3d _t; // transportation
		//Quaterniond _qR;     // rotation
		Sophus::SO3 _R;
	};

}
#endif // VISUALSTATE_H
