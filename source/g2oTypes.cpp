#include "stdafx.h"
#include "g2oTypes.h"

namespace g2o
{
	using namespace std;
	using namespace ORB_SLAM2;

	void EdgeVSIPR::computeError()
	{
		const VertexVisualState* _vsi = static_cast<const VertexVisualState*>(_vertices[0]);
		const VertexVisualState* _vsj = static_cast<const VertexVisualState*>(_vertices[1]);
		const VertexInitialParameterR* _IniR = static_cast<const VertexInitialParameterR*>(_vertices[2]);

		VisualState vsi, vsj;
		vsi = _vsi->estimate();
		vsj = _vsj->estimate();
		Sophus::SO3 Ri = vsi.Get_R();
		Vector3d ti = vsi.Get_t();
		Sophus::SO3 Rj = vsj.Get_R();
		Vector3d tj = vsj.Get_t();

		Sophus::SO3 IniR = _IniR->estimate();
		Sophus::SO3 Rm = _measurement.Get_R();
		Vector3d tm = _measurement.Get_t();

		Vector6d error;
		error.setZero();
		error.segment<3>(0) = (Rm.inverse()*Rj*Ri.inverse()).log();
		error.segment<3>(3) = Rm.inverse().matrix()*(tj - Rj*Ri.inverse()*ti - Rj*IniR*tm);
		_error = error;
	}

	void EdgeVSIPR::linearizeOplus()
	{
		const VertexVisualState* _vsi = static_cast<const VertexVisualState*>(_vertices[0]);
		const VertexVisualState* _vsj = static_cast<const VertexVisualState*>(_vertices[1]);
		const VertexInitialParameterR* _IniR = static_cast<const VertexInitialParameterR*>(_vertices[2]);

		VisualState vsi, vsj;
		vsi = _vsi->estimate();
		vsj = _vsj->estimate();
		Matrix3d Ri = vsi.Get_RotMatrix();
		Vector3d ti = vsi.Get_t();
		Matrix3d Rj = vsj.Get_RotMatrix();
		Vector3d tj = vsj.Get_t();

		Matrix3d IniR = _IniR->estimate().matrix();
		Matrix3d Rm = _measurement.Get_RotMatrix();
		Vector3d tm = _measurement.Get_t();

		Vector3d errR = _error.segment<3>(0);
		Matrix3d JrInv_errR = Sophus::SO3::JacobianRInv(errR);
		
		Matrix6d JVSi;
		JVSi.setZero();
		JVSi.block<3, 3>(0, 0) = -JrInv_errR*Ri;
		JVSi.block<3, 3>(3, 0) = -Rm.inverse()*Rj*Sophus::SO3::hat(Ri.inverse()*ti);
		JVSi.block<3, 3>(3, 3) = -Rm.inverse()*Rj*Ri.inverse();
		
		Matrix6d JVSj;
		JVSj.setZero();
		JVSj.block<3, 3>(0, 0) = JrInv_errR*Ri;
		JVSj.block<3, 3>(3, 0) = Rm.inverse()*Rj*Sophus::SO3::hat(Ri.inverse()*ti) + Rm.inverse()*Rj*Sophus::SO3::hat(IniR*tm);
		JVSj.block<3, 3>(3, 3) = Rm.inverse();

		Matrix<double, 6, 3> JIP;
		JIP.setZero();
		JIP.block<3, 3>(3, 0) = -Rm.inverse()*Rj*IniR*Sophus::SO3::hat(tm);

		// Evaluate _jacobianOplus
		_jacobianOplus[0] = JVSi;
		_jacobianOplus[1] = JVSj;
		_jacobianOplus[2] = JIP;
	}

	void EdgeVSPointXYZOnlyPose::linearizeOplus()
	{
		const VertexVisualState* vVisualState = static_cast<const VertexVisualState*>(_vertices[0]);

		Matrix3d Rcw = vVisualState->estimate().Get_RotMatrix();
		Vector3d tcw = vVisualState->estimate().Get_t();

		Vector3d Pc = Rcw*Pw + tcw;
		double x = Pc[0];
		double y = Pc[1];
		double z = Pc[2];

		// Jacobian of camera projection
		Matrix<double, 2, 3> Maux;
		Maux.setZero();
		Maux(0, 0) = fx;
		Maux(0, 1) = 0;
		Maux(0, 2) = -x / z*fx;
		Maux(1, 0) = 0;
		Maux(1, 1) = fy;
		Maux(1, 2) = -y / z*fy;
		Matrix<double, 2, 3> Jpi = Maux / z;

		Matrix<double, 2, 3> JdR = Jpi*Sophus::SO3::hat(Rcw*Pw)*Rcw;
		Matrix<double, 2, 3> Jdt = -Jpi*Rcw;

		Matrix<double, 2, 6> JVisualState = Matrix<double, 2, 6>::Zero();
		JVisualState.block<2, 3>(0, 0) = JdR;
		JVisualState.block<2, 3>(0, 3) = Jdt;

		_jacobianOplusXi = JVisualState;
	}

	void EdgeStereoVSPointXYZOnlyPose::linearizeOplus()
	{
		const VertexVisualState* vVisualState = static_cast<const VertexVisualState*>(_vertices[0]);

		Matrix3d Rcw = vVisualState->estimate().Get_RotMatrix();
		Vector3d tcw = vVisualState->estimate().Get_t();

		Vector3d Pc = Rcw*Pw + tcw;
		double x = Pc[0];
		double y = Pc[1];
		double z = Pc[2];

		// Jacobian of camera projection
		Matrix<double, 3, 3> Maux;
		Maux.setZero();
		Maux(0, 0) = fx;
		Maux(0, 1) = 0;
		Maux(0, 2) = -x / z*fx;
		Maux(1, 0) = 0;
		Maux(1, 1) = fy;
		Maux(1, 2) = -y / z*fy;
		Maux(2, 0) = fx;
		Maux(2, 1) = 0;
		Maux(2, 2) = -x / z*fx + bf / z;
		Matrix<double, 3, 3> Jpi = Maux / z;

		Matrix<double, 3, 3> JdR = Jpi*Sophus::SO3::hat(Rcw*Pw)*Rcw;
		Matrix<double, 3, 3> Jdt = -Jpi*Rcw;

		Matrix<double, 3, 6> JVisualState = Matrix<double, 3, 6>::Zero();
		JVisualState.block<3, 3>(0, 0) = JdR;
		JVisualState.block<3, 3>(0, 3) = Jdt;

		_jacobianOplusXi = JVisualState;
	}

	void EdgeVSPointXYZ::linearizeOplus()
	{
		const VertexVisualState* vVisualState = static_cast<const VertexVisualState*>(_vertices[1]);
		const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

		Matrix3d Rcw = vVisualState->estimate().Get_RotMatrix();
		Vector3d tcw = vVisualState->estimate().Get_t();
		Vector3d Pw = vPoint->estimate();

		Vector3d Pc = Rcw*Pw + tcw;
		double x = Pc[0];
		double y = Pc[1];
		double z = Pc[2];

		// Jacobian of camera projection
		Matrix<double, 2, 3> Maux;
		Maux.setZero();
		Maux(0, 0) = fx;
		Maux(0, 1) = 0;
		Maux(0, 2) = -x / z*fx;
		Maux(1, 0) = 0;
		Maux(1, 1) = fy;
		Maux(1, 2) = -y / z*fy;
		Matrix<double, 2, 3> Jpi = Maux / z;

		Matrix<double, 2, 3> JdR = Jpi*Sophus::SO3::hat(Rcw*Pw)*Rcw;
		Matrix<double, 2, 3> Jdt = -Jpi*Rcw;

		Matrix<double, 2, 6> JVisualState = Matrix<double, 2, 6>::Zero();
		JVisualState.block<2, 3>(0, 0) = JdR;
		JVisualState.block<2, 3>(0, 3) = Jdt;

		_jacobianOplusXi = Jdt;
		_jacobianOplusXj = JVisualState;
	}

	void EdgeStereoVSPointXYZ::linearizeOplus()
	{
		const VertexVisualState* vVisualState = static_cast<const VertexVisualState*>(_vertices[1]);
		const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

		Matrix3d Rcw = vVisualState->estimate().Get_RotMatrix();
		Vector3d tcw = vVisualState->estimate().Get_t();
		Vector3d Pw = vPoint->estimate();

		Vector3d Pc = Rcw*Pw + tcw;
		double x = Pc[0];
		double y = Pc[1];
		double z = Pc[2];

		// Jacobian of camera projection
		Matrix<double, 3, 3> Maux;
		Maux.setZero();
		Maux(0, 0) = fx;
		Maux(0, 1) = 0;
		Maux(0, 2) = -x / z*fx;
		Maux(1, 0) = 0;
		Maux(1, 1) = fy;
		Maux(1, 2) = -y / z*fy;
		Maux(2, 0) = fx;
		Maux(2, 1) = 0;
		Maux(2, 2) = -x / z*fx + bf / z;
		Matrix<double, 3, 3> Jpi = Maux / z;

		Matrix<double, 3, 3> JdR = Jpi*Sophus::SO3::hat(Rcw*Pw)*Rcw;
		Matrix<double, 3, 3> Jdt = -Jpi*Rcw;

		Matrix<double, 3, 6> JVisualState = Matrix<double, 3, 6>::Zero();
		JVisualState.block<3, 3>(0, 0) = JdR;
		JVisualState.block<3, 3>(0, 3) = Jdt;

		_jacobianOplusXi = Jdt;
		_jacobianOplusXj = JVisualState;
	}

	void EdgeIPR::computeError()
	{
		const VertexInitialParameterR* _IniR = static_cast<const VertexInitialParameterR*>(_vertices[0]);
		const VertexVisualState* _vsj = static_cast<const VertexVisualState*>(_vertices[1]);

		Matrix3d Rj = _vsj->estimate().Get_RotMatrix();
		Vector3d tj = _vsj->estimate().Get_t();
		Matrix3d IniR = _IniR->estimate().matrix();

		Vector3d t_m = _measurement.Get_t();

		Vector3d error;
		error = tj - Rj*IniR*t_m;

		_error = error;
	}

	void EdgeIPR::linearizeOplus()
	{
		const VertexInitialParameterR* _IniR = static_cast<const VertexInitialParameterR*>(_vertices[0]);
		const VertexVisualState* _vsj = static_cast<const VertexVisualState*>(_vertices[1]);

		Matrix3d Rj = _vsj->estimate().Get_RotMatrix();
		Matrix3d IniR = _IniR->estimate().matrix();

		Vector3d t_m = _measurement.Get_t();

		Matrix3d JIPR;
		JIPR = Rj*IniR*Sophus::SO3::hat(t_m);

		Matrix<double, 3, 6> JVS;
		JVS.setZero();

		_jacobianOplusXi = JIPR;
		_jacobianOplusXj = JVS;
	}

}