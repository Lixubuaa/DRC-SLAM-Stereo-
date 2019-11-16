#ifndef G2OTYPES_H
#define G2OTYPES_H

#include "so3.h"
#include "VisualState.h"
#include "Converter.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/sba/types_six_dof_expmap.h"


namespace  g2o
{
	using namespace ORB_SLAM2;

	class VertexVisualState :public BaseVertex<6, VisualState>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			VertexVisualState() : BaseVertex<6, VisualState>() {}

		bool read(std::istream& is){ return true; }

		bool write(std::ostream& os) const{ return true; }

		virtual void setToOriginImpl() {
			_estimate = VisualState();
		}

		virtual void oplusImpl(const double* update_)  {
			Eigen::Map<const Vector6d> update(update_);
			
			_estimate.updateVS(update);
		}
	};

	class VertexInitialParameterR :public BaseVertex<3, Sophus::SO3>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			VertexInitialParameterR() : BaseVertex<3, Sophus::SO3>() {}

		bool read(std::istream& is){ return true; }

		bool write(std::ostream& os) const { return true; }

		virtual void setToOriginImpl() {
			_estimate = Sophus::SO3();
		}

		virtual void oplusImpl(const double* update_)  {
			Eigen::Map<const Vector3d> update(update_);
			setEstimate(estimate()*Sophus::SO3::exp(update));
		}
	};

	class EdgeVSPointXYZOnlyPose : public BaseUnaryEdge<2, Vector2d, VertexVisualState>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			EdgeVSPointXYZOnlyPose() :BaseUnaryEdge<2, Vector2d, VertexVisualState>() {}

		bool read(std::istream& is){ return true; }

		bool write(std::ostream& os) const{ return true; }

		void computeError()  {
			const VertexVisualState* v1 = static_cast<const VertexVisualState*>(_vertices[0]);
			Vector2d obs(_measurement);
			_error = obs - cam_project(computePc());
		}

		Vector3d computePc() {
			const VertexVisualState* vVS = static_cast<const VertexVisualState*>(_vertices[0]);

			const VisualState& ns = vVS->estimate();
			Matrix3d R = ns.Get_RotMatrix();
			Vector3d t = ns.Get_t();

			Vector3d Pc = R*Pw + t;

			return Pc;
		}

		bool isDepthPositive() {
			const VertexVisualState* v1 = static_cast<const VertexVisualState*>(_vertices[0]);
			return (computePc())(2)>0.0;
		}


		inline Vector2d project2d(const Vector3d& v) const {
			Vector2d res;
			res(0) = v(0) / v(2);
			res(1) = v(1) / v(2);
			return res;
		}
		Vector2d cam_project(const Vector3d & trans_xyz) const {
			Vector2d proj = project2d(trans_xyz);
			Vector2d res;
			res[0] = proj[0] * fx + cx;
			res[1] = proj[1] * fy + cy;
			return res;
		}

		//
		virtual void linearizeOplus();

		void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,
			 const Vector3d& Pw_) {
			fx = fx_;
			fy = fy_;
			cx = cx_;
			cy = cy_;
			Pw = Pw_;
		}

	protected:
		// Camera intrinsics
		double fx, fy, cx, cy;

		// Point position in world frame
		Vector3d Pw;
	};

	class EdgeStereoVSPointXYZOnlyPose : public BaseUnaryEdge<3, Vector3d, VertexVisualState>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			EdgeStereoVSPointXYZOnlyPose(): BaseUnaryEdge<3, Vector3d, VertexVisualState>() {}

		bool read(std::istream& is) { return true; }

		bool write(std::ostream& os) const { return true; }

		void computeError()  {
			const VertexVisualState* v1 = static_cast<const VertexVisualState*>(_vertices[0]);
			Vector3d obs(_measurement);
			_error = obs - cam_project(computePc());
		}

		Vector3d computePc() {
			const VertexVisualState* vVS = static_cast<const VertexVisualState*>(_vertices[0]);

			const VisualState& ns = vVS->estimate();
			Matrix3d R = ns.Get_RotMatrix();
			Vector3d t = ns.Get_t();

			Vector3d Pc = R*Pw + t;

			return Pc;
		}

		bool isDepthPositive() {
			const VertexVisualState* v1 = static_cast<const VertexVisualState*>(_vertices[0]);
			return (computePc())(2)>0.0;
		}


		virtual void linearizeOplus();

		Vector3d cam_project(const Vector3d & trans_xyz) const
		{
			const float invz = 1.0f / trans_xyz[2];
			Vector3d res;
			res[0] = trans_xyz[0] * invz*fx + cx;
			res[1] = trans_xyz[1] * invz*fy + cy;
			res[2] = res[0] - bf*invz;
			return res;
		}

		void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,
			const double& bf_, const const Vector3d& Pw_) {
			fx = fx_;
			fy = fy_;
			cx = cx_;
			cy = cy_;
			bf = bf_;
			Pw = Pw_;
		}

	protected:
		Vector3d Pw;
		double fx, fy, cx, cy, bf;
	};

	class EdgeVSPointXYZ : public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexVisualState>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			EdgeVSPointXYZ() :BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexVisualState>() {}

		bool read(std::istream& is){ return true; }

		bool write(std::ostream& os) const{ return true; }

		void computeError()  {
			//const VertexVisualState* v1 = static_cast<const VertexVisualState*>(_vertices[1]);
			//const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
			Vector2d obs(_measurement);
			_error = obs - cam_project(computePc());
		}

		Vector3d computePc() {
			const VertexVisualState* vVS = static_cast<const VertexVisualState*>(_vertices[1]);
			const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

			const VisualState& ns = vVS->estimate();

			Vector3d Pw = vPoint->estimate();
			Matrix3d R = ns.Get_RotMatrix();
			Vector3d t = ns.Get_t();

			Vector3d Pc = R*Pw + t;

			return Pc;
		}

		bool isDepthPositive() {
			//const VertexVisualState* v1 = static_cast<const VertexVisualState*>(_vertices[0]);
			return (computePc())(2)>0.0;
		}


		inline Vector2d project2d(const Vector3d& v) const {
			Vector2d res;
			res(0) = v(0) / v(2);
			res(1) = v(1) / v(2);
			return res;
		}
		Vector2d cam_project(const Vector3d & trans_xyz) const {
			Vector2d proj = project2d(trans_xyz);
			Vector2d res;
			res[0] = proj[0] * fx + cx;
			res[1] = proj[1] * fy + cy;
			return res;
		}

		//
		virtual void linearizeOplus();

		void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_) {
			fx = fx_;
			fy = fy_;
			cx = cx_;
			cy = cy_;
		}

	protected:
		// Camera intrinsics
		double fx, fy, cx, cy;

	};

	class EdgeStereoVSPointXYZ : public BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexVisualState>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			EdgeStereoVSPointXYZ() : BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexVisualState>() {}

		bool read(std::istream& is) { return true; }

		bool write(std::ostream& os) const { return true; }

		void computeError()  {
			//const VertexVisualState* v1 = static_cast<const VertexVisualState*>(_vertices[0]);
			Vector3d obs(_measurement);
			_error = obs - cam_project(computePc());
		}

		Vector3d computePc() {
			const VertexVisualState* vVS = static_cast<const VertexVisualState*>(_vertices[1]);
			const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

			const VisualState& ns = vVS->estimate();

			Vector3d Pw = vPoint->estimate();
			Matrix3d R = ns.Get_RotMatrix();
			Vector3d t = ns.Get_t();

			Vector3d Pc = R*Pw + t;

			return Pc;
		}

		bool isDepthPositive() {
			//const VertexVisualState* v1 = static_cast<const VertexVisualState*>(_vertices[0]);
			return (computePc())(2)>0.0;
		}


		virtual void linearizeOplus();

		Vector3d cam_project(const Vector3d & trans_xyz) const
		{
			const float invz = 1.0f / trans_xyz[2];
			Vector3d res;
			res[0] = trans_xyz[0] * invz*fx + cx;
			res[1] = trans_xyz[1] * invz*fy + cy;
			res[2] = res[0] - bf*invz;
			return res;
		}

		void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_,
			const double& bf_) {
			fx = fx_;
			fy = fy_;
			cx = cx_;
			cy = cy_;
			bf = bf_;
		}

	protected:
		double fx, fy, cx, cy, bf;
	};

	class EdgeVSIPR :public BaseMultiEdge<6, VisualState>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			EdgeVSIPR() : BaseMultiEdge<6, VisualState>() {
				resize(3);
			}

		bool read(std::istream& is){ return true; }

		bool write(std::ostream& os) const{ return true; }

		void computeError();

		virtual void linearizeOplus();
	};

	class EdgeIPR : public BaseBinaryEdge<3, VisualState, VertexInitialParameterR, VertexVisualState>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			EdgeIPR() : BaseBinaryEdge<3, VisualState, VertexInitialParameterR, VertexVisualState>() {
			}

		bool read(std::istream& is){ return true; }

		bool write(std::ostream& os) const{ return true; }

		void computeError();

		virtual void linearizeOplus();
	};

}
#endif // G2OTYPES_H