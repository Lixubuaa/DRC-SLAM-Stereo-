/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "BowVector.h"
#include "FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "VisualState.h"

#include<boost/thread.hpp>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;
class VisualState;

class KeyFrame
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

	//记录存储序号
	void SetSaveNum(int _SaveNum);
	int GetSaveNum();

	//盲道、障碍物、地面等参数获取函数
	void SetObstacle(std::vector<pair<cv::Point3f, int>> _obstacle);
	void SetBlindwaypar(std::vector<cv::Point3f> _blindwaypar);
	std::vector<pair<cv::Point3f, int>> GetObstacle();
	std::vector<cv::Point3f> GetBlindwaypar();

	//Get Left&Right Image
	cv::Mat GetImage();
	cv::Mat GetRightImage();

	cv::Mat GetColorImage();
	cv::Mat GetColorRightImage();
    // Pose functions
    void SetPose(const cv::Mat &Tcw);
	void SetIntPose(const cv::Mat &Zcw);
    cv::Mat GetPose();
	cv::Mat GetIntPose();
    cv::Mat GetPoseInverse();
	cv::Mat GetIntPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
	cv::Mat GetIntRotation();
    cv::Mat GetTranslation();
	cv::Mat GetIntTranslation();
	cv::Mat dynamic_mask;
    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
	vector<cv::Point3f> GetDynamicPoints();
	void AddDynamicPoint(cv::Point3f DP);
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);
	cv::Mat UnprojectStereo_dynamic_keyframe(const int &i);
    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

	//VisualState function
	void UpdatePoseFromVS();
	void UpdateVisualState(const Matrix3d &deltaR, const Vector3d &deltat);
	const VisualState GetVisualState();
	void SetVisualState(const VisualState& vs);

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
	//存储序号
	int SaveNum;
	//障碍物坐标
	std::vector<pair<cv::Point3f, int>> obstacle;
	//相机坐标系下盲道参数
	std::vector<cv::Point3f> blindwaypar;

	//Stereo Image
	cv::Mat im_l, im_r;
	cv::Mat im_l_color, im_r_color;
    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
	const std::vector<cv::KeyPoint> mvKeysUn_left_dynamic;
	const std::vector<cv::KeyPoint> mvKeysUn_right_dynamic;
	std::vector<float> mvDepth_dynamic;
	std::vector<cv::Point3f> mvpDynamicPoints;
//	const std::vector<cv::KeyPoint> left
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;
	cv::Mat UnprojectStereo_dynamic(const int &i);
    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

	//Standard deviation
	cv::Mat mCov;

	//gnss_fix_type
	unsigned int mFixtype;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;
	

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
	cv::Mat Twc;
    cv::Mat Ow;
	// True Pose
	cv::Mat Zcw;
	cv::Mat Zwc;

    cv::Mat Cw; // Stereo middel point. Only for visualization
	cv::Mat mRwc;
	cv::Mat mOw;
    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;
	//std::vector<MapPoint*> mvpDynamicPoints;
    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

	boost::mutex mMutexImage;
	boost::mutex mMutexPose;
	boost::mutex mMutexConnections;
	boost::mutex mMutexFeatures;

	boost::mutex mMutexSaveNum;
	boost::mutex mMutexObstacle;
	boost::mutex mMutexBlindwaypar;

	boost::mutex mMutexVisualState;
	VisualState mVisualState;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
