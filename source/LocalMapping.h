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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Map.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <boost/thread.hpp>

namespace ORB_SLAM2
{

class Tracking;
class Map;

class LocalMapping
{
public:
	bool GetMapUpdateFlagForTracking();
	void SetMapUpdateFlagInTracking(bool bflag);
public:
    LocalMapping(Map* pMap, const float bMonocular);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        boost::mutex::scoped_lock lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }
	bool isdynamic(KeyFrame *pKF, MapPoint* Point);
	// True if Integration is deactivated and we are performing only SLAM
	bool mbIntegration;
	void CreateDynamicPoint_frame_LP(KeyFrame*);
	//void CreateDynamicPoint();
protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    boost::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    boost::mutex mMutexFinish;

    Map* mpMap;

    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    boost::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    boost::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    boost::mutex mMutexAccept;

	//Map update Flag
	boost::mutex mMutexMapUpdateFlag;
	bool mbMapUpdateFlagForTracking;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
