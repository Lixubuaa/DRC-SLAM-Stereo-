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

#include "stdafx.h"
#include "System.h"
#include "Converter.h"
#include "path_planning.h"
#include <boost/thread.hpp>
#include <pangolin/pangolin.h>
#include <Windows.h>

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const int iMode):mSensor(sensor),mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

	mpVocabulary = new ORBVocabulary();
	bool bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
	if (!bVocLoad)
	{
		cerr << "Wrong path to vocabulary. " << endl;
		cerr << "Falied to open at: " << strVocFile << endl;
		exit(-1);
	}
	cout << "Vocabulary loaded!" << endl << endl;

	//Initialize the ReadNAV982 thread
	mpReader = new ReadNAV982();
	mptReading = new boost::thread(&ORB_SLAM2::ReadNAV982::Run, mpReader);

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new boost::thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    //mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    //mptLoopClosing = new boost::thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

	//Initialize the Detection thread and launch
	mpDetection = new Detection(mpMap);
	mptDetection = new boost::thread(&ORB_SLAM2::Detection::Run, mpDetection);

	//mpDynamicProcess = new DynamicProcess(mpMap);
	//mptDynamicProcess = new boost::thread(&ORB_SLAM2::DynamicProcess::Run, mpDynamicProcess);

	//Initialize the ObstacleReconstruction thread and launch
	mpObstacleReconstruction = new ObstacleReconstruction(mpMap);
	mptObstacleReconstruction = new boost::thread(&ORB_SLAM2::ObstacleReconstruction::Run, mpObstacleReconstruction);

    //Initialize the Viewer thread and launch
    mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
    if(bUseViewer)
        mptViewer = new boost::thread(&Viewer::Run, mpViewer);

    mpTracker->SetViewer(mpViewer);

    //Set pointers between threads
	mpObstacleReconstruction->SetReader(mpReader);
	mpObstacleReconstruction->SetDetector(mpDetection);

    mpTracker->SetLocalMapper(mpLocalMapper);

	mpTracker->SetDetection(mpDetection);
	mpTracker->SetObstacleReconstruction(mpObstacleReconstruction);

    mpLocalMapper->SetTracker(mpTracker);

	ModeChoose(iMode);

}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const cv::Mat &Zcw, const cv::Mat &Cov, const unsigned int &Fixtype)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        boost::mutex::scoped_lock lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
				cout << "mpLocalMapper->isStopped()=" << mpLocalMapper->isStopped() << endl;
                Sleep(1);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    boost::mutex::scoped_lock lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

	return mpTracker->GrabImageStereo(imLeft, imRight, timestamp, Zcw, Cov, Fixtype);
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        boost::mutex::scoped_lock lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                Sleep(1);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
		boost::mutex::scoped_lock lock(mMutexReset);
		if(mbReset)
		{
			mpTracker->Reset();
			mbReset = false;
		}
    }

    return mpTracker->GrabImageRGBD(im,depthmap,timestamp);
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        boost::mutex::scoped_lock lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                Sleep(1);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    boost::mutex::scoped_lock lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageMonocular(im,timestamp);
}

void System::ActivateLocalizationMode()
{
    boost::mutex::scoped_lock lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    boost::mutex::scoped_lock lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::Reset()
{
    boost::mutex::scoped_lock lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    //mpLoopCloser->RequestFinish();
    mpViewer->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() /*|| !mpLoopCloser->isFinished()*/  ||
          !mpViewer->isFinished()      /*|| mpLoopCloser->isRunningGBA()*/)
    {
        Sleep(5);
    }

    pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SettoWalkMode()
{
	mpTracker->mbIntegration = false;
	mpLocalMapper->mbIntegration = false;
	mpObstacleReconstruction->mbRoute = false;
}

void System::SettoArriveMode()
{
	mpTracker->mbIntegration = true;
	mpLocalMapper->mbIntegration = true;
	mpObstacleReconstruction->mbRoute = true;
	mpObstacleReconstruction->meRouteMode = eRouteMode::Input;
}

void System::SettoSchoolGPSMode()
{
	mpTracker->mbIntegration = true;
	mpLocalMapper->mbIntegration = true;
	mpObstacleReconstruction->mbRoute = true;
	mpObstacleReconstruction->meRouteMode = eRouteMode::School;

	//开启砖缝线跟踪
	mpDetection->meLineTrackMode = eLineTrackMode::BrickLine;
	mpDetection->mbBlindway = false;
}

void System::SettoSchoolBlindwayMode()
{
	mpTracker->mbIntegration = false;
	mpLocalMapper->mbIntegration = false;
	mpObstacleReconstruction->mbRoute = false;
}

void System::SettoBeihangGPSMode()
{
	mpTracker->mbIntegration = true;
	mpLocalMapper->mbIntegration = true;
	mpObstacleReconstruction->mbRoute = true;
	mpObstacleReconstruction->meRouteMode = eRouteMode::Beihang;
	
	//开启砖缝线跟踪
	mpDetection->meLineTrackMode = eLineTrackMode::BrickLine;
	mpDetection->mbBlindway = false;
}

void System::SettoBeihangBlindwayMode()
{
	mpTracker->mbIntegration = false;
	mpLocalMapper->mbIntegration = false;
	mpObstacleReconstruction->mbRoute = false;
}

void System::SettoObstacleMode()
{
	mpTracker->mbIntegration = false;
	mpLocalMapper->mbIntegration = false;
	mpObstacleReconstruction->mbRoute = false;
	mpDetection->mbBlindway = false;
}

void System::SettoBlindwayMode()
{
	mpTracker->mbIntegration = false;
	mpLocalMapper->mbIntegration = false;
	mpObstacleReconstruction->mbRoute = false;
	mpDetection->mbObstacle = false;

	//开启盲道跟踪
	//mpDetection->meLineTrackMode = eLineTrackMode::Blindway;
}

void System::SettoCircleMode()
{
	mpTracker->mbIntegration = true;
	mpLocalMapper->mbIntegration = true;
	mpObstacleReconstruction->mbRoute = true;
	mpObstacleReconstruction->meRouteMode = eRouteMode::Circle;
	mpDetection->mbObstacle = false;
}

void System::SettoStraightLineMode()
{
	mpTracker->mbIntegration = true;
	mpLocalMapper->mbIntegration = true;
	mpObstacleReconstruction->mbRoute = true;
	mpObstacleReconstruction->meRouteMode = eRouteMode::Straight;
	mpDetection->mbObstacle = false;
	mpDetection->mbBlindway = false;

}

void System::SettoBlindwayStraightLineMode()
{
	mpTracker->mbIntegration = false;
	mpLocalMapper->mbIntegration = false;
	mpObstacleReconstruction->mbRoute = true;
	mpObstacleReconstruction->meRouteMode = eRouteMode::Blindway;
	mpDetection->mbObstacle = false;
}

void System::ModeChoose(int iMode)
{
	switch (iMode)
	{
	//散步模式
	case 21:
		SettoWalkMode();
		break;
	
	//到达模式
	case 22:
		SettoArriveMode();
		break;
	
	//盲校模式
	case 3111:
		SettoSchoolGPSMode();
		break;

	case 3112:
		SettoSchoolBlindwayMode();
		break;
	
	//北航模式
	case 3121:
		SettoBeihangGPSMode();
		break;

	case 3122:
		SettoBeihangBlindwayMode();
		break;
	
	//障碍物模式
	case 321:
		SettoObstacleMode();
		break;
	
	//盲道模式
	case 322:
		SettoBlindwayMode();
		break;
	
	//圆轨迹模式
	case 3231:
		SettoCircleMode();
		break;
	
	//直线轨迹模式
	case 3232:
		SettoStraightLineMode();
		break;

	case 3233:
		SettoBlindwayStraightLineMode();
		break;
	}
}

} //namespace ORB_SLAM
