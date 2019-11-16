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
#include "Viewer.h"
#include <pangolin/pangolin.h>
#include "generalDataFuction.h"

//#include <Windows.h>
extern vector<string> data_dir;
namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

void Viewer::Run()
{
    mbFinished = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",640,480);	
	HWND hwnd_mapviewer = FindWindow(NULL, "ORB-SLAM2: Map Viewer");
	SetWindowPos(hwnd_mapviewer, HWND_TOP, 0, 0, 640, 480, SWP_ASYNCWINDOWPOS | SWP_DEFERERASE | SWP_NOSIZE | SWP_NOACTIVATE | SWP_NOZORDER);


    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(100));	
	pangolin::Var<bool> menuShowGround("menu.Show Ground", true, true);
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
	pangolin::Var<bool> menuShowObstacle("menu.Show Obstacle", true, true);
	pangolin::Var<bool> menuShowBlindway("menu.Show Blindway", true, true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640,480,mViewpointF,mViewpointF,320,240,0.1,500),	
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(100), 1.0, -1024.0f/768.0f)	
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");

	int count = 0;
    bool bFollow = true;
    bool bLocalizationMode = false;
	bool bSLAMMode = false;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

		

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();
			mpMapDrawer->DrawDynamicPoints();
		if (menuShowObstacle)
			mpMapDrawer->DrawObstaclePoints();
		if (menuShowBlindway)
			mpMapDrawer->DrawBlindwayLines();
		if (menuShowGround)
			mpMapDrawer->DrawGround();

        pangolin::FinishFrame();
		
		cv::Mat im = mpFrameDrawer->DrawFrame();
		cv::imshow("ORB-SLAM2: Current Frame", im);
		HWND hwnd_CurrentFrame = FindWindow(NULL, "ORB-SLAM2: Current Frame");
		SetWindowPos(hwnd_CurrentFrame, HWND_TOP, 0, 480, 640, 480, SWP_ASYNCWINDOWPOS | SWP_DEFERERASE | SWP_NOACTIVATE | SWP_NOZORDER|SWP_NOSIZE);
		stringstream ss;
		ss << setfill('0') << setw(6) << count;
		string strFrame = data_dir[8] + "/";
		//cout << data_dir[8] << endl;
		cv::imwrite(strFrame + ss.str() + ".png", im);
		cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
			menuShowObstacle = true;
			menuShowBlindway = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                Sleep(3);
            }
        }
		count++;
        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    boost::mutex::scoped_lock lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    boost::mutex::scoped_lock lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    boost::mutex::scoped_lock lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    boost::mutex::scoped_lock lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    boost::mutex::scoped_lock lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    mbStopped = false;
}

}
