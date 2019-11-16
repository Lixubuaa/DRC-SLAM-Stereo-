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
#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
extern vector<cv::Point3f> dynamic_center_vector;
namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawGround()
{
	cv::Mat WPlane = mpMap->GetWPlane();

	if (WPlane.empty())
		return;
	WPlane = WPlane.t();
	cv::Mat Rcw(3, 3, CV_32F);
	cv::Mat tcw(3, 1, CV_32F);
	{
		boost::mutex::scoped_lock lock(mMutexCamera);
		Rcw = mCameraPose.rowRange(0, 3).colRange(0, 3);
		tcw = mCameraPose.rowRange(0, 3).col(3);
	}
	cv::Mat Ow(3, 1, CV_32F);
	cv::Mat Ow_z(3, 1, CV_32F);
	cv::Mat	Ow_g(3, 1, CV_32F);
	cv::Mat Ow_z_g(3, 1, CV_32F);
	cv::Mat axis_z = cv::Mat::zeros(3, 1, CV_32F);
	axis_z.at<float>(2) = 1;
	Ow = -Rcw.t()*tcw;
	Ow_z = -Rcw.t()*(tcw - axis_z);
	float dist = abs(WPlane.dot(Ow) - 1) / sqrt(WPlane.dot(WPlane));
	float dist_z = abs(WPlane.dot(Ow_z) - 1) / sqrt(WPlane.dot(WPlane));
	cv::Mat WPlane_norm(3, 1, CV_32F);
	WPlane_norm = WPlane / sqrt(WPlane.dot(WPlane));
	if (WPlane_norm.at<float>(1) > 0)
	{
		Ow_g = Ow + dist*WPlane_norm;
		Ow_z_g = Ow_z + dist_z*WPlane_norm;
	}
	else
	{
		Ow_g = Ow - dist*WPlane_norm;
		Ow_z_g = Ow_z - dist_z*WPlane_norm;
	}
	cv::Mat forward(3, 1, CV_32F);
	cv::Mat lateral(3, 1, CV_32F);
	forward = Ow_z_g - Ow_g;
	forward = forward / sqrt(forward.dot(forward));
	lateral = forward.cross(WPlane_norm);
	cv::Mat x1, x2, x3, x4;
	x1 = Ow_g + 5 * forward - 2.5*lateral;
	x2 = Ow_g + 5 * forward + 2.5*lateral;
	x3 = Ow_g + 2.5*lateral;
	x4 = Ow_g - 2.5*lateral;
	glBegin(GL_QUADS);
	glColor3f(0.0, 0.0, 1.0);
	glVertex3f(x1.at<float>(0), x1.at<float>(1), x1.at<float>(2));
	glVertex3f(x2.at<float>(0), x2.at<float>(1), x2.at<float>(2));
	glVertex3f(x3.at<float>(0), x3.at<float>(1), x3.at<float>(2));
	glVertex3f(x4.at<float>(0), x4.at<float>(1), x4.at<float>(2));
	glEnd();
}
void MapDrawer::DrawDynamicPoints(){ 
	const vector<cv::Point3f> &vpDPs = mpMap->GetAllDynamicPoints();
	if (vpDPs.empty()){
		//cout << "no dynamic point return" << endl;
		return;
	}
	cv::Point3f dynamic_center;
	
	//cout << "" << endl;
	glPointSize(mPointSize);
	glBegin(GL_POINTS);
	glColor3f(0.2, 0.8, 0.2);

	for (size_t i = 0, iend = vpDPs.size(); i<iend; i++)
	{
		//cv::Mat pos = vpMPs[i]->GetWorldPos();
		glVertex3f(vpDPs[i].x, vpDPs[i].y, vpDPs[i].z);
	
	}
	glEnd();

	glPointSize(mPointSize*2);
	glBegin(GL_POINTS);
	
	glColor3f(0.8, 0.2, 0.8);
	for (int i = 0; i < dynamic_center_vector.size(); i++){
		glVertex3f(dynamic_center_vector[i].x, dynamic_center_vector[i].y, dynamic_center_vector[i].z);
	}
	glEnd();
	if (dynamic_center_vector.size()>1){

		glLineWidth(mKeyFrameLineWidth);
		glColor3f(0.0f, 0.0f, 1.0f);
		glBegin(GL_LINES);
		for (int i = 1; i < dynamic_center_vector.size(); i++){
			glVertex3f(dynamic_center_vector[i - 1].x, dynamic_center_vector[i - 1].y, dynamic_center_vector[i - 1].z);
			glVertex3f(dynamic_center_vector[i].x, dynamic_center_vector[i].y, dynamic_center_vector[i].z);
		}
		glEnd();
	}
}
void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawObstaclePoints()
{
	const vector<cv::Point3f> &vpOPs = mpMap->GetAllObstaclePoints();
	if (vpOPs.empty())
		return;

	glPointSize(mPointSize);
	glBegin(GL_POINTS);
	glColor3f(0.0, 1.0,1.0);
	for (size_t i = 0, iend = vpOPs.size(); i<iend; i++)
	{
		glVertex3f(vpOPs[i].x, vpOPs[i].y, vpOPs[i].z);
	}
	glEnd();
}

void MapDrawer::DrawBlindwayLines()
{
	const vector<cv::Point3f> &vpOBs = mpMap->GetAllBlindwayPoints();
	if (vpOBs.empty())
		return;

	glPointSize(mPointSize);

	glLineWidth(mKeyFrameLineWidth);
	glColor3f(1.0, 0.0, 1.0);
	glBegin(GL_LINES);
	for (int i = 0; i < vpOBs.size()-2; i=i+2)
	{
		glVertex3f(vpOBs[i].x / 1000, vpOBs[i].y / 1000, vpOBs[i].z / 1000);
		glVertex3f(vpOBs[i + 1].x / 1000, vpOBs[i + 1].y / 1000, vpOBs[i + 1].z / 1000);
	}
	glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();
			cv::Mat Zwc = pKF->GetIntPose().inv().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

			glPushMatrix();

			glMultMatrixf(Zwc.ptr<GLfloat>(0));

			glLineWidth(mKeyFrameLineWidth);
			glColor3f(0.0f, 1.0f, 0.0f);
			glBegin(GL_LINES);
			glVertex3f(0, 0, 0);
			glVertex3f(w, h, z);
			glVertex3f(0, 0, 0);
			glVertex3f(w, -h, z);
			glVertex3f(0, 0, 0);
			glVertex3f(-w, -h, z);
			glVertex3f(0, 0, 0);
			glVertex3f(-w, h, z);

			glVertex3f(w, h, z);
			glVertex3f(w, -h, z);

			glVertex3f(-w, h, z);
			glVertex3f(-w, -h, z);

			glVertex3f(-w, h, z);
			glVertex3f(w, h, z);

			glVertex3f(-w, -h, z);
			glVertex3f(w, -h, z);
			glEnd();

			glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    boost::mutex::scoped_lock lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            boost::mutex::scoped_lock lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
