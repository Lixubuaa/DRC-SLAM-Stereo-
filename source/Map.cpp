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
#include "Map.h"

#include<boost/thread.hpp>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::AddDynamicPoint(cv::Point3f DP)
{
	boost::mutex::scoped_lock lock(mMutexMap);
	//cout << "add dynamic point to Map" << endl;
	mspDynamicPoints.push_back(DP);
}

void Map::ClearDynamicPoint()
{
	boost::mutex::scoped_lock lock(mMutexMap);
	//cout << "add dynamic point to Map" << endl;
	mspDynamicPoints.clear();
}

//obstacle
void Map::AddObstaclePoint(cv::Point3f OP)
{
	boost::mutex::scoped_lock lock(mMutexMap);
	msObstaclePoints.push_back(OP);

}

void Map::RemoveObstaclePoints()
{
	boost::mutex::scoped_lock lock(mMutexMap);
	msObstaclePoints.clear();
}

//blindway
void Map::AddBlindwayPoint(cv::Point3f OB)
{
	boost::mutex::scoped_lock lock(mMutexMap);
	msBlindwayPoints.push_back(OB);

}

void Map::RemoveBlindwayPoints()
{
	boost::mutex::scoped_lock lock(mMutexMap);
	msBlindwayPoints.clear();
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::SetWPlane(const cv::Mat& WPlane)
{
	boost::mutex::scoped_lock lock(mMutexMap);
	WPlane.copyTo(mWPlane);
}

cv::Mat Map::GetWPlane()
{
	boost::mutex::scoped_lock lock(mMutexMap);
	return mWPlane;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

vector<cv::Point3f> Map::GetAllDynamicPoints()
{
	boost::mutex::scoped_lock lock(mMutexMap);
	return vector<cv::Point3f>(mspDynamicPoints.begin(), mspDynamicPoints.end());
}

vector<cv::Point3f> Map::GetAllObstaclePoints()
{
	boost::mutex::scoped_lock lock(mMutexMap);
	return vector<cv::Point3f>(msObstaclePoints.begin(), msObstaclePoints.end());
}

vector<cv::Point3f> Map::GetAllBlindwayPoints()
{
	boost::mutex::scoped_lock lock(mMutexMap);
	return vector<cv::Point3f>(msBlindwayPoints.begin(), msBlindwayPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;
	mspDynamicPoints.clear();
    mspMapPoints.clear();
    mspKeyFrames.clear();
	msObstaclePoints.clear();
	msBlindwayPoints.clear();
	mWPlane.release();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
