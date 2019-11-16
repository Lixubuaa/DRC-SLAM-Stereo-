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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include<boost/thread.hpp>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

class Map
{
public:
    Map();
	cv::Point3f Dynamic_center;
    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
	void AddDynamicPoint(cv::Point3f DP);
	void AddObstaclePoint(cv::Point3f OP);
	void RemoveObstaclePoints();
	void AddBlindwayPoint(cv::Point3f OB);
	void RemoveBlindwayPoints();
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
	void SetWPlane(const cv::Mat& WPlane);
	void ClearDynamicPoint();
	cv::Mat GetWPlane();
    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
	std::vector<cv::Point3f> GetAllDynamicPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();
	std::vector<cv::Point3f> GetAllObstaclePoints();
	std::vector<cv::Point3f> GetAllBlindwayPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    boost::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    boost::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
	//std::set<MapPoint*> mspDynamicPoints;
	std::vector<cv::Point3f> mspDynamicPoints;
	
    std::set<KeyFrame*> mspKeyFrames;
	std::vector<cv::Point3f> msObstaclePoints;
	std::vector<cv::Point3f> msBlindwayPoints;

    std::vector<MapPoint*> mvpReferenceMapPoints;

	cv::Mat mWPlane;

    long unsigned int mnMaxKFid;

    boost::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
