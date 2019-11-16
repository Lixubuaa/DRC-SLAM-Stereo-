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
#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"

extern int filenum;

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
im_l(F.im_l), im_r(F.im_r), mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mCov(F.mCov), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
	mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn), dynamic_mask(F.dynamic_mask), mvKeysUn_left_dynamic(F.mvKeysUn_left_dynamic), mvKeysUn_right_dynamic(F.mvKeysUn_right_dynamic),
	mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()), mRwc(F.mRwc), mOw(F.mOw),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
	mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb / 2), mpMap(pMap), SaveNum(filenum), obstacle(0), blindwaypar(1, { 0, 0, 0 }), mFixtype(F.mFixtype), im_l_color(F.im_l_color), im_r_color(F.im_r_color)
{
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

	SetVisualState(F.GetVisualState());
	UpdatePoseFromVS();
	SetIntPose(F.mZcw);
	if (Converter::toMatrix3d(Tcw.rowRange(0, 3).colRange(0, 3)).isIdentity())
	{
		SetPose(F.mTcw);
	}
}

cv::Mat KeyFrame::UnprojectStereo_dynamic(const int &i)
{
	//mvDepth_dynamic[i] = ;
	//cout << "start calculate Z" << endl;
	//cout << mvKeysUn_left_dynamic.size() << " " << mvKeysUn_right_dynamic.size() << endl;
	//cout << mbf << endl;
	//cout << mvKeysUn_left_dynamic[i].pt.x - mvKeysUn_right_dynamic[i].pt.x << endl;
	float z = mbf / (mvKeysUn_left_dynamic[i].pt.x - mvKeysUn_right_dynamic[i].pt.x);
	//cout << z << endl;
//	cout << "dynamic point z = " << z << endl;
	if (z>0)
	{
		const float u = mvKeysUn_left_dynamic[i].pt.x;
		const float v = mvKeysUn_left_dynamic[i].pt.y;
		const float x = (u - cx)*z*invfx;
		const float y = (v - cy)*z*invfy;
		cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
		//return Tcw.rowRange(0, 3).colRange(0, 3)*x3Dc + Tcw.rowRange(0, 3).col(3);
		return Twc.rowRange(0, 3).colRange(0, 3)*x3Dc + Twc.rowRange(0, 3).col(3);
		//return x3Dc;
	}
	else
		return cv::Mat();
}

//存储序号
void KeyFrame::SetSaveNum(int _SaveNum)
{
	boost::mutex::scoped_lock lock(mMutexSaveNum);
	SaveNum = _SaveNum;
}
int KeyFrame::GetSaveNum()
{
	boost::mutex::scoped_lock lock(mMutexSaveNum);
	return SaveNum;
}

//盲道、障碍物、地面等参数设置获取函数
void KeyFrame::SetObstacle(std::vector<pair<cv::Point3f, int>> _obstacle)
{
	boost::mutex::scoped_lock lock(mMutexObstacle);
	obstacle = _obstacle;
}
void KeyFrame::SetBlindwaypar(std::vector<cv::Point3f> _blindwaypar)
{
	boost::mutex::scoped_lock lock(mMutexBlindwaypar);
	blindwaypar = _blindwaypar;
}

std::vector<pair<cv::Point3f,int>> KeyFrame::GetObstacle()
{
	boost::mutex::scoped_lock lock(mMutexObstacle);
	return obstacle;
}
std::vector<cv::Point3f> KeyFrame::GetBlindwaypar()
{
	boost::mutex::scoped_lock lock(mMutexBlindwaypar);
	return blindwaypar;
}

cv::Mat KeyFrame::GetImage()
{
	boost::mutex::scoped_lock lock(mMutexImage);
	return im_l;
}

cv::Mat KeyFrame::GetRightImage()
{
	boost::mutex::scoped_lock lock(mMutexImage);
	return im_r;
}

cv::Mat KeyFrame::GetColorImage()
{
	boost::mutex::scoped_lock lock(mMutexImage);
	return im_l_color;
}

cv::Mat KeyFrame::GetColorRightImage()
{
	boost::mutex::scoped_lock lock(mMutexImage);
	return im_r_color;
}
void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

cv::Mat KeyFrame::UnprojectStereo_dynamic_keyframe(const int &i)
{
	float disp = mvKeysUn_left_dynamic[i].pt.x - mvKeysUn_right_dynamic[i].pt.x;
	const float z = mbf / disp;
	if (disp<(mbf / mb)){
		if (z>0)
		{
			const float u = mvKeysUn_left_dynamic[i].pt.x;
			const float v = mvKeysUn_left_dynamic[i].pt.y;
			const float x = (u - cx)*z*invfx;
			const float y = (v - cy)*z*invfy;
			cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
			return Twc.rowRange(0, 3).colRange(0, 3)*x3Dc + Twc.rowRange(0, 3).col(3);
			//return x3Dc;
		}
		else return cv::Mat();
	}
	else
		return cv::Mat();
}
void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    boost::mutex::scoped_lock lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;
}

void KeyFrame::SetIntPose(const cv::Mat &Zcw_)
{
	boost::mutex::scoped_lock lock(mMutexPose);
	Zcw_.copyTo(Zcw);
	cv::Mat Rcw = Zcw.rowRange(0, 3).colRange(0, 3);
	cv::Mat tcw = Zcw.rowRange(0, 3).col(3);
	cv::Mat Rwc = Rcw.t();
	cv::Mat Owc = -Rwc*tcw;

	Zwc = cv::Mat::eye(4, 4, Zcw.type());
	Rwc.copyTo(Zwc.rowRange(0, 3).colRange(0, 3));
	Owc.copyTo(Zwc.rowRange(0, 3).col(3));
	//cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
	//Cw = Zwc*center;
}

cv::Mat KeyFrame::GetPose()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetIntPose()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	return Zcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetIntPoseInverse()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	return Zwc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return Cw.clone();
}


cv::Mat KeyFrame::GetRotation()
{
    boost::mutex::scoped_lock lock(mMutexPose);
	return Tcw.rowRange(0, 3).colRange(0, 3).clone();
}

cv::Mat KeyFrame::GetIntRotation()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	return Zcw.rowRange(0, 3).colRange(0, 3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    boost::mutex::scoped_lock lock(mMutexPose);
	return Tcw.rowRange(0, 3).col(3).clone();
}

cv::Mat KeyFrame::GetIntTranslation()
{
	boost::mutex::scoped_lock lock(mMutexPose);
	return Zcw.rowRange(0, 3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    boost::mutex::scoped_lock lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}
void KeyFrame::AddDynamicPoint(cv::Point3f DP)
{
	boost::mutex::scoped_lock lock(mMutexFeatures);
	mvpDynamicPoints.push_back(DP);
}
void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}
vector<cv::Point3f> KeyFrame::GetDynamicPoints()
{
	boost::mutex::scoped_lock lock(mMutexFeatures);
	return mvpDynamicPoints;
}
set<MapPoint*> KeyFrame::GetMapPoints()
{
     boost::mutex::scoped_lock lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
   boost::mutex::scoped_lock lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        boost::mutex::scoped_lock lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
	//该参数可适当调整
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        boost::mutex::scoped_lock lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
     boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{   
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);
    {
				boost::mutex::scoped_lock lock(mMutexConnections);
 				boost::mutex::scoped_lock lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
			KeyFrame* pC = NULL;
			KeyFrame* pP = NULL;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        mTcp = Tcw*mpParent->GetPoseInverse();
        mbBad = true;
    }


    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        boost::mutex::scoped_lock lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        boost::mutex::scoped_lock lock(mMutexFeatures);
        boost::mutex::scoped_lock lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::UpdatePoseFromVS()
{
	boost::mutex::scoped_lock lock(mMutexVisualState);
	cv::Mat R = Converter::toCvMat(mVisualState.Get_RotMatrix());
	cv::Mat t = Converter::toCvMat(mVisualState.Get_t());

	cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
	R.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
	t.copyTo(Tcw.rowRange(0, 3).col(3));

	SetPose(Tcw);
}

void KeyFrame::UpdateVisualState(const Matrix3d &deltaR, const Vector3d &deltat)
{
	boost::mutex::scoped_lock lock(mMutexVisualState);
	Matrix3d R = mVisualState.Get_RotMatrix();
	Vector3d t = mVisualState.Get_t();

	t += R*deltat;
	R = R*deltaR;

	mVisualState.Set_Tra(t);
	mVisualState.Set_Rot(R);
}

const VisualState KeyFrame::GetVisualState()
{
	boost::mutex::scoped_lock lock(mMutexVisualState);
	return mVisualState;
}

void KeyFrame::SetVisualState(const VisualState& vs)
{
	boost::mutex::scoped_lock lock(mMutexVisualState);
	mVisualState = vs;
}

} //namespace ORB_SLAM
