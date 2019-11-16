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
#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "DynamicProcess.h"

#include "generalDataFuction.h"

extern bool eliminate_dynamic;
namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;
cv::Mat preframe;
//extern vector<string> data_dir;
Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
	:im_l(frame.im_l), im_r(frame.im_r), mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
	mTimeStamp(frame.mTimeStamp), mZcw(frame.mZcw), mCov(frame.mCov), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
	 mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mvbDynamic(frame.mvbDynamic) ,mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
	 mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2), mVisualState(frame.GetVisualState()), dynamic_mask(frame.dynamic_mask),
	 im_l_color(frame.im_l_color)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, const cv::Mat &Zcw, const cv::Mat &Cov, const unsigned int &Fixtype, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, const cv::Mat &imLeft_color, const cv::Mat &imRight_color)
:im_l(imLeft), im_r(imRight), mpORBvocabulary(voc), mZcw(Zcw), mCov(Cov), mFixtype(Fixtype), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
mpReferenceKF(static_cast<KeyFrame*>(NULL)), im_l_color(imLeft_color), im_r_color(imRight_color)
{
    // Frame ID
    mnId=nNextId++;


    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

	// This is done only for the first Frame (or after a change in the calibration)
	if (mbInitialComputations)
	{
		ComputeImageBounds(imLeft);

		mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
		mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

		fx = K.at<float>(0, 0);
		fy = K.at<float>(1, 1);
		cx = K.at<float>(0, 2);
		cy = K.at<float>(1, 2);
		invfx = 1.0f / fx;
		invfy = 1.0f / fy;

		mbInitialComputations = false;
	}

	mb = mbf / fx;
	
	
	DynamicProcess dyna;
	
	if (mnId ==0){
		preframe = im_l_color;
	}
	else{
		double start, end;
		start = (double)clock() / CLOCKS_PER_SEC;
		dynamic_Rect = dyna.Process(im_l_color, im_r_color, preframe, dynamic_mask);
		end = (double)clock() / CLOCKS_PER_SEC;
		cout << "dynamic process cost = " << end - start << " s " << endl;
		//dynamic_mask = dyna.Process(im_l_color, im_r_color, preframe);
		preframe = im_l_color;
		
		/*stringstream ss;
		ss << setfill('0') << setw(6) << mnId;
		string savedynamic = data_dir[8] + " / ";
		cv::imwrite(savedynamic + ss.str() + ".png", dynamic_mask);*/
		
	}
	if (!dynamic_mask.empty()&&mnId >= 3){
		//cv::imshow("dynamic_mask", dynamic_mask*255);
		// ORB extraction,zhenghui
		//dynamic_mask
		//cout << "start processing dynamic mask" << endl;
		//cv::waitKey(3);
		boost::thread threadLeft(&Frame::ExtractORB_mask, this, 0, imLeft, dynamic_mask);
		boost::thread threadRight(&Frame::ExtractORB_mask, this, 1, imRight,dynamic_mask);
		threadLeft.join();
		threadRight.join();  
	}
	else{
		boost::thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
		boost::thread threadRight(&Frame::ExtractORB, this, 1, imRight);
		threadLeft.join();
		threadRight.join();
	}

	if (mvKeys.empty() || mDescriptors.empty()){
		cout << "ORB extractor error" << endl;
		return;
	}
	/*if (left_dynamic_keypoints.size() != 0){
		Mat out_img;
		out_img = im_l_color.clone();
		for (int i = 0; i < left_dynamic_keypoints.size(); i++){
			cv::circle(out_img, Point(left_dynamic_keypoints[i].pt.x, left_dynamic_keypoints[i].pt.y), 2, Scalar(255, 0, 0), 1);
		}
		imshow("out_img", out_img);
		waitKey(3);
	}*/
	
	//if (!dynamic_mask.empty()){
	//	EliminateDynamicPoint();
	//}
    N = mvKeys.size();
	N_dynamic = left_dynamic_keypoints.size();
	//cout << "Frame N = " << endl;
	//Mat dynamic_left;
	//dynamic_left = im_l_color.clone();
	//if (!mvDynamicFeature.size()){
	//	for (int i = 0; i < mvDynamicFeature.size(); i++){
	//		cv::circle(dynamic_left, Point(mvDynamicFeature[i].pt.x, mvDynamicFeature[i].pt.y), 2, Scalar(255, 0, 0), 1);
	//	}
	//}
	//
	//imshow("dynamic_left", dynamic_left);
	//waitKey(1);
	/*Mat out_img_r;
	out_img_r = im_r_color.clone();
	for (int i = 0; i < mvKeysRight.size(); i++){
		cv::circle(out_img_r, Point(mvKeysRight[i].pt.x, mvKeysRight[i].pt.y), 2, Scalar(255, 0, 0), 1);
	}
	if (!dynamic_Rect.empty()){
		for (int i = 0; i < dynamic_Rect.size(); i++){
			cv::rectangle(out_img_r, Point(dynamic_Rect[i].tl().x * 2, dynamic_Rect[i].tl().y * 2), Point(dynamic_Rect[i].br().x * 2, dynamic_Rect[i].br().y* 2), Scalar(0, 0, 255), 2);
		}
	}*/
	
	//imshow("out_img_r", out_img_r);
	//waitKey(1);
	if (left_dynamic_keypoints.size() != 0){
		N_dynamic_matched = 0;
		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
		std::vector<DMatch> matches;
		matcher->match(mDescriptors_dynamic, mDescriptorsRight_dynamic, matches);
		//-- 第四步:匹配点对筛选
		double min_dist = 10000, max_dist = 0;

		//找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
		/*for (int i = 0; i < mDescriptors_dynamic.rows; i++)
		{
			double dist = matches[i].distance;
			if (dist < min_dist) min_dist = dist;
			if (dist > max_dist) max_dist = dist;
		}
		std::vector< DMatch > good_matches;
		cout << "min dist = " << min_dist << endl;
		for (int i = 0; i < mDescriptors_dynamic.rows; i++)
		{
			if (matches[i].distance <= max(2 * min_dist, 30.0))
			{
				good_matches.push_back(matches[i]);
				left_dynamic_matched.push_back(left_dynamic_keypoints[i]);
				right_dynamic_matched.push_back(right_dynamic_keypoints[i]);
				N_dynamic_matched++;
			}
		}*/
		std::vector< DMatch > good_matches;
		if (matches.size() < 8){
			good_matches = matches;
		}
		else{
			//Prepare data for findHomography
			vector<Point2f>  srcPoints(matches.size());
			vector<Point2f>  dstPoints(matches.size());

			for (size_t i = 0; i < matches.size(); i++) {
				srcPoints[i] = right_dynamic_keypoints[matches[i].trainIdx].pt;
				dstPoints[i] = left_dynamic_keypoints[matches[i].queryIdx].pt;
			}

			//find homography matrix and get inliers mask
			vector<uchar>  inliersMask(srcPoints.size());
			Mat homography = findHomography(srcPoints, dstPoints, CV_FM_RANSAC, 3, inliersMask);

			//vector<DMatch> inliers;
			for (size_t i = 0; i < inliersMask.size(); i++){
				if (inliersMask[i]){
					good_matches.push_back(matches[i]);
					//left_dynamic_matched.push_back(left_dynamic_keypoints[i]);
					//right_dynamic_matched.push_back(right_dynamic_keypoints[i]);
					left_dynamic_matched.push_back(left_dynamic_keypoints[matches[i].queryIdx]);
					right_dynamic_matched.push_back(right_dynamic_keypoints[matches[i].trainIdx]);
					N_dynamic_matched++;
				}
					
			}
			//matches.swap(good_matches)
		}
	/*	Mat img_match;
		Mat img_goodmatch;
		drawMatches(im_l_color, left_dynamic_keypoints, im_r_color, right_dynamic_keypoints, matches, img_match);
		drawMatches(im_l_color, left_dynamic_keypoints, im_r_color, right_dynamic_keypoints, good_matches, img_goodmatch);
		cv::imshow("match1", img_match);
		waitKey(1);
		imshow("match2", img_goodmatch);
		waitKey(1);
		stringstream ss;
		ss << setfill('0') << setw(6) << mnId;
		imwrite("match/raw_match/" + ss.str() + ".png", img_match);
		imwrite("match/good_match/" + ss.str() + ".png", img_goodmatch);
		cout << "matched points num = " << good_matches.size() << endl;
		cout << "left image matched points num = " << left_dynamic_matched.size() << endl;
		cout << "right image matched points num = " << right_dynamic_matched.size() << endl;*/
	}
	Undistortdynamicpoint();
    UndistortKeyPoints();
    ComputeStereoMatches();
	
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
    mvbOutlier = vector<bool>(N,false);
	mvbDynamic = vector<bool>(N, false);
    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
	if (flag == 0){
		(*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors , left_dynamic_keypoints, mDescriptors_dynamic);
	}
    else
		(*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight, right_dynamic_keypoints, mDescriptorsRight_dynamic);
}

void Frame::ExtractORB_mask(int flag, const cv::Mat &im,cv::Mat mask)
{
	if (flag == 0){
		(*mpORBextractorLeft)(im, mask, mvKeys, mDescriptors, left_dynamic_keypoints, mDescriptors_dynamic);
	}
	else{
		(*mpORBextractorRight)(im, mask, mvKeysRight, mDescriptorsRight, right_dynamic_keypoints, mDescriptorsRight_dynamic);
	}
		
}


void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}



void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,mfLogScaleFactor);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = floor((kp.pt.x-mnMinX)*mfGridElementWidthInv+0.5f);
    posY = floor((kp.pt.y-mnMinY)*mfGridElementHeightInv+0.5f);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
	mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
		mvKeysUn[i] = kp;
    }
}

void Frame::Undistortdynamicpoint(){
	if (mDistCoef.at<float>(0) == 0.0)
	{
		mvKeysUn_left_dynamic = left_dynamic_matched;
		mvKeysUn_right_dynamic = right_dynamic_matched;
		return;
	}

	// Fill matrix with points
	cv::Mat mat(N_dynamic_matched, 2, CV_32F);
	cv::Mat mat_r(N_dynamic_matched, 2, CV_32F);
	for (int i = 0; i<N_dynamic_matched; i++)
	{
		mat.at<float>(i, 0) = left_dynamic_matched[i].pt.x;
		mat.at<float>(i, 1) = left_dynamic_matched[i].pt.y;
		mat_r.at<float>(i, 0) = right_dynamic_matched[i].pt.x;
		mat_r.at<float>(i, 1) = right_dynamic_matched[i].pt.y;
	}

	// Undistort points
	mat = mat.reshape(2); 
	mat_r = mat_r.reshape(2);
	cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
	mat = mat.reshape(1);
	cv::undistortPoints(mat_r, mat_r, mK, mDistCoef, cv::Mat(), mK);
	mat_r = mat_r.reshape(1);

	// Fill undistorted keypoint vector
	mvKeysUn_left_dynamic.resize(N_dynamic_matched);
	for (int i = 0; i<N_dynamic; i++)
	{
		cv::KeyPoint kp = left_dynamic_matched[i];
		cv::KeyPoint kp_r = right_dynamic_matched[i];
		kp.pt.x = mat.at<float>(i, 0);
		kp.pt.y = mat.at<float>(i, 1);
		kp_r.pt.x = mat_r.at<float>(i, 0);
		kp_r.pt.y = mat_r.at<float>(i, 1);
		mvKeysUn_left_dynamic[i] = kp;
		mvKeysUn_right_dynamic[i] = kp_r;
	}
}

void Frame::EliminateDynamicPoint(){
	cout << "Before Eliminate point size = " << mvKeys.size() << endl;
	vector<KeyPoint>::iterator it;
	//KeyPoint keypoint_;
	imshow("_mask", dynamic_mask*255);
	waitKey(1);
	for (it = mvKeys.begin(); it != mvKeys.end();){
		//keypoint_ = *it;
		if (dynamic_mask.at<uchar>((*it).pt.y / 2, (*it).pt.x / 2) == 0){
			mvKeys.erase(it);
		}
		else{
			++it;
		}
	}
	cout << "AfterEliminate point size = " << mvKeys.size() << endl;
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);
	vector<cv::KeyPoint> left_matched;
    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
	const float maxD = mbf / minZ /*/ 10*/;//？？调整搜索范围
    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);
	int matched_size = 0;
    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;
		
		//if (eliminate_dynamic){
		//	if (!this->dynamic_mask.empty()){
		//		//cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
		//		if (this->dynamic_mask.at<uchar>(Point(uL / 2, vL / 2)) == 0){
		//			this->eliminate_num++;
		//			continue;
		//		}
		//			
		//	}
		//}
		//
		
        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<ORBmatcher::TH_HIGH)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = floor(kpL.pt.x*scaleFactor+0.5f);
            const float scaledvL = floor(kpL.pt.y*scaleFactor+0.5f);
            const float scaleduR0 = floor(uR0*scaleFactor+0.5f);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0-L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity<maxD)
            {
                if(disparity<=0)
                {
					
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
				
				mvDepth[iL] = mbf / disparity;
                mvuRight[iL] = bestuR;
				matched_size++;
				left_matched.push_back(mvKeysUn[iL]);
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }

        }
    }
	
    std::sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = /*1.5f*1.4f**/median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
			mvDepth[vDistIdx[i].second] = -1;
        }
    }

	//cout << "matched point size = " << mvDepth.size() << endl;
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

cv::Mat Frame::UnprojectStereo_dynamic_frame(const int &i)
{
	const float z = mbf / (mvKeysUn_left_dynamic[i].pt.x - mvKeysUn_right_dynamic[i].pt.x);
	if (z>0)
	{
		const float u = mvKeysUn_left_dynamic[i].pt.x;
		const float v = mvKeysUn_left_dynamic[i].pt.y;
		const float x = (u - cx)*z*invfx;
		const float y = (v - cy)*z*invfy;
		cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
		return mRwc*x3Dc + mOw;
		//return x3Dc;
	}
	else
		return cv::Mat();
}

void Frame::UpdatePoseFromVS()
{
	cv::Mat R = Converter::toCvMat(mVisualState.Get_RotMatrix());
	cv::Mat t = Converter::toCvMat(mVisualState.Get_t());

	cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
	R.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
	t.copyTo(Tcw.rowRange(0, 3).col(3));

	SetPose(Tcw);
}

void Frame::UpdateVisualState(const Matrix3d &deltaR, const Vector3d &deltat)
{
	Matrix3d R = mVisualState.Get_RotMatrix();
	Vector3d t = mVisualState.Get_t();

	t = deltaR*t + deltat;
	R = deltaR*R;

	mVisualState.Set_Tra(t);
	mVisualState.Set_Rot(R);
}

const VisualState& Frame::GetVisualState() const
{
	return mVisualState;
}

void Frame::SetVisualState(const VisualState& vs)
{
	mVisualState = vs;
}

cv::Mat Frame::GetColorImage()
{
	//boost::mutex::scoped_lock lock(mMutexFrameImage);
	return im_l_color;
}

cv::Mat Frame::GetColorRightImage()
{
	//boost::mutex::scoped_lock lock(mMutexFrameImage);
	return im_r_color;
}



} //namespace ORB_SLAM
