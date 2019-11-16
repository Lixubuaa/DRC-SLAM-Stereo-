#include "Detection.h"
#include "Converter.h"

namespace ORB_SLAM2{
	Detection::Detection(ORB_SLAM2::Map* World) :
		sWorld(World), obstacle(0), blindwaypar(1, { 0, 0, 0 }), mbObstacle(true), mbBlindway(true), meLineTrackMode(eLineTrackMode::Close),
		mbRestartNeed(true), mbSelectNeed(false), mvCandilinesWpoints(0), mvBestlineWpoints(0), mbStopped(false), mbStopRequested(false)
	{
		SetCalibrateParams();
	}

	void Detection::Run(){
		FramesNumLast = 0;
		ofstream outfile(data + "/DetectionTime.txt", ios::app);

		int recognum = 0;
		float recograte = 0.;


		while (1){
			if (Stop())
			{
				while (isStopped())
					Sleep(3);
			}
			FramesNum = sWorld->KeyFramesInMap();

			if (FramesNum){
				FramesNumLast = FramesNum;

				vector<ORB_SLAM2::KeyFrame*> KFs = sWorld->GetAllKeyFrames();
				sort(KFs.begin(), KFs.end(), ORB_SLAM2::KeyFrame::lId);
				//if (KFs[FramesNum - 1]->GetObstacle().size()>0 || KFs[FramesNum - 1]->GetBlindwaypar().size()>1)
				//	continue;
				mCurrentKeyFrame = KFs[FramesNum - 1];
				im = mCurrentKeyFrame->GetImage();
				im_r = mCurrentKeyFrame->GetRightImage();
				mvpMapPoints = mCurrentKeyFrame->GetMapPointMatches();

				if (im.rows != 0){
					double t0, t1, t2, t3, t4, t5;
					t0 = (double)clock() / CLOCKS_PER_SEC;

					mCurrentKeyFrame->SetSaveNum(++filenum);

					//���弰��ʼ���洢string
					string str1 = data_dir[0] + "/Left", str2 = data_dir[1] + "/Right",
						str3 = data_dir[2] + "/Disparity", str4 = data_dir[3] + "/Showimage";
					string end = to_string(filenum) + ".jpg";

					cvtColor(im, im, CV_GRAY2BGR);
					cvtColor(im_r, im_r, CV_GRAY2BGR);

					//�洢����ͼ��
					imwrite(str1 + end, im);
					imwrite(str2 + end, im_r);

					Mat half_imL, half_imR;
					cv::resize(im, half_imL, cv::Size(im.cols / 2, im.rows / 2));
					cv::resize(im_r, half_imR, cv::Size(im_r.cols / 2, im_r.rows / 2));

					im.copyTo(showimage);
					im.copyTo(pathimage);
					t1 = (double)clock() / CLOCKS_PER_SEC;

					//slam��TΪ���ƣ�Ϊֱ�ߵȼ��㣬����תΪ�����ƣ�Detection�о��Ժ�����Ϊ��λ
					CameraR = mCurrentKeyFrame->GetPose().rowRange(0, 3).colRange(0, 3).clone();
					CameraT = 1000*mCurrentKeyFrame->GetPose().rowRange(0, 3).col(3).clone();

					cv::Mat WPlane = sWorld->GetWPlane();

					Mat disp;
					//�����ϰ���ʶ��λģ����
					ob_detection ob_detection;
					homography = ob_detection.findH(half_imL, half_imR, CPlane, CGroundPoints, filenum, WPlane);
					if (homography.empty()){
						cout << "Homography is empty! We will continue next frame..." << endl;
						continue;
					}
					t2 = (double)clock() / CLOCKS_PER_SEC;

					if (mbObstacle)
						obstacle = ob_detection.obstacletest(half_imL, half_imR, CPlane, homography, disp, showimage);
					mCurrentKeyFrame->SetObstacle(obstacle);
					t3 = (double)clock() / CLOCKS_PER_SEC;
					cout << "CPlane:" << CPlane.x << "," << CPlane.y << "," << CPlane.z << endl;
					if (WPlane.empty()||mbResetWPlane)
					{
						boost::mutex::scoped_lock lock(mMutexResetWPlane);
						mbResetWPlane = false;
						cv::Mat MCPlane(1, 3, CV_32F);
						MCPlane.at<float>(0) = CPlane.x;
						MCPlane.at<float>(1) = CPlane.y;
						MCPlane.at<float>(2) = CPlane.z;
						cv::Mat At;
						At = MCPlane*CameraT;
						WPlane = MCPlane*CameraR / (1 - At.at<float>(0)) * 1000;
						sWorld->SetWPlane(WPlane);
					}

					if (mbBlindway)  //�������ä��ģʽ
						blindwaypar = GetBlindwayTrackLine();
					else if (meLineTrackMode == eLineTrackMode::BrickLine)  //���û�п���ä��ģʽ������ש���߸���
						blindwaypar = GetBrickTrackLine();
					KFs[FramesNum - 1]->SetBlindwaypar(blindwaypar);
					t4 = (double)clock() / CLOCKS_PER_SEC;

					//ͳ��ä��ʶ����
					if (blindwaypar.size() == 2)
					{
						recognum++;
						recograte = (float)recognum / filenum * 100;
					}

					Mat showimage_show;
					resize(showimage,showimage_show,Size(320,240));	//����
					imshow("showimage", showimage_show);	//��ʾ
					HWND hwnd_showimage = FindWindow(NULL,"showimage");
					SetWindowPos(hwnd_showimage,HWND_NOTOPMOST,960,661,320,240,SWP_ASYNCWINDOWPOS|SWP_DEFERERASE|SWP_NOSIZE|SWP_NOZORDER|SWP_NOACTIVATE);
					//�洢�Ӳ�ͼ����ʾͼ
					imwrite(str3 + end, disp);
					imwrite(str4 + end, showimage);
					t5 = (double)clock() / CLOCKS_PER_SEC;

					outfile << "filenum = " << filenum << endl;
					outfile << "Using FramesNum = " << FramesNum << endl;
					outfile << "Matching and ground computing time: " << t2 - t1 << " s" << endl;
					outfile << "Obstacle detection time: " << t3 - t2 << " s" << endl;
					outfile << "Blindway recognition time: " << t4 - t3 << " s" << endl;
					outfile << "Detection Module total time: " << t5 - t0 << " s" << endl;
					outfile << "Blindway recognition rate: " << setprecision(4) << recograte << " %" << endl;
					outfile << endl;

					waitKey(33);

					cout << "detection over!" << endl;
				}
			}
		}
		outfile.close();
	}
	void Detection::RequestStop()
	{

		boost::mutex::scoped_lock lock(mMutexStop);
		mbStopRequested = true;



	}

	bool Detection::Stop()
	{
		boost::mutex::scoped_lock lock(mMutexStop);
		if (mbStopRequested)
		{
			FramesNumLast = 0;
			mbStopped = true;
			mbStopRequested = false;
		}

		return true;
	}

	bool Detection::isStopped()
	{
		boost::mutex::scoped_lock lock(mMutexStop);
		return mbStopped;
	}
	void Detection::Release()
	{
		boost::mutex::scoped_lock lock(mMutexStop);
		mbStopped = false;
	}

	vector<cv::Point3f> Detection::GetBlindwayTrackLine()
	{
		vector<Point3f> w_blindwaypar(0);

		//ä��ģʽ����Ҫ���٣�����ä��ʶ���㷨������������������ϵ������
		if (meLineTrackMode != eLineTrackMode::Blindway)
		{
			vector<Point3f> c_blindwaypar = bw_location.blindwaytest(im, im_r, homography, showimage, CameraR, CameraT, CPlane);
			for (int j = 0; j < c_blindwaypar.size(); j++){
				cv::Mat Xc(3, 1, CV_32F);
				Xc.at<float>(0) = c_blindwaypar[j].x;
				Xc.at<float>(1) = c_blindwaypar[j].y;
				Xc.at<float>(2) = c_blindwaypar[j].z;
				cv::Mat X(3, 1, CV_32F);
				X = CameraR.t()*(Xc - CameraT);
				float x = X.at<float>(0);
				float y = X.at<float>(1);
				float z = X.at<float>(2);
				w_blindwaypar.push_back(cv::Point3f(x, y, z));
			}
		}
		//ä��ģʽ����Ҫ����
		if (meLineTrackMode == eLineTrackMode::Blindway)
		{
			//����ä����������������ϵ����
			if (blindwaypar.size() == 2)
			{
				rl_tracking.setCurrentParams(im, homography, CameraR, CameraT, CPlane);
				w_blindwaypar = rl_tracking.linesTrack(blindwaypar, showimage);
			}
			else //��������ä��ʶ���㷨������������������ϵ������
			{
				vector<Point3f> c_blindwaypar = bw_location.blindwaytest(im, im_r, homography, showimage, CameraR, CameraT, CPlane);
				for (int j = 0; j < c_blindwaypar.size(); j++){
					cv::Mat Xc(3, 1, CV_32F);
					Xc.at<float>(0) = c_blindwaypar[j].x;
					Xc.at<float>(1) = c_blindwaypar[j].y;
					Xc.at<float>(2) = c_blindwaypar[j].z;
					cv::Mat X(3, 1, CV_32F);
					X = CameraR.t()*(Xc - CameraT);
					float x = X.at<float>(0);
					float y = X.at<float>(1);
					float z = X.at<float>(2);
					w_blindwaypar.push_back(cv::Point3f(x, y, z));
				}
			}
		}

		return w_blindwaypar;
	}

	vector<cv::Point3f> Detection::GetBrickTrackLine()
	{
		vector<Point3f> itBricklinepar(1, { 0, 0, 0 });

		rl_tracking.setCurrentParams(im, homography, CameraR, CameraT, CPlane);

		//��ѡֱ�����
		if (!getSelectNeedBool())
		{
			//��ȡ��ѡ��ֱ��
			if (mvBestlineWpoints.size() == 2)
			{
				blindwaypar = mvBestlineWpoints;
				mvBestlineWpoints.clear();
			}

			if (blindwaypar.size() == 2)
			{	//������һ֡�����ߵ��������꣬���ص�ǰ֡�����ߵ���������
				itBricklinepar = rl_tracking.linesTrack(blindwaypar, showimage);
				//����ʧ��������
				if (itBricklinepar.size() < 2)
					setRestartNeedBool(true);
			}
		}
		//����
		if (getRestartNeedBool())
		{
			//��ȡ��ɸѡֱ��
			mvCandilinesWpoints = rl_tracking.linesRefind(showimage);

			setRestartNeedBool(false);
			setSelectNeedBool(true);
		}

		return itBricklinepar;
	}

	void Detection::setSelectNeedBool(bool SelectNeed_)
	{
		boost::mutex::scoped_lock lock(mMutexSelectNeedBool);
		mbSelectNeed = SelectNeed_;
	}

	bool Detection::getSelectNeedBool()
	{
		boost::mutex::scoped_lock lock(mMutexSelectNeedBool);
		return mbSelectNeed;
	}

	void Detection::setRestartNeedBool(bool RestartNeed_)
	{
		boost::mutex::scoped_lock lock(mMutexRestartNeedBool);
		mbRestartNeed = RestartNeed_;
	}

	bool Detection::getRestartNeedBool()
	{
		boost::mutex::scoped_lock lock(mMutexRestartNeedBool);
		return mbRestartNeed;
	}

	void Detection::ResetWPlane()
	{
		boost::mutex::scoped_lock lock(mMutexResetWPlane);
		mbResetWPlane = true;
	}
}
