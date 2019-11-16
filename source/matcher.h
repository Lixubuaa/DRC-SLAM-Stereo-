#if !defined MATCHER
#define MATCHER

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "string.h"
#include <fstream>
#include <time.h>

using namespace std;
using namespace cv;

Mat empty;

class RobustMatcher {

  private:

	  // pointer to the feature point detector object
	  cv::Ptr<cv::FeatureDetector> detector;
	  // pointer to the feature descriptor extractor object
	  cv::Ptr<cv::DescriptorExtractor> extractor;
	  float ratio; // max ratio between 1st and 2nd NN
	  bool refineF; // if true will refine the F matrix
	  double distance; // min distance to epipolar
	  double confidence; // confidence level (probability)

  public:

	  RobustMatcher() : ratio(0.65f), refineF(true), confidence(0.99), distance(3.0) {	  

		  // SURF is the default feature
		  detector= new SurfFeatureDetector();
		  extractor= new SurfDescriptorExtractor();
	  }

	  // Set the feature detector
	  void setFeatureDetector(cv::Ptr<cv::FeatureDetector>& detect) {

		  detector= detect;
	  }

	  // Set descriptor extractor
	  void setDescriptorExtractor(cv::Ptr<cv::DescriptorExtractor>& desc) {

		  extractor= desc;
	  }

	  // Set the minimum distance to epipolar in RANSAC
	  void setMinDistanceToEpipolar(double d) {

		  distance= d;
	  }

	  // Set confidence level in RANSAC
	  void setConfidenceLevel(double c) {

		  confidence= c;
	  }

	  // Set the NN ratio
	  void setRatio(float r) {

		  ratio= r;
	  }

	  // if you want the F matrix to be recalculated
	  void refineFundamental(bool flag) {

		  refineF= flag;
	  }

	  // Clear matches for which NN ratio is > than threshold
	  // return the number of removed points 
	  // (corresponding entries being cleared, i.e. size will be 0)
	  int ratioTest(std::vector<std::vector<cv::DMatch>>& matches) {

		int removed=0;

        // for all matches
		for (vector<vector<DMatch>>::iterator matchIterator= matches.begin();
			 matchIterator!= matches.end(); ++matchIterator) {
			/*if (matchIterator->size() > 100){
				matchIterator->clear();
				break;
			}*/
				 // if 2 NN has been identified
				 if (matchIterator->size() > 1) {
					 float a = (*matchIterator)[0].distance;
					 float b = (*matchIterator)[1].distance;
					 // check distance ratio
					 if (( a/b ) > ratio) {

						 matchIterator->clear(); // remove match
						 removed++;
					 }

				 } else { // does not have 2 neighbours

					 matchIterator->clear(); // remove match
					 removed++;
				 }
		}

		return removed;
	  }

	  // Insert symmetrical matches in symMatches vector
	  void symmetryTest(const vector<vector<DMatch>>& matches1,
		                const vector<vector<DMatch>>& matches2,
					    vector<DMatch>& symMatches) {
			
		// for all matches image 1 -> image 2
		for (vector<vector<DMatch>>::const_iterator matchIterator1= matches1.begin();
			 matchIterator1!= matches1.end(); ++matchIterator1) {

			if (matchIterator1->size() < 2) // ignore deleted matches 
				continue;

			// for all matches image 2 -> image 1
			for (vector<vector<DMatch>>::const_iterator matchIterator2= matches2.begin();
				matchIterator2!= matches2.end(); ++matchIterator2) {

				if (matchIterator2->size() < 2) // ignore deleted matches 
					continue;

				// Match symmetry test
				if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx  && 
					(*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx) {

						// add symmetrical match
						symMatches.push_back(DMatch((*matchIterator1)[0].queryIdx,
									  				    (*matchIterator1)[0].trainIdx,
													    (*matchIterator1)[0].distance));
						break; // next match in image 1 -> image 2
				}
			}
		}
	  }

	  // Identify good matches using RANSAC
	  // Return fundemental matrix
	  Mat ransacTest(const vector<DMatch>& matches,
		             const vector<KeyPoint>& keypoints1, 
				     const vector<KeyPoint>& keypoints2,
					 vector<DMatch>& outMatches) {

		// Convert keypoints into Point2f	
		vector<Point2f> points1, points2;	
		for (vector<DMatch>::const_iterator it= matches.begin();
			 it!= matches.end(); ++it) {

			 // Get the position of left keypoints
			 float x= keypoints1[it->queryIdx].pt.x;
			 float y= keypoints1[it->queryIdx].pt.y;
			 points1.push_back(Point2f(x,y));
			 // Get the position of right keypoints
			 x= keypoints2[it->trainIdx].pt.x;
			 y= keypoints2[it->trainIdx].pt.y;
			 points2.push_back(Point2f(x,y));
	    }

		// Compute F matrix using RANSAC
		vector<uchar> inliers(points1.size(),0);
		Mat fundemental= findFundamentalMat(
			Mat(points1),Mat(points2), // matching points
		    inliers,      // match status (inlier ou outlier)  
		    CV_FM_RANSAC, // RANSAC method
		    distance,     // distance to epipolar line
		    confidence);  // confidence probability
	
		// extract the surviving (inliers) matches
		vector<uchar>::const_iterator itIn= inliers.begin();
		vector<DMatch>::const_iterator itM= matches.begin();
		// for all matches
		for ( ;itIn!= inliers.end(); ++itIn, ++itM) {

			if (*itIn) { // it is a valid match

				outMatches.push_back(*itM);
			}
		}

		cout << "Number of matched points (after cleaning): " << outMatches.size() << endl;
		if (outMatches.size() == 0)
			return empty;

		if (refineF) {
		// The F matrix will be recomputed with all accepted matches

			// Convert keypoints into Point2f for final F computation	
			points1.clear();
			points2.clear();
	
			for (vector<DMatch>::const_iterator it= outMatches.begin();
				 it!= outMatches.end(); ++it) {

				 // Get the position of left keypoints
				 float x= keypoints1[it->queryIdx].pt.x;
				 float y= keypoints1[it->queryIdx].pt.y;
				 points1.push_back(cv::Point2f(x,y));
				 // Get the position of right keypoints
				 x= keypoints2[it->trainIdx].pt.x;
				 y= keypoints2[it->trainIdx].pt.y;
				 points2.push_back(Point2f(x,y));
			}

			// Compute 8-point F from all accepted matches
			fundemental= findFundamentalMat(
				Mat(points1),Mat(points2), // matching points
				CV_FM_8POINT); // 8-point method
		}

		return fundemental;
	  }

	  // Match feature points using symmetry test and RANSAC
	  // returns fundemental matrix
	  Mat match(Mat& image1, Mat& image2, // input images 
		  vector<DMatch>& matches, // output matches and keypoints
		  vector<KeyPoint>& keypoints1, vector<KeyPoint>& keypoints2) {
		
		double t0, t1, t2, t3, t4, t5, t6, t7;
		t0 = (double)clock() / CLOCKS_PER_SEC;

		fstream myfile;
		myfile.open(data + "/MatchTime.txt", ios::app);   //创建一个文件
		if (!myfile)                        //检查文件是否创建成功
		{
			cout << "error open" << endl;
			exit(0);
		}
		myfile << "filenum = " << filenum << endl;
		t1 = (double)clock() / CLOCKS_PER_SEC;

		// 1a. Detection of the SURF features
		detector->detect(image1,keypoints1);
		detector->detect(image2,keypoints2);

		cout << "Number of SURF points (1): " << keypoints1.size() << endl;
		cout << "Number of SURF points (2): " << keypoints2.size() << endl;

		t2 = (double)clock() / CLOCKS_PER_SEC;

		if (keypoints1.size() == 0 || keypoints2.size() == 0)
			return empty;

		// 1b. Extraction of the SURF descriptors
		Mat descriptors1, descriptors2;
		extractor->compute(image1,keypoints1,descriptors1);
		extractor->compute(image2,keypoints2,descriptors2);

		cout << "descriptor matrix size: " << descriptors1.rows << " by " << descriptors1.cols << endl;
		t3 = (double)clock() / CLOCKS_PER_SEC;

		if (descriptors1.empty() || descriptors2.empty())
			return empty;

		// 2. Match the two image descriptors
		// Construction of the matcher 
		BruteForceMatcher<L2<float>> matcher;

		//双向匹配
		// from image 1 to image 2  
		// based on k nearest neighbours (with k=2)
		vector<vector<DMatch>> matches1;//1->2的匹配
		matcher.knnMatch(descriptors1,descriptors2, 
			matches1, // vector of matches (up to 2 per entry) 
			2);		  // return 2 nearest neighbours

		// from image 2 to image 1
		// based on k nearest neighbours (with k=2)
		vector<vector<DMatch>> matches2;//2->1的匹配
		matcher.knnMatch(descriptors2,descriptors1, 
			matches2, // vector of matches (up to 2 per entry) 
			2);		  // return 2 nearest neighbours

		cout << "Number of matched points 1->2: " << matches1.size() << endl;
		cout << "Number of matched points 2->1: " << matches2.size() << endl;
		t4 = (double)clock() / CLOCKS_PER_SEC;

		if (matches1.size() == 0 || matches2.size() == 0)
			return empty;

		// 3. Remove matches for which NN ratio is > than threshold

		// clean image 1 -> image 2 matches
		int removed= ratioTest(matches1);
		cout << "Number of matched points 1->2 (ratio test) : " << matches1.size()-removed << endl;
		// clean image 2 -> image 1 matches
		removed= ratioTest(matches2);
		cout << "Number of matched points 1->2 (ratio test) : " << matches2.size()-removed << endl;
		t5 = (double)clock() / CLOCKS_PER_SEC;

		if (matches1.size() == 0 || matches2.size() == 0)
			return empty;

		// 4. Remove non-symmetrical matches
	    vector<DMatch> symMatches;
		symmetryTest(matches1,matches2,symMatches);
		cout << "Number of matched points (symmetry test): " << symMatches.size() << endl;
		t6 = (double)clock() / CLOCKS_PER_SEC;

		if (symMatches.size() == 0)
			return empty;

		// 5. Validate matches using RANSAC
		Mat fundemental= ransacTest(symMatches, keypoints1, keypoints2, matches);

		t7 = (double)clock() / CLOCKS_PER_SEC;

		myfile << "特征检测耗时： " << t2-t1 << " s" << endl;
		myfile << "SURF描绘子提取耗时： " << t3-t2 << " s" << endl;
		myfile << "最近邻测试耗时： " << t4-t3 << " s" << endl;
		myfile << "比率测试耗时： " << t5-t4 << " s" << endl;
		myfile << "对称性测试耗时： " << t6-t5 << " s" << endl;
		myfile << "基础矩阵计算及极线匹配耗时： " << t7-t6 << " s" << endl;
		myfile << "匹配部分总耗时： " << t7-t0 << " s" << endl;
		myfile << endl;
		myfile.close();

		return fundemental;
		
	}
};

#endif
