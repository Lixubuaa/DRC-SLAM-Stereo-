/*
*包括keyline计算、LBD描绘子计算、直线匹配
*修改日期：2017.5.3
*/
#include "linematcher.h"
#define _USE_MATH_DEFINES
#include <math.h>

#define NUM_OF_BANDS 9 //default 9
#define Horizontal  255//if |dx|<|dy|;
#define Vertical    0//if |dy|<=|dx|;
#define UpDir       1
#define RightDir    2
#define DownDir     3
#define LeftDir     4
#define TryTime     6
#define SkipEdgePoint 2

/* combinations of internal indeces for binary descriptor extractor */
static const int combinations[32][2] =
{
	{ 0, 1 },
	{ 0, 2 },
	{ 0, 3 },
	{ 0, 4 },
	{ 0, 5 },
	{ 0, 6 },
	{ 1, 2 },
	{ 1, 3 },
	{ 1, 4 },
	{ 1, 5 },
	{ 1, 6 },
	{ 2, 3 },
	{ 2, 4 },
	{ 2, 5 },
	{ 2, 6 },
	{ 2, 7 },
	{ 2, 8 },
	{ 3, 4 },
	{ 3, 5 },
	{ 3, 6 },
	{ 3, 7 },
	{ 3, 8 },
	{ 4, 5 },
	{ 4, 6 },
	{ 4, 7 },
	{ 4, 8 },
	{ 5, 6 },
	{ 5, 7 },
	{ 5, 8 },
	{ 6, 7 },
	{ 6, 8 },
	{ 7, 8 } };

/* return default parameters */
LineDescriptor::Params::Params()
{
	numOfOctave_ = 1; //1
	widthOfBand_ = 7; //7
	reductionRatio = 2; //2
	ksize_ = 5; //5
}

/* setters and getters */
int LineDescriptor::getNumOfOctaves()
{
	return params.numOfOctave_;
}

void LineDescriptor::setNumOfOctaves(int octaves)
{
	params.numOfOctave_ = octaves;
}

int LineDescriptor::getWidthOfBand()
{
	return params.widthOfBand_;
}

void LineDescriptor::setWidthOfBand(int width)
{
	params.widthOfBand_ = width;

	/* reserve enough space for EDLine objects and images in Gaussian pyramid */
	edLineVec_.resize(params.numOfOctave_);
	images_sizes.resize(params.numOfOctave_);

	for (int i = 0; i < params.numOfOctave_; i++)
		edLineVec_[i] = Ptr < EDLineDetector >(new EDLineDetector());

	/* prepare a vector to host local weights F_l*/
	gaussCoefL_.resize(params.widthOfBand_ * 3);

	/* compute center of central band (every computation involves 2-3 bands) */
	double u = (params.widthOfBand_ * 3 - 1) / 2;

	/* compute exponential part of F_l */
	double sigma = (params.widthOfBand_ * 2 + 1) / 2;  // (widthOfBand_*2+1)/2;
	double invsigma2 = -1 / (2 * sigma * sigma);

	/* compute all local weights */
	double dis;
	for (int i = 0; i < params.widthOfBand_ * 3; i++)
	{
		dis = i - u;
		gaussCoefL_[i] = exp(dis * dis * invsigma2);
	}

	/* prepare a vector for global weights F_g*/
	gaussCoefG_.resize(NUM_OF_BANDS * params.widthOfBand_);

	/* compute center of LSR */
	u = (NUM_OF_BANDS * params.widthOfBand_ - 1) / 2;

	/* compute exponential part of F_g */
	sigma = u;
	invsigma2 = -1 / (2 * sigma * sigma);
	for (int i = 0; i < NUM_OF_BANDS * params.widthOfBand_; i++)
	{
		dis = i - u;
		gaussCoefG_[i] = exp(dis * dis * invsigma2);
	}
}

int LineDescriptor::getReductionRatio()
{
	return params.reductionRatio;
}

void LineDescriptor::setReductionRatio(int rRatio)
{
	params.reductionRatio = rRatio;
}

void LineDescriptor::setKsize(int ksize)
{
	params.ksize_ = ksize;
}

/* read parameters from a FileNode object and store them (struct function) */
void LineDescriptor::Params::read(const cv::FileNode& fn)
{
	numOfOctave_ = fn["numOfOctave_"];
	widthOfBand_ = fn["widthOfBand_"];
	reductionRatio = fn["reductionRatio"];
}

/* store parameters to a FileStorage object (struct function) */
void LineDescriptor::Params::write(cv::FileStorage& fs) const
{
	fs << "numOfOctave_" << numOfOctave_;
	fs << "numOfBand_" << NUM_OF_BANDS;
	fs << "widthOfBand_" << widthOfBand_;
	fs << "reductionRatio" << reductionRatio;
}

Ptr<LineDescriptor> LineDescriptor::createBinaryDescriptor()
{
	return Ptr < LineDescriptor >(new LineDescriptor());
}

Ptr<LineDescriptor> LineDescriptor::createBinaryDescriptor(Params parameters)
{
	return Ptr < LineDescriptor >(new LineDescriptor(parameters));
}

/* construct a BinaryDescrptor object and compute external private parameters */
LineDescriptor::LineDescriptor(const LineDescriptor::Params &parameters) :
params(parameters)
{
	/* reserve enough space for EDLine objects and images in Gaussian pyramid */
	edLineVec_.resize(params.numOfOctave_);
	images_sizes.resize(params.numOfOctave_);

	for (int i = 0; i < params.numOfOctave_; i++)
		edLineVec_[i] = Ptr < EDLineDetector >(new EDLineDetector());

	/* prepare a vector to host local weights F_l*/
	gaussCoefL_.resize(params.widthOfBand_ * 3);

	/* compute center of central band (every computation involves 2-3 bands) */
	double u = (params.widthOfBand_ * 3 - 1) / 2;

	/* compute exponential part of F_l */
	double sigma = (params.widthOfBand_ * 2 + 1) / 2;  // (widthOfBand_*2+1)/2;
	double invsigma2 = -1 / (2 * sigma * sigma);

	/* compute all local weights */
	double dis;
	for (int i = 0; i < params.widthOfBand_ * 3; i++)
	{
		dis = i - u;
		gaussCoefL_[i] = exp(dis * dis * invsigma2);
	}

	/* prepare a vector for global weights F_g*/
	gaussCoefG_.resize(NUM_OF_BANDS * params.widthOfBand_);

	/* compute center of LSR */
	u = (NUM_OF_BANDS * params.widthOfBand_ - 1) / 2;

	/* compute exponential part of F_g */
	sigma = u;
	invsigma2 = -1 / (2 * sigma * sigma);
	for (int i = 0; i < NUM_OF_BANDS * params.widthOfBand_; i++)
	{
		dis = i - u;
		gaussCoefG_[i] = exp(dis * dis * invsigma2);
	}
}

/* definition of operator () */
void LineDescriptor::operator()(InputArray image, InputArray mask, CV_OUT std::vector<KeyLine>& keylines, OutputArray descriptors,
	bool useProvidedKeyLines, bool returnFloatDescr) const
{

	/* create some matrix objects */
	cv::Mat imageMat, maskMat, descrMat;

	/* store reference to input matrices */
	imageMat = image.getMat();
	maskMat = mask.getMat();

	/* require drawing KeyLines detection if demanded */
	if (!useProvidedKeyLines)
	{
		keylines.clear();
		LineDescriptor *bn = const_cast<LineDescriptor*>(this);
		bn->edLineVec_.clear();
		bn->edLineVec_.resize(params.numOfOctave_);

		for (int i = 0; i < params.numOfOctave_; i++)
			bn->edLineVec_[i] = Ptr<EDLineDetector>(new EDLineDetector());

		detectImpl(imageMat, keylines, maskMat);

	}

	/* initialize output matrix */
	//descriptors.create( Size( 32, (int) keylines.size() ), CV_8UC1 );
	/* store reference to output matrix */
	//descrMat = descriptors.getMat();
	/* compute descriptors */
	if (!useProvidedKeyLines)
		computeImpl(imageMat, keylines, descrMat, returnFloatDescr, true);

	else
		computeImpl(imageMat, keylines, descrMat, returnFloatDescr, false);

	descrMat.copyTo(descriptors);
}

LineDescriptor::~LineDescriptor()
{

}

/* read parameters from a FileNode object and store them (class function ) */
void LineDescriptor::read(const cv::FileNode& fn)
{
	params.read(fn);
}

/* store parameters to a FileStorage object (class function) */
void LineDescriptor::write(cv::FileStorage& fs) const
{
	params.write(fs);
}

/* return norm mode */
int LineDescriptor::defaultNorm() const
{
	return NORM_HAMMING;
}

/* return data type */
int LineDescriptor::descriptorType() const
{
	return CV_8U;
}

/*return descriptor size */
int LineDescriptor::descriptorSize() const
{
	return 32 * 8;
}

/* power function with error management */
static inline int get2Pow(int i)
{
	if (i >= 0 && i <= 7)
		return (int)pow(2, (double)i);

	else
	{
		throw std::runtime_error("Invalid power argument");
	}
}

/* compute Gaussian pyramids */
void LineDescriptor::computeGaussianPyramid(const Mat& image, const int numOctaves)
{
	/* clear class fields */
	images_sizes.clear();
	octaveImages.clear();

	/* insert input image into pyramid */
	cv::Mat currentMat = image.clone();
	cv::GaussianBlur(currentMat, currentMat, cv::Size(5, 5), 1);
	octaveImages.push_back(currentMat);
	images_sizes.push_back(currentMat.size());

	/* fill Gaussian pyramid */
	for (int pyrCounter = 1; pyrCounter < numOctaves; pyrCounter++)
	{
		/* compute and store next image in pyramid and its size */
		pyrDown(currentMat, currentMat, Size(currentMat.cols / params.reductionRatio, currentMat.rows / params.reductionRatio));
		octaveImages.push_back(currentMat);
		images_sizes.push_back(currentMat.size());
	}
}

/* compute Sobel's derivatives */
void LineDescriptor::computeSobel(const cv::Mat& image, const int numOctaves)
{

	/* compute Gaussian pyramids */
	computeGaussianPyramid(image, numOctaves);

	/* reinitialize class structures */
	dxImg_vector.clear();
	dyImg_vector.clear();

	//  dxImg_vector.resize( params.numOfOctave_ );
	//  dyImg_vector.resize( params.numOfOctave_ );

	dxImg_vector.resize(octaveImages.size());
	dyImg_vector.resize(octaveImages.size());

	/* compute derivatives */
	for (size_t sobelCnt = 0; sobelCnt < octaveImages.size(); sobelCnt++)
	{
		dxImg_vector[sobelCnt].create(images_sizes[sobelCnt].height, images_sizes[sobelCnt].width, CV_16SC1);
		dyImg_vector[sobelCnt].create(images_sizes[sobelCnt].height, images_sizes[sobelCnt].width, CV_16SC1);

		cv::Sobel(octaveImages[sobelCnt], dxImg_vector[sobelCnt], CV_16SC1, 1, 0, 3);
		cv::Sobel(octaveImages[sobelCnt], dyImg_vector[sobelCnt], CV_16SC1, 0, 1, 3);
	}
}

/* utility function for conversion of an LBD descriptor to its binary representation */
unsigned char LineDescriptor::binaryConversion(float* f1, float* f2)
{
	uchar result = 0;
	for (int i = 0; i < 8; i++)
	{
		if (f1[i] > f2[i])
			result += (uchar)get2Pow(i);
	}

	return result;

}

KeyLine LineDescriptor::calKeyline(Mat imageSrc, Point startpoint, Point endpoint, int id)
{
	Point transpoint;
	//以y最大的点作为起点
	if (startpoint.y < endpoint.y)
	{
		transpoint = startpoint;
		startpoint = endpoint;
		endpoint = transpoint;
	}

	KeyLine k;
	k.octave = 0;
	k.class_id = id;

	k.startPointX = startpoint.x;
	k.startPointY = startpoint.y;
	k.endPointX = endpoint.x;
	k.endPointY = endpoint.y;

	k.sPointInOctaveX = startpoint.x;
	k.sPointInOctaveY = startpoint.y;
	k.ePointInOctaveX = endpoint.x;
	k.ePointInOctaveY = endpoint.y;

	k.pt = Point2f((endpoint.x + startpoint.x) / 2, (endpoint.y + startpoint.y) / 2);
	
	float dx = endpoint.x - startpoint.x + 1e-5;  //x1-x2
	float dy = endpoint.y - startpoint.y;  //y1-y2
	k.size = abs(dx*dy);
	k.lineLength = sqrt(dx * dx + dy * dy);

	k.response = k.lineLength / max(imageSrc.cols, imageSrc.rows);

	float angle;
	if (dx > 1e-6 && dy > 1e-6)
		angle = atan2(dy, dx);
	if (dx > 1e-6 && dy < -1e-6)
		angle = -atan2(-dy, dx);
	if (dx < -1e-6 && dy > 1e-6)
		angle = M_PI - atan2(dy, -dx);
	if (dx < -1e-6 && dy < -1e-6)
		angle = atan2(-dy, -dx) - M_PI;
	k.angle = angle;

	LineIterator it(imageSrc, startpoint, endpoint);
	k.numOfPixels = it.count;

	return k;
}

void LineDescriptor::calKeylineVec(Mat imageSrc, vector<vector<Point>> linepoints, vector<KeyLine>& keyline)
{
	for (int i = 0; i < linepoints.size(); i++)
	{
		KeyLine k = calKeyline(imageSrc, linepoints[i][0], linepoints[i][1], i);
		keyline.push_back(k);
	}
}


/* requires line detection (only one image) */
void LineDescriptor::detect(const Mat& image, CV_OUT std::vector<KeyLine>& keylines, const Mat& mask)
{
	if (image.data == NULL)
	{
		std::cout << "Error: input image for detection is empty" << std::endl;
		return;
	}

	if (mask.data != NULL && (mask.size() != image.size() || mask.type() != CV_8UC1))
		throw std::runtime_error("Mask error while detecting lines: please check its dimensions and that data type is CV_8UC1");

	else
		detectImpl(image, keylines, mask);
}

/* requires line detection (more than one image) */
void LineDescriptor::detect(const std::vector<Mat>& images, std::vector<std::vector<KeyLine> >& keylines, const std::vector<Mat>& masks) const
{

	if (images.size() == 0)
	{
		std::cout << "Error: input image for detection is empty" << std::endl;
		return;
	}

	/* detect lines from each image */
	for (size_t counter = 0; counter < images.size(); counter++)
	{
		if (masks[counter].data != NULL && (masks[counter].size() != images[counter].size() || masks[counter].type() != CV_8UC1))
			throw std::runtime_error("Masks error while detecting lines: please check their dimensions and that data types are CV_8UC1");

		else
			detectImpl(images[counter], keylines[counter], masks[counter]);
	}
}

void LineDescriptor::detectImpl(const Mat& imageSrc, std::vector<KeyLine>& keylines, const Mat& mask) const
{

	cv::Mat image;
	if (imageSrc.channels() != 1)
	{
		cvtColor(imageSrc, image, COLOR_BGR2GRAY);
	}
	else
		image = imageSrc.clone();

	/*check whether image depth is different from 0 */
	if (image.depth() != 0)
		throw std::runtime_error("Warning, depth image!= 0");

	/* create a pointer to self */
	LineDescriptor *bn = const_cast<LineDescriptor*>(this);

	/* detect and arrange lines across octaves */
	ScaleLines sl;
	bn->OctaveKeyLines(image, sl);

	/* fill KeyLines vector */
	for (int i = 0; i < (int)sl.size(); i++)
	{
		for (size_t j = 0; j < sl[i].size(); j++)
		{
			/* get current line */
			OctaveSingleLine osl = sl[i][j];

			/* create a KeyLine object */
			KeyLine kl;

			/* fill KeyLine's fields */
			kl.startPointX = osl.startPointX;  //extremes[0];
			kl.startPointY = osl.startPointY;  //extremes[1];
			kl.endPointX = osl.endPointX;  //extremes[2];
			kl.endPointY = osl.endPointY;  //extremes[3];
			kl.sPointInOctaveX = osl.sPointInOctaveX;
			kl.sPointInOctaveY = osl.sPointInOctaveY;
			kl.ePointInOctaveX = osl.ePointInOctaveX;
			kl.ePointInOctaveY = osl.ePointInOctaveY;
			kl.lineLength = osl.lineLength;
			kl.numOfPixels = osl.numOfPixels;

			kl.angle = osl.direction;
			kl.class_id = i;
			kl.octave = osl.octaveCount;
			kl.size = (osl.endPointX - osl.startPointX) * (osl.endPointY - osl.startPointY);
			kl.response = osl.lineLength / max(images_sizes[osl.octaveCount].width, images_sizes[osl.octaveCount].height);
			kl.pt = Point2f((osl.endPointX + osl.startPointX) / 2, (osl.endPointY + osl.startPointY) / 2);

			/* store KeyLine */
			keylines.push_back(kl);
		}

	}

	/* delete undesired KeyLines, according to input mask */
	if (!mask.empty())
	{
		for (size_t keyCounter = 0; keyCounter < keylines.size(); keyCounter++)
		{
			KeyLine kl = keylines[keyCounter];
			if (mask.at < uchar >((int)kl.startPointY, (int)kl.startPointX) == 0 && mask.at < uchar >((int)kl.endPointY, (int)kl.endPointX) == 0)
				keylines.erase(keylines.begin() + keyCounter);
		}
	}

}

/* requires descriptors computation (only one image) */
void LineDescriptor::compute(const Mat& image, CV_OUT CV_IN_OUT std::vector<KeyLine>& keylines, CV_OUT Mat& descriptors,
	bool returnFloatDescr) const
{
	computeImpl(image, keylines, descriptors, returnFloatDescr, false);
}

/* requires descriptors computation (more than one image) */
void LineDescriptor::compute(const std::vector<Mat>& images, std::vector<std::vector<KeyLine> >& keylines, std::vector<Mat>& descriptors,
	bool returnFloatDescr) const
{
	for (size_t i = 0; i < images.size(); i++)
		computeImpl(images[i], keylines[i], descriptors[i], returnFloatDescr, false);
}

/* implementation of descriptors computation */
void LineDescriptor::computeImpl(const Mat& imageSrc, std::vector<KeyLine>& keylines, Mat& descriptors, bool returnFloatDescr,
	bool useDetectionData) const
{
	/* convert input image to gray scale */
	cv::Mat image;
	if (imageSrc.channels() != 1)
		cvtColor(imageSrc, image, COLOR_BGR2GRAY);
	else
		image = imageSrc.clone();

	/*check whether image's depth is different from 0 */
	if (image.depth() != 0)
		throw std::runtime_error("Error, depth of image != 0");

	/* keypoints list can't be empty */
	if (keylines.size() == 0)
	{
		std::cout << "Error: keypoint list is empty" << std::endl;
		return;
	}

	LineDescriptor* bd = const_cast<LineDescriptor*>(this);

	/* get maximum class_id and octave*/
	int numLines = 0;
	int octaveIndex = -1;
	for (size_t l = 0; l < keylines.size(); l++)
	{
		if (keylines[l].class_id > numLines)
			numLines = keylines[l].class_id;

		if (keylines[l].octave > octaveIndex)
			octaveIndex = keylines[l].octave;
	}

	if (!useDetectionData)
		bd->computeSobel(image, octaveIndex + 1);

	/* create a ScaleLines object */
	OctaveSingleLine fictiousOSL;
	//  fictiousOSL.octaveCount = params.numOfOctave_ + 1;
	//  LinesVec lv( params.numOfOctave_, fictiousOSL );
	fictiousOSL.octaveCount = octaveIndex + 1;
	LinesVec lv(octaveIndex + 1, fictiousOSL);
	ScaleLines sl(numLines + 1, lv);

	/* create a map to record association between KeyLines and their position
	in ScaleLines vector */
	std::map<std::pair<int, int>, size_t> correspondences;

	/* fill ScaleLines object */
	for (size_t slCounter = 0; slCounter < keylines.size(); slCounter++)
	{
		/* get a KeyLine object and create a new line */
		KeyLine kl = keylines[slCounter];
		OctaveSingleLine osl;

		/* insert data in newly created line */
		osl.startPointX = kl.startPointX;
		osl.startPointY = kl.startPointY;
		osl.endPointX = kl.endPointX;
		osl.endPointY = kl.endPointY;
		osl.sPointInOctaveX = kl.sPointInOctaveX;
		osl.sPointInOctaveY = kl.sPointInOctaveY;
		osl.ePointInOctaveX = kl.ePointInOctaveX;
		osl.ePointInOctaveY = kl.ePointInOctaveY;
		osl.lineLength = kl.lineLength;
		osl.numOfPixels = kl.numOfPixels;
		osl.salience = kl.response;

		osl.direction = kl.angle;
		osl.octaveCount = kl.octave;

		/* store new line */
		sl[kl.class_id][kl.octave] = osl;

		/* update map */
		int id = kl.class_id;
		int oct = kl.octave;
		correspondences.insert(std::pair<std::pair<int, int>, size_t>(std::pair<int, int>(id, oct), slCounter));
	}

	/* delete useless OctaveSingleLines */
	for (size_t i = 0; i < sl.size(); i++)
	{
		for (size_t j = 0; j < sl[i].size(); j++)
		{
			//if( (int) ( sl[i][j] ).octaveCount > params.numOfOctave_ )
			if ((int)(sl[i][j]).octaveCount > octaveIndex)
				(sl[i]).erase((sl[i]).begin() + j);
		}
	}

	/* compute LBD descriptors */
	bd->computeLBD(sl, useDetectionData);

	/* resize output matrix */
	if (!returnFloatDescr)
		descriptors = cv::Mat((int)keylines.size(), 32, CV_8UC1);

	else
		descriptors = cv::Mat((int)keylines.size(), NUM_OF_BANDS * 8, CV_32FC1);

	/* fill output matrix with descriptors */
	for (int k = 0; k < (int)sl.size(); k++)
	{
		for (int lineC = 0; lineC < (int)sl[k].size(); lineC++)
		{
			/* get original index of keypoint */
			int lineOctave = (sl[k][lineC]).octaveCount;
			int originalIndex = (int)correspondences.find(std::pair<int, int>(k, lineOctave))->second;

			if (!returnFloatDescr)
			{
				/* get a pointer to correspondent row in output matrix */
				uchar* pointerToRow = descriptors.ptr(originalIndex);

				/* get LBD data */
				float* desVec = &sl[k][lineC].descriptor.front();

				/* fill current row with binary descriptor */
				for (int comb = 0; comb < 32; comb++)
				{
					*pointerToRow = bd->binaryConversion(&desVec[8 * combinations[comb][0]], &desVec[8 * combinations[comb][1]]);
					pointerToRow++;
				}
			}

			else
			{
				/* get a pointer to correspondent row in output matrix */
				float* pointerToRow = descriptors.ptr<float>(originalIndex);

				/* get LBD data */
				std::vector<float> desVec = sl[k][lineC].descriptor;

				for (int count = 0; count < (int)desVec.size(); count++)
				{
					*pointerToRow = desVec[count];
					pointerToRow++;
				}
			}

		}
	}

}

int LineDescriptor::OctaveKeyLines(cv::Mat& image, ScaleLines &keyLines)
{

	/* final number of extracted lines 最终提取的直线数目*/
	unsigned int numOfFinalLine = 0;

	/* sigma values and reduction factor used in Gaussian pyramids 高斯函数中的sigma参数和octave层之间的缩放因子*/
	float preSigma2 = 0;  //orignal image is not blurred, has zero sigma;原图没有滤波，sigma=0
	float curSigma2 = 1.0;  //[sqrt(2)]^0=1;
	double factor = sqrt(2.0);  //the down sample factor between connective two octave images 相邻两层图像间的下采样参数

	/* loop over number of octaves 循环每个octave层*/
	for (int octaveCount = 0; octaveCount < params.numOfOctave_; octaveCount++)
	{
		/* matrix storing results from blurring processes 滤波后的图像*/
		cv::Mat blur;

		/* apply Gaussian blur 进行高斯滤波*/
		float increaseSigma = sqrt(curSigma2 - preSigma2);
		cv::GaussianBlur(image, blur, cv::Size(params.ksize_, params.ksize_), increaseSigma);
		images_sizes[octaveCount] = blur.size();

		/* for current octave, extract lines 对当前层提取直线，提取成功返回1*/
		if ((edLineVec_[octaveCount]->EDline(blur)) != 1)
		{
			return -1;
		}

		/* update number of total extracted lines 更新检测到的直线数目总和*/
		numOfFinalLine += edLineVec_[octaveCount]->lines_.numOfLines;

		/* resize image for next level of pyramid 金字塔缩放图像，获取下一层图像，每层缩放1/2*/
		cv::resize(blur, image, cv::Size(), (1.f / factor), (1.f / factor));

		/* update sigma values 更新sigma参数*/
		preSigma2 = curSigma2;
		curSigma2 = curSigma2 * 2;

	} /* end of loop over number of octaves */

	/* prepare a vector to store octave information associated to extracted lines 用来存储每层直线的信息的向量*/
	std::vector < OctaveLine > octaveLines(numOfFinalLine);

	/* set lines' counter to 0 for reuse 清零*/
	numOfFinalLine = 0;

	/* counter to give a unique ID to lines in LineVecs 直线的ID*/
	unsigned int lineIDInScaleLineVec = 0;

	/* floats to compute lines' lengths */
	float dx, dy;

	/* loop over lines extracted from scale 0 (original image) 循环0层（原图）的每条直线*/
	for (unsigned int lineCurId = 0; lineCurId < edLineVec_[0]->lines_.numOfLines; lineCurId++)
	{
		/* FOR CURRENT LINE: */

		/* set octave from which it was extracted 记录其所处的层级*/
		octaveLines[numOfFinalLine].octaveCount = 0;
		/* set ID within its octave 记录直线原ID*/
		octaveLines[numOfFinalLine].lineIDInOctave = lineCurId;
		/* set a unique ID among all lines extracted in all octaves 设置新的ID，从0开始*/
		octaveLines[numOfFinalLine].lineIDInScaleLineVec = lineIDInScaleLineVec;

		/* compute absolute value of difference between X coordinates of line's extreme points */
		dx = fabs(edLineVec_[0]->lineEndpoints_[lineCurId][0] - edLineVec_[0]->lineEndpoints_[lineCurId][2]);
		/* compute absolute value of difference between Y coordinates of line's extreme points */
		dy = fabs(edLineVec_[0]->lineEndpoints_[lineCurId][1] - edLineVec_[0]->lineEndpoints_[lineCurId][3]);
		/* compute line's length 计算直线的长度*/
		octaveLines[numOfFinalLine].lineLength = sqrt(dx * dx + dy * dy);

		/* update counters */
		numOfFinalLine++;
		lineIDInScaleLineVec++;
	}

	/* create and fill an array to store scale factors 存储尺度因子*/
	float *scale = new float[params.numOfOctave_];
	scale[0] = 1;
	for (int octaveCount = 1; octaveCount < params.numOfOctave_; octaveCount++)
	{
		scale[octaveCount] = (float)(factor * scale[octaveCount - 1]);
	}

	/* some variables' declarations */
	float rho1, rho2, tempValue;
	float direction, diffNear, length;
	unsigned int octaveID, lineIDInOctave;

	/*more than one octave image, organize lines in scale space. 不止一层时，在尺度空间里对直线操作
	*lines corresponding to the same line in octave images should have the same index in the ScaleLineVec */
	if (params.numOfOctave_ > 1)
	{
		/* some other variables' declarations */
		double twoPI = 2 * M_PI;
		unsigned int closeLineID = 0;
		float endPointDis, minEndPointDis, minLocalDis, maxLocalDis;
		float lp0, lp1, lp2, lp3, np0, np1, np2, np3;

		/* loop over list of octaves */
		for (int octaveCount = 1; octaveCount < params.numOfOctave_; octaveCount++)
		{
			/*for each line in current octave image, find their corresponding lines in the octaveLines,
			*give them the same value of lineIDInScaleLineVec*/

			/* loop over list of lines extracted from current octave */
			for (unsigned int lineCurId = 0; lineCurId < edLineVec_[octaveCount]->lines_.numOfLines; lineCurId++)
			{
				/* get (scaled) known term from equation of current line */
				rho1 = (float)(scale[octaveCount] * fabs(edLineVec_[octaveCount]->lineEquations_[lineCurId][2]));

				/*nearThreshold depends on the distance of the image coordinate origin to current line.
				*so nearThreshold = rho1 * nearThresholdRatio, where nearThresholdRatio = 1-cos(10*pi/180) = 0.0152*/
				tempValue = (float)(rho1 * 0.0152);
				float diffNearThreshold = (tempValue > 6) ? (tempValue) : 6;
				diffNearThreshold = (diffNearThreshold < 12) ? diffNearThreshold : 12;

				/* compute scaled lenght of current line */
				dx = fabs(edLineVec_[octaveCount]->lineEndpoints_[lineCurId][0] - edLineVec_[octaveCount]->lineEndpoints_[lineCurId][2]);  //x1-x2
				dy = fabs(edLineVec_[octaveCount]->lineEndpoints_[lineCurId][1] - edLineVec_[octaveCount]->lineEndpoints_[lineCurId][3]);  //y1-y2
				length = scale[octaveCount] * sqrt(dx * dx + dy * dy);

				minEndPointDis = 12;
				/* loop over the octave representations of all lines */
				for (unsigned int lineNextId = 0; lineNextId < numOfFinalLine; lineNextId++)
				{
					/* if a line from same octave is encountered,
					a comparison with it shouldn't be considered */
					octaveID = octaveLines[lineNextId].octaveCount;
					if ((int)octaveID == octaveCount)
					{  //lines in the same layer of octave image should not be compared.
						break;
					}

					/* take ID in octave of line to be compared */
					lineIDInOctave = octaveLines[lineNextId].lineIDInOctave;

					/*first check whether current line and next line are parallel.
					*If line1:a1*x+b1*y+c1=0 and line2:a2*x+b2*y+c2=0 are parallel, then
					*-a1/b1=-a2/b2, i.e., a1b2=b1a2.
					*we define parallel=fabs(a1b2-b1a2)
					*note that, in EDLine class, we have normalized the line equations
					*to make a1^2+ b1^2 = a2^2+ b2^2 = 1*/
					direction = fabs(edLineVec_[octaveCount]->lineDirection_[lineCurId] - edLineVec_[octaveID]->lineDirection_[lineIDInOctave]);

					/* the angle between two lines are larger than 10degrees
					(i.e. 10*pi/180=0.1745), they are not close to parallel */
					if (direction > 0.1745 && (twoPI - direction > 0.1745))
					{
						continue;
					}
					/*now check whether current line and next line are near to each other.
					*If line1:a1*x+b1*y+c1=0 and line2:a2*x+b2*y+c2=0 are near in image, then
					*rho1 = |a1*0+b1*0+c1|/sqrt(a1^2+b1^2) and rho2 = |a2*0+b2*0+c2|/sqrt(a2^2+b2^2) should close.
					*In our case, rho1 = |c1| and rho2 = |c2|, because sqrt(a1^2+b1^2) = sqrt(a2^2+b2^2) = 1;
					*note that, lines are in different octave images, so we define near =  fabs(scale*rho1 - rho2) or
					*where scale is the scale factor between to octave images*/

					/* get known term from equation to be compared */
					rho2 = (float)(scale[octaveID] * fabs(edLineVec_[octaveID]->lineEquations_[lineIDInOctave][2]));
					/* compute difference between known ters */
					diffNear = fabs(rho1 - rho2);

					/* two lines are not close in the image */
					if (diffNear > diffNearThreshold)
					{
						continue;
					}

					/*now check the end points distance between two lines, the scale of  distance is in the original image size.
					* find the minimal and maximal end points distance*/

					/* get the extreme points of the two lines */
					lp0 = scale[octaveCount] * edLineVec_[octaveCount]->lineEndpoints_[lineCurId][0];
					lp1 = scale[octaveCount] * edLineVec_[octaveCount]->lineEndpoints_[lineCurId][1];
					lp2 = scale[octaveCount] * edLineVec_[octaveCount]->lineEndpoints_[lineCurId][2];
					lp3 = scale[octaveCount] * edLineVec_[octaveCount]->lineEndpoints_[lineCurId][3];
					np0 = scale[octaveID] * edLineVec_[octaveID]->lineEndpoints_[lineIDInOctave][0];
					np1 = scale[octaveID] * edLineVec_[octaveID]->lineEndpoints_[lineIDInOctave][1];
					np2 = scale[octaveID] * edLineVec_[octaveID]->lineEndpoints_[lineIDInOctave][2];
					np3 = scale[octaveID] * edLineVec_[octaveID]->lineEndpoints_[lineIDInOctave][3];

					/* get the distance between the two leftmost extremes of lines
					L1(0,1)<->L2(0,1) */
					dx = lp0 - np0;
					dy = lp1 - np1;
					endPointDis = sqrt(dx * dx + dy * dy);

					/* set momentaneously min and max distance between lines to
					the one between left extremes */
					minLocalDis = endPointDis;
					maxLocalDis = endPointDis;

					/* compute distance between right extremes
					L1(2,3)<->L2(2,3) */
					dx = lp2 - np2;
					dy = lp3 - np3;
					endPointDis = sqrt(dx * dx + dy * dy);

					/* update (if necessary) min and max distance between lines */
					minLocalDis = (endPointDis < minLocalDis) ? endPointDis : minLocalDis;
					maxLocalDis = (endPointDis > maxLocalDis) ? endPointDis : maxLocalDis;

					/* compute distance between left extreme of current line and
					right extreme of line to be compared
					L1(0,1)<->L2(2,3) */
					dx = lp0 - np2;
					dy = lp1 - np3;
					endPointDis = sqrt(dx * dx + dy * dy);

					/* update (if necessary) min and max distance between lines */
					minLocalDis = (endPointDis < minLocalDis) ? endPointDis : minLocalDis;
					maxLocalDis = (endPointDis > maxLocalDis) ? endPointDis : maxLocalDis;

					/* compute distance between right extreme of current line and
					left extreme of line to be compared
					L1(2,3)<->L2(0,1) */
					dx = lp2 - np0;
					dy = lp3 - np1;
					endPointDis = sqrt(dx * dx + dy * dy);

					/* update (if necessary) min and max distance between lines */
					minLocalDis = (endPointDis < minLocalDis) ? endPointDis : minLocalDis;
					maxLocalDis = (endPointDis > maxLocalDis) ? endPointDis : maxLocalDis;

					/* check whether conditions for considering line to be compared
					worth to be inserted in the same LineVec are satisfied */
					if ((maxLocalDis < 0.8 * (length + octaveLines[lineNextId].lineLength)) && (minLocalDis < minEndPointDis))
					{  //keep the closest line
						minEndPointDis = minLocalDis;
						closeLineID = lineNextId;
					}
				}

				/* add current line into octaveLines */
				if (minEndPointDis < 12)
				{
					octaveLines[numOfFinalLine].lineIDInScaleLineVec = octaveLines[closeLineID].lineIDInScaleLineVec;
				}
				else
				{
					octaveLines[numOfFinalLine].lineIDInScaleLineVec = lineIDInScaleLineVec;
					lineIDInScaleLineVec++;
				}
				octaveLines[numOfFinalLine].octaveCount = octaveCount;
				octaveLines[numOfFinalLine].lineIDInOctave = lineCurId;
				octaveLines[numOfFinalLine].lineLength = length;
				numOfFinalLine++;
			}
		}  //end for(unsigned int octaveCount = 1; octaveCount<numOfOctave_; octaveCount++)
	}  //end if(numOfOctave_>1)

	////////////////////////////////////
	//Reorganize the detected lines into keyLines 将检测到的直线存入keyline
	keyLines.clear();
	keyLines.resize(lineIDInScaleLineVec);
	unsigned int tempID;
	float s1, e1, s2, e2;
	bool shouldChange;
	OctaveSingleLine singleLine;
	for (unsigned int lineID = 0; lineID < numOfFinalLine; lineID++)
	{
		lineIDInOctave = octaveLines[lineID].lineIDInOctave; //直线的新ID
		octaveID = octaveLines[lineID].octaveCount; //直线的旧ID，如果只有一层，则和新ID一致
		direction = edLineVec_[octaveID]->lineDirection_[lineIDInOctave]; //直线的角度
		singleLine.octaveCount = octaveID;
		singleLine.direction = direction;
		singleLine.lineLength = octaveLines[lineID].lineLength;
		singleLine.salience = edLineVec_[octaveID]->lineSalience_[lineIDInOctave];
		singleLine.numOfPixels = edLineVec_[octaveID]->lines_.sId[lineIDInOctave + 1] - edLineVec_[octaveID]->lines_.sId[lineIDInOctave];
		//decide the start point and end point
		shouldChange = false;
		s1 = edLineVec_[octaveID]->lineEndpoints_[lineIDInOctave][0];  //sx
		s2 = edLineVec_[octaveID]->lineEndpoints_[lineIDInOctave][1];  //sy
		e1 = edLineVec_[octaveID]->lineEndpoints_[lineIDInOctave][2];  //ex
		e2 = edLineVec_[octaveID]->lineEndpoints_[lineIDInOctave][3];  //ey
		dx = e1 - s1;  //ex-sx
		dy = e2 - s2;  //ey-sy
		if (direction >= -0.75 * M_PI && direction < -0.25 * M_PI)
		{
			if (dy > 0)
			{
				shouldChange = true;
			}
		}
		if (direction >= -0.25 * M_PI && direction < 0.25 * M_PI)
		{
			if (dx < 0)
			{
				shouldChange = true;
			}
		}
		if (direction >= 0.25 * M_PI && direction < 0.75 * M_PI)
		{
			if (dy < 0)
			{
				shouldChange = true;
			}
		}
		if ((direction >= 0.75 * M_PI && direction < M_PI) || (direction >= -M_PI && direction < -0.75 * M_PI))
		{
			if (dx > 0)
			{
				shouldChange = true;
			}
		}
		tempValue = scale[octaveID];
		if (shouldChange)
		{
			singleLine.sPointInOctaveX = e1;
			singleLine.sPointInOctaveY = e2;
			singleLine.ePointInOctaveX = s1;
			singleLine.ePointInOctaveY = s2;
			singleLine.startPointX = tempValue * e1;
			singleLine.startPointY = tempValue * e2;
			singleLine.endPointX = tempValue * s1;
			singleLine.endPointY = tempValue * s2;
		}
		else
		{
			singleLine.sPointInOctaveX = s1;
			singleLine.sPointInOctaveY = s2;
			singleLine.ePointInOctaveX = e1;
			singleLine.ePointInOctaveY = e2;
			singleLine.startPointX = tempValue * s1;
			singleLine.startPointY = tempValue * s2;
			singleLine.endPointX = tempValue * e1;
			singleLine.endPointY = tempValue * e2;
		}
		tempID = octaveLines[lineID].lineIDInScaleLineVec;
		keyLines[tempID].push_back(singleLine);
	}

	delete[] scale;
	return 1;
}

int LineDescriptor::computeLBD(ScaleLines &keyLines, bool useDetectionData)
{
	//the default length of the band is the line length. //Band的长度等于直线的长度
	short numOfFinalLine = (short)keyLines.size();
	float *dL = new float[2];  //line direction cos(dir), sin(dir) //直线的方向
	float *dO = new float[2];  //the clockwise orthogonal vector of line direction. //直线的顺时针垂直方向
	short heightOfLSP = (short)(params.widthOfBand_ * NUM_OF_BANDS);  //the height of line support region; //LSR区域的高度 = Band的宽度（7）*Band的个数（9）
	short descriptor_size = NUM_OF_BANDS * 8;  //each band, we compute the m( pgdL, ngdL,  pgdO, ngdO) and std( pgdL, ngdL,  pgdO, ngdO); //每个Band计算其均值和标准差
	float pgdLRowSum;  //the summation of {g_dL |g_dL>0 } for each row of the region; //
	float ngdLRowSum;  //the summation of {g_dL |g_dL<0 } for each row of the region;
	float pgdL2RowSum;  //the summation of {g_dL^2 |g_dL>0 } for each row of the region;
	float ngdL2RowSum;  //the summation of {g_dL^2 |g_dL<0 } for each row of the region;
	float pgdORowSum;  //the summation of {g_dO |g_dO>0 } for each row of the region;
	float ngdORowSum;  //the summation of {g_dO |g_dO<0 } for each row of the region;
	float pgdO2RowSum;  //the summation of {g_dO^2 |g_dO>0 } for each row of the region;
	float ngdO2RowSum;  //the summation of {g_dO^2 |g_dO<0 } for each row of the region;

	float *pgdLBandSum = new float[NUM_OF_BANDS];  //the summation of {g_dL |g_dL>0 } for each band of the region;
	float *ngdLBandSum = new float[NUM_OF_BANDS];  //the summation of {g_dL |g_dL<0 } for each band of the region;
	float *pgdL2BandSum = new float[NUM_OF_BANDS];  //the summation of {g_dL^2 |g_dL>0 } for each band of the region;
	float *ngdL2BandSum = new float[NUM_OF_BANDS];  //the summation of {g_dL^2 |g_dL<0 } for each band of the region;
	float *pgdOBandSum = new float[NUM_OF_BANDS];  //the summation of {g_dO |g_dO>0 } for each band of the region;
	float *ngdOBandSum = new float[NUM_OF_BANDS];  //the summation of {g_dO |g_dO<0 } for each band of the region;
	float *pgdO2BandSum = new float[NUM_OF_BANDS];  //the summation of {g_dO^2 |g_dO>0 } for each band of the region;
	float *ngdO2BandSum = new float[NUM_OF_BANDS];  //the summation of {g_dO^2 |g_dO<0 } for each band of the region;

	short numOfBitsBand = NUM_OF_BANDS * sizeof(float);
	short lengthOfLSP;  //the length of line support region, varies with lines //LSR区域的长度，与直线长度相关
	short halfHeight = (heightOfLSP - 1) / 2;
	short halfWidth;
	short bandID;
	float coefInGaussion;
	float lineMiddlePointX, lineMiddlePointY;
	float sCorX, sCorY, sCorX0, sCorY0;
	short tempCor, xCor, yCor;  //pixel coordinates in image plane
	short dx, dy;
	float gDL;  //store the gradient projection of pixels in support region along dL vector
	float gDO;  //store the gradient projection of pixels in support region along dO vector
	short imageWidth, imageHeight, realWidth;
	short *pdxImg, *pdyImg;
	float *desVec;

	short sameLineSize;
	short octaveCount;
	OctaveSingleLine *pSingleLine;
	/* loop over list of LineVec */
	for (short lineIDInScaleVec = 0; lineIDInScaleVec < numOfFinalLine; lineIDInScaleVec++)
	{
		sameLineSize = (short)(keyLines[lineIDInScaleVec].size());
		/* loop over current LineVec's lines */
		for (short lineIDInSameLine = 0; lineIDInSameLine < sameLineSize; lineIDInSameLine++)
		{
			/* get a line in current LineVec and its original ID in its octave */
			pSingleLine = &(keyLines[lineIDInScaleVec][lineIDInSameLine]);
			octaveCount = (short)pSingleLine->octaveCount;

			if (useDetectionData)
			{
				/* retrieve associated dxImg and dyImg */
				pdxImg = edLineVec_[octaveCount]->dxImg_.ptr<short>();
				pdyImg = edLineVec_[octaveCount]->dyImg_.ptr<short>();

				/* get image size to work on from real one */
				realWidth = (short)edLineVec_[octaveCount]->imageWidth;
				imageWidth = realWidth - 1;
				imageHeight = (short)(edLineVec_[octaveCount]->imageHeight - 1);
			}

			else
			{
				/* retrieve associated dxImg and dyImg */
				pdxImg = dxImg_vector[octaveCount].ptr<short>();
				pdyImg = dyImg_vector[octaveCount].ptr<short>();

				/* get image size to work on from real one */
				realWidth = (short)images_sizes[octaveCount].width;
				imageWidth = realWidth - 1;
				imageHeight = (short)(images_sizes[octaveCount].height - 1);
			}

			/* initialize memory areas */
			memset(pgdLBandSum, 0, numOfBitsBand);
			memset(ngdLBandSum, 0, numOfBitsBand);
			memset(pgdL2BandSum, 0, numOfBitsBand);
			memset(ngdL2BandSum, 0, numOfBitsBand);
			memset(pgdOBandSum, 0, numOfBitsBand);
			memset(ngdOBandSum, 0, numOfBitsBand);
			memset(pgdO2BandSum, 0, numOfBitsBand);
			memset(ngdO2BandSum, 0, numOfBitsBand);

			/* get length of line and its half */
			lengthOfLSP = (short)keyLines[lineIDInScaleVec][lineIDInSameLine].numOfPixels;
			halfWidth = (lengthOfLSP - 1) / 2;

			/* get middlepoint of line */
			lineMiddlePointX = (float)(0.5 * (pSingleLine->sPointInOctaveX + pSingleLine->ePointInOctaveX));
			lineMiddlePointY = (float)(0.5 * (pSingleLine->sPointInOctaveY + pSingleLine->ePointInOctaveY));

			/*1.rotate the local coordinate system to the line direction (direction is the angle
			between positive line direction and positive X axis)
			*2.compute the gradient projection of pixels in line support region*/

			/* get the vector representing original image reference system after rotation to aligh with
			line's direction */
			dL[0] = cos(pSingleLine->direction);
			dL[1] = sin(pSingleLine->direction);

			/* set the clockwise orthogonal vector of line direction */
			dO[0] = -dL[1];
			dO[1] = dL[0];

			/* get rotated reference frame */
			sCorX0 = -dL[0] * halfWidth + dL[1] * halfHeight + lineMiddlePointX;  //hID =0; wID = 0;
			sCorY0 = -dL[1] * halfWidth - dL[0] * halfHeight + lineMiddlePointY;

			/* BIAS::Matrix<float> gDLMat(heightOfLSP,lengthOfLSP) */
			for (short hID = 0; hID < heightOfLSP; hID++)
			{
				/*initialization */
				sCorX = sCorX0;
				sCorY = sCorY0;

				pgdLRowSum = 0;
				ngdLRowSum = 0;
				pgdORowSum = 0;
				ngdORowSum = 0;

				for (short wID = 0; wID < lengthOfLSP; wID++)
				{
					tempCor = (short)round(sCorX);
					xCor = (tempCor < 0) ? 0 : (tempCor > imageWidth) ? imageWidth : tempCor;
					tempCor = (short)round(sCorY);
					yCor = (tempCor < 0) ? 0 : (tempCor > imageHeight) ? imageHeight : tempCor;

					/* To achieve rotation invariance, each simple gradient is rotated aligned with
					* the line direction and clockwise orthogonal direction.*/
					dx = pdxImg[yCor * realWidth + xCor];
					dy = pdyImg[yCor * realWidth + xCor];
					gDL = dx * dL[0] + dy * dL[1];
					gDO = dx * dO[0] + dy * dO[1];
					if (gDL > 0)
					{
						pgdLRowSum += gDL;
					}
					else
					{
						ngdLRowSum -= gDL;
					}
					if (gDO > 0)
					{
						pgdORowSum += gDO;
					}
					else
					{
						ngdORowSum -= gDO;
					}
					sCorX += dL[0];
					sCorY += dL[1];
					/* gDLMat[hID][wID] = gDL; */
				}
				sCorX0 -= dL[1];
				sCorY0 += dL[0];
				coefInGaussion = (float)gaussCoefG_[hID];
				pgdLRowSum = coefInGaussion * pgdLRowSum;
				ngdLRowSum = coefInGaussion * ngdLRowSum;
				pgdL2RowSum = pgdLRowSum * pgdLRowSum;
				ngdL2RowSum = ngdLRowSum * ngdLRowSum;
				pgdORowSum = coefInGaussion * pgdORowSum;
				ngdORowSum = coefInGaussion * ngdORowSum;
				pgdO2RowSum = pgdORowSum * pgdORowSum;
				ngdO2RowSum = ngdORowSum * ngdORowSum;

				/* compute {g_dL |g_dL>0 }, {g_dL |g_dL<0 },
				{g_dO |g_dO>0 }, {g_dO |g_dO<0 } of each band in the line support region
				first, current row belong to current band */
				bandID = (short)(hID / params.widthOfBand_);
				coefInGaussion = (float)(gaussCoefL_[hID % params.widthOfBand_ + params.widthOfBand_]);
				pgdLBandSum[bandID] += coefInGaussion * pgdLRowSum;
				ngdLBandSum[bandID] += coefInGaussion * ngdLRowSum;
				pgdL2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdL2RowSum;
				ngdL2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdL2RowSum;
				pgdOBandSum[bandID] += coefInGaussion * pgdORowSum;
				ngdOBandSum[bandID] += coefInGaussion * ngdORowSum;
				pgdO2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdO2RowSum;
				ngdO2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdO2RowSum;

				/* In order to reduce boundary effect along the line gradient direction,
				* a row's gradient will contribute not only to its current band, but also
				* to its nearest upper and down band with gaussCoefL_.*/
				bandID--;
				if (bandID >= 0)
				{/* the band above the current band */
					coefInGaussion = (float)(gaussCoefL_[hID % params.widthOfBand_ + 2 * params.widthOfBand_]);
					pgdLBandSum[bandID] += coefInGaussion * pgdLRowSum;
					ngdLBandSum[bandID] += coefInGaussion * ngdLRowSum;
					pgdL2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdL2RowSum;
					ngdL2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdL2RowSum;
					pgdOBandSum[bandID] += coefInGaussion * pgdORowSum;
					ngdOBandSum[bandID] += coefInGaussion * ngdORowSum;
					pgdO2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdO2RowSum;
					ngdO2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdO2RowSum;
				}
				bandID = bandID + 2;
				if (bandID < NUM_OF_BANDS)
				{/*the band below the current band */
					coefInGaussion = (float)(gaussCoefL_[hID % params.widthOfBand_]);
					pgdLBandSum[bandID] += coefInGaussion * pgdLRowSum;
					ngdLBandSum[bandID] += coefInGaussion * ngdLRowSum;
					pgdL2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdL2RowSum;
					ngdL2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdL2RowSum;
					pgdOBandSum[bandID] += coefInGaussion * pgdORowSum;
					ngdOBandSum[bandID] += coefInGaussion * ngdORowSum;
					pgdO2BandSum[bandID] += coefInGaussion * coefInGaussion * pgdO2RowSum;
					ngdO2BandSum[bandID] += coefInGaussion * coefInGaussion * ngdO2RowSum;
				}
			}
			/* gDLMat.Save("gDLMat.txt");
			return 0; */

			/* construct line descriptor */
			pSingleLine->descriptor.resize(descriptor_size);
			desVec = &pSingleLine->descriptor.front();

			short desID;

			/*Note that the first and last bands only have (lengthOfLSP * widthOfBand_ * 2.0) pixels
			* which are counted. */
			float invN2 = (float)(1.0 / (params.widthOfBand_ * 2.0));
			float invN3 = (float)(1.0 / (params.widthOfBand_ * 3.0));
			float invN, temp;
			for (bandID = 0; bandID < NUM_OF_BANDS; bandID++)
			{
				if (bandID == 0 || bandID == NUM_OF_BANDS - 1)
				{
					invN = invN2;
				}
				else
				{
					invN = invN3;
				}
				desID = bandID * 8;
				temp = pgdLBandSum[bandID] * invN;
				desVec[desID] = temp;/* mean value of pgdL; */
				desVec[desID + 4] = sqrt(pgdL2BandSum[bandID] * invN - temp * temp);  //std value of pgdL;
				temp = ngdLBandSum[bandID] * invN;
				desVec[desID + 1] = temp;  //mean value of ngdL;
				desVec[desID + 5] = sqrt(ngdL2BandSum[bandID] * invN - temp * temp);  //std value of ngdL;

				temp = pgdOBandSum[bandID] * invN;
				desVec[desID + 2] = temp;  //mean value of pgdO;
				desVec[desID + 6] = sqrt(pgdO2BandSum[bandID] * invN - temp * temp);  //std value of pgdO;
				temp = ngdOBandSum[bandID] * invN;
				desVec[desID + 3] = temp;  //mean value of ngdO;
				desVec[desID + 7] = sqrt(ngdO2BandSum[bandID] * invN - temp * temp);  //std value of ngdO;
			}

			// normalize;
			float tempM, tempS;
			tempM = 0;
			tempS = 0;
			desVec = &pSingleLine->descriptor.front();

			int base = 0;
			for (short i = 0; i < (short)(NUM_OF_BANDS * 8); ++base, i = (short)(base * 8))
			{
				tempM += *(desVec + i) * * (desVec + i);  //desVec[8*i+0] * desVec[8*i+0];
				tempM += *(desVec + i + 1) * * (desVec + i + 1);  //desVec[8*i+1] * desVec[8*i+1];
				tempM += *(desVec + i + 2) * * (desVec + i + 2);  //desVec[8*i+2] * desVec[8*i+2];
				tempM += *(desVec + i + 3) * * (desVec + i + 3);  //desVec[8*i+3] * desVec[8*i+3];
				tempS += *(desVec + i + 4) * * (desVec + i + 4);  //desVec[8*i+4] * desVec[8*i+4];
				tempS += *(desVec + i + 5) * * (desVec + i + 5);  //desVec[8*i+5] * desVec[8*i+5];
				tempS += *(desVec + i + 6) * * (desVec + i + 6);  //desVec[8*i+6] * desVec[8*i+6];
				tempS += *(desVec + i + 7) * * (desVec + i + 7);  //desVec[8*i+7] * desVec[8*i+7];
			}

			tempM = 1 / sqrt(tempM);
			tempS = 1 / sqrt(tempS);
			desVec = &pSingleLine->descriptor.front();
			base = 0;
			for (short i = 0; i < (short)(NUM_OF_BANDS * 8); ++base, i = (short)(base * 8))
			{
				*(desVec + i) = *(desVec + i) * tempM;  //desVec[8*i] =  desVec[8*i] * tempM;
				*(desVec + 1 + i) = *(desVec + 1 + i) * tempM;  //desVec[8*i+1] =  desVec[8*i+1] * tempM;
				*(desVec + 2 + i) = *(desVec + 2 + i) * tempM;  //desVec[8*i+2] =  desVec[8*i+2] * tempM;
				*(desVec + 3 + i) = *(desVec + 3 + i) * tempM;  //desVec[8*i+3] =  desVec[8*i+3] * tempM;
				*(desVec + 4 + i) = *(desVec + 4 + i) * tempS;  //desVec[8*i+4] =  desVec[8*i+4] * tempS;
				*(desVec + 5 + i) = *(desVec + 5 + i) * tempS;  //desVec[8*i+5] =  desVec[8*i+5] * tempS;
				*(desVec + 6 + i) = *(desVec + 6 + i) * tempS;  //desVec[8*i+6] =  desVec[8*i+6] * tempS;
				*(desVec + 7 + i) = *(desVec + 7 + i) * tempS;  //desVec[8*i+7] =  desVec[8*i+7] * tempS;
			}

			/* In order to reduce the influence of non-linear illumination,
			* a threshold is used to limit the value of element in the unit feature
			* vector no larger than this threshold. In Z.Wang's work, a value of 0.4 is found
			* empirically to be a proper threshold.*/
			desVec = &pSingleLine->descriptor.front();
			for (short i = 0; i < descriptor_size; i++)
			{
				if (desVec[i] > 0.4)
				{
					desVec[i] = (float) 0.4;
				}
			}

			//re-normalize desVec;
			temp = 0;
			for (short i = 0; i < descriptor_size; i++)
			{
				temp += desVec[i] * desVec[i];
			}

			temp = 1 / sqrt(temp);
			for (short i = 0; i < descriptor_size; i++)
			{
				desVec[i] = desVec[i] * temp;
			}
		}/* end for(short lineIDInSameLine = 0; lineIDInSameLine<sameLineSize;
		 lineIDInSameLine++) */

		cv::Mat appoggio = cv::Mat(1, 32, CV_32FC1);
		float* pointerToRow = appoggio.ptr<float>(0);
		for (int g = 0; g < 32; g++)
		{
			/* get LBD data */
			float* des_Vec = &keyLines[lineIDInScaleVec][0].descriptor.front();
			*pointerToRow = des_Vec[g];
			pointerToRow++;

		}

	}/* end for(short lineIDInScaleVec = 0;
	 lineIDInScaleVec<numOfFinalLine; lineIDInScaleVec++) */

	delete[] dL;
	delete[] dO;
	delete[] pgdLBandSum;
	delete[] ngdLBandSum;
	delete[] pgdL2BandSum;
	delete[] ngdL2BandSum;
	delete[] pgdOBandSum;
	delete[] ngdOBandSum;
	delete[] pgdO2BandSum;
	delete[] ngdO2BandSum;

	return 1;

}

LineDescriptor::EDLineDetector::EDLineDetector()
{
	//set parameters for line segment detection
	ksize_ = 15;  //15
	sigma_ = 30.0;  //30
	gradienThreshold_ = 80;  // ***** ORIGINAL WAS 25
	anchorThreshold_ = 8;  //8
	scanIntervals_ = 2;  //2
	minLineLen_ = 15;  //15
	lineFitErrThreshold_ = 1.6;  //1.4
	InitEDLine_();
}
LineDescriptor::EDLineDetector::EDLineDetector(EDLineParam param)
{
	//set parameters for line segment detection
	ksize_ = param.ksize;
	sigma_ = param.sigma;
	gradienThreshold_ = (short)param.gradientThreshold;
	anchorThreshold_ = (unsigned char)param.anchorThreshold;
	scanIntervals_ = param.scanIntervals;
	minLineLen_ = param.minLineLen;
	lineFitErrThreshold_ = param.lineFitErrThreshold;
	InitEDLine_();
}
void LineDescriptor::EDLineDetector::InitEDLine_()
{
	bValidate_ = true;
	ATA = cv::Mat_<int>(2, 2);
	ATV = cv::Mat_<int>(1, 2);
	tempMatLineFit = cv::Mat_<int>(2, 2);
	tempVecLineFit = cv::Mat_<int>(1, 2);
	fitMatT = cv::Mat_<int>(2, minLineLen_);
	fitVec = cv::Mat_<int>(1, minLineLen_);
	for (int i = 0; i < minLineLen_; i++)
	{
		fitMatT[1][i] = 1;
	}
	dxImg_.create(1, 1, CV_16SC1);
	dyImg_.create(1, 1, CV_16SC1);
	gImgWO_.create(1, 1, CV_8SC1);
	pFirstPartEdgeX_ = NULL;
	pFirstPartEdgeY_ = NULL;
	pFirstPartEdgeS_ = NULL;
	pSecondPartEdgeX_ = NULL;
	pSecondPartEdgeY_ = NULL;
	pSecondPartEdgeS_ = NULL;
	pAnchorX_ = NULL;
	pAnchorY_ = NULL;
}

LineDescriptor::EDLineDetector::~EDLineDetector()
{
	if (pFirstPartEdgeX_ != NULL)
	{
		delete[] pFirstPartEdgeX_;
		delete[] pFirstPartEdgeY_;
		delete[] pSecondPartEdgeX_;
		delete[] pSecondPartEdgeY_;
		delete[] pAnchorX_;
		delete[] pAnchorY_;
	}
	if (pFirstPartEdgeS_ != NULL)
	{
		delete[] pFirstPartEdgeS_;
		delete[] pSecondPartEdgeS_;
	}
}

int LineDescriptor::EDLineDetector::EdgeDrawing(cv::Mat &image, EdgeChains &edgeChains)
{
	imageWidth = image.cols;
	imageHeight = image.rows;
	unsigned int pixelNum = imageWidth * imageHeight;

	unsigned int edgePixelArraySize = pixelNum / 5;
	unsigned int maxNumOfEdge = edgePixelArraySize / 20;
	//compute dx, dy images
	if (gImg_.cols != (int)imageWidth || gImg_.rows != (int)imageHeight)
	{
		if (pFirstPartEdgeX_ != NULL)
		{
			delete[] pFirstPartEdgeX_;
			delete[] pFirstPartEdgeY_;
			delete[] pSecondPartEdgeX_;
			delete[] pSecondPartEdgeY_;
			delete[] pFirstPartEdgeS_;
			delete[] pSecondPartEdgeS_;
			delete[] pAnchorX_;
			delete[] pAnchorY_;
		}

		dxImg_.create(imageHeight, imageWidth, CV_16SC1);
		dyImg_.create(imageHeight, imageWidth, CV_16SC1);
		gImgWO_.create(imageHeight, imageWidth, CV_16SC1);
		gImg_.create(imageHeight, imageWidth, CV_16SC1);
		dirImg_.create(imageHeight, imageWidth, CV_8UC1);
		edgeImage_.create(imageHeight, imageWidth, CV_8UC1);
		pFirstPartEdgeX_ = new unsigned int[edgePixelArraySize];
		pFirstPartEdgeY_ = new unsigned int[edgePixelArraySize];
		pSecondPartEdgeX_ = new unsigned int[edgePixelArraySize];
		pSecondPartEdgeY_ = new unsigned int[edgePixelArraySize];
		pFirstPartEdgeS_ = new unsigned int[maxNumOfEdge];
		pSecondPartEdgeS_ = new unsigned int[maxNumOfEdge];
		pAnchorX_ = new unsigned int[edgePixelArraySize];
		pAnchorY_ = new unsigned int[edgePixelArraySize];
	}
	cv::Sobel(image, dxImg_, CV_16SC1, 1, 0, 3);
	cv::Sobel(image, dyImg_, CV_16SC1, 0, 1, 3);

	//compute gradient and direction images
	cv::Mat dxABS_m = cv::abs(dxImg_);
	cv::Mat dyABS_m = cv::abs(dyImg_);
	cv::Mat sumDxDy;
	cv::add(dyABS_m, dxABS_m, sumDxDy);

	cv::threshold(sumDxDy, gImg_, gradienThreshold_ + 1, 255, cv::THRESH_TOZERO);
	gImg_ = gImg_ / 4;
	gImgWO_ = sumDxDy / 4;
	cv::compare(dxABS_m, dyABS_m, dirImg_, cv::CMP_LT);

	short *pgImg = gImg_.ptr<short>();
	unsigned char *pdirImg = dirImg_.ptr();

	//extract the anchors in the gradient image, store into a vector
	memset(pAnchorX_, 0, edgePixelArraySize * sizeof(unsigned int));  //initialization
	memset(pAnchorY_, 0, edgePixelArraySize * sizeof(unsigned int));
	unsigned int anchorsSize = 0;
	int indexInArray;
	unsigned char gValue1, gValue2, gValue3;
	for (unsigned int w = 1; w < imageWidth - 1; w = w + scanIntervals_)
	{
		for (unsigned int h = 1; h < imageHeight - 1; h = h + scanIntervals_)
		{
			indexInArray = h * imageWidth + w;
			//gValue1 = pdirImg[indexInArray];
			if (pdirImg[indexInArray] == Horizontal)
			{  //if the direction of pixel is horizontal, then compare with up and down
				//gValue2 = pgImg[indexInArray];
				if (pgImg[indexInArray] >= pgImg[indexInArray - imageWidth] + anchorThreshold_
					&& pgImg[indexInArray] >= pgImg[indexInArray + imageWidth] + anchorThreshold_)
				{       // (w,h) is accepted as an anchor
					pAnchorX_[anchorsSize] = w;
					pAnchorY_[anchorsSize++] = h;
				}
			}
			else
			{       // if(pdirImg[indexInArray]==Vertical){//it is vertical edge, should be compared with left and right
				//gValue2 = pgImg[indexInArray];
				if (pgImg[indexInArray] >= pgImg[indexInArray - 1] + anchorThreshold_ && pgImg[indexInArray] >= pgImg[indexInArray + 1] + anchorThreshold_)
				{       // (w,h) is accepted as an anchor
					pAnchorX_[anchorsSize] = w;
					pAnchorY_[anchorsSize++] = h;
				}
			}
		}
	}
	if (anchorsSize > edgePixelArraySize)
	{
		std::cout << "anchor size is larger than its maximal size. anchorsSize=" << anchorsSize << ", maximal size = " << edgePixelArraySize << std::endl;
		return -1;
	}

	//link the anchors by smart routing
	edgeImage_.setTo(0);
	unsigned char *pEdgeImg = edgeImage_.data;
	memset(pFirstPartEdgeX_, 0, edgePixelArraySize * sizeof(unsigned int));       //initialization
	memset(pFirstPartEdgeY_, 0, edgePixelArraySize * sizeof(unsigned int));
	memset(pSecondPartEdgeX_, 0, edgePixelArraySize * sizeof(unsigned int));
	memset(pSecondPartEdgeY_, 0, edgePixelArraySize * sizeof(unsigned int));
	memset(pFirstPartEdgeS_, 0, maxNumOfEdge * sizeof(unsigned int));
	memset(pSecondPartEdgeS_, 0, maxNumOfEdge * sizeof(unsigned int));
	unsigned int offsetPFirst = 0, offsetPSecond = 0;
	unsigned int offsetPS = 0;

	unsigned int x, y;
	unsigned int lastX = 0;
	unsigned int lastY = 0;
	unsigned char lastDirection;        //up = 1, right = 2, down = 3, left = 4;
	unsigned char shouldGoDirection;        //up = 1, right = 2, down = 3, left = 4;
	int edgeLenFirst, edgeLenSecond;
	for (unsigned int i = 0; i < anchorsSize; i++)
	{
		x = pAnchorX_[i];
		y = pAnchorY_[i];
		indexInArray = y * imageWidth + x;
		if (pEdgeImg[indexInArray])
		{       //if anchor i is already been an edge pixel.
			continue;
		}
		/*The walk stops under 3 conditions:
		* 1. We move out of the edge areas, i.e., the thresholded gradient value
		*    of the current pixel is 0.
		* 2. The current direction of the edge changes, i.e., from horizontal
		*    to vertical or vice versa.?? (This is turned out not correct. From the online edge draw demo
		*    we can figure out that authors don't implement this rule either because their extracted edge
		*    chain could be a circle which means pixel directions would definitely be different
		*    in somewhere on the chain.)
		* 3. We encounter a previously detected edge pixel. */
		pFirstPartEdgeS_[offsetPS] = offsetPFirst;
		if (pdirImg[indexInArray] == Horizontal)
		{       //if the direction of this pixel is horizontal, then go left and right.
			//fist go right, pixel direction may be different during linking.
			lastDirection = RightDir;
			while (pgImg[indexInArray] > 0 && !pEdgeImg[indexInArray])
			{
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pFirstPartEdgeX_[offsetPFirst] = x;
				pFirstPartEdgeY_[offsetPFirst++] = y;
				shouldGoDirection = 0;        //unknown
				if (pdirImg[indexInArray] == Horizontal)
				{        //should go left or right
					if (lastDirection == UpDir || lastDirection == DownDir)
					{        //change the pixel direction now
						if (x > lastX)
						{        //should go right
							shouldGoDirection = RightDir;
						}
						else
						{        //should go left
							shouldGoDirection = LeftDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == RightDir || shouldGoDirection == RightDir)
					{        //go right
						if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-right
							x = x + 1;
							y = y + 1;
						}
						else
						{        //straight-right
							x = x + 1;
						}
						lastDirection = RightDir;
					}
					else if (lastDirection == LeftDir || shouldGoDirection == LeftDir)
					{        //go left
						if (x == 0 || y == 0 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						gValue2 = (unsigned char)pgImg[indexInArray - 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-left
							x = x - 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-left
							x = x - 1;
							y = y + 1;
						}
						else
						{        //straight-left
							x = x - 1;
						}
						lastDirection = LeftDir;
					}
				}
				else
				{        //should go up or down.
					if (lastDirection == RightDir || lastDirection == LeftDir)
					{        //change the pixel direction now
						if (y > lastY)
						{        //should go down
							shouldGoDirection = DownDir;
						}
						else
						{        //should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == DownDir || shouldGoDirection == DownDir)
					{        //go down
						if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //down-right
							x = x + 1;
							y = y + 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-left
							x = x - 1;
							y = y + 1;
						}
						else
						{        //straight-down
							y = y + 1;
						}
						lastDirection = DownDir;
					}
					else if (lastDirection == UpDir || shouldGoDirection == UpDir)
					{        //go up
						if (x == 0 || x == imageWidth - 1 || y == 0)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //up-left
							x = x - 1;
							y = y - 1;
						}
						else
						{        //straight-up
							y = y - 1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y * imageWidth + x;
			}        //end while go right
			//then go left, pixel direction may be different during linking.
			x = pAnchorX_[i];
			y = pAnchorY_[i];
			indexInArray = y * imageWidth + x;
			pEdgeImg[indexInArray] = 0;     //mark the anchor point be a non-edge pixel and
			lastDirection = LeftDir;
			pSecondPartEdgeS_[offsetPS] = offsetPSecond;
			while (pgImg[indexInArray] > 0 && !pEdgeImg[indexInArray])
			{
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pSecondPartEdgeX_[offsetPSecond] = x;
				pSecondPartEdgeY_[offsetPSecond++] = y;
				shouldGoDirection = 0;        //unknown
				if (pdirImg[indexInArray] == Horizontal)
				{        //should go left or right
					if (lastDirection == UpDir || lastDirection == DownDir)
					{        //change the pixel direction now
						if (x > lastX)
						{        //should go right
							shouldGoDirection = RightDir;
						}
						else
						{        //should go left
							shouldGoDirection = LeftDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == RightDir || shouldGoDirection == RightDir)
					{        //go right
						if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-right
							x = x + 1;
							y = y + 1;
						}
						else
						{        //straight-right
							x = x + 1;
						}
						lastDirection = RightDir;
					}
					else if (lastDirection == LeftDir || shouldGoDirection == LeftDir)
					{        //go left
						if (x == 0 || y == 0 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						gValue2 = (unsigned char)pgImg[indexInArray - 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-left
							x = x - 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-left
							x = x - 1;
							y = y + 1;
						}
						else
						{        //straight-left
							x = x - 1;
						}
						lastDirection = LeftDir;
					}
				}
				else
				{        //should go up or down.
					if (lastDirection == RightDir || lastDirection == LeftDir)
					{        //change the pixel direction now
						if (y > lastY)
						{        //should go down
							shouldGoDirection = DownDir;
						}
						else
						{        //should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == DownDir || shouldGoDirection == DownDir)
					{        //go down
						if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //down-right
							x = x + 1;
							y = y + 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-left
							x = x - 1;
							y = y + 1;
						}
						else
						{        //straight-down
							y = y + 1;
						}
						lastDirection = DownDir;
					}
					else if (lastDirection == UpDir || shouldGoDirection == UpDir)
					{        //go up
						if (x == 0 || x == imageWidth - 1 || y == 0)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //up-left
							x = x - 1;
							y = y - 1;
						}
						else
						{        //straight-up
							y = y - 1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y * imageWidth + x;
			}        //end while go left
			//end anchor is Horizontal
		}
		else
		{     //the direction of this pixel is vertical, go up and down
			//fist go down, pixel direction may be different during linking.
			lastDirection = DownDir;
			while (pgImg[indexInArray] > 0 && !pEdgeImg[indexInArray])
			{
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pFirstPartEdgeX_[offsetPFirst] = x;
				pFirstPartEdgeY_[offsetPFirst++] = y;
				shouldGoDirection = 0;        //unknown
				if (pdirImg[indexInArray] == Horizontal)
				{        //should go left or right
					if (lastDirection == UpDir || lastDirection == DownDir)
					{        //change the pixel direction now
						if (x > lastX)
						{        //should go right
							shouldGoDirection = RightDir;
						}
						else
						{        //should go left
							shouldGoDirection = LeftDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == RightDir || shouldGoDirection == RightDir)
					{        //go right
						if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-right
							x = x + 1;
							y = y + 1;
						}
						else
						{        //straight-right
							x = x + 1;
						}
						lastDirection = RightDir;
					}
					else if (lastDirection == LeftDir || shouldGoDirection == LeftDir)
					{        //go left
						if (x == 0 || y == 0 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						gValue2 = (unsigned char)pgImg[indexInArray - 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-left
							x = x - 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-left
							x = x - 1;
							y = y + 1;
						}
						else
						{        //straight-left
							x = x - 1;
						}
						lastDirection = LeftDir;
					}
				}
				else
				{        //should go up or down.
					if (lastDirection == RightDir || lastDirection == LeftDir)
					{        //change the pixel direction now
						if (y > lastY)
						{        //should go down
							shouldGoDirection = DownDir;
						}
						else
						{        //should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == DownDir || shouldGoDirection == DownDir)
					{        //go down
						if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //down-right
							x = x + 1;
							y = y + 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-left
							x = x - 1;
							y = y + 1;
						}
						else
						{        //straight-down
							y = y + 1;
						}
						lastDirection = DownDir;
					}
					else if (lastDirection == UpDir || shouldGoDirection == UpDir)
					{        //go up
						if (x == 0 || x == imageWidth - 1 || y == 0)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //up-left
							x = x - 1;
							y = y - 1;
						}
						else
						{        //straight-up
							y = y - 1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y * imageWidth + x;
			}        //end while go down
			//then go up, pixel direction may be different during linking.
			lastDirection = UpDir;
			x = pAnchorX_[i];
			y = pAnchorY_[i];
			indexInArray = y * imageWidth + x;
			pEdgeImg[indexInArray] = 0;     //mark the anchor point be a non-edge pixel and
			pSecondPartEdgeS_[offsetPS] = offsetPSecond;
			while (pgImg[indexInArray] > 0 && !pEdgeImg[indexInArray])
			{
				pEdgeImg[indexInArray] = 1;        // Mark this pixel as an edge pixel
				pSecondPartEdgeX_[offsetPSecond] = x;
				pSecondPartEdgeY_[offsetPSecond++] = y;
				shouldGoDirection = 0;        //unknown
				if (pdirImg[indexInArray] == Horizontal)
				{        //should go left or right
					if (lastDirection == UpDir || lastDirection == DownDir)
					{        //change the pixel direction now
						if (x > lastX)
						{        //should go right
							shouldGoDirection = RightDir;
						}
						else
						{        //should go left
							shouldGoDirection = LeftDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == RightDir || shouldGoDirection == RightDir)
					{        //go right
						if (x == imageWidth - 1 || y == 0 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the right and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-right
							x = x + 1;
							y = y + 1;
						}
						else
						{        //straight-right
							x = x + 1;
						}
						lastDirection = RightDir;
					}
					else if (lastDirection == LeftDir || shouldGoDirection == LeftDir)
					{        //go left
						if (x == 0 || y == 0 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the left and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						gValue2 = (unsigned char)pgImg[indexInArray - 1];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-left
							x = x - 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-left
							x = x - 1;
							y = y + 1;
						}
						else
						{        //straight-left
							x = x - 1;
						}
						lastDirection = LeftDir;
					}
				}
				else
				{        //should go up or down.
					if (lastDirection == RightDir || lastDirection == LeftDir)
					{        //change the pixel direction now
						if (y > lastY)
						{        //should go down
							shouldGoDirection = DownDir;
						}
						else
						{        //should go up
							shouldGoDirection = UpDir;
						}
					}
					lastX = x;
					lastY = y;
					if (lastDirection == DownDir || shouldGoDirection == DownDir)
					{        //go down
						if (x == 0 || x == imageWidth - 1 || y == imageHeight - 1)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the down and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray + imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray + imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray + imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //down-right
							x = x + 1;
							y = y + 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //down-left
							x = x - 1;
							y = y + 1;
						}
						else
						{        //straight-down
							y = y + 1;
						}
						lastDirection = DownDir;
					}
					else if (lastDirection == UpDir || shouldGoDirection == UpDir)
					{        //go up
						if (x == 0 || x == imageWidth - 1 || y == 0)
						{        //reach the image border
							break;
						}
						// Look at 3 neighbors to the up and pick the one with the max. gradient value
						gValue1 = (unsigned char)pgImg[indexInArray - imageWidth + 1];
						gValue2 = (unsigned char)pgImg[indexInArray - imageWidth];
						gValue3 = (unsigned char)pgImg[indexInArray - imageWidth - 1];
						if (gValue1 >= gValue2 && gValue1 >= gValue3)
						{        //up-right
							x = x + 1;
							y = y - 1;
						}
						else if (gValue3 >= gValue2 && gValue3 >= gValue1)
						{        //up-left
							x = x - 1;
							y = y - 1;
						}
						else
						{        //straight-up
							y = y - 1;
						}
						lastDirection = UpDir;
					}
				}
				indexInArray = y * imageWidth + x;
			}        //end while go up
		}        //end anchor is Vertical
		//only keep the edge chains whose length is larger than the minLineLen_;
		edgeLenFirst = offsetPFirst - pFirstPartEdgeS_[offsetPS];
		edgeLenSecond = offsetPSecond - pSecondPartEdgeS_[offsetPS];
		if (edgeLenFirst + edgeLenSecond < minLineLen_ + 1)
		{   //short edge, drop it
			offsetPFirst = pFirstPartEdgeS_[offsetPS];
			offsetPSecond = pSecondPartEdgeS_[offsetPS];
		}
		else
		{
			offsetPS++;
		}
	}
	//store the last index
	pFirstPartEdgeS_[offsetPS] = offsetPFirst;
	pSecondPartEdgeS_[offsetPS] = offsetPSecond;
	if (offsetPS > maxNumOfEdge)
	{
		std::cout << "Edge drawing Error: The total number of edges is larger than MaxNumOfEdge, "
			"numofedge = " << offsetPS << ", MaxNumOfEdge=" << maxNumOfEdge << std::endl;
		return -1;
	}
	if (offsetPFirst > edgePixelArraySize || offsetPSecond > edgePixelArraySize)
	{
		std::cout << "Edge drawing Error: The total number of edge pixels is larger than MaxNumOfEdgePixels, "
			"numofedgePixel1 = " << offsetPFirst << ",  numofedgePixel2 = " << offsetPSecond << ", MaxNumOfEdgePixel=" << edgePixelArraySize << std::endl;
		return -1;
	}

	/*now all the edge information are stored in pFirstPartEdgeX_, pFirstPartEdgeY_,
	*pFirstPartEdgeS_,  pSecondPartEdgeX_, pSecondPartEdgeY_, pSecondPartEdgeS_;
	*we should reorganize them into edgeChains for easily using. */
	int tempID;
	edgeChains.xCors.resize(offsetPFirst + offsetPSecond);
	edgeChains.yCors.resize(offsetPFirst + offsetPSecond);
	edgeChains.sId.resize(offsetPS + 1);
	unsigned int *pxCors = &edgeChains.xCors.front();
	unsigned int *pyCors = &edgeChains.yCors.front();
	unsigned int *psId = &edgeChains.sId.front();
	offsetPFirst = 0;
	offsetPSecond = 0;
	unsigned int indexInCors = 0;
	unsigned int numOfEdges = 0;
	for (unsigned int edgeId = 0; edgeId < offsetPS; edgeId++)
	{
		//step1, put the first and second parts edge coordinates together from edge start to edge end
		psId[numOfEdges++] = indexInCors;
		indexInArray = pFirstPartEdgeS_[edgeId];
		offsetPFirst = pFirstPartEdgeS_[edgeId + 1];
		for (tempID = offsetPFirst - 1; tempID >= indexInArray; tempID--)
		{   //add first part edge
			pxCors[indexInCors] = pFirstPartEdgeX_[tempID];
			pyCors[indexInCors++] = pFirstPartEdgeY_[tempID];
		}
		indexInArray = pSecondPartEdgeS_[edgeId];
		offsetPSecond = pSecondPartEdgeS_[edgeId + 1];
		for (tempID = indexInArray + 1; tempID < (int)offsetPSecond; tempID++)
		{   //add second part edge
			pxCors[indexInCors] = pSecondPartEdgeX_[tempID];
			pyCors[indexInCors++] = pSecondPartEdgeY_[tempID];
		}
	}
	psId[numOfEdges] = indexInCors;   //the end index of the last edge
	edgeChains.numOfEdges = numOfEdges;

	return 1;
}

int LineDescriptor::EDLineDetector::EDline(cv::Mat &image, LineChains &lines)
{

	//first, call EdgeDrawing function to extract edges
	EdgeChains edges;
	if ((EdgeDrawing(image, edges)) != 1)
	{
		std::cout << "Line Detection not finished" << std::endl;
		return -1;
	}

	//detect lines
	unsigned int linePixelID = edges.sId[edges.numOfEdges];
	lines.xCors.resize(linePixelID);
	lines.yCors.resize(linePixelID);
	lines.sId.resize(5 * edges.numOfEdges);
	unsigned int *pEdgeXCors = &edges.xCors.front();
	unsigned int *pEdgeYCors = &edges.yCors.front();
	unsigned int *pEdgeSID = &edges.sId.front();
	unsigned int *pLineXCors = &lines.xCors.front();
	unsigned int *pLineYCors = &lines.yCors.front();
	unsigned int *pLineSID = &lines.sId.front();
	logNT_ = 2.0 * (log10((double)imageWidth) + log10((double)imageHeight));
	double lineFitErr = 0;    //the line fit error;
	std::vector<double> lineEquation(2, 0);
	lineEquations_.clear();
	lineEndpoints_.clear();
	lineDirection_.clear();
	unsigned char *pdirImg = dirImg_.data;
	unsigned int numOfLines = 0;
	unsigned int newOffsetS = 0;
	unsigned int offsetInEdgeArrayS, offsetInEdgeArrayE;    //start index and end index
	unsigned int offsetInLineArray = 0;
	float direction;    //line direction

	for (unsigned int edgeID = 0; edgeID < edges.numOfEdges; edgeID++)
	{
		offsetInEdgeArrayS = pEdgeSID[edgeID];
		offsetInEdgeArrayE = pEdgeSID[edgeID + 1];
		while (offsetInEdgeArrayE > offsetInEdgeArrayS + minLineLen_)
		{   //extract line segments from an edge, may find more than one segments
			//find an initial line segment
			while (offsetInEdgeArrayE > offsetInEdgeArrayS + minLineLen_)
			{
				lineFitErr = LeastSquaresLineFit_(pEdgeXCors, pEdgeYCors, offsetInEdgeArrayS, lineEquation);
				if (lineFitErr <= lineFitErrThreshold_)
					break;      //ok, an initial line segment detected
				offsetInEdgeArrayS += SkipEdgePoint;  //skip the first two pixel in the chain and try with the remaining pixels
			}
			if (lineFitErr > lineFitErrThreshold_)
				break;  //no line is detected
			//An initial line segment is detected. Try to extend this line segment
			pLineSID[numOfLines] = offsetInLineArray;
			double coef1 = 0;     //for a line ax+by+c=0, coef1 = 1/sqrt(a^2+b^2);
			double pointToLineDis;      //for a line ax+by+c=0 and a point(xi, yi), pointToLineDis = coef1*|a*xi+b*yi+c|
			bool bExtended = true;
			bool bFirstTry = true;
			int numOfOutlier;     //to against noise, we accept a few outlier of a line.
			int tryTimes = 0;
			if (pdirImg[pEdgeYCors[offsetInEdgeArrayS] * imageWidth + pEdgeXCors[offsetInEdgeArrayS]] == Horizontal)
			{     //y=ax+b, i.e. ax-y+b=0
				while (bExtended)
				{
					tryTimes++;
					if (bFirstTry)
					{
						bFirstTry = false;
						for (int i = 0; i < minLineLen_; i++)
						{     //First add the initial line segment to the line array
							pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
							pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						}
					}
					else
					{     //after each try, line is extended, line equation should be re-estimated
						//adjust the line equation
						lineFitErr = LeastSquaresLineFit_(pLineXCors, pLineYCors, pLineSID[numOfLines], newOffsetS, offsetInLineArray, lineEquation);
					}
					coef1 = 1 / sqrt(lineEquation[0] * lineEquation[0] + 1);
					numOfOutlier = 0;
					newOffsetS = offsetInLineArray;
					while (offsetInEdgeArrayE > offsetInEdgeArrayS)
					{
						pointToLineDis = fabs(lineEquation[0] * pEdgeXCors[offsetInEdgeArrayS] - pEdgeYCors[offsetInEdgeArrayS] + lineEquation[1]) * coef1;
						pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
						pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						if (pointToLineDis > lineFitErrThreshold_)
						{
							numOfOutlier++;
							if (numOfOutlier > 3)
								break;
						}
						else
						{           //we count number of connective outliers.
							numOfOutlier = 0;
						}
					}
					//pop back the last few outliers from lines and return them to edge chain
					offsetInLineArray -= numOfOutlier;
					offsetInEdgeArrayS -= numOfOutlier;
					if (offsetInLineArray - newOffsetS > 0 && tryTimes < TryTime)
					{           //some new pixels are added to the line
					}
					else
					{
						bExtended = false;            //no new pixels are added.
					}
				}
				//the line equation coefficients,for line w1x+w2y+w3 =0, we normalize it to make w1^2+w2^2 = 1.
				std::vector<double> lineEqu(3, 0);
				lineEqu[0] = lineEquation[0] * coef1;
				lineEqu[1] = -1 * coef1;
				lineEqu[2] = lineEquation[1] * coef1;
				if (LineValidation_(pLineXCors, pLineYCors, pLineSID[numOfLines], offsetInLineArray, lineEqu, direction))
				{           //check the line
					//store the line equation coefficients
					lineEquations_.push_back(lineEqu);
					/*At last, compute the line endpoints and store them.
					*we project the first and last pixels in the pixelChain onto the best fit line
					*to get the line endpoints.
					*xp= (w2^2*x0-w1*w2*y0-w3*w1)/(w1^2+w2^2)
					*yp= (w1^2*y0-w1*w2*x0-w3*w2)/(w1^2+w2^2)  */
					std::vector<float> lineEndP(4, 0);          //line endpoints
					double a1 = lineEqu[1] * lineEqu[1];
					double a2 = lineEqu[0] * lineEqu[0];
					double a3 = lineEqu[0] * lineEqu[1];
					double a4 = lineEqu[2] * lineEqu[0];
					double a5 = lineEqu[2] * lineEqu[1];
					unsigned int Px = pLineXCors[pLineSID[numOfLines]];         //first pixel
					unsigned int Py = pLineYCors[pLineSID[numOfLines]];
					lineEndP[0] = (float)(a1 * Px - a3 * Py - a4);         //x
					lineEndP[1] = (float)(a2 * Py - a3 * Px - a5);         //y
					Px = pLineXCors[offsetInLineArray - 1];         //last pixel
					Py = pLineYCors[offsetInLineArray - 1];
					lineEndP[2] = (float)(a1 * Px - a3 * Py - a4);         //x
					lineEndP[3] = (float)(a2 * Py - a3 * Px - a5);         //y
					lineEndpoints_.push_back(lineEndP);
					lineDirection_.push_back(direction);
					numOfLines++;
				}
				else
				{
					offsetInLineArray = pLineSID[numOfLines];         // line was not accepted, the offset is set back
				}
			}
			else
			{         //x=ay+b, i.e. x-ay-b=0
				while (bExtended)
				{
					tryTimes++;
					if (bFirstTry)
					{
						bFirstTry = false;
						for (int i = 0; i < minLineLen_; i++)
						{         //First add the initial line segment to the line array
							pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
							pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						}
					}
					else
					{         //after each try, line is extended, line equation should be re-estimated
						//adjust the line equation
						lineFitErr = LeastSquaresLineFit_(pLineXCors, pLineYCors, pLineSID[numOfLines], newOffsetS, offsetInLineArray, lineEquation);
					}
					coef1 = 1 / sqrt(1 + lineEquation[0] * lineEquation[0]);
					numOfOutlier = 0;
					newOffsetS = offsetInLineArray;
					while (offsetInEdgeArrayE > offsetInEdgeArrayS)
					{
						pointToLineDis = fabs(pEdgeXCors[offsetInEdgeArrayS] - lineEquation[0] * pEdgeYCors[offsetInEdgeArrayS] - lineEquation[1]) * coef1;
						pLineXCors[offsetInLineArray] = pEdgeXCors[offsetInEdgeArrayS];
						pLineYCors[offsetInLineArray++] = pEdgeYCors[offsetInEdgeArrayS++];
						if (pointToLineDis > lineFitErrThreshold_)
						{
							numOfOutlier++;
							if (numOfOutlier > 3)
								break;
						}
						else
						{           //we count number of connective outliers.
							numOfOutlier = 0;
						}
					}
					//pop back the last few outliers from lines and return them to edge chain
					offsetInLineArray -= numOfOutlier;
					offsetInEdgeArrayS -= numOfOutlier;
					if (offsetInLineArray - newOffsetS > 0 && tryTimes < TryTime)
					{           //some new pixels are added to the line
					}
					else
					{
						bExtended = false;            //no new pixels are added.
					}
				}
				//the line equation coefficients,for line w1x+w2y+w3 =0, we normalize it to make w1^2+w2^2 = 1.
				std::vector<double> lineEqu(3, 0);
				lineEqu[0] = 1 * coef1;
				lineEqu[1] = -lineEquation[0] * coef1;
				lineEqu[2] = -lineEquation[1] * coef1;

				if (LineValidation_(pLineXCors, pLineYCors, pLineSID[numOfLines], offsetInLineArray, lineEqu, direction))
				{           //check the line
					//store the line equation coefficients
					lineEquations_.push_back(lineEqu);
					/*At last, compute the line endpoints and store them.
					*we project the first and last pixels in the pixelChain onto the best fit line
					*to get the line endpoints.
					*xp= (w2^2*x0-w1*w2*y0-w3*w1)/(w1^2+w2^2)
					*yp= (w1^2*y0-w1*w2*x0-w3*w2)/(w1^2+w2^2)  */
					std::vector<float> lineEndP(4, 0);          //line endpoints
					double a1 = lineEqu[1] * lineEqu[1];
					double a2 = lineEqu[0] * lineEqu[0];
					double a3 = lineEqu[0] * lineEqu[1];
					double a4 = lineEqu[2] * lineEqu[0];
					double a5 = lineEqu[2] * lineEqu[1];
					unsigned int Px = pLineXCors[pLineSID[numOfLines]];         //first pixel
					unsigned int Py = pLineYCors[pLineSID[numOfLines]];
					lineEndP[0] = (float)(a1 * Px - a3 * Py - a4);         //x
					lineEndP[1] = (float)(a2 * Py - a3 * Px - a5);         //y
					Px = pLineXCors[offsetInLineArray - 1];         //last pixel
					Py = pLineYCors[offsetInLineArray - 1];
					lineEndP[2] = (float)(a1 * Px - a3 * Py - a4);         //x
					lineEndP[3] = (float)(a2 * Py - a3 * Px - a5);         //y
					lineEndpoints_.push_back(lineEndP);
					lineDirection_.push_back(direction);
					numOfLines++;
				}
				else
				{
					offsetInLineArray = pLineSID[numOfLines];         // line was not accepted, the offset is set back
				}
			}
			//Extract line segments from the remaining pixel; Current chain has been shortened already.
		}
	}         //end for(unsigned int edgeID=0; edgeID<edges.numOfEdges; edgeID++)

	pLineSID[numOfLines] = offsetInLineArray;
	lines.numOfLines = numOfLines;

	return 1;
}

double LineDescriptor::EDLineDetector::LeastSquaresLineFit_(unsigned int *xCors, unsigned int *yCors, unsigned int offsetS,
	std::vector<double> &lineEquation)
{

	float * pMatT;
	float * pATA;
	double fitError = 0;
	double coef;
	unsigned char *pdirImg = dirImg_.data;
	unsigned int offset = offsetS;
	/*If the first pixel in this chain is horizontal,
	*then we try to find a horizontal line, y=ax+b;*/
	if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Horizontal)
	{
		/*Build the system,and solve it using least square regression: mat * [a,b]^T = vec
		* [x0,1]         [y0]
		* [x1,1] [a]     [y1]
		*    .   [b]  =   .
		* [xn,1]         [yn]*/
		pMatT = fitMatT.ptr<float>();         //fitMatT = [x0, x1, ... xn; 1,1,...,1];
		for (int i = 0; i < minLineLen_; i++)
		{
			//*(pMatT+minLineLen_) = 1; //the value are not changed;
			*(pMatT++) = (float)xCors[offsetS];
			fitVec[0][i] = (float)yCors[offsetS++];
		}
		ATA = fitMatT * fitMatT.t();
		ATV = fitMatT * fitVec.t();
		/* [a,b]^T = Inv(mat^T * mat) * mat^T * vec */
		pATA = ATA.ptr<float>();
		coef = 1.0 / (double(pATA[0]) * double(pATA[3]) - double(pATA[1]) * double(pATA[2]));
		//    lineEquation = svd.Invert(ATA) * matT * vec;
		lineEquation[0] = coef * (double(pATA[3]) * double(ATV[0][0]) - double(pATA[1]) * double(ATV[0][1]));
		lineEquation[1] = coef * (double(pATA[0]) * double(ATV[0][1]) - double(pATA[2]) * double(ATV[0][0]));
		/*compute line fit error */
		for (int i = 0; i < minLineLen_; i++)
		{
			//coef = double(yCors[offset]) - double(xCors[offset++]) * lineEquation[0] - lineEquation[1];
			coef = double(yCors[offset]) - double(xCors[offset]) * lineEquation[0] - lineEquation[1];
			offset++;
			fitError += coef * coef;
		}
		return sqrt(fitError);
	}
	/*If the first pixel in this chain is vertical,
	*then we try to find a vertical line, x=ay+b;*/
	if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Vertical)
	{
		/*Build the system,and solve it using least square regression: mat * [a,b]^T = vec
		* [y0,1]         [x0]
		* [y1,1] [a]     [x1]
		*    .   [b]  =   .
		* [yn,1]         [xn]*/
		pMatT = fitMatT.ptr<float>();         //fitMatT = [y0, y1, ... yn; 1,1,...,1];
		for (int i = 0; i < minLineLen_; i++)
		{
			//*(pMatT+minLineLen_) = 1;//the value are not changed;
			*(pMatT++) = (float)yCors[offsetS];
			fitVec[0][i] = (float)xCors[offsetS++];
		}
		ATA = fitMatT * (fitMatT.t());
		ATV = fitMatT * fitVec.t();
		/* [a,b]^T = Inv(mat^T * mat) * mat^T * vec */
		pATA = ATA.ptr<float>();
		coef = 1.0 / (double(pATA[0]) * double(pATA[3]) - double(pATA[1]) * double(pATA[2]));
		//    lineEquation = svd.Invert(ATA) * matT * vec;
		lineEquation[0] = coef * (double(pATA[3]) * double(ATV[0][0]) - double(pATA[1]) * double(ATV[0][1]));
		lineEquation[1] = coef * (double(pATA[0]) * double(ATV[0][1]) - double(pATA[2]) * double(ATV[0][0]));
		/*compute line fit error */
		for (int i = 0; i < minLineLen_; i++)
		{
			//coef = double(xCors[offset]) - double(yCors[offset++]) * lineEquation[0] - lineEquation[1];
			coef = double(xCors[offset]) - double(yCors[offset]) * lineEquation[0] - lineEquation[1];
			offset++;
			fitError += coef * coef;
		}
		return sqrt(fitError);
	}
	return 0;
}
double LineDescriptor::EDLineDetector::LeastSquaresLineFit_(unsigned int *xCors, unsigned int *yCors, unsigned int offsetS,
	unsigned int newOffsetS, unsigned int offsetE, std::vector<double> &lineEquation)
{
	int length = offsetE - offsetS;
	int newLength = offsetE - newOffsetS;
	if (length <= 0 || newLength <= 0)
	{
		std::cout << "EDLineDetector::LeastSquaresLineFit_ Error:"
			" the expected line index is wrong...offsetE = " << offsetE << ", offsetS=" << offsetS << ", newOffsetS=" << newOffsetS << std::endl;
		return -1;
	}
	if (lineEquation.size() != 2)
	{
		std::cout << "SHOULD NOT BE != 2" << std::endl;
	}
	cv::Mat_<float> matT(2, newLength);
	cv::Mat_<float> vec(newLength, 1);
	float * pMatT;
	float * pATA;
	double coef;
	unsigned char *pdirImg = dirImg_.data;
	/*If the first pixel in this chain is horizontal,
	*then we try to find a horizontal line, y=ax+b;*/
	if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Horizontal)
	{
		/*Build the new system,and solve it using least square regression: mat * [a,b]^T = vec
		* [x0',1]         [y0']
		* [x1',1] [a]     [y1']
		*    .    [b]  =   .
		* [xn',1]         [yn']*/
		pMatT = matT.ptr<float>();          //matT = [x0', x1', ... xn'; 1,1,...,1]
		for (int i = 0; i < newLength; i++)
		{
			*(pMatT + newLength) = 1;
			*(pMatT++) = (float)xCors[newOffsetS];
			vec[0][i] = (float)yCors[newOffsetS++];
		}
		/* [a,b]^T = Inv(ATA + mat^T * mat) * (ATV + mat^T * vec) */
		tempMatLineFit = matT * matT.t();
		tempVecLineFit = matT * vec;
		ATA = ATA + tempMatLineFit;
		ATV = ATV + tempVecLineFit;
		pATA = ATA.ptr<float>();
		coef = 1.0 / (double(pATA[0]) * double(pATA[3]) - double(pATA[1]) * double(pATA[2]));
		lineEquation[0] = coef * (double(pATA[3]) * double(ATV[0][0]) - double(pATA[1]) * double(ATV[0][1]));
		lineEquation[1] = coef * (double(pATA[0]) * double(ATV[0][1]) - double(pATA[2]) * double(ATV[0][0]));

		return 0;
	}
	/*If the first pixel in this chain is vertical,
	*then we try to find a vertical line, x=ay+b;*/
	if (pdirImg[yCors[offsetS] * imageWidth + xCors[offsetS]] == Vertical)
	{
		/*Build the system,and solve it using least square regression: mat * [a,b]^T = vec
		* [y0',1]         [x0']
		* [y1',1] [a]     [x1']
		*    .    [b]  =   .
		* [yn',1]         [xn']*/
		pMatT = matT.ptr<float>();          //matT = [y0', y1', ... yn'; 1,1,...,1]
		for (int i = 0; i < newLength; i++)
		{
			*(pMatT + newLength) = 1;
			*(pMatT++) = (float)yCors[newOffsetS];
			vec[0][i] = (float)xCors[newOffsetS++];
		}
		/* [a,b]^T = Inv(ATA + mat^T * mat) * (ATV + mat^T * vec) */
		//    matT.MultiplyWithTransposeOf(matT, tempMatLineFit);
		tempMatLineFit = matT * matT.t();
		tempVecLineFit = matT * vec;
		ATA = ATA + tempMatLineFit;
		ATV = ATV + tempVecLineFit;
		//    pATA = ATA.GetData();
		pATA = ATA.ptr<float>();
		coef = 1.0 / (double(pATA[0]) * double(pATA[3]) - double(pATA[1]) * double(pATA[2]));
		lineEquation[0] = coef * (double(pATA[3]) * double(ATV[0][0]) - double(pATA[1]) * double(ATV[0][1]));
		lineEquation[1] = coef * (double(pATA[0]) * double(ATV[0][1]) - double(pATA[2]) * double(ATV[0][0]));

	}
	return 0;
}

bool LineDescriptor::EDLineDetector::LineValidation_(unsigned int *xCors, unsigned int *yCors, unsigned int offsetS, unsigned int offsetE,
	std::vector<double> &lineEquation, float &direction)
{
	if (bValidate_)
	{
		int n = offsetE - offsetS;
		/*first compute the direction of line, make sure that the dark side always be the
		*left side of a line.*/
		int meanGradientX = 0, meanGradientY = 0;
		short *pdxImg = dxImg_.ptr<short>();
		short *pdyImg = dyImg_.ptr<short>();
		double dx, dy;
		std::vector<double> pointDirection;
		int index;
		for (int i = 0; i < n; i++)
		{
			index = yCors[offsetS] * imageWidth + xCors[offsetS];
			offsetS++;
			meanGradientX += pdxImg[index];
			meanGradientY += pdyImg[index];
			dx = (double)pdxImg[index];
			dy = (double)pdyImg[index];
			pointDirection.push_back(atan2(-dx, dy));
		}
		dx = fabs(lineEquation[1]);
		dy = fabs(lineEquation[0]);
		if (meanGradientX == 0 && meanGradientY == 0)
		{         //not possible, if happens, it must be a wrong line,
			return false;
		}
		if (meanGradientX > 0 && meanGradientY >= 0)
		{         //first quadrant, and positive direction of X axis.
			direction = (float)atan2(-dy, dx);         //line direction is in fourth quadrant
		}
		if (meanGradientX <= 0 && meanGradientY > 0)
		{         //second quadrant, and positive direction of Y axis.
			direction = (float)atan2(dy, dx);          //line direction is in first quadrant
		}
		if (meanGradientX < 0 && meanGradientY <= 0)
		{         //third quadrant, and negative direction of X axis.
			direction = (float)atan2(dy, -dx);         //line direction is in second quadrant
		}
		if (meanGradientX >= 0 && meanGradientY < 0)
		{         //fourth quadrant, and negative direction of Y axis.
			direction = (float)atan2(-dy, -dx);          //line direction is in third quadrant
		}
		/*then check whether the line is on the border of the image. We don't keep the border line.*/
		if (fabs(direction) < 0.15 || M_PI - fabs(direction) < 0.15)
		{         //Horizontal line
			if (fabs(lineEquation[2]) < 10 || fabs(imageHeight - fabs(lineEquation[2])) < 10)
			{         //upper border or lower border
				return false;
			}
		}
		if (fabs(fabs(direction) - M_PI * 0.5) < 0.15)
		{         //Vertical line
			if (fabs(lineEquation[2]) < 10 || fabs(imageWidth - fabs(lineEquation[2])) < 10)
			{         //left border or right border
				return false;
			}
		}
		//count the aligned points on the line which have the same direction as the line.
		double disDirection;
		int k = 0;
		for (int i = 0; i < n; i++)
		{
			disDirection = fabs(direction - pointDirection[i]);
			if (fabs(2 * M_PI - disDirection) < 0.392699 || disDirection < 0.392699)
			{         //same direction, pi/8 = 0.392699081698724
				k++;
			}
		}
		//now compute NFA(Number of False Alarms)
		double ret = nfa(n, k, 0.125, logNT_);

		return (ret > 0);  //0 corresponds to 1 mean false alarm
	}
	else
	{
		return true;
	}
}

int LineDescriptor::EDLineDetector::EDline(cv::Mat &image)
{
	if ((EDline(image, lines_/*, smoothed*/)) != 1)
	{
		return -1;
	}
	lineSalience_.clear();
	lineSalience_.resize(lines_.numOfLines);
	unsigned char *pgImg = gImgWO_.ptr();
	unsigned int indexInLineArray;
	unsigned int *pXCor = &lines_.xCors.front();
	unsigned int *pYCor = &lines_.yCors.front();
	unsigned int *pSID = &lines_.sId.front();
	for (unsigned int i = 0; i < lineSalience_.size(); i++)
	{
		int salience = 0;
		for (indexInLineArray = pSID[i]; indexInLineArray < pSID[i + 1]; indexInLineArray++)
		{
			salience += pgImg[pYCor[indexInLineArray] * imageWidth + pXCor[indexInLineArray]];
		}
		lineSalience_[i] = (float)salience;
	}
	return 1;
}

LineMatcher::LineMatcher() :ratio_(2.f)
{}

// Set the NN ratio
void LineMatcher::setRatio(float r) {
	ratio_ = r;
}

int LineMatcher::hammingMatch(const Mat& queryDescriptors, const Mat& trainDescriptors)
{
	int minDistance = 1000;
	int bestID = 0;
	for (int i = 0; i < trainDescriptors.rows; i++)
	{
		int distance = norm(queryDescriptors, trainDescriptors.rowRange(i, i + 1), NORM_HAMMING);
		if (distance < minDistance)
		{
			minDistance = distance;
			bestID = i;
		}
	}

	return bestID;
}

// Clear matches for which NN ratio is > than threshold
// return the number of removed points 
// (corresponding entries being cleared, i.e. size will be 0)
int LineMatcher::ratioTest(std::vector<std::vector<cv::DMatch>>& matches) {

	int removed = 0;

	// for all matches
	for (vector<vector<DMatch>>::iterator matchIterator = matches.begin();
		matchIterator != matches.end(); ++matchIterator) {
		/*if (matchIterator->size() > 100){
		matchIterator->clear();
		break;
		}*/
		// if 2 NN has been identified
		if (matchIterator->size() > 1) {
			float a = (*matchIterator)[0].distance;
			float b = (*matchIterator)[1].distance;
			// check distance ratio
			if ((a / b) > ratio_) {

				matchIterator->clear(); // remove match
				removed++;
			}

		}
		else { // does not have 2 neighbours

			matchIterator->clear(); // remove match
			removed++;
		}
	}

	return removed;
}

// Insert symmetrical matches in symMatches vector
void LineMatcher::symmetryTest(const vector<vector<DMatch>>& matches1,
	const vector<vector<DMatch>>& matches2,
	vector<DMatch>& symMatches, bool singleFlag) {

	// for all matches image 1 -> image 2
	for (vector<vector<DMatch>>::const_iterator matchIterator1 = matches1.begin();
		matchIterator1 != matches1.end(); ++matchIterator1) {

		if (matchIterator1->size() < 2) // ignore deleted matches 
			continue;

		// for all matches image 2 -> image 1
		for (vector<vector<DMatch>>::const_iterator matchIterator2 = matches2.begin();
			matchIterator2 != matches2.end(); ++matchIterator2) {

			if (!singleFlag && matchIterator2->size() < 2) // ignore deleted matches 当singleflag为false时执行，否则不执行
				continue;

			// Match symmetry test
			if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx &&
				(*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx) {

				// add symmetrical match
				symMatches.push_back(DMatch((*matchIterator1)[0].queryIdx,
					(*matchIterator1)[0].trainIdx, (*matchIterator1)[0].imgIdx,
					(*matchIterator1)[0].distance));
				break; // next match in image 1 -> image 2
			}
		}
	}
}

/* draw extracted lines on original image */
void LineMatcher::drawKeylines(const Mat& image, const std::vector<KeyLine>& keylines, Mat& outImage, const Scalar& color, int flags)
{
	if (flags == 1)
		outImage = image.clone();

	for (size_t i = 0; i < keylines.size(); i++)
	{
		/* decide lines' color  */
		Scalar lineColor;
		if (color == Scalar::all(-1))
		{
			int R = (rand() % (int)(255 + 1));
			int G = (rand() % (int)(255 + 1));
			int B = (rand() % (int)(255 + 1));

			lineColor = Scalar(R, G, B);
		}

		else
			lineColor = color;

		/* get line */
		KeyLine k = keylines[i];

		/* draw line */
		line(outImage, Point2f(k.startPointX, k.startPointY), Point2f(k.endPointX, k.endPointY), lineColor, 1);
	}
}

/* draw matches between two images */
void LineMatcher::drawLineMatches(const Mat& img1, const std::vector<KeyLine>& keylines1, const Mat& img2, const std::vector<KeyLine>& keylines2,
	const std::vector<DMatch>& matches1to2, Mat& outImg, const Scalar& matchColor, const Scalar& singleLineColor,
	const std::vector<char>& matchesMask, int flags)
{

	if (img1.type() != img2.type())
	{
		std::cout << "Input images have different types" << std::endl;
		CV_Assert(img1.type() == img2.type());
	}

	/* initialize output matrix (if necessary) */
	if (flags == 1)
	{
		/* check how many rows are necessary for output matrix */
		int totalRows = img1.rows >= img2.rows ? img1.rows : img2.rows;

		/* initialize output matrix */
		outImg = Mat::zeros(totalRows, img1.cols + img2.cols, img1.type());

	}

	/* initialize random seed: */
	//srand((unsigned int)time(NULL));

	Scalar singleLineColorRGB;
	if (singleLineColor == Scalar::all(-1))
	{
		int R = (rand() % (int)(255 + 1));
		int G = (rand() % (int)(255 + 1));
		int B = (rand() % (int)(255 + 1));

		singleLineColorRGB = Scalar(R, G, B);
	}

	else
		singleLineColorRGB = singleLineColor;

	/* copy input images to output images */
	Mat roi_left(outImg, Rect(0, 0, img1.cols, img1.rows));
	Mat roi_right(outImg, Rect(img1.cols, 0, img2.cols, img2.rows));
	img1.copyTo(roi_left);
	img2.copyTo(roi_right);

	/* get columns offset */
	int offset = img1.cols;

	/* if requested, draw lines from both images */
	if (flags != 0)
	{
		for (size_t i = 0; i < keylines1.size(); i++)
		{
			KeyLine k1 = keylines1[i];
			//line( outImg, Point2f( k1.startPointX, k1.startPointY ), Point2f( k1.endPointX, k1.endPointY ), singleLineColorRGB, 2 );
			line(outImg, Point2f(k1.sPointInOctaveX, k1.sPointInOctaveY), Point2f(k1.ePointInOctaveX, k1.ePointInOctaveY), singleLineColorRGB, 2);

			//添加序号
			ostringstream oss;//创建一个流
			oss << i;//把值传递如流中
			putText(outImg, oss.str(), Point((k1.sPointInOctaveX + k1.ePointInOctaveX) / 2, (k1.sPointInOctaveY + k1.ePointInOctaveY) / 2), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0, 0, 0));
		}

		for (size_t j = 0; j < keylines2.size(); j++)
		{
			KeyLine k2 = keylines2[j];
			line(outImg, Point2f(k2.sPointInOctaveX + offset, k2.sPointInOctaveY), Point2f(k2.ePointInOctaveX + offset, k2.ePointInOctaveY), singleLineColorRGB, 2);

			//添加序号
			ostringstream oss;//创建一个流
			oss << j;//把值传递如流中
			putText(outImg, oss.str(), Point((k2.sPointInOctaveX + k2.ePointInOctaveX) / 2 + offset, (k2.sPointInOctaveY + k2.ePointInOctaveY) / 2), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0, 0, 0));
		}
	}

	/* draw matches */
	for (size_t counter = 0; counter < /*matches1to2.size()*/1; counter++)
	{
		if (/*matchesMask[counter] != 0*/1)
		{
			DMatch dm = matches1to2[counter];
			KeyLine left = keylines1[dm.queryIdx];
			KeyLine right = keylines2[dm.trainIdx];

			Scalar matchColorRGB;
			if (matchColor == Scalar::all(-1))
			{
				int R = (rand() % (int)(255 + 1));
				int G = (rand() % (int)(255 + 1));
				int B = (rand() % (int)(255 + 1));

				matchColorRGB = Scalar(R, G, B);

				if (singleLineColor == Scalar::all(-1))
					singleLineColorRGB = matchColorRGB;
			}

			else
				matchColorRGB = matchColor;

			/* draw lines if necessary */
			//      line( outImg, Point2f( left.startPointX, left.startPointY ), Point2f( left.endPointX, left.endPointY ), singleLineColorRGB, 2 );
			//
			//      line( outImg, Point2f( right.startPointX + offset, right.startPointY ), Point2f( right.endPointX + offset, right.endPointY ), singleLineColorRGB,
			//            2 );
			//
			//      /* link correspondent lines */
			//      line( outImg, Point2f( left.startPointX, left.startPointY ), Point2f( right.startPointX + offset, right.startPointY ), matchColorRGB, 1 );

			line(outImg, Point2f(left.sPointInOctaveX, left.sPointInOctaveY), Point2f(left.ePointInOctaveX, left.ePointInOctaveY), singleLineColorRGB, 2);

			line(outImg, Point2f(right.sPointInOctaveX + offset, right.sPointInOctaveY), Point2f(right.ePointInOctaveX + offset, right.ePointInOctaveY), singleLineColorRGB,
				2);

			/* link correspondent lines */
			line(outImg, Point2f(left.ePointInOctaveX, left.ePointInOctaveY), Point2f(right.ePointInOctaveX + offset, right.ePointInOctaveY), matchColorRGB, 1);
		}
	}
}