// stdafx.h : ��׼ϵͳ�����ļ��İ����ļ���
// ���Ǿ���ʹ�õ��������ĵ�
// �ض�����Ŀ�İ����ļ�
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // ĳЩ CString ���캯��������ʽ��
#define _AFX_NO_MFC_CONTROLS_IN_DIALOGS         // �Ƴ��Ի����е� MFC �ؼ�֧��

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // �� Windows ͷ���ų�����ʹ�õ�����
#endif

//#include <afx.h>
#include <afxwin.h>         // MFC ��������ͱ�׼���
#include <afxext.h>         // MFC ��չ
#include "afxinet.h"
#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // MFC �� Internet Explorer 4 �����ؼ���֧��
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>                     // MFC �� Windows �����ؼ���֧��
#endif // _AFX_NO_AFXCMN_SUPPORT

//opencv headers
#include<opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv/cv.h>
//
////Third Party headers
//#include <boost/thread.hpp>
//#include<Eigen/Dense>
//#include <Eigen/Core>
//#include <Eigen/StdVector>
//#include <Eigen/Geometry>
#include<pangolin/pangolin.h>
//#include"g2o/types/sba/types_six_dof_expmap.h"
//#include"g2o/types/sim3/types_seven_dof_expmap.h"
//#include "g2o/core/base_vertex.h"
//#include "g2o/core/base_unary_edge.h"
//#include "g2o/core/base_binary_edge.h"
//#include "g2o/core/base_multi_edge.h"
//
//VS headers
#include <list>
#include <set>
#include <graphics.h>
#include <conio.h>
#include <stdlib.h> 
#include <math.h>
#include <sstream>
#include <iostream>
//#include <string>
//#include <vector>
//#include <fstream>
//#include <time.h>
//
//#include <windows.h>

#pragma comment(lib,"dxgi.lib")
#include <dxgi.h>

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
// TODO:  �ڴ˴����ó�����Ҫ������ͷ�ļ�
