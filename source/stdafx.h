// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // 某些 CString 构造函数将是显式的
#define _AFX_NO_MFC_CONTROLS_IN_DIALOGS         // 移除对话框中的 MFC 控件支持

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // 从 Windows 头中排除极少使用的资料
#endif

//#include <afx.h>
#include <afxwin.h>         // MFC 核心组件和标准组件
#include <afxext.h>         // MFC 扩展
#include "afxinet.h"
#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // MFC 对 Internet Explorer 4 公共控件的支持
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>                     // MFC 对 Windows 公共控件的支持
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
// TODO:  在此处引用程序需要的其他头文件
