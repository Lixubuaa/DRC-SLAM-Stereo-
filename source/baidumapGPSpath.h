#pragma once

#include "GPS_GlobalDataFuction.h"
#include <string>
#include <vector>
#include <fstream>
#include <time.h>

using namespace std;

bool isTypeRight(CString stringHtml);
bool isStatusRight(CString stringHtml);
vector<CString> getPathLL(CString stringHtml);
vector<CString> getLatLon(vector<CString> pathLonLat);
vector<CString> coordinateTran(vector<CString> pathLonLat_Ed);
vector<CString> getShort(vector<CString> pathLonLat_Ed);
vector<CString> getWGS84(vector<CString> GPSpath);
vector<Point2d> getDoubleVector(vector<CString> GPSLL);
CString getBaiduPath(CString originLatGPS, CString originLonGPS, CString destination);
vector<Point2d> getGPSroute(CString originLatGPS, CString originLonGPS, CString destination);
vector<Point2d> getStartEnd(CString originLatGPS, CString originLonGPS, CString destination);
vector<Point2d> getBaiduBreaks(CString originLatGPS, CString originLonGPS, CString destination);
Point2d getBaiduLatLon(Point3d originLatLon);

vector<Point2d> getBeihangFixStartEnd();
vector<Point2d> getSchoolFixStartEnd();

vector<Point2d> getStartEndline(vector<Point2d> GoalLatLon);
vector<Point2d> getCurrentGoalLatLon(cv::Point3d IniLatLonHeight, vector<Point2d> line, vector<Point2d> GoalLatLon_origin);