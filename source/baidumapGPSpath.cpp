#include "stdafx.h"
#include "baidumapGPSpath.h"
#include <sstream>

using namespace std;

int destiChose = 11;

int ChoseSchoolDestination(char buf[100])
{
	if (!strcmp(buf, "北门"))
		return 2;
	else if (!strcmp(buf, "西门"))
		return 5;
	else if (!strcmp(buf, "南门"))
		return 7;
	else if (!strcmp(buf, "全程"))
		return 11;
	else
	{
		cout << "Your destination isn't in school !!!" << endl;
		return 0;
	}
}

vector<CString> getPathLL(CString stringHtml) {
	int i = 0;
	int k = 0;
	int Offset = 7;//path":"共七个字符
	int length = 4;
	CString tem;

	vector<CString> pathLL;
	while (1)
	{

		k = stringHtml.Find("path");
		if (k == -1) {//无path则返回-1
			break;
		}
		i = k + Offset;
		tem += (";");
		while (stringHtml.GetAt(i) !='"')//CStringArray类(动态CString)的GetAt根据一个输入参数“ID”,返回其对应的字符串
		{
			tem += stringHtml.GetAt(i);
			i++;
		}
		tem += ";";

		pathLL.push_back(tem);
		tem.Empty();//清空tem
		stringHtml.Delete(k, length);//Delete 'length=4' characters, starting at index 'k'

	}
	return pathLL;
}

//Find()函数如果查找成功则输出查找到的第一个位置，否则返回-1 ；
bool isTypeRight(CString stringHtml) {
	int i = 0, offset = 6;
	i = stringHtml.Find("type") + offset;
	if (stringHtml.GetAt(i) == '2') {//??
		cout << "json data is available for navigation" << endl;
		return TRUE;
	}
	else {
		return FALSE;
	}
}

bool isStatusRight(CString stringHtml) {
	int i = 0, offset = 8;
	i = stringHtml.Find("status") + offset;
	if (stringHtml.GetAt(i) == '0') {
		cout << "json data status is right" << endl;
		return TRUE;
	}
	else {
		return FALSE;
	}

}

vector<CString> getLatLon(vector<CString> pathLonLat) {



	vector<CString> pathLonLat_Ed;
	for (int i = 0;i < pathLonLat.size();i++) {
		CString tem;
		int offset = 1;
		int k = 0;
		int ind = 0;
		int count = 0;

		while (1) {
			//Lat
			k = pathLonLat[i].Find(",");
			if (k == -1) {
				break;
			}
			ind = k + offset;
			while (pathLonLat[i].GetAt(ind) !=';')
			{
				tem += pathLonLat[i].GetAt(ind);
				ind++;
			}
			tem += ",";





			//Lon
			k = pathLonLat[i].Find(";");
			ind = k + offset;
			while (pathLonLat[i].GetAt(ind) != ',')
			{
				tem += pathLonLat[i].GetAt(ind);
				ind++;
			}
			tem += ";";


			//delete
			k = pathLonLat[i].Find(";");
			pathLonLat[i].Delete(k, 1);
			k = pathLonLat[i].Find(",");
			pathLonLat[i].Delete(k, 1);


		}
		int Leng = tem.GetLength();
		tem.Delete(Leng - 1, 1);
		pathLonLat_Ed.push_back(tem);

	}
	return pathLonLat_Ed;

}

vector<CString> coordinateTran(vector<CString> pathLonLat_Ed) {


	vector<CString> GPSpath;


	for (int i = 0;i < pathLonLat_Ed.size();i++) {
		CInternetSession sessionSPG;
		CHttpFile *fileSPG = NULL;

		//地图坐标转换API-convert
		CString strURL_SPG = "http://api.gpsspg.com/convert/coord/?oid=3181&key=F76293B1C3556C557D4AF03CFF6AEED2&from=2&to=0&latlng=" + pathLonLat_Ed[i];
		CString strSPG = "";
		try {

			fileSPG = (CHttpFile*)sessionSPG.OpenURL(strURL_SPG);

		}
		catch (CInternetException * m_pException) {

			fileSPG = NULL;

			m_pException->m_dwError;

			m_pException->Delete();

			sessionSPG.Close();

			MessageBox(NULL, "CInternetExeption", "InternetError", MB_ICONERROR);

		}

		CString strLine_SPG;

		if (fileSPG != NULL) {

			while (fileSPG->ReadString(strLine_SPG) != NULL) {

				strSPG += strLine_SPG;

			}


		}
		else {

			MessageBox(NULL, "fail", "ReadError", MB_ICONERROR);

		}

		GPSpath.push_back(strSPG);

		fileSPG->Close();
		delete fileSPG;
		fileSPG = NULL;
		sessionSPG.Close();
	}



	return GPSpath;
}

vector<CString> getShort(vector<CString> pathLonLat_Ed) {
	vector<CString> pathLonLat_S;
	for (int i = 0;i < pathLonLat_Ed.size();i++) {
		int count = 0;
		int isEmp = 0;
		CString copy = pathLonLat_Ed[i];
		while (1) {
			isEmp = copy.Find(',');
			if (isEmp == -1) {
				break;
			}
			else
			{
				count++;
			}
			copy.Delete(isEmp, 1);
		}
		//---------------------------------------------------------------------------------------------------------------------------------------------------//
		if ((count > 19) && (count<41))
		{
			int k = 0;
			int num = 0;
			CString tem;
			while (1) {
				tem += pathLonLat_Ed[i].GetAt(k);
				if (pathLonLat_Ed[i].GetAt(k) == ';') {
					num++;
				}
				k++;
				if (num > 17) {
					int lengOf = tem.GetLength();
					tem.Delete(lengOf - 1, 1);
					pathLonLat_S.push_back(tem);
					break;
				}
			}
			CString tem_;
			while (k<pathLonLat_Ed[i].GetLength()) {
				tem_ += pathLonLat_Ed[i].GetAt(k);
				k++;
			}
			pathLonLat_S.push_back(tem_);

		}
		//---------------------------------------------------------------------------------------------------------------------------------------------------//
		else if (count > 41) {
			int k = 0;
			int num = 0;
			CString tem;
			while (1) {
				tem += pathLonLat_Ed[i].GetAt(k);
				if (pathLonLat_Ed[i].GetAt(k) == ';') {
					num++;
				}
				k++;
				if (num > 17) {
					int lengOf = tem.GetLength();
					tem.Delete(lengOf - 1, 1);
					pathLonLat_S.push_back(tem);
					break;
				}
			}
			num = 0;
			CString tem_;
			while (1) {
				tem_ += pathLonLat_Ed[i].GetAt(k);
				if (pathLonLat_Ed[i].GetAt(k) ==';') {
					num++;
				}
				k++;
				if (num > 17) {
					int lengOf = tem_.GetLength();
					tem_.Delete(lengOf - 1, 1);
					pathLonLat_S.push_back(tem_);
					break;
				}
			}

			CString tem__;
			while (k<pathLonLat_Ed[i].GetLength()) {
				tem__ += pathLonLat_Ed[i].GetAt(k);
				k++;
			}
			pathLonLat_S.push_back(tem__);
		}


		//---------------------------------------------------------------------------------------------------------------------------------------------------//


		else
		{
			pathLonLat_S.push_back(pathLonLat_Ed[i]);
		}


	}
	return pathLonLat_S;
}

vector<CString> getWGS84(vector<CString> GPSpath) {

	vector<CString> GPSLL;
	for (int i = 0;i < GPSpath.size();i++) {
		int k = 0;
		int offset = 6;
		int index = 0;
		int length = 3;
		CString tem;



		while (1) {
			k = GPSpath[i].Find("lat");
			if (k == -1) {
				break;
			}

			index = k + offset;
			while (GPSpath[i].GetAt(index) != '"') {
				tem += GPSpath[i].GetAt(index);
				index++;
			}
			tem += ',';
			GPSpath[i].Delete(k, length);


			k = GPSpath[i].Find("lng");
			index = k + offset;
			while (GPSpath[i].GetAt(index) != '"') {
				tem += GPSpath[i].GetAt(index);
				index++;
			}

			tem += ';';
			GPSpath[i].Delete(k, length);
		}

		GPSLL.push_back(tem);
	}


	return GPSLL;

}

vector<Point2d> getDoubleVector(vector<CString> GPSLL) {



	vector<Point2d> GPS_LL_NUM;
	for (int i = 0;i < GPSLL.size();i++) {


		int p = 0;
		int k = 0;
		while (p<GPSLL[i].GetLength()) {
			string latitude;
			string longitude;



			while (GPSLL[i].GetAt(p) != ',') {
				latitude += GPSLL[i].GetAt(p);
				p++;
			}
			p++;
			while (GPSLL[i].GetAt(p) != ';') {
				longitude += GPSLL[i].GetAt(p);
				p++;
			}
			p++;
			Point2d tem_gps;
			stringstream lat_str;
			lat_str << latitude;
			lat_str >> tem_gps.x;
			stringstream lng_str;
			lng_str << longitude;
			lng_str >> tem_gps.y;



			GPS_LL_NUM.push_back(tem_gps);
		}
	}


	return GPS_LL_NUM;
}

CString getBaiduPath(CString originLatGPS, CString originLonGPS, CString destination) {
	CInternetSession session;

	CHttpFile *file = NULL;


	//ak=ajzZSeCRePGj4mRcWxFXOZOZQyZEd04i
	CString strURL = "http://api.map.baidu.com/direction/v1?mode=walking&origin=" + originLatGPS + ',' + originLonGPS + "&destination=" + destination + "&region=北京&output=json&coord_type=wgs84&ak=ajzZSeCRePGj4mRcWxFXOZOZQyZEd04i";

	CString strHtml = "";//初始化为空串


	//异常处理
	try {

		file = (CHttpFile*)session.OpenURL(strURL);

	}
	catch (CInternetException * m_pException) {//指针

		file = NULL;

		m_pException->m_dwError;//->指向结构体成员运算符

		m_pException->Delete();

		session.Close();

		MessageBox(NULL, "CInternetExeption", "InternetError", MB_ICONERROR);

	}

	CString strLine;

	if (file != NULL) {
		while (file->ReadString(strLine) != NULL) {

			strHtml += strLine;//ReadString是按行读的？

		}

	}
	else {

		MessageBox(NULL, "fail", "ReadError", MB_ICONERROR);

	}



	file->Close();
	delete file;
	file = NULL;
	session.Close();


	return strHtml;
}

vector<Point2d> getGPSroute(CString originLatGPS, CString originLonGPS, CString destination) {


	CString strHtml;
	//
	strHtml = getBaiduPath(originLatGPS, originLonGPS, destination);


	//json数据分析
	//1.判断返回的json数据是否可用
	//1.1status
	if (!isStatusRight(strHtml)) {
		MessageBox(NULL, "status error, navigation fail", "Error", MB_ICONERROR);
		exit(1);
	}
	//1.2type
	if (!isTypeRight(strHtml)) {
		MessageBox(NULL, "destination is not clear, please try a specific one", "Error", MB_ICONERROR);
		exit(1);
	}

	//得到path的经纬度
	vector<CString> pathLonLat;
	pathLonLat = getPathLL(strHtml);


	//将经度,纬度转为纬度,经度？？？？？？
	vector<CString> pathLonLat_Ed;
	pathLonLat_Ed = getLatLon(pathLonLat);


	//将长度限制在20以内？？？？？？
	vector<CString> pathLonLat_S;
	pathLonLat_S = getShort(pathLonLat_Ed);



	//发送请求，得到GPS经纬度JSON数据
	vector<CString> GPSpath;
	GPSpath = coordinateTran(pathLonLat_S);

	//得到WGS84经纬度
	vector<CString> GPSLL;
	GPSLL = getWGS84(GPSpath);


	//将数组转化为double结构
	vector<Point2d> GPS_LL_NUM;
	GPS_LL_NUM = getDoubleVector(GPSLL);

	return GPS_LL_NUM;


}

vector<Point2d> getStartEnd(CString originLatGPS, CString originLonGPS, CString destination) {
	CString strHtml;
	//
	strHtml = getBaiduPath(originLatGPS, originLonGPS, destination);


	//json数据分析
	//1.判断返回的json数据是否可用
	//1.1status
	if (!isStatusRight(strHtml)) {
		MessageBox(NULL, "status error, navigation fail", "Error", MB_ICONERROR);
		exit(1);
	}
	//1.2type
	if (!isTypeRight(strHtml)) {
		MessageBox(NULL, "destination is not clear, please try a specific one", "Error", MB_ICONERROR);
		exit(1);
	}

	//得到path的经纬度

	vector<CString> pathLonLat;
	pathLonLat = getPathLL(strHtml);


	//将经度,纬度转为纬度,经度
	vector<CString> pathLonLat_Ed;
	pathLonLat_Ed = getLatLon(pathLonLat);


	vector<CString> startEnd;
	for (int i = 0;i < pathLonLat_Ed.size();i++) {

		CString tem;

		if (pathLonLat_Ed[i].Find(';') == -1) {
			int index_ = 0;
			while (index_<pathLonLat_Ed[i].GetLength()) {
				tem += pathLonLat_Ed[i].GetAt(index_);
				index_++;
			}
			startEnd.push_back(tem);
		}
		else {
			int index = 0;
			while (pathLonLat_Ed[i].GetAt(index) != ';') {//？？？
				tem += pathLonLat_Ed[i].GetAt(index);
				index++;
			}
			tem += ';';
			int count = 0;
			int k1 = 0, k2 = 0;
			while (1) {
				k1 = pathLonLat_Ed[i].Find(';', k2);
				if (k1 == -1) {
					break;
				}
				k2 = k1 + 1;
			}
			while (k2 < pathLonLat_Ed[i].GetLength()) {
				tem += pathLonLat_Ed[i].GetAt(k2);
				k2++;
			}

			startEnd.push_back(tem);

		}


	}

	vector<CString> startEnd_json;
	startEnd_json = coordinateTran(startEnd);

	vector<CString> startEnd_GPSLL;
	startEnd_GPSLL = getWGS84(startEnd_json);

	vector<Point2d> startEnd_NUM;
	startEnd_NUM = getDoubleVector(startEnd_GPSLL);

	return startEnd_NUM;

}

vector<Point2d> getBaiduBreaks(CString originLatGPS, CString originLonGPS, CString destination) {
	CString strHtml;
	//
	strHtml = getBaiduPath(originLatGPS, originLonGPS, destination);

	//json数据分析
	//1.判断返回的json数据是否可用
	//1.1status
	if (!isStatusRight(strHtml)) {
		MessageBox(NULL, "status error, navigation fail", "Error", MB_ICONERROR);
		exit(1);
	}
	//1.2type
	if (!isTypeRight(strHtml)) {
		MessageBox(NULL, "destination is not clear, please try a specific one", "Error", MB_ICONERROR);
		exit(1);
	}

	//得到path的经纬度

	vector<CString> pathLonLat;
	pathLonLat = getPathLL(strHtml);


	//将经度,纬度转为纬度,经度
	vector<CString> pathLonLat_Ed;
	pathLonLat_Ed = getLatLon(pathLonLat);


	vector<CString> startEnd;
	for (int i = 0; i < pathLonLat_Ed.size(); i++) {

		CString tem;

		if (pathLonLat_Ed[i].Find(';') == -1) {
			int index_ = 0;
			while (index_ < pathLonLat_Ed[i].GetLength()) {
				tem += pathLonLat_Ed[i].GetAt(index_);
				index_++;
			}
			tem += ';';

			startEnd.push_back(tem);
		}
		else {
			int index = 0;
			while (pathLonLat_Ed[i].GetAt(index) != ';') {//？？？
				tem += pathLonLat_Ed[i].GetAt(index);
				index++;
			}
			tem += ';';
			int count = 0;
			int k1 = 0, k2 = 0;
			while (1) {
				k1 = pathLonLat_Ed[i].Find(';', k2);
				if (k1 == -1) {
					break;
				}
				k2 = k1 + 1;
			}
			while (k2 < pathLonLat_Ed[i].GetLength()) {
				tem += pathLonLat_Ed[i].GetAt(k2);
				k2++;
			}
			tem += ';';

			startEnd.push_back(tem);

		}


	}
	vector<Point2d> startEnd_NUM;
	startEnd_NUM = getDoubleVector(startEnd);

	return startEnd_NUM;

}

Point2d getBaiduLatLon(Point3d originLatLon) {
	CInternetSession session;
	CHttpFile *file = NULL;

	CString cstringLatGPS;
	cstringLatGPS.Format("%f", originLatLon.x * RADIANS_TO_DEGREES);
	CString cstringLonGPS;
	cstringLonGPS.Format("%f", originLatLon.y * RADIANS_TO_DEGREES);

	//ak=ajzZSeCRePGj4mRcWxFXOZOZQyZEd04i//ZbE5zHhuR6rgSul4TUQbPQh83NVn3Oga
	CString strURL = "http://api.map.baidu.com/geoconv/v1/?coords=" + cstringLonGPS + ',' + cstringLatGPS + "&from=1&to=5&ak=ajzZSeCRePGj4mRcWxFXOZOZQyZEd04i";

	CString strHtml = "";//初始化为空串

	//异常处理
	try {

		file = (CHttpFile*)session.OpenURL(strURL);

	}
	catch (CInternetException * m_pException) {//指针

		file = NULL;

		m_pException->m_dwError;//->指向结构体成员运算符

		m_pException->Delete();

		session.Close();

		MessageBox(NULL, "CInternetExeption", "InternetError", MB_ICONERROR);

	}

	CString strLine;

	if (file != NULL) {
		while (file->ReadString(strLine) != NULL) {

			strHtml += strLine;//ReadString是按行读的？

		}

	}
	else {

		MessageBox(NULL, "fail", "ReadError", MB_ICONERROR);

	}

	file->Close();
	delete file;
	file = NULL;
	session.Close();

	//json数据分析
	//1.判断返回的json数据是否可用
	//1.1status
	if (!isStatusRight(strHtml)) {
		MessageBox(NULL, "status error, navigation fail", "Error", MB_ICONERROR);
		exit(1);
	}

	int k = strHtml.Find("y");
	CString cLat, cLon;
	int i = k + 3;
	while (strHtml.GetAt(i) != '}')
	{
		cLat += strHtml.GetAt(i);
		i++;
	}
	k = strHtml.Find("x");
	i = k + 3;
	while (strHtml.GetAt(i) != ',')
	{
		cLon += strHtml.GetAt(i);
		i++;
	}

	string sLat, sLon;
	int p = 0;
	while (p < cLat.GetLength()) {
		sLat += cLat.GetAt(p);
		p++;
	}
	p = 0;
	while (p < cLon.GetLength()) {
		sLon += cLon.GetAt(p);
		p++;
	}

	Point2d baidu_coordinate;
	stringstream lat_str;
	lat_str << sLat;
	lat_str >> baidu_coordinate.x;
	stringstream lng_str;
	lng_str << sLon;
	lng_str >> baidu_coordinate.y;

	return baidu_coordinate;
}

////逸夫楼拐角测试
//vector<Point2d> getBeihangFixStartEnd()
//{
//	vector<Point2d> GoalLatLon;
//	GoalLatLon.push_back(Point2d(39.9871817409, 116.356841575));
//	GoalLatLon.push_back(Point2d(39.987189, 116.356455));
//	GoalLatLon.push_back(Point2d(39.987268, 116.356455));
//	GoalLatLon.push_back(Point2d(39.987268, 116.356282));
//	GoalLatLon.push_back(Point2d(39.987768, 116.356282));
//
//	return GoalLatLon;
//}
//
////北航百度固定长距离 图书馆广场一周 垃圾桶-图书馆东南-图书馆西南-办公楼西北-垃圾桶
//vector<Point2d> getBeihangFixStartEnd()
//{
//	vector<Point2d> GoalLatLon;
//	GoalLatLon.push_back(Point2d(39.9883523197, 116.355619917));  //图书馆东南角 
//	GoalLatLon.push_back(Point2d(39.9886768786, 116.355597315));
//	GoalLatLon.push_back(Point2d(39.988948349, 116.355585238));
//	GoalLatLon.push_back(Point2d(39.9889385615, 116.355450795));
//	GoalLatLon.push_back(Point2d(39.9889236973, 116.355290376));
//	GoalLatLon.push_back(Point2d(39.9886654892, 116.355298941));
//	GoalLatLon.push_back(Point2d(39.9883270161, 116.355312613));
//	GoalLatLon.push_back(Point2d(39.9883301948, 116.355437051));
//	GoalLatLon.push_back(Point2d(39.9883361219, 116.355596919));
//
//	return GoalLatLon;
//}

//北航百度固定长距离
vector<Point2d> getBeihangFixStartEnd()
{
	vector<Point2d> GoalLatLon;
	GoalLatLon.push_back(Point2d(39.9872541899, 116.359297343));  //近东南门口
	GoalLatLon.push_back(Point2d(39.9871992231, 116.358210895));
	GoalLatLon.push_back(Point2d(39.9871608825, 116.357254725));
	GoalLatLon.push_back(Point2d(39.9871517409, 116.356841575));
	GoalLatLon.push_back(Point2d(39.9871811171, 116.356311179));
	GoalLatLon.push_back(Point2d(39.9881249127, 116.356261772));

	return GoalLatLon;
}

////盲校百度固定长距离
//vector<Point2d> getSchoolFixStartEnd()
//{
//	vector<Point2d> SetGoalLatLon;
//	SetGoalLatLon.push_back(Point2d(39.9428590591, 116.29015215));
//	SetGoalLatLon.push_back(Point2d(39.9431281263, 116.290093303));
//	SetGoalLatLon.push_back(Point2d(39.9430824977, 116.289703841)); //北门2
//	SetGoalLatLon.push_back(Point2d(39.9430191118, 116.289208798));
//	SetGoalLatLon.push_back(Point2d(39.9429348942, 116.289150394));
//	SetGoalLatLon.push_back(Point2d(39.9426137202, 116.289235275)); //西门5
//	SetGoalLatLon.push_back(Point2d(39.9422294869, 116.289360331));
//	SetGoalLatLon.push_back(Point2d(39.9422676433, 116.289750018)); //南门7
//	SetGoalLatLon.push_back(Point2d(39.9423402659, 116.290172356));
//	SetGoalLatLon.push_back(Point2d(39.9424420694, 116.290153647));
//	SetGoalLatLon.push_back(Point2d(39.9424582927, 116.290243282));
//	SetGoalLatLon.push_back(Point2d(39.9428590591, 116.29015215));
//
//	vector<Point2d> OutputGoalLatLon;
//	for (int i = 0; i <= destiChose; i++)
//	{
//		OutputGoalLatLon.push_back(SetGoalLatLon[i]);
//	}
//
//	return OutputGoalLatLon;
//}

////盲校百度固定长距离(内测小改1.0)
//vector<Point2d> getSchoolFixStartEnd()
//{
//	vector<Point2d> SetGoalLatLon;
//	SetGoalLatLon.push_back(Point2d(39.9428590591, 116.29015215));
//	SetGoalLatLon.push_back(Point2d(39.9431281263, 116.290093303));
//	SetGoalLatLon.push_back(Point2d(39.9430824977, 116.289703841)); //北门2
//	SetGoalLatLon.push_back(Point2d(39.9430191118, 116.289208798));
//	SetGoalLatLon.push_back(Point2d(39.9429348942, 116.289150394));
//	SetGoalLatLon.push_back(Point2d(39.9426137202, 116.289235275)); //西门5
//	SetGoalLatLon.push_back(Point2d(39.9422294869, 116.289360331));
//	SetGoalLatLon.push_back(Point2d(39.9422676433, 116.289750018)); //南门7
//	SetGoalLatLon.push_back(Point2d(39.9423343123, 116.290092232));
//	SetGoalLatLon.push_back(Point2d(39.9424610832, 116.290070815));
//	SetGoalLatLon.push_back(Point2d(39.9424582927, 116.290243282));
//	SetGoalLatLon.push_back(Point2d(39.9428590591, 116.29015215));
//
//	vector<Point2d> OutputGoalLatLon;
//	for (int i = 0; i <= destiChose; i++)
//	{
//		OutputGoalLatLon.push_back(SetGoalLatLon[i]);
//	}
//
//	return OutputGoalLatLon;
//}

//盲校百度固定长距离(内测小改1.1)
vector<Point2d> getSchoolFixStartEnd()
{
	vector<Point2d> SetGoalLatLon;
	SetGoalLatLon.push_back(Point2d(39.9428590591, 116.29015215));
	SetGoalLatLon.push_back(Point2d(39.9431281263, 116.290093303));
	SetGoalLatLon.push_back(Point2d(39.9430824977, 116.289703841)); //北门2
	SetGoalLatLon.push_back(Point2d(39.9430191118, 116.289208798));
	SetGoalLatLon.push_back(Point2d(39.9429348942, 116.289150394));
	SetGoalLatLon.push_back(Point2d(39.9426137202, 116.289235275)); //西门5
	SetGoalLatLon.push_back(Point2d(39.9422294869, 116.289360331));
	SetGoalLatLon.push_back(Point2d(39.9422676433, 116.289750018)); //南门7
	SetGoalLatLon.push_back(Point2d(39.9423255037, 116.290051219));
	SetGoalLatLon.push_back(Point2d(39.9424771811, 116.290044747));
	SetGoalLatLon.push_back(Point2d(39.9424936964, 116.290206354));
	SetGoalLatLon.push_back(Point2d(39.9428590591, 116.29015215));

	vector<Point2d> OutputGoalLatLon;
	for (int i = 0; i <= destiChose; i++)
	{
		OutputGoalLatLon.push_back(SetGoalLatLon[i]);
	}

	return OutputGoalLatLon;
}

vector<Point2d> getStartEndline(vector<Point2d> Goal)
{
	//求大地图分段路径直线方程
	vector<Point2d> line(Goal.size() - 1);
	for (size_t i = 0; i < Goal.size() - 1; i++){
		line[i].x = (Goal[i].y - Goal[i + 1].y) / float(Goal[i].x - Goal[i + 1].x);//k(i)
		line[i].y = (Goal[i].x*Goal[i + 1].y - Goal[i + 1].x*Goal[i].y) / float(Goal[i].x - Goal[i + 1].x);//b(i)
	}
	return line;
}

vector<Point2d> getCurrentGoalLatLon(cv::Point3d IniLatLonHeight, vector<Point2d> line, vector<Point2d> GoalLatLon_origin)
{
	//由起点在大地图给定路径的位置，得出当前将要走的路径集合
	vector<Point2d> GoalLatLon(0);
	double distance = 1000;
	double distance1 = 0;
	int count = 0;
	for (size_t i = 0; i < line.size(); i++){
		distance1 = abs(line[i].x*IniLatLonHeight.x - IniLatLonHeight.y + line[i].y) / sqrt(1 + pow(line[i].x, 2));
		if ((IniLatLonHeight.x - GoalLatLon_origin[i].x)*(IniLatLonHeight.x - GoalLatLon_origin[i + 1].x) < 0 || (IniLatLonHeight.y - GoalLatLon_origin[i].y)*(IniLatLonHeight.y - GoalLatLon_origin[i + 1].y) < 0)
		{
			if (distance1 < distance){
				distance = distance1;
				count = i;
			}
		}
	}
	for (size_t j = count; j < GoalLatLon_origin.size(); j++){
		GoalLatLon.push_back(Point2d(GoalLatLon_origin[j].x, GoalLatLon_origin[j].y));
	}
	return GoalLatLon;
}