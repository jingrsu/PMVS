#ifndef IMAGE_H
#define IMAGE_H
#include <opencv2\opencv.hpp>
#include "tex.h"
#include "feature.h"

using namespace std;
using namespace cv;
class Feature;
class Image
{
public:
	Image(const string& fileName, int i, const Mat_<double>& k, const Mat_<double>& rt);
	int id;
	Mat data;
	Mat kmat;
	Mat pmat;
	Mat rmat;
	Mat tmat;
	Mat rtmat;
	Mat cameraCenter;
	Mat xaxis;
	Mat yaxis;
	Mat zaxis;
	vector<vector<set<int>>> qf;
	vector<vector<set<int>>> qt;
	vector<vector<int>> depth;
	vector<Feature> features;
	Mat descriptors;
	vector<Image*> nimages;
	void detectFeatures();
	double getDistanceToCameraCenter(const Mat& point);
	Mat project(const Mat& point);
	void grabTex(const Mat coord, const Mat& pxaxis, const Mat& pyaxis, Tex& pTex);

private:

};

#endif
