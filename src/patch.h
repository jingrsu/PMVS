#ifndef PATCH_H
#define PATCH_H
#include <opencv2\opencv.hpp>
#include <nlopt.hpp>
#include <memory>
#include "image.h"
#include "tex.h"

using namespace std;
using namespace cv;

class  Patch
{
public:
	Patch();
	Mat_<double> center;
	Mat_<double> normal;
	Mat_<double> ray;
	Image* rimage;
	vector<Image*> simages;
	vector<Image*> timages;
	vector<pair<int, pair<int, int>>> qs;
	vector<pair<int, pair<int, int>>> qt;
	void encode(double &depth, double &alpha, double &beta)const;
	void decode(double depth, double alpha, double beta);
	void getPAxes(Mat &pxaxis, Mat &pyaxis)const;
	double averageCost()const;
	double cost(const Tex&tex1, const Tex&tex2) const;
	void optimze();
	void updateImage(double alpha1, double alpha2);
	void updateImage();
	void updateImageCell(int pid);
	void showResult();
	bool isNeighborPatch(Patch&p);

private:
	bool isInTheImage(Image* image);
};
typedef  shared_ptr<Patch> PPatch;
#endif