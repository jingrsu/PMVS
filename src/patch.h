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
	Mat center;
	Mat normal;
	Mat ray;
	Image* rimage;
	vector<Image*> simages;
	vector<Image*> timages;
	void encode(double &depth, double &alpha, double &beta)const;
	void decode(double depth, double alpha, double beta);
	void getPAxes(Mat &pxaxis, Mat &pyaxis)const;
	double averageCost()const;
	double cost(const Tex&tex1, const Tex&tex2) const;
	void optimze();
	void updateImage(double alpha1, double alpha2);
	void updateImageCell(int pid);
	void showResult();

private:
	bool isInTheImage(Image* image);
};
typedef  shared_ptr<Patch> PPatch;
#endif