#ifndef FEATURE_H
#define FEATURE_H

#include <opencv2\opencv.hpp>
#include "image.h"
using namespace cv;
class Image;
class Feature
{
public:
	Image * image;
	double x;
	double y;
	Mat point4D;
	double getDistanceToCameraCenter(const Mat&cameraCenter) const;//distance to camera center
	bool isInEmptyCell()const;
	Mat toHomogeneous()const;
	void findFeatures(vector<Feature>&featuresNearEpipolarLine)const;

private:

};
#endif
