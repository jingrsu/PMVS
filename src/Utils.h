#ifndef UTILS_H
#define UTILS_H
#include <opencv2\opencv.hpp>
#include <cmath>
#include "feature.h"
#include "image.h"
using namespace std;

namespace utils {
	inline void triangluate(const Feature& f1, const Feature& f2, Mat& point)
	{
		Mat points4D;
		vector<Point2d> points1, points2;
		points1.push_back(Point2d(f1.x, f1.y));
		points2.push_back(Point2d(f2.x, f2.y));
		triangulatePoints(f1.image->pmat, f2.image->pmat, points1, points2, points4D);
		point = points4D;
		point /= point.at<double>(3, 0);
		point.at<double>(3, 0) = 1;
	}
	inline double cosangle(const Mat& point, const Mat& c1, const Mat& c2)
	{
		Mat v1 = point - c1;
		Mat v2 = point - c2;
		double ret = v1.dot(v2) / (norm(v1)*norm(v2));
		assert(std::isnan(ret) == 0);

		return ret;
	}
	inline double cosangle(const Image* image1, const Image* image2)
	{
		Mat v1 = Mat::zeros(3,1, CV_64F);
		v1.at<double>(2, 0) = 1;
		v1 = image1->rmat.inv()*v1;
		Mat v2 = Mat::zeros(3, 1, CV_64F);
		v2.at<double>(2, 0) = 1;
		v2 = image1->rmat.inv()*v2;
		double ret = v1.dot(v2) / (norm(v1)*norm(v2));
		assert(std::isnan(ret) == 0);

		return ret;
	}
	inline void computeEpipolarLine(const Mat &p1, const Mat &p2, const Mat&c1, const Feature &f, Mat &line)
	{
		Mat pinvP1(3, 3, CV_64F);
		pinvP1 = p1.t()*(p1*p1.t()).inv();
		Mat x(3, 1, CV_64F);
		x.at<double>(0, 0) = f.x;
		x.at<double>(1, 0) = f.y;
		x.at<double>(2, 0) = 1;
		line = (p2*c1).cross(p2*pinvP1*x);
	}
}


#endif // !UTILS_H

