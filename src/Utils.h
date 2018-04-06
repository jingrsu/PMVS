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
	inline Mat CalPlaneLineIntersectPoint(Mat_<double>&planeVector, Mat_<double>&planePoint, Mat_<double>&lineVector, Mat_<double>&linePoint)
	{
		double vp1, vp2, vp3, n1, n2, n3, v1, v2, v3, m1, m2, m3, t, vpt;
		vp1 = planeVector(0, 0);
		vp2 = planeVector(1, 0);
		vp3 = planeVector(2, 0);
		n1 = planePoint(0, 0);
		n2 = planePoint(1, 0);
		n3 = planePoint(2, 0);
		v1 = lineVector(0, 0);
		v2 = lineVector(1, 0);
		v3 = lineVector(2, 0);
		m1 = linePoint(0, 0);
		m2 = linePoint(1, 0);
		m3 = linePoint(2, 0);
		vpt = v1 * vp1 + v2 * vp2 + v3 * vp3;
		assert(fabs(vpt) > 1e-6);
		t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt;
		Mat_<double> ret(4, 1);
		ret(0, 0) = m1 + v1 * t;
		ret(1, 0) = m2 + v2 * t;
		ret(2, 0) = m3 + v3 * t;
		ret(3, 0) = 1;
		return ret;
	}
}


#endif // !UTILS_H

