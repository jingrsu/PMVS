#include "image.h"
#include "feature.h"

Image::Image(const string& fileName, int i, const Mat_<double>& k, const Mat_<double>& rt)
{
	cout << "Loading image " << i << ' ' << fileName << endl;
	id = i;
	data = imread(fileName);
	int row = data.rows;
	int col = data.cols;
	kmat = k;
	rtmat = rt;
	pmat = k * rt;
	rmat = rt(Range(0, 3), Range(0, 3));
	tmat = rt.col(3);
	Mat_<double> c = -rmat.inv() * tmat;
	Mat hc = (Mat_<double>(4, 1) << c(0, 0), c(1, 0), c(2, 0), 1);
	cameraCenter = hc;
	qf.resize(ceil(row / 2.0), vector<set<int>>(ceil(col / 2.0)));
	qt.resize(ceil(row / 2.0), vector<set<int>>(ceil(col / 2.0)));
	depth.resize(row - 1, vector<int>(col - 1, -1));
	Mat_<double> xaxis_(4, 1), yaxis_(4, 1), zaxis_(4, 1);
	xaxis_ = rt(0, 0);
	xaxis_(1, 0) = rt(0, 1);
	xaxis_(2, 0) = rt(0, 2);
	xaxis_(3, 0) = 0;
	xaxis = xaxis_;
	yaxis_(0, 0) = rt(1, 0);
	yaxis_(1, 0) = rt(1, 1);
	yaxis_(2, 0) = rt(1, 2);
	yaxis_(3, 0) = 0;
	yaxis = yaxis_;
	zaxis_(0, 0) = rt(2, 0);
	zaxis_(1, 0) = rt(2, 1);
	zaxis_(2, 0) = rt(2, 2);
	zaxis_(3, 0) = 0;
	zaxis = zaxis_;
}

void Image::detectFeatures()
{
	vector<Point2d> corners;
	Mat dst;
	cvtColor(data, dst, cv::COLOR_BGR2GRAY);
	goodFeaturesToTrack(dst, corners, 500, 0.02, 10);
	for (size_t i = 0; i < corners.size(); i++)
	{
		Feature f;
		f.image = this;
		f.x = corners[i].x;
		f.y = corners[i].y;
		features.push_back(f);
	}
}

double Image::getDistanceToCameraCenter(const Mat &point)
{
	return norm(cameraCenter - point);
}

Mat Image::project(const Mat& point)
{
	Mat m(3, 1, CV_64F);
	m = pmat * point;
	m /= m.at<double>(2, 0);
	return m;
}

void Image::grabTex(const Mat coord, const Mat &pxaxis, const Mat &pyaxis, Tex &pTex)
{
	pTex.points.clear();
	pTex.values.clear();
	Mat center = project(coord);
	Mat dx = project(coord + pxaxis) - center;
	Mat dy = project(coord + pyaxis) - center;

	for (int i = -2; i <= 2; i++)
		for (int j = -2; j <= 2; j++)
		{
			Mat p(3, 1, CV_64F);
			p = center + j * dx + i * dy;
			Mat v1, v2;
			double px = p.at<double>(0, 0);
			double py = p.at<double>(1, 0);
			pTex.points.push_back(Point2d(px, py));
			if (px<0 || px>data.cols - 1 || py<0 || py>data.rows - 1 || std::isnan(px) || std::isnan(py)) {
				pTex.values.push_back(-1);
				pTex.values.push_back(-1);
				pTex.values.push_back(-1);
			}
			else {
				Mat v;
				cv::getRectSubPix(data, Size(1, 1), Point2f(px, py), v);
				pTex.values.push_back(v.at<Vec3b>(0, 0)[0]);
				pTex.values.push_back(v.at<Vec3b>(0, 0)[1]);
				pTex.values.push_back(v.at<Vec3b>(0, 0)[2]);
			}
		}
	pTex.size = norm(4 * dx + 4 * dy);
}