#ifndef TEX_H
#define TEX_H

#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;

class Tex
{
public:
	int id;
	vector<Point2d> points;
	vector<double> values;
	double size;
	double ncc(const Tex&other)const;
	void getMinMaxXMinMaxY(int&minX, int&maxX, int&minY, int&maxY);
	void updateCell(int id, vector<vector<set<int>>>&cell);

private:

};

#endif
