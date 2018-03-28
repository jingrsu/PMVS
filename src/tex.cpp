#include "tex.h"

double Tex::ncc(const Tex &other)const
{
	//Mat v1=values*size;
	//Mat v2=other.values*other.size;
	double ratio = other.size>size ? size / other.size : other.size / size;
	if (std::isnan(ratio))
		return -1;
	double m1 = mean(values)[0];
	double m2 = mean(other.values)[0];
	double m = m1 * values.size() + m2 * other.values.size();
	m = m / (values.size() * 2);
	double n1 = 0;
	double n2 = 0;
	for (int i = 0; i<values.size(); i++) {
		if (values[i] == -1 || other.values[i] == -1) {
			return -1;
		}
		n1 += pow(values[i] - m, 2);
		n2 += pow(other.values[i] - m, 2);
	}
	n1 = sqrt(n1);
	n2 = sqrt(n2);
	if (n1 == 0 || n2 == 0)
		return -1;
	double sum = 0;
	for (int i = 0; i<values.size(); i++) {
		if (values[i] == -1 || other.values[i] == -1) {
			return -1;
		}
		sum += (values[i] - m)*(other.values[i] - m);
	}
	//if(sum/(n1*n2)*ratio>0.9)
	//    cout<<""<<endl;
	//return sum/(n1*n2)*ratio;
	return sum / (n1*n2);
}
void Tex::updateCell(int id, vector<vector<set<int> > >&cell)
{
	int minX, minY, maxX, maxY;
	getMinMaxXMinMaxY(minX, maxX, minY, maxY);
	for (int i = minX; i<maxX; i++)
		for (int j = minY; j<maxY; j++)
		{
			cell[j][i].insert(id);
		}
}
void Tex::getMinMaxXMinMaxY(int&minX, int&maxX, int&minY, int&maxY)
{
	minX = INT_MAX;
	minY = INT_MAX;
	maxX = INT_MIN;
	maxY = INT_MIN;
	for (vector<Point2d>::iterator it = points.begin(); it != points.end(); it++) {
		if (floor(it->x)<minX)
		{
			minX = floor(it->x);
		}
		if (ceil(it->x)>maxX)
		{
			maxX = ceil(it->x);
		}
		if (floor(it->y)<minY)
		{
			minY = floor(it->y);
		}
		if (ceil(it->y)>maxY)
		{
			maxY = ceil(it->y);
		}
	}
}