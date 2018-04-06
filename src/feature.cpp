#include "feature.h"
#include "Utils.h"

bool Feature::isInEmptyCell()const
{

	return image->qt[floor(y/2)][floor(x/2)].empty() && image->qf[floor(y/2)][floor(x/2)].empty();
}
Mat Feature::toHomogeneous()const
{
	Mat m(3, 1, CV_64F);
	m.at<double>(0, 0) = x;
	m.at<double>(1, 0) = y;
	m.at<double>(2, 0) = 1;
	return m;
}
double Feature::getDistanceToCameraCenter(const Mat &cameraCenter)const
{
	return norm(cameraCenter - point4D);
}


void Feature::findFeatures(vector<Feature>&featuresNearEpipolarLine)const
{

	for (Image *nimage : image->nimages) {
		if (utils::cosangle(nimage, image) < 0.0)
			continue;
		Mat line(3, 1, CV_64F);
		utils::computeEpipolarLine(image->pmat, nimage->pmat, image->cameraCenter, *this, line);
		double a = line.at<double>(0, 0);
		double b = line.at<double>(1, 0);
		double c = line.at<double>(2, 0);
		//cout<<nimage->features.size()<<endl;
		for (Feature &f : nimage->features) {
			if (f.isInEmptyCell())
			{
				double d = abs(a*f.x + b * f.y + c) / sqrt(a*a + b * b);
				if (d <= 1)
				{
					featuresNearEpipolarLine.push_back(f);
				}
			}
		}

	}
}
