#ifndef PMVS_H
#define PMVS_H
#include <opencv2\opencv.hpp>
#include "image.h"
#include "patch.h"
#include "tex.h"
#include "feature.h"

using namespace cv;
using namespace std;

class PMVS
{
public:
	PMVS(const string&fileName, const string&imageDir);
	void seed();
	void expand();
	void filter();
	void savePatches(const string& fileName);
	

private:
	vector<Image> images;
	vector<PPatch> patches;
	void loadImages(const string&fileName, const string&imageDir);
	void findNeighbors(Patch p, vector<pair<int, pair<int, int>>>& cells);
	void findNeighbors8(Patch p, vector<PPatch>& patches);
};

#endif // !PMVS_H

