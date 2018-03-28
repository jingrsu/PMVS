#include "PMVS.h"
#include "Utils.h"

PMVS::PMVS(const string&fileName, const string&imageDir)
{
	loadImages(fileName, imageDir);
	for (Image &image:images)
	{
		image.detectFeatures();
	}

	for (size_t i = 0; i < images.size(); i++)
	{
		for (size_t j = i + 1; j < images.size(); j++)
		{
			images[i].nimages.push_back(&images[j]);
			images[j].nimages.push_back(&images[i]);
		}
	}
}

void PMVS::loadImages(const string&fileName, const string&imageDir)
{
	ifstream fin(fileName.c_str());
	int n;
	fin >> n;
	cout << n << endl;
	system("pause");
	for (size_t i = 0; i < n; i++)
	{
		string name;
		Mat_<double> k(3, 3), rt(3, 4);
		fin >> name
			>> k(0, 0) >> k(0, 1) >> k(0, 2)
			>> k(1, 0) >> k(1, 1) >> k(1, 2)
			>> k(2, 0) >> k(2, 1) >> k(2, 2)
			>> rt(0, 0) >> rt(0, 1) >> rt(0, 2)
			>> rt(1, 0) >> rt(1, 1) >> rt(1, 2)
			>> rt(2, 0) >> rt(2, 1) >> rt(2, 2)
			>> rt(0, 3) >> rt(1, 3) >> rt(2, 3);
		images.push_back(Image(imageDir + '/' + name, i, k, rt));
	}
}

void PMVS::seed()
{
	for (Image &rimage:images)
	{
		for (const Feature &f1:rimage.features)
		{
			if (f1.isInEmptyCell())
			{
				vector<Feature> features;
				f1.findFeatures(features);
				PPatch bestPatch;
				double bestCost = -1;

				for (Feature &f2:features)
				{
					utils::triangluate(f1, f2, f2.point4D);
					PPatch p = PPatch(new Patch());
					p->center = f2.point4D;
					p->normal = rimage.cameraCenter - f2.point4D;
					double n = norm(p->normal);

					p->normal /= n;
					p->rimage = &rimage;
					p->updateImage(0.4, 0.4);
					
					if (p->timages.size() == 0)
					{
						continue;
					}

					p->optimze();

					p->updateImage(0.7, 0.7);
					if (p->timages.size() >= 3)
					{
						if (p->averageCost() > bestCost)
						{
							bestPatch = p;
							bestCost = p->averageCost();
						}
					}
				}
				if (bestPatch.get() != NULL)
				{
					patches.push_back(bestPatch);

					int pid = (int)patches.size();
					bestPatch->updateImageCell(pid);
				}
			}
		}
	}
}

void PMVS::expand()
{

}


void PMVS::savePatches(const string &fileName)
{
	ofstream fout(fileName);

	fout << "ply" << endl;
	fout << "format ascii 1.0" << endl;
	fout << "element vertex " << patches.size() << endl;
	fout << "property float x" << endl;
	fout << "property float y" << endl;
	fout << "property float z" << endl;
	fout << "property uchar diffuse_red" << endl;
	fout << "property uchar diffuse_green" << endl;
	fout << "property uchar diffuse_blue" << endl;
	fout << "end_header" << endl;
	for (int i = 0; i<patches.size(); i++) {
		Mat_<double> center(4, 1);
		center = patches[i]->center;
		double r = 0, g = 0, b = 0;
		for (Image *image : patches[i]->timages)
		{

			Mat_<double> x(3, 1);
			x = image->project(center);
			Mat patch;
			getRectSubPix(image->data, Size(1, 1), Point2d(x(0, 0), x(1, 0)), patch);
			b += patch.at<Vec3b>(0, 0)[0];
			g += patch.at<Vec3b>(0, 0)[1];
			r += patch.at<Vec3b>(0, 0)[2];
		}
		r /= patches[i]->timages.size();
		g /= patches[i]->timages.size();
		b /= patches[i]->timages.size();
		fout << center(0, 0) << " " << center(1, 0) << " " << center(2, 0) << " " << (int)r << " " << (int)g << " " << (int)b << endl;
	}
	fout.close();
	cout << "±£´æ½áÊø" << endl;
}
