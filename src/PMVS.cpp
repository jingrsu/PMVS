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

					int pid = (int)patches.size() - 1;
					bestPatch->updateImageCell(pid);
				}
			}
		}
	}
}

void PMVS::expand()
{
	size_t idx = 0;
	while (idx < patches.size())
	{
		PPatch &p = patches[idx];
		vector<pair<int, pair<int, int>>> NeighborsCells;
		findNeighbors(*p, NeighborsCells);
		Mat_<double>normal(4, 1), center(4, 1);
		normal = p->normal;
		center = p->center;
		Image*rimage = p->rimage;
		vector<Image*>timages = p->timages;
		for (pair<int, pair<int, int>>&cell: NeighborsCells)
		{
			PPatch newp = PPatch(new Patch());
			normal.copyTo(newp->normal);
			newp->rimage = rimage;
			newp->timages = timages;
			Image& image = images[cell.first];
			float x = 2 * cell.second.first + 0.5;
			float y = 2 * cell.second.second + 0.5;
			x -= image.kmat.at<double>(0, 2);
			y -= image.kmat.at<double>(1, 2);
			Mat_<double>planeVector(3, 1), planePoint(3, 1), lineVector(3, 1), linePoint(3, 1);
			lineVector(0, 0) = x;
			lineVector(1, 0) = y;
			lineVector(2, 0) = 0.5*(image.kmat.at<double>(0, 0) + image.kmat.at<double>(1, 1));
			lineVector = image.rmat.inv()*lineVector;
			image.cameraCenter.rowRange(0, 3).copyTo(linePoint);
			normal.rowRange(0, 3).copyTo(planeVector);
			center.rowRange(0, 3).copyTo(planePoint);
			newp->center = utils::CalPlaneLineIntersectPoint(planeVector, planePoint, lineVector, linePoint);
			newp->optimze();
			newp->updateImage();
			if (newp->timages.size() >= 3)
			{
				patches.push_back(newp);
				int pid = (int)patches.size() - 1;
				newp->updateImageCell(pid);
				cout << "pid: " << pid << " expand a new patch!" << endl;
				if (pid > 10000)
					return;
			}
		}
		idx++;
	}
}

void PMVS::findNeighbors(Patch p, vector<pair<int, pair<int, int>>>& cells)
{
	for (pair<int, pair<int, int>>&cell : p.qs)
	{
		Image &image = images[cell.first];
		for (int dx = -1; dx < 2; dx += 2)
		{
			int x = cell.second.first + dx;
			int y = cell.second.second;

			if (x < 0 || x >= ceil(image.data.cols / 2.0))
			{
				//cout << "edge" << endl;
				continue;
			}
			if (y < 0 || y >= ceil(image.data.rows / 2.0))
			{
				//cout << "edge" << endl;
				continue;
			}
			if (!image.qt[y][x].empty())
			{
				//cout << "empty" << endl;
				continue;
			}

			int isNeighbor = 0;
			for (int patch_id:image.qf[y][x])
			{
				PPatch newpatch = patches[patch_id];
				if (p.isNeighborPatch(*newpatch))
				{
					isNeighbor = 1;
					//cout << "Neighbor" << endl;
					break;
				}
			}

			if (isNeighbor)
				continue;

			cells.push_back(make_pair(cell.first, make_pair(x, y)));
			
		}

		for (int dy = -1; dy < 2; dy += 2)
		{
			int x = cell.second.first;
			int y = cell.second.second + dy;

			if (x < 0 || x >= ceil(image.data.cols / 2.0))
			{
				//cout << "edge" << endl;
				continue;
			}
			if (y < 0 || y >= ceil(image.data.rows / 2.0))
			{
				//cout << "edge" << endl;
				continue;
			}
			if (!image.qt[y][x].empty())
			{
				//cout << "empty" << endl;
				continue;
			}

			int isNeighbor = 0;
			for (int patch_id : image.qf[y][x])
			{
				PPatch newpatch = patches[patch_id];
				if (p.isNeighborPatch(*newpatch))
				{
					isNeighbor = 1;
					//cout << "Neighbor" << endl;
					break;
				}
			}

			if (isNeighbor)
				continue;

			cells.push_back(make_pair(cell.first, make_pair(x, y)));

		}
	}
}

void PMVS::findNeighbors8(Patch p, vector<PPatch>& ppatches)
{
	for (pair<int, pair<int, int>>&cell : p.qs)
	{
		Image &image = images[cell.first];
		int x = cell.second.first;
		int y = cell.second.second;
		if (x < 1 || x >= ceil(image.data.cols / 2.0) - 1)
		{
			continue;
		}
		if (y < 1 || y >= ceil(image.data.rows / 2.0) - 1)
		{
			continue;
		}
		for (int dx = -1; dx < 2; dx++)
		{
			for (int dy = -1; dy < 2; dy++)
			{
				if (dx == 0 && dy == 0)
					continue;

				int x_ = x + dx;
				int y_ = y + dy;


				for (int idx : image.qf[y_][x_])
				{
					if (patches[idx] == NULL)
						continue;
					ppatches.push_back(patches[idx]);
				}
			}
		}
	}
}

void PMVS::filter()
{
	for (PPatch &p:patches)
	{
		vector<PPatch> Up;
		for (pair<int,pair<int,int>>cell:p->qs)
		{
			for (int idx : images[cell.first].qf[cell.second.second][cell.second.first])
			{
				PPatch pp = patches[idx];
				if (pp == NULL)
					continue;
				if (p->isNeighborPatch(*pp))
					continue;
				Up.push_back(pp);
			}
		}
		double a = p->timages.size()*p->averageCost();
		double sum = 0;
		for (PPatch &pp:Up)
		{
			sum += pp->averageCost();
		}
		if (a<sum)
		{
			p = NULL;
			cout << "delete a patch" << endl;
		}
		else
		{
			vector<PPatch> Neighbors8CellsPatches;
			findNeighbors8(*p, Neighbors8CellsPatches);
			int num = 0;
			for (PPatch Neighbors8CellsPatche : Neighbors8CellsPatches)
			{
				if (p->isNeighborPatch(*Neighbors8CellsPatche))
					num++;
			}
			float rate = (float)num / Neighbors8CellsPatches.size();
			if (rate < 2.5)
			{
				p = NULL;
				cout << "delete a patch" << endl;
			}
		}
	}
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
		if (patches[i] == NULL)
			continue;
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
