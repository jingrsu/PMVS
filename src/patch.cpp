#include "patch.h"
#include "Utils.h"

Patch::Patch()
{
	center = Mat(4, 1, CV_64F);
	normal = Mat(4, 1, CV_64F);
	ray = Mat(4, 1, CV_64F);
}

void Patch::updateImage(double alpha1, double alpha2)
{
	timages.clear();
	simages.clear();
	Tex pTex1;
	Mat_<double> pxaxis(4, 1), pyaxis(4, 1);
	getPAxes(pxaxis, pyaxis);
	rimage->grabTex(center, pxaxis, pyaxis, pTex1);

	for (Image *nimage:rimage->nimages)
	{
		if (isInTheImage(nimage))
		{
			simages.push_back(nimage);

			Tex pTex2;
			nimage->grabTex(center, pxaxis, pyaxis, pTex2);
			double v = pTex1.ncc(pTex2);

			if (v > alpha1)
			{
				timages.push_back(nimage);
			}
		}
	}
}

void Patch::encode(double &depth, double &alpha, double &beta)const
{
	//detph：patch中心到相机的距离
	//alpha：patch法线与相机Y轴的夹角(-π/2,π/2)
	//beta：patch法线在XOZ平面上的投影与相机Z轴的夹角(-π/2,π/2)
	depth = rimage->getDistanceToCameraCenter(center);

	double fx = rimage->xaxis.dot(normal);
	double fy = rimage->yaxis.dot(normal);
	double fz = rimage->zaxis.dot(normal);
	alpha = asin(fy);
	double cosAlpha = cos(alpha);
	double sinBeta = fx / cosAlpha; //fx=sinβ*cosα
	double cosBeta = -fz / cosAlpha; //-fz=cosβ*cosα
	beta = acos(cosBeta);
	if (sinBeta<0) {
		beta = -beta;
	}
	//cout<<sin(beta)*cos(alpha)-fx<<endl;
	//cout<<sin(alpha)-fy<<endl;
	//cout<<-cos(beta)*cos(alpha)-fz<<endl;
	//cout<<depth<<" "<<alpha<<" "<<beta<<endl;

}

void Patch::decode(double depth, double alpha, double beta)
{

	double fx = sin(beta)*cos(alpha);
	double fy = sin(alpha);
	double fz = -cos(beta)*cos(alpha);

	normal = fx * rimage->xaxis + fy * rimage->yaxis + fz * rimage->zaxis;
	normal.at<double>(3, 0) = 0;
	center = rimage->cameraCenter + ray * depth;
}

void Patch::getPAxes(Mat &pxaxis, Mat &pyaxis)const
{

	Image &image = *rimage;
	Vec3d zaxis(normal.at<double>(0, 0), normal.at<double>(1, 0), normal.at<double>(2, 0));
	Vec3d xaxis = Vec3d(image.xaxis.at<double>(0, 0), image.xaxis.at<double>(1, 0), image.xaxis.at<double>(2, 0));
	Vec3d yaxis = zaxis.cross(xaxis);
	yaxis = yaxis / norm(yaxis);
	xaxis = yaxis.cross(zaxis);
	double depth = norm(center - image.cameraCenter);
	double scale = 2 * depth / (image.kmat.at<double>(0, 0) + image.kmat.at<double>(1, 1));
	pxaxis.at<double>(0, 0) = xaxis[0];
	pxaxis.at<double>(1, 0) = xaxis[1];
	pxaxis.at<double>(2, 0) = xaxis[2];
	pxaxis.at<double>(3, 0) = 0;
	pyaxis.at<double>(0, 0) = yaxis[0];
	pyaxis.at<double>(1, 0) = yaxis[1];
	pyaxis.at<double>(2, 0) = yaxis[2];
	pyaxis.at<double>(3, 0) = 0;
	pxaxis = pxaxis * scale;
	pyaxis = pyaxis * scale;
	double xdis = norm(image.project(center + pxaxis) - image.project(center));
	double ydis = norm(image.project(center + pyaxis) - image.project(center));
	pxaxis = pxaxis / xdis;
	pyaxis = pyaxis / ydis;

}

double Patch::averageCost()const
{
	if (timages.size() == 0) {
		return -1;
	}
	Tex pTex1;
	Mat_<double> pxaxis(4, 1), pyaxis(4, 1);
	getPAxes(pxaxis, pyaxis);
	rimage->grabTex(center, pxaxis, pyaxis, pTex1);

	double sum = 0;
	for (Image *timage : timages)
	{
		Tex pTex2;
		timage->grabTex(center, pxaxis, pyaxis, pTex2);
		sum += cost(pTex1, pTex2);
	}

	double ret = sum / timages.size();
	if (std::isnan(ret) || std::isinf(ret))
		return -1;
	return ret;
}

double Patch::cost(const Tex &tex1, const Tex &tex2) const
{
	return tex1.ncc(tex2);
}

double optimizeFun(const vector<double> &x, vector<double> &grad, void *fdata)
{
	Patch p;
	Patch *oldP = (Patch*)fdata;
	p.timages = oldP->timages;
	p.rimage = oldP->rimage;
	p.ray = p.ray;
	p.decode(x[0], x[1], x[2]);
	return p.averageCost();

}

void Patch::optimze()
{
	double depth, alpha, beta;
	ray = center - rimage->cameraCenter;
	ray = ray / norm(ray);
	encode(depth, alpha, beta);
	nlopt::opt opt(nlopt::LN_BOBYQA, 3);
	opt.set_max_objective(optimizeFun, this);
	opt.set_xtol_rel(1.e-7);
	opt.set_maxeval(100);
	vector<double> x(3), lb(3), ub(3);
	lb[0] = 0.1; //HUGE_VAL极大值
	lb[1] = -M_PI_2 / 3; //-pi/2
	lb[2] = -M_PI_2 / 3; //-pi/2
	ub[0] = HUGE_VAL;
	ub[1] = M_PI_2 / 3;
	ub[2] = M_PI_2 / 3;
	x[0] = depth;
	x[1] = alpha;
	x[2] = beta;
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);
	bool success = false;
	double maxf = averageCost();
	try {
		nlopt::result result = opt.optimize(x, maxf);
		success = (result == nlopt::SUCCESS
			|| result == nlopt::STOPVAL_REACHED
			|| result == nlopt::FTOL_REACHED
			|| result == nlopt::XTOL_REACHED);
		//cout<<"maxf:"<<maxf<<endl;
	}
	catch (std::exception &e)
	{
		success = false;
	}
	if (success) {
		decode(x[0], x[1], x[2]);
	}

}

void Patch::updateImageCell(int pid)
{
	Tex pTex1;
	Mat_<double> pxaxis(4, 1), pyaxis(4, 1);
	getPAxes(pxaxis, pyaxis);
	rimage->grabTex(center, pxaxis, pyaxis, pTex1);
	pTex1.updateCell(pid, rimage->qt);
	//    for(Image *nimage:rimage->nimages)
	//    {
	//        if(find(timages.begin(), timages.end(), nimage)!=timages.end())
	//        {
	//            
	//            Tex pTex2;
	//            nimage->grabTex(center,pxaxis,pyaxis,pTex2);
	//            pTex2.updateCell(pid, nimage->qt);
	//            
	//        }else if(find(simages.begin(), simages.end(), nimage)!=simages.end()){
	//            
	//            Tex pTex2;
	//            nimage->grabTex(center,pxaxis,pyaxis,pTex2);
	//            pTex2.updateCell(pid, nimage->qf);
	//        }
	//        
	//    }
}

bool Patch::isInTheImage(Image* image)
{
	Mat_<double> v(4, 1);
	v = image->cameraCenter - center;
	double ret = v.dot(normal) / (norm(v)*norm(normal));
	if (ret > 0.0)
		return true;
	return false;
}