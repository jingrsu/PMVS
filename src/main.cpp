#include "PMVS.h"

using namespace std;

int main()
{
	PMVS pmvs("../images/imagesparameters.txt", "../images");

	pmvs.seed();
	pmvs.savePatches("a.ply");

	system("pause");
}