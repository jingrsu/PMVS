#include "PMVS.h"

using namespace std;

int main()
{
	PMVS pmvs("../images/imagesParameters.txt", "../images");

	pmvs.seed();
	pmvs.expand();
	//pmvs.filter();
	//pmvs.expand();
	pmvs.savePatches("b.ply");

	system("pause");
}