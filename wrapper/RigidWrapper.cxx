#include <string>
#include <dirent.h>

#include "RigidWrapperCLP.h"
#include "RigidAlignment.h"

using namespace std;

int main(int argc, char* argv[])
{
	PARSE_ARGS;
	
	if (argc < 3)
	{
		cout << "Usage: " << argv[0] << " --help" << endl;
		return -1;
	}
	
	DIR *dir;
	dirent *pdir;
	vector<char *> landmarkList;

	dir = opendir(landmark.c_str());

	while (pdir = readdir(dir))
	{
		if ('.' == pdir->d_name[0]) continue;
		landmarkList.push_back(pdir->d_name);
	}

	RigidAlignment *RAlign = new RigidAlignment(landmark.c_str(), landmarkList, sphere.c_str(), output.c_str(), lmtype);
	
	if (!outputLM.empty()) RAlign->saveLM(outputLM.c_str());
	
	delete RAlign;
}
