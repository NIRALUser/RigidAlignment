#include <string>
#include <dirent.h>
#include <map>
#include <iterator>
// #include <ifstream>

#include "RigidAlignmentCLP.h"
#include "RigidAlignmentImpl.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkDoubleArray.h>
#include <vtkPolyDataReader.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyDataWriter.h>
#include <vtkPointLocator.h>


using namespace std;

#define NB_LINES 250
#define NB_WORDS 250


bool getListFile(string path, vector<string> &list, const string &suffix);
void getTrimmedList(vector<string> &list, const vector<string> &name);
void convertLandmarksToID(std::map<std::string, std::vector<int> > &landmarksMap, const vector<string> &meshList, const vector<string> &landmarkList);
void getIDlandmarks(std::string mesh, std::string landmarks, std::vector<int> &landmarkPids);




int main(int argc, char* argv[])
{
  PARSE_ARGS;

  if (argc < 3)
  {
    cout << "Usage: " << argv[0] << " --help" << endl;
    return -1;
  }


  vector<std::string> landmarkList;
  vector<std::string> meshList;

  if (inputCSV != "0")
  {
    std::ifstream ifs;
    ifs.open(inputCSV.c_str(), std::ifstream::in);
    if ( ifs.good() != true )
    {
      std::cout << " File CSV couldn't be open " << std::endl;
      return EXIT_FAILURE;
    }
    else
    {
      while ( ifs.good() )
      {
        istringstream line_stream;
        std::string line, mesh, fid;

        std::getline (ifs, line, '\n');

          line_stream.str(line); // set the input stream to line
          getline(line_stream, mesh, ',');
          getline(line_stream, fid, ',');

          std::size_t pos_vtk = mesh.find(".vtk");
          std::size_t pos_fcsv = fid.find(".fcsv");
          if ( pos_vtk <= mesh.length() && pos_fcsv <= fid.length() )
        {
          landmarkList.push_back(fid);
          meshList.push_back(mesh);
        }
      }
    }
  }
  else
  {
    // Landmarks are initially stored in a fcsv files (fiducials list from Slicer, obtained with Q3DC for example)
    if (!landmark.empty() && landmarkList.empty()) {
          if(! getListFile(landmark.c_str(), landmarkList, "fcsv"))
              return EXIT_FAILURE;
      }
    if (!mesh.empty() && meshList.empty()) {
          if(! getListFile(mesh.c_str(), meshList, "vtk"))
              return EXIT_FAILURE;
      }
  }

  // std::vector<std::string>::iterator it = landmarkList.begin(), it_end = landmarkList.end();
  // std::cout<<" --- Landmark files"<<std::endl;
  // for (; it != it_end; it ++)
  //  std::cout<<*it<<std::endl;

  // std::cout<<" --- Mesh files"<<std::endl;
  // it = meshList.begin(), it_end = meshList.end();
  // for (; it != it_end; it ++)
  //  std::cout<<*it<<std::endl;


  std::map<std::string, std::vector<int> > landmarksMap;
  convertLandmarksToID(landmarksMap, meshList, landmarkList);

  // std::map<std::string, std::vector<int> >::iterator itt = landmarksMap.begin(), itt_end = landmarksMap.end();
  // for (; itt != itt_end; itt ++)
  //  cout << itt->first <<endl;

  RigidAlignment *RAlign = new RigidAlignment(landmarksMap, sphere.c_str(), output.c_str(), lmtype);

  if (!outputLM.empty()) RAlign->saveLM(outputLM.c_str());

  delete RAlign;

  return 0;
}



bool getListFile(string path, vector<string> &list, const string &suffix)
{

    DIR *dir = opendir(path.c_str());
    if (dir != NULL)
    {
        while (dirent *entry = readdir(dir))
        {
            string filename = entry->d_name;
            if(filename.find(suffix) != string::npos && filename.find_last_of(suffix) == filename.size() - 1)
            {
                list.push_back(path + "/" + filename);
            }
        }
        closedir(dir);
        sort(list.begin(), list.begin() + list.size());
        return true;
    }else{

        cerr<<"The directory does not exist! "<<path<<endl;
        return false;
    }
}

void getTrimmedList(vector<string> &list, const vector<string> &name)
{
    int i = 0;
    while (i < list.size())
    {
        size_t found;
        for (int j = 0; j < name.size(); j++)
        {
            found = list[i].find(name[j].substr(name[j].rfind('/') + 1));
            if (string::npos != found) break;
        }
        if (string::npos == found) list.erase(list.begin() + i);
        else i++;
    }
    sort(list.begin(), list.begin() + list.size());
}

void convertLandmarksToID(std::map<std::string, std::vector<int> > &landmarksMap, const vector<string> &meshList, const vector<string> &landmarkList)
{
    // --------- subject names --------
    int nSubj = meshList.size();
    vector<string> subjName;
    if (nSubj > 0)
    {
        for (int i = 0; i < nSubj; i++)
        {
            int pivot = meshList[i].rfind('/') + 1;

            std::string filename = meshList[i].substr(pivot);

            std::string name;
            std::string suffixe_procalign = "_pp_surfSPHARM_procalign.vtk", suffixe_surfSPHARM = "_pp_surfSPHARM.vtk";
            int suffixe_size;

            if ( filename.substr(filename.length() - suffixe_surfSPHARM.length()) == suffixe_surfSPHARM )
                suffixe_size = suffixe_surfSPHARM.length();

            else if ( filename.substr(filename.length() - suffixe_procalign.length()) == suffixe_procalign )
                suffixe_size = suffixe_procalign.length();

            name = filename.substr(0, filename.length() - suffixe_size);
            subjName.push_back(name);
            // std::cout << "Name du subject " << i << " :: " << name << std::endl;

            // Find corresponding fiducials list
            std::vector< std::string > :: const_iterator it = landmarkList.begin(), it_end = landmarkList.end();
            for (; it != it_end; it++)
            {
              string suffix_fid = "_fid.fcsv";
              string landmarkName = (*it).substr(0, (*it).length() - suffix_fid.length());
              pivot = landmarkName.rfind('/') + 1;

              landmarkName = landmarkName.substr(pivot);
              if (name == landmarkName) // On a trouve le bon!
              {
                std::vector<int> landmarkPids;
                getIDlandmarks(meshList[i], *it, landmarkPids);
                landmarksMap[name] = landmarkPids;
                break;
              }

            }
        }
    }
}


void getIDlandmarks(std::string mesh, std::string landmarks, std::vector<int> &landmarkPids)
{
  // cout << "mesh  " << mesh.c_str() << endl;

  // Get all surface data from the file
    vtkSmartPointer<vtkPolyDataReader> surfacereader = vtkSmartPointer<vtkPolyDataReader>::New();
    surfacereader->SetFileName(mesh.c_str());
    surfacereader->Update();

  vtkPolyData* inputPolyData = surfacereader->GetOutput();
  // std::cout << "Input surface has " << inputPolyData->GetNumberOfPoints() << " points." << std::endl;

  // Build a locator
  vtkPointLocator *pointLocator = vtkPointLocator::New();
  pointLocator->SetDataSet(inputPolyData);
  pointLocator->BuildLocator();

  // -------------------- Reading FCSV file --------------------

  // Get the Surface filename from the command line
  std::fstream fcsvfile(landmarks.c_str());
  std::string line, mot;
  std::string words[NB_LINES][NB_WORDS]; // !!!! WARNING DEFINE AND TO PROTECT IF SUPERIOR TO 20
  int i,j, NbPoints;

  if(fcsvfile)
  {
    getline(fcsvfile, line);
    fcsvfile>>mot;
    while(mot=="#")
    {
      if(getline(fcsvfile, line))
        fcsvfile>>mot;
      else
        mot="#";
    }

    i=0;
    do
    {

      std::size_t pos_end;// = mot.find(",,");
      std::size_t pos1;
      j=0;
      do
      {
        std::size_t pos0 = 0;
        pos1 = mot.find(',');
        pos_end = mot.find(",,");
        words[i][j] = mot.substr(pos0, pos1-pos0);
        mot = mot.substr(pos1+1);
        j++;
      } while(pos1+1<pos_end);
      i++;
    } while(fcsvfile>>mot);

      NbPoints = i;
    for (int i = 0; i < NbPoints; ++i)
    {
      double x = atof(words[i][1].c_str());
      double y = atof(words[i][2].c_str());
      double z = atof(words[i][3].c_str());

                      // Find closest point
                      vtkIdType ptId;
                      double p[] = {0.0, 0.0, 0.0};
                      p[0] = x; p[1] = y; p[2] = z;
                      ptId = pointLocator->FindClosestPoint(p);
                      landmarkPids.push_back(ptId);
    }
  }
  else
    std::cout<<"Error !";

}



