#include <string>
#include <algorithm>
#include <map>
#include <iterator>

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
#include <vtksys/Directory.hxx>
#include <vtksys/SystemTools.hxx>

using namespace std;

#define NB_LINES 250
#define NB_WORDS 250


bool getListFile(const string& path, vector<string> &list, const string &suffix);
std::map<std::string, std::string> convertLandmarksToID(std::map<std::string, std::vector<int> > &landmarksMap, const vector<string> &meshList, const vector<string> &landmarkList, const string& suffixe_procalign = "_pp_surfSPHARM_procalign.vtk", const string& suffixe_surfSPHARM = "_pp_surfSPHARM.vtk");
void getIDlandmarks(const std::string& mesh, const std::string& landmarks, std::vector<int> &landmarkPids);


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
      std::cerr << " File CSV couldn't be open " << std::endl;
      return EXIT_FAILURE;
    }
    else
    {
      while ( ifs.good() )
      {
        istringstream line_stream;
        std::string line, mesh, fid;

        std::getline (ifs, line);

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

  std::map<std::string, std::vector<int> > landmarksMap;
  std::map<std::string, std::string > matchedNamesMap =  convertLandmarksToID(landmarksMap, meshList, landmarkList, suffixe_procalign, suffixe_surfSPHARM);

  if(landmarksMap.size() == 0){
    cerr<<"No matched landmark files and mesh. Please check that the suffix from SPHARM files match the inputs and that landmark files have the right extensions."<<endl;
    return 1;
  }
  std::map<std::string, std::string >::iterator itt = matchedNamesMap.begin(), itt_end = matchedNamesMap.end();
  cout <<endl<<"Matched files with landmarks:"<<endl;
  for (; itt != itt_end; itt ++)
    cout << itt->first << " " << itt->second << endl;
  cout << endl;

  std::unique_ptr<RigidAlignment> RAlign(new RigidAlignment(landmarksMap, sphere.c_str(), output.c_str(), lmtype));

  if (!outputLM.empty()) RAlign->saveLM(outputLM.c_str());

  return 0;
}

bool endsWith(const string& value, const string& suffix) {
  return value.size() >= suffix.size() && 0 == value.compare(value.size() - suffix.size(), suffix.size(), suffix);
}

bool getListFile(const string& path, vector<string> &list, const string &suffix)
{
  vtksys::Directory dir;
  std::string errorMessage = "";
  dir.Load(path.c_str(), &errorMessage);
  if (errorMessage != "")
  {
    cerr << "Failed to list directory " << path << ": " << errorMessage << endl;
    return false;
  }
  for (unsigned long fileNum=0; fileNum < dir.GetNumberOfFiles(); ++fileNum)
  {
    std::string filename = std::string(dir.GetFile(fileNum));
    std::string filepath = path + "/" + filename;
    if (vtksys::SystemTools::FileIsDirectory(filepath))
    {
      continue;
    }
    if (endsWith(filename, suffix))
    {
      list.push_back(filepath);
    }
  }
  sort(list.begin(), list.end());
  return true;
}

std::string getFilenameComponent(const std::string& filepath)
{
  std::string fixedFilepath = filepath;
  // Move all slashes to forward slash
  std::replace(fixedFilepath.begin(), fixedFilepath.end(), '\\', '/');
  int pivot = fixedFilepath.rfind('/') + 1;
  return fixedFilepath.substr(pivot);
}

std::map<std::string, std::string> convertLandmarksToID(std::map<std::string, std::vector<int> > &landmarksMap, const vector<string> &meshList, const vector<string> &landmarkList, const string& suffixe_procalign, const string& suffixe_surfSPHARM)
{
  // --------- subject names --------
  int nSubj = meshList.size();
  vector<string> subjName;
  std::map<std::string, std::string> matchedNames;
  if (nSubj > 0)
  {
    for (int i = 0; i < nSubj; i++)
    {
      std::string filename = getFilenameComponent(meshList[i]);

      int suffixe_size = 0;

      if ( filename.substr(filename.length() - suffixe_surfSPHARM.length()).compare(suffixe_surfSPHARM) == 0 )
        suffixe_size = suffixe_surfSPHARM.length();

      else if ( filename.substr(filename.length() - suffixe_procalign.length()).compare(suffixe_procalign) == 0 )
        suffixe_size = suffixe_procalign.length();

      std::string name = filename.substr(0, filename.length() - suffixe_size);
      subjName.push_back(name);

      // Find corresponding fiducials list
      std::vector< std::string > :: const_iterator it = landmarkList.begin(), it_end = landmarkList.end();
      for (; it != it_end; it++)
      {
        string suffix_fid = "_fid.fcsv";
        string landmarkName = (*it).substr(0, (*it).length() - suffix_fid.length());
        landmarkName = getFilenameComponent(landmarkName);
        if (name == landmarkName) // On a trouve le bon!
        {
          std::vector<int> landmarkPids;
          getIDlandmarks(meshList[i], *it, landmarkPids);
          landmarksMap[name] = landmarkPids;
          matchedNames[name] = landmarkName;
          break;
        }
      }
    }
  }
  else
  {
    std::cerr << "No subjects found" << std::endl;
  }
  return matchedNames;
}

void getIDlandmarks(const std::string& mesh, const std::string& landmarks, std::vector<int> &landmarkPids)
{
  // Get all surface data from the file
  vtkSmartPointer<vtkPolyDataReader> surfacereader = vtkSmartPointer<vtkPolyDataReader>::New();
  surfacereader->SetFileName(mesh.c_str());
  surfacereader->Update();

  vtkPolyData* inputPolyData = surfacereader->GetOutput();

  // Build a locator
  vtkSmartPointer<vtkPointLocator> pointLocator = vtkSmartPointer<vtkPointLocator>::New();
  pointLocator->SetDataSet(inputPolyData);
  pointLocator->BuildLocator();

  // -------------------- Reading FCSV file --------------------

  // Get the Surface filename from the command line
  std::fstream fcsvfile(landmarks.c_str());
  std::string line, mot;
  std::vector< std::vector< std::string > > words(NB_LINES); // !!!! WARNING DEFINE AND TO PROTECT IF SUPERIOR TO 20
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
      words[i].resize(NB_WORDS);
      std::size_t pos_end;
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
