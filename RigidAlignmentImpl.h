#pragma once
#include <algorithm>
#include <vector>
#include <iostream>
#include <cstring>
#include <map>
#include <iterator>
#include "Mesh.h"
#include "newuoa.h"
#include "Geom.h"

using namespace std;

class RigidAlignment
{
public:
  RigidAlignment(void);
  RigidAlignment(const char *landmarkDir, vector<char *> landmarkList, const char *sphere, const char *outdir = NULL, bool lmCoordType = false);
  RigidAlignment(std::map<std::string, std::vector<int> > landmarksMap, const char *sphere, const char *outdir = NULL, bool lmCoordType = false);
  ~RigidAlignment(void);
  float cost(float *coeff);
  const float *rot(void);
  void saveSphere(const char *dir);
  void saveLM(const char *lm);

private:
  void setup(const char *landmarkDir, vector<char *> landmarkList, const char *sphere);
  void setup(std::map<std::string, std::vector<int> > landmarksMap, const char *sphere);
  void setup3f(const char *landmarkDir, vector<char *> landmarkList, const char *sphere);
  void readPoint(const char *filename);
  void readPoint3f(const char *filename);
  void optimization(void);
  void update(void);
  void updateAxis(const float phi, const float theta, const float *axis_old, float *axis_new);
  float landmarkVariance(void);

private:
  vector<vector<int> > m_point;
  // vector<const char *> m_filename;
  vector<string> m_filename;
  Mesh *m_sphere;
  char m_spherename[255];
  float *m_rot;

  int nIter = 0;
  int m_nSubj;
  int m_nLM;

  // workspace
  float *fpoint;  // landmarks
  float *fmean; // mean for landmarks
  float *faxis; // center of subject
};

class cost_function
{
public:
    cost_function (RigidAlignment *instance)
    {
        m_instance = instance;
    }

    double operator () (float *arg)
    {
    float cost = m_instance->cost(arg);
        return (double)cost;
    }

private:
  RigidAlignment *m_instance;
};
