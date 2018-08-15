#include "AABB.h"
#include "RigidAlignmentImpl.h"

RigidAlignment::RigidAlignment(void)
{
}

RigidAlignment::RigidAlignment(const char *landmarkDir, vector<char *> landmarkList, const char *sphere, const char *outdir, bool lmCoordType)
{
  strcpy(m_spherename, sphere);
  cout << "Loading Files..\n";
  if (!lmCoordType) setup(landmarkDir, landmarkList, sphere);
  else setup3f(landmarkDir, landmarkList, sphere);
  update();

  cout << "Optimziation\n";
  optimization();

  if (outdir != NULL)
  {
    cout << "Saving Aligned Spheres..\n";
    saveSphere(outdir);
  }
}

RigidAlignment::RigidAlignment(std::map<std::string, std::vector<int> > landmarksMap, const char *sphere, const char *outdir, bool lmCoordType)
{
  strcpy(m_spherename, sphere);
  cout << "Loading Files..\n";
  setup(landmarksMap, sphere);
  update();

  cout << "Optimziation\n";
  optimization();

  if (outdir != NULL)
  {
    cout << "Saving Aligned Spheres..\n";
    saveSphere(outdir);
  }
}

RigidAlignment::~RigidAlignment(void)
{
  delete m_sphere;
  delete [] m_rot;
  delete [] fpoint;
  delete [] fmean;
  delete [] faxis;
}

void RigidAlignment::setup(const char *landmarkDir, vector<char *> landmarkList, const char *sphere)
{
  m_sphere = new Mesh();
  m_sphere->openFile(sphere);

  m_nSubj = landmarkList.size();
  for (int i = 0; i < m_nSubj; i++)
  {
    char fullpath[1024];
    sprintf(fullpath, "%s/%s", landmarkDir, landmarkList[i]);
    cout << "[" << i << "] " << landmarkList[i] << endl;
    readPoint(fullpath);
    m_filename.push_back(landmarkList[i]);
  }

  // rotation angle
  m_rot = new float[m_nSubj * 3];
  memset(m_rot, 0, sizeof(float) * m_nSubj * 3);
  m_nLM = m_point[0].size();

  // workspace
  fpoint = new float[m_nSubj * m_nLM * 3];
  fmean = new float[m_nLM * 3];
  faxis = new float[m_nSubj * 3];

  // axis
  memset(faxis, 0, sizeof(float) * m_nSubj * 3);
  for (int i = 0; i < m_nSubj; i++)
  {
    float *axis = &faxis[i * 3];
    for (int j = 0; j < m_nLM; j++)
    {
      float *p = &fpoint[(i * m_nLM + j) * 3];
      int id = m_point[i][j];
      memcpy(p, m_sphere->vertex(id)->fv(), sizeof(float) * 3);
      for (int k = 0; k < 3; k++) axis[k] += p[k];
    }

    // axis
    float norm = axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2];
    norm = sqrt(norm);
    for (int k = 0; k < 3; k++) axis[k] /= norm;
  }
}

void RigidAlignment::setup(std::map<std::string, std::vector<int> > landmarksMap, const char *sphere)
{
  m_sphere = new Mesh();
  m_sphere->openFile(sphere);

  m_nSubj = landmarksMap.size();

  int i = 0;
  std::map<std::string, std::vector<int> >::iterator it = landmarksMap.begin(), it_end = landmarksMap.end();
  for (; it != it_end; it++)
  {
    // string name = ;
    cout << "[" << i << "] " << it->first << endl;
    // readPoint(fullpath);
    if (!(it->second).empty()) m_point.push_back((it->second));
    m_filename.push_back((it->first).c_str());
    i++;
  }

  // rotation angle
  m_rot = new float[m_nSubj * 3];
  memset(m_rot, 0, sizeof(float) * m_nSubj * 3);
  m_nLM = m_point[0].size();

  // workspace
  fpoint = new float[m_nSubj * m_nLM * 3];
  fmean = new float[m_nLM * 3];
  faxis = new float[m_nSubj * 3];

  // axis
  memset(faxis, 0, sizeof(float) * m_nSubj * 3);
  for (int i = 0; i < m_nSubj; i++)
  {
    float *axis = &faxis[i * 3];
    for (int j = 0; j < m_nLM; j++)
    {
      float *p = &fpoint[(i * m_nLM + j) * 3];
      int id = m_point[i][j];
      memcpy(p, m_sphere->vertex(id)->fv(), sizeof(float) * 3);
      for (int k = 0; k < 3; k++) axis[k] += p[k];
    }

    // axis
    float norm = axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2];
    norm = sqrt(norm);
    for (int k = 0; k < 3; k++) axis[k] /= norm;
  }
}

void RigidAlignment::setup3f(const char *landmarkDir, vector<char *> landmarkList, const char *sphere)
{
  m_sphere = new Mesh();
  m_sphere->openFile(sphere);

  m_nSubj = landmarkList.size();
  for (int i = 0; i < m_nSubj; i++)
  {
    char fullpath[1024];
    sprintf(fullpath, "%s/%s", landmarkDir, landmarkList[i]);
    cout << "[" << i << "] " << landmarkList[i] << endl;
    readPoint3f(fullpath);
    m_filename.push_back(landmarkList[i]);
  }

  // rotation angle
  m_rot = new float[m_nSubj * 3];
  memset(m_rot, 0, sizeof(float) * m_nSubj * 3);
  m_nLM = m_point[0].size();

  // workspace
  fpoint = new float[m_nSubj * m_nLM * 3];
  fmean = new float[m_nLM * 3];
  faxis = new float[m_nSubj * 3];

  // axis
  memset(faxis, 0, sizeof(float) * m_nSubj * 3);
  for (int i = 0; i < m_nSubj; i++)
  {
    float *axis = &faxis[i * 3];
    for (int j = 0; j < m_nLM; j++)
    {
      float *p = &fpoint[(i * m_nLM + j) * 3];
      int id = m_point[i][j];
      memcpy(p, m_sphere->vertex(id)->fv(), sizeof(float) * 3);
      for (int k = 0; k < 3; k++) axis[k] += p[k];
    }

    // axis
    float norm = axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2];
    norm = sqrt(norm);
    for (int k = 0; k < 3; k++) axis[k] /= norm;
  }
}

void RigidAlignment::readPoint(const char *filename)
{
  int i = 0;
  FILE *fp = fopen(filename,"r");
  vector<int> point;
  while (!feof(fp))
  {
    int id;
    fscanf(fp, "%d", &id);
    point.push_back(id);
  }
  point.pop_back();
  fclose(fp);

  if (!point.empty()) m_point.push_back(point);
}

void RigidAlignment::readPoint3f(const char *filename)
{
  AABB *tree = new AABB(m_sphere);
  int i = 0;
  FILE *fp = fopen(filename,"r");
  vector<int> point;
  while (!feof(fp))
  {
    float v[3];
    fscanf(fp, "%f %f %f", &v[0], &v[1], &v[2]);
    float coeffs[3];

    // find the closest face
    int fid = tree->closestFace(v, coeffs, 0.01);
    Face *f = (Face *)m_sphere->face(fid);
    const int *vid = f->list();
    int id = vid[0];

    // find the closest vertex
    if (coeffs[1] >= coeffs[2] && coeffs[1] >= coeffs[0]) id = vid[1];
    else if (coeffs[2] >= coeffs[0] && coeffs[2] >= coeffs[1]) id = vid[2];
    point.push_back(id);
  }
  point.pop_back();
  fclose(fp);

  if (!point.empty()) m_point.push_back(point);

  delete tree;
}

float RigidAlignment::landmarkVariance(void)
{
  float cost = 0;
  for (int i = 0; i < m_nLM; i++)
  {
    float sd = 0;
    for (int j = 0; j < m_nSubj; j++)
    {
      float inner = 0;
      float norm1 = 0;
      float norm2 = 0;
      for (int k = 0; k < 3; k++)
      {
        inner += fmean[i * 3 + k] * fpoint[(j * m_nLM + i) * 3 + k];
        norm1 += fmean[i * 3 + k] * fmean[i * 3 + k];
        norm2 += fpoint[(j * m_nLM + i) * 3 + k] * fpoint[(j * m_nLM + i) * 3 + k];
      }
      norm1 = sqrt(norm1);
      norm2 = sqrt(norm2);
      inner /= norm1 * norm2;
      if (inner > 1) inner = 1;
      else if (inner < -1) inner = -1;
      float arclen = acos(inner);
      sd += arclen * arclen;
    }
    cost += sqrt(sd);
  }

  return cost;
}

void RigidAlignment::update(void)
{
  // new point
  for (int i = 0; i < m_nSubj; i++)
  {
    const float *axis = &faxis[i * 3];

    // new axis
    float axis2[3];
    updateAxis(m_rot[i * 3 + 1], m_rot[i * 3 + 2], axis, axis2);

    Vector ax(axis), ax2(axis2);
    float inner = ax * ax2;
    if (inner > 1) inner = 1;
    else if (inner < -1) inner = -1;
    float deg = acos(inner);

    // matrix
    float mat[9];
    Vector ax3 = ax.cross(ax2); ax3.unit();
    if (ax3.norm() != 0)
    {
      Coordinate::rotation(ax3.fv(), deg, mat);

      // axis rotation
      for (int j = 0; j < m_nLM; j++)
      {
        float newp[3];
        float *p = &fpoint[(i * m_nLM + j) * 3];
        int id = m_point[i][j];
        memcpy(p, m_sphere->vertex(id)->fv(), sizeof(float) * 3);
        Coordinate::rotPoint(p, mat, newp);
        memcpy(p, newp, sizeof(float) * 3);
      }
    }

    // matrix
    Coordinate::rotation(axis2, m_rot[i * 3], mat);

    // rotation
    for (int j = 0; j < m_nLM; j++)
    {
      float newp[3];
      float *p = &fpoint[(i * m_nLM + j) * 3];
      Coordinate::rotPoint(p, mat, newp);
      memcpy(p, newp, sizeof(float) * 3);
    }
  }

  // mean
  memset(fmean, 0, sizeof(float) * m_nLM * 3);
  for (int i = 0; i < m_nLM; i++)
    for (int j = 0; j < m_nSubj; j++)
      for (int k = 0; k < 3; k++)
        fmean[i * 3 + k] += fpoint[(j * m_nLM + i) * 3 + k] / m_nSubj;
  for (int i = 0; i < m_nLM; i++)
  {
    float norm = fmean[i * 3] * fmean[i * 3] + fmean[i * 3 + 1] * fmean[i * 3 + 1] + fmean[i * 3 + 2] * fmean[i * 3 + 2];
    norm = sqrt(norm);
    for (int j = 0; j < 3; j++)
      fmean[i * 3 + j] /= norm;
  }

  /*for (int i = 0; i < m_nLM; i++)
    cout << fmean[i * 3] << " " << fmean[i * 3 + 1] << " " << fmean[i * 3 + 2] << endl;*/
}

void RigidAlignment::optimization(void)
{
  cost_function costFunc(this);
  nIter = 0;
  min_newuoa(m_nSubj * 3, m_rot, costFunc, (float)M_PI, 1e-6f, 20000);
}

float RigidAlignment::cost(float *coeff)
{
  nIter++;

  update();
  float cost = landmarkVariance();

  //
  if (nIter % 10 == 0)
    cout << "[" << nIter << "] " << cost << endl;

  return cost;
}

const float * RigidAlignment::rot(void)
{
  return m_rot;
}

void RigidAlignment::saveSphere(const char *dir)
{
  for (int i = 0; i < m_nSubj; i++)
  {
    const float *axis = &faxis[i * 3];
    float axis2[3];
    updateAxis(m_rot[i * 3 + 1], m_rot[i * 3 + 2], axis, axis2);

    Vector ax(axis), ax2(axis2);
    float inner = ax * ax2;
    if (inner > 1) inner = 1;
    else if (inner < -1) inner = -1;
    float deg = acos(inner);

    Mesh *sphere = new Mesh();
    sphere->openFile(m_spherename);

    // matrix
    Vector ax3 = ax.cross(ax2); ax3.unit();
    if (ax3.norm() != 0)
      sphere->rotation(ax3.fv(), deg);
    sphere->rotation(axis2, m_rot[i * 3]);

    // output

    // char filename[1024];
    // sprintf(filename, "%s/%s.vtk", dir, m_filename[i]);

    string filename;
    string temp = "/";
    string temp2 = "_rotSphere.vtk";
    filename = dir + temp + m_filename[i] + temp2;

    // cout << "m_filename " << i << " :: " << m_filename[i] << endl;
    sphere->saveFile(filename.c_str(), "vtk");

    delete sphere;
  }
}

void RigidAlignment::saveLM(const char *dir)
{
  for (int i = 0; i < m_nSubj; i++)
  {
    // char filename[1024];
    // sprintf(filename, "%s/%s.txt", dir, m_filename[i]);
    string filename;
    string temp = "/";
    string temp2 = ".txt";
    filename = dir + temp + m_filename[i] + temp2;

    FILE *fp = fopen(filename.c_str(), "w");
    for (int j = 0; j < m_nLM; j++)
    {
      int id = m_point[i][j];
      fprintf(fp, "%d\n", id);
    }
    fclose(fp);
  }
}

void RigidAlignment::updateAxis(const float phi, const float theta, const float *axis_old, float *axis_new)
{
  float phi_, theta_;
  Coordinate::cart2sph(axis_old, &phi_, &theta_);
  phi_ += phi;
  theta_ += theta;
  Coordinate::sph2cart(phi_, theta_, axis_new);
}
