#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include "glm/glm.hpp"
#include "utilities.h"
#include "sceneStructs.h"
#include "objmesh.h"

using namespace std;

class Scene {
private:
    ifstream fp_in;
    int loadMaterial(string materialid);
    int loadGeom(string objectid);
    int loadCamera();
public:
    Scene(string filename);
    ~Scene();
    void loadObj(string filepath, string mtlpath);

    std::vector<Geom> geoms;
    std::vector<Material> materials;
    RenderState state;
    
    ObjMesh* objmesh;
    int obj_numshapes;
    int* obj_numpolyverts;
    int** obj_polysidx;
    float* obj_verts;
    float* obj_norms;
    float* obj_texts;
    float* obj_bboxes;
    // shading
    float* obj_RGB;
    float* obj_SPECEX;
    float* obj_SPECRGB;
    float* obj_REFL;
    float* obj_REFR;
    float* obj_REFRIOR;
    int* obj_materialOffsets;
    bool hasObj;

    int* obj_polyoffsets;
    int* obj_polysidxflat;
    int polyidxcount;
};
