#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <stdlib.h>
#include "glm/glm.hpp"
#include "utilities.h"
#include "sceneStructs.h"
#include "objmesh.h"
#include "KDnode.h"
#include "KDtree.h"

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
    void getKDnodes_(KDN::KDnode* root, std::vector<KDN::KDnode*>& nodes);
    void getKDnodesLoop_(KDN::KDnode* root, std::vector<KDN::KDnode*>& nodes);
    void getKDnodesLoopDeref_(KDN::KDnode* root, vector<KDN::KDnode>& nodes);
    std::vector<int> cacheTriangles_(KDN::KDnode* nodes, int numNodes, std::vector<KDN::Triangle>& newTriangles);
    std::vector<int> cacheTriangles_(std::vector<KDN::KDnode*> nodes, std::vector<KDN::Triangle>& newTriangles);
    std::vector<int> cacheTriangles_(std::vector<KDN::KDnode> nodes, std::vector<KDN::Triangle>& newTriangles);
    void deleteTree_(KDN::KDnode* root);
    //bool nodeComparator_(const void* a, const void* b);
    std::vector<KDN::Triangle*> getTrianglesFromScene_(void);
    void loadObj(string filepath, string mtlpath);
    void cacheNodesBare();
    void cacheTrianglesBare();

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

    // KD
    int numNodes;
    int numTriangles;
    std::vector<KDN::Triangle*> triangles;
    KDtree* KDT;
    std::vector<KDN::KDnode*> nodes;
    std::vector<KDN::Triangle> Triangles;
    std::vector<int> offsets;
    std::vector<KDN::KDnode*> nodesLoop;
    std::vector<KDN::KDnode> nodesLoopDeref;
    KDN::KDnode* newNodes;
    KDN::Triangle* newTriangles;
    KDN::NodeBare* newNodesBare;
    KDN::TriBare* newTrianglesBare;
};
