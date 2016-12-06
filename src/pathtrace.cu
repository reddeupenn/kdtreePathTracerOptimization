#include <cstdio>
#include <cuda.h>
#include <cmath>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/remove.h>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/zip_iterator.h>

#include "sceneStructs.h"
#include "scene.h"
#include "glm/glm.hpp"
#include "glm/gtx/norm.hpp"
#include <glm/gtc/matrix_inverse.hpp>
#include "utilities.h"
#include "pathtrace.h"
#include "intersections.h"
#include "interactions.h"

#include <algorithm>
#include <stdlib.h>

#include <random>
#include <vector>
#include <stack>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <glm/gtx/intersect.hpp>

#include "KDnode.h"
#include "KDtree.h"

#define ERRORCHECK 1

#define FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define checkCUDAError(msg) checkCUDAErrorFn(msg, FILENAME, __LINE__)
void checkCUDAErrorFn(const char *msg, const char *file, int line) {
#if ERRORCHECK
    cudaDeviceSynchronize();
    cudaError_t err = cudaGetLastError();
    if (cudaSuccess == err) {
        return;
    }

    fprintf(stderr, "CUDA error");
    if (file) {
        fprintf(stderr, " (%s:%d)", file, line);
    }
    fprintf(stderr, ": %s: %s\n", msg, cudaGetErrorString(err));
#  ifdef _WIN32
    getchar();
#  endif
    exit(EXIT_FAILURE);
#endif
}

__host__ __device__
thrust::default_random_engine makeSeededRandomEngine(int iter, int index, int depth) {
    int h = utilhash((1 << 31) | (depth << 22) | iter) ^ utilhash(index);
    return thrust::default_random_engine(h);
}

//Kernel that writes the image to the OpenGL PBO directly.
__global__ void sendImageToPBO(uchar4* pbo, glm::ivec2 resolution,
    int iter, glm::vec3* image) {
    int x = (blockIdx.x * blockDim.x) + threadIdx.x;
    int y = (blockIdx.y * blockDim.y) + threadIdx.y;

    if (x < resolution.x && y < resolution.y) {
        int index = x + (y * resolution.x);
        glm::vec3 pix = image[index];

        glm::ivec3 color;
        color.x = glm::clamp((int)(pix.x / iter * 255.0), 0, 255);
        color.y = glm::clamp((int)(pix.y / iter * 255.0), 0, 255);
        color.z = glm::clamp((int)(pix.z / iter * 255.0), 0, 255);

        // Each thread writes one pixel location in the texture (textel)
        pbo[index].w = 0;
        pbo[index].x = color.x;
        pbo[index].y = color.y;
        pbo[index].z = color.z;
    }
}













static Scene * hst_scene = NULL;
static glm::vec3 * dev_image = NULL;
static Geom * dev_geoms = NULL;
static Material * dev_materials = NULL;
static PathSegment * dev_paths = NULL;
static PathSegment * dev_paths_cache = NULL;
static ShadeableIntersection * dev_intersections = NULL;

static const int STACK_SIZE = 2000;


struct is_zero_bounce
{
    __host__ __device__
        bool operator()(const PathSegment p)
    {
        return (p.remainingBounces == 0);
    }
};

struct by_material_id
{
    const PathSegment a;
    by_material_id(PathSegment _a) : a(_a) {}
    __host__ __device__
        int operator()(const PathSegment& x, const PathSegment& y) const
    {
        //x.
        return x.color.r + y.color.r;
    }
};

__host__ __device__ bool operator<(const PathSegment &lhs, const PathSegment &rhs)
{
    return lhs.materialIdHit < rhs.materialIdHit;
}

__host__ __device__ bool operator<(const ShadeableIntersection &lhs, const ShadeableIntersection &rhs)
{
    return lhs.materialId < rhs.materialId;
}


struct NodeStack{
    KDN::NodeBare* node;
    float tmin;
    float tmax;
    glm::vec3 origin;
};




// ------------------------------------------------------------------------
// --------------------------- KD TREE UTILITIES --------------------------
// ------------------------------------------------------------------------

std::vector<KDN::Triangle*> getTrianglesFromFile(const char* path)
{
    std::vector<KDN::Triangle*>triangles;

    string line;
    ifstream file(path);

    if (file.is_open())
    {
        while (getline(file, line))
        {
            float x1 = atof(line.c_str());
            getline(file, line); float y1 = atof(line.c_str());
            getline(file, line); float z1 = atof(line.c_str());
            getline(file, line); float x2 = atof(line.c_str());
            getline(file, line); float y2 = atof(line.c_str());
            getline(file, line); float z2 = atof(line.c_str());
            getline(file, line); float x3 = atof(line.c_str());
            getline(file, line); float y3 = atof(line.c_str());
            getline(file, line); float z3 = atof(line.c_str());

            KDN::Triangle* t = new KDN::Triangle(x1, y1, z1,
                                                 x2, y2, z2,
                                                 x3, y3, z3);
            triangles.push_back(t);
        }
    }
    return triangles;
}


/*
// fast AABB intersection
__host__ __device__ //__global__
void intersectAABB(Ray r, KDN::BoundingBox b, float& dist, bool& result)
{

    glm::vec3 invdir(1.0f / r.direction.x,
                     1.0f / r.direction.y,
                     1.0f / r.direction.z);

    float v1 = (b.mins[0] - r.origin.x)*invdir.x;
    float v2 = (b.maxs[0] - r.origin.x)*invdir.x;
    float v3 = (b.mins[1] - r.origin.y)*invdir.y;
    float v4 = (b.maxs[1] - r.origin.y)*invdir.y;
    float v5 = (b.mins[2] - r.origin.z)*invdir.z;
    float v6 = (b.maxs[2] - r.origin.z)*invdir.z;

    float dmin = max(max(min(v1, v2), min(v3, v4)), min(v5, v6));
    float dmax = min(min(max(v1, v2), max(v3, v4)), max(v5, v6));

    if (dmax < 0)
    {
        dist = dmax;
        result = false;
        return;
    }
    if (dmin > dmax)
    {
        dist = dmax;
        result = false;
        return;
    }
    dist = dmin;
    result = true;
    return;
}
*/

/*
void getKDnodes(KDN::KDnode* root, vector<KDN::KDnode*>& nodes)
{
    if (root != NULL)
    {
        nodes.push_back(root);
        getKDnodes(root->left, nodes);
        getKDnodes(root->right, nodes);
    }
}

void getKDnodesLoop(KDN::KDnode* root, vector<KDN::KDnode*>& nodes)
{
    KDN::KDnode* currNode = root;
    while (true)
    {
        if (currNode == NULL)
            break;

        if (currNode->left != NULL && currNode->left->visited != true)
            currNode = currNode->left;
        else if (currNode->right != NULL && currNode->right->visited != true)
            currNode = currNode->right;
        else if (currNode->visited == false)
        {
            //std::cout << "NODE LOOP: " << currNode << std::endl;
            nodes.push_back(currNode);
            currNode->visited = true;
        }
        else
            currNode = currNode->parent;
    }

    // reset visited to false
    currNode = root;
    while (true)
    {
        if (currNode == NULL)
            break;

        if (currNode->left != NULL && currNode->left->visited != false)
            currNode = currNode->left;
        else if (currNode->right != NULL && currNode->right->visited != false)
            currNode = currNode->right;
        else if (currNode->visited == true)
            currNode->visited = false;
        else
            currNode = currNode->parent;
    }
}


void getKDnodesLoopDeref(KDN::KDnode* root, vector<KDN::KDnode>& nodes)
{
    KDN::KDnode* currNode = root;

    while (true)
    {
        if (currNode == NULL)
            break;

        if (currNode->left != NULL && currNode->left->visited != true)
            currNode = currNode->left;
        else if (currNode->right != NULL && currNode->right->visited != true)
            currNode = currNode->right;
        else if (currNode->visited == false)
        {
            //std::cout << "NODE LOOP: " << currNode << std::endl;
            nodes.push_back(currNode[0]);
            currNode->visited = true;
        }
        else
            currNode = currNode->parent;
    }

    // reset visited to false
    currNode = root;
    while (true)
    {
        if (currNode == NULL)
            break;

        if (currNode->left != NULL && currNode->left->visited != false)
            currNode = currNode->left;
        else if (currNode->right != NULL && currNode->right->visited != false)
            currNode = currNode->right;
        else if (currNode->visited == true)
            currNode->visited = false;
        else
            currNode = currNode->parent;
    }
}



vector<int> cacheTriangles(KDN::KDnode* nodes, int numNodes, vector<KDN::Triangle>& newTriangles)
{

    int triCount = 0;
    vector<int> offsets;

    if (numNodes == 0)
        return offsets;

    for (int i = 0; i < numNodes; i++)
    {
        int numTriangles = nodes[i].triangles.size();
        if (numTriangles > 0)
        {
            // update node triangle lookup
            nodes[i].triIdStart = triCount;
            nodes[i].triIdSize = numTriangles;

            triCount += numTriangles;
            offsets.push_back(triCount);

            for (int j = 0; j < numTriangles; j++)
            {
                newTriangles.push_back(nodes[i].triangles[j][0]);
            }
        }

        std::cout << "node: " << nodes[i].ID << " numtris: " << numTriangles << std::endl;
    }


    return offsets;
}


vector<int> cacheTriangles(std::vector<KDN::KDnode*> nodes, vector<KDN::Triangle>& newTriangles)
{

    int triCount = 0;
    vector<int> offsets;

    if (nodes.size() == 0)
        return offsets;

    for (int i = 0; i < nodes.size(); i++)
    {
        int numTriangles = nodes[i]->triangles.size();
        if (numTriangles > 0)
        {
            // update node triangle lookup
            nodes[i]->triIdStart = triCount;
            nodes[i]->triIdSize = numTriangles;

            triCount += numTriangles;
            offsets.push_back(triCount);

            for (int j = 0; j < numTriangles; j++)
            {
                newTriangles.push_back(nodes[i]->triangles[j][0]);
            }
        }

        std::cout << "node: " << nodes[i]->ID << " numtris: " << numTriangles << std::endl;
    }


    return offsets;
}

vector<int> cacheTriangles(std::vector<KDN::KDnode> nodes, vector<KDN::Triangle>& newTriangles)
{

    int triCount = 0;
    vector<int> offsets;

    if (nodes.size() == 0)
        return offsets;

    for (int i = 0; i < nodes.size(); i++)
    {
        int numTriangles = nodes[i].triangles.size();
        if (numTriangles > 0)
        {
            // update node triangle lookup
            nodes[i].triIdStart = triCount;
            nodes[i].triIdSize = numTriangles;

            triCount += numTriangles;
            offsets.push_back(triCount);

            for (int j = 0; j < numTriangles; j++)
            {
                newTriangles.push_back(nodes[i].triangles[j][0]);
            }
        }

        std::cout << "node: " << nodes[i].ID << " numtris: " << numTriangles << std::endl;
    }


    return offsets;
}

void deleteTree(KDN::KDnode* root)
{
    if (root != NULL)
    {
        deleteTree(root->left);
        deleteTree(root->right);
        //delete root;

        if (root->left != NULL)
            root->left = NULL;
        if (root->right != NULL)
            root->right = NULL;

        delete root;
        root = NULL;
    }
}

bool nodeComparator(const void* a, const void* b)
{
    int ida = (*(KDN::KDnode*)a).ID;
    int idb = (*(KDN::KDnode*)b).ID;

    if (ida <= idb)
        return true;
    else if (ida > idb)
        return false;
}


std::vector<KDN::Triangle*> getTrianglesFromScene(Scene* scene)
{
    vector<KDN::Triangle*> triangles;

    int iterator = 0;

    int pidxo1 = 0;
    int pidxo2 = 0;
    int pidxo3 = 0;

    for (int i = 0; i < scene->obj_numshapes; i++)
    {
        for (int j = iterator; j < iterator + scene->obj_polyoffsets[i]; j += 3)
        {
            pidxo1 = 3 * scene->obj_polysidxflat[j];
            pidxo2 = 3 * scene->obj_polysidxflat[j + 1];
            pidxo3 = 3 * scene->obj_polysidxflat[j + 2];

            KDN::Triangle* t = new KDN::Triangle(
                scene->obj_verts[pidxo1],
                scene->obj_verts[pidxo1 + 1],
                scene->obj_verts[pidxo1 + 2],
                scene->obj_verts[pidxo2],
                scene->obj_verts[pidxo2 + 1],
                scene->obj_verts[pidxo2 + 2],
                scene->obj_verts[pidxo3],
                scene->obj_verts[pidxo3 + 1],
                scene->obj_verts[pidxo3 + 2],
                scene->obj_norms[pidxo1],
                scene->obj_norms[pidxo1 + 1],
                scene->obj_norms[pidxo1 + 2],
                scene->obj_norms[pidxo2],
                scene->obj_norms[pidxo2 + 1],
                scene->obj_norms[pidxo2 + 2],
                scene->obj_norms[pidxo3],
                scene->obj_norms[pidxo3 + 1],
                scene->obj_norms[pidxo3 + 2]);

            triangles.push_back(t);
        }

        iterator += scene->obj_polyoffsets[i];
    }

    return triangles;
}
*/


// ------------------------------------------------------------------------
// ------------------------ KD TREE UTILITIES END -------------------------
// ------------------------------------------------------------------------













int obj_numshapes = 0;
int* obj_numpolyverts = NULL;
//int** obj_polysidx = NULL;
float* obj_verts = NULL;
float* obj_norms = NULL;
float* obj_texts = NULL;
int* obj_polyoffsets = NULL;
int* obj_polysidxflat = NULL;
float* obj_polysbboxes = NULL;

// KD DATA
//KDN::KDnode* kd_nodes = NULL;
//KDN::Triangle* kd_triangles = NULL;
KDN::NodeBare* kd_nodesBare = NULL;
KDN::TriBare* kd_trianglesBare = NULL;
static int numNodes = 0;
static int numTriangles = 0;
//

/*
float* obj_RGB = NULL;
float* obj_SPECEX = NULL;
float* obj_SPECRGB = NULL;
float* obj_REFL = NULL;
float* obj_REFR = NULL;
float* obj_REFRIOR = NULL;
*/
int* obj_materialOffsets = NULL;

void pathtraceInit(Scene *scene, bool enablekd) {
    hst_scene = scene;
    const Camera &cam = hst_scene->state.camera;
    const int pixelcount = cam.resolution.x * cam.resolution.y;

    cudaMalloc(&dev_image, pixelcount * sizeof(glm::vec3));
    cudaMemset(dev_image, 0, pixelcount * sizeof(glm::vec3));

    cudaMalloc(&dev_paths, pixelcount * sizeof(PathSegment));
    cudaMalloc(&dev_paths_cache, pixelcount * sizeof(PathSegment));

    cudaMalloc(&dev_geoms, scene->geoms.size() * sizeof(Geom));
    cudaMemcpy(dev_geoms, scene->geoms.data(), scene->geoms.size() * sizeof(Geom), cudaMemcpyHostToDevice);

    cudaMalloc(&dev_materials, scene->materials.size() * sizeof(Material));
    cudaMemcpy(dev_materials, scene->materials.data(), scene->materials.size() * sizeof(Material), cudaMemcpyHostToDevice);

    cudaMalloc(&dev_intersections, pixelcount * sizeof(ShadeableIntersection));
    cudaMemset(dev_intersections, 0, pixelcount * sizeof(ShadeableIntersection));
 

    // objloader part
    if (scene->hasObj)
    {
        if (enablekd == false)
        {
            cudaMalloc((void**)&obj_numpolyverts, scene->obj_numshapes * sizeof(int));
            cudaMalloc((void**)&obj_polyoffsets, scene->obj_numshapes * sizeof(int));
            cudaMalloc((void**)&obj_polysidxflat, scene->polyidxcount * sizeof(int));
            cudaMalloc((void**)&obj_verts, scene->objmesh->attrib.vertices.size()* sizeof(float));
            cudaMalloc((void**)&obj_norms, scene->objmesh->attrib.normals.size()* sizeof(float));
            cudaMalloc((void**)&obj_texts, scene->objmesh->attrib.texcoords.size()* sizeof(float));
            cudaMalloc((void**)&obj_polysbboxes, scene->obj_numshapes * 6 * sizeof(float));
        }
        cudaMalloc((void**)&obj_materialOffsets, scene->obj_numshapes * sizeof(int));


        // ------------------------------------------------------------------
        // KD DATA PART
        // ------------------------------------------------------------------
        /*
        // This section assembles a KD tree and flattens out the entire tree
        // as an array of KDnodes, an array of Triangles and an array of 
        // indices with the triangle offsets so that we can traverse the tree
        // non recursively.
        std::vector<KDN::Triangle*> triangles = getTrianglesFromScene(scene);

        KDtree* KDT = new KDtree(triangles);
        KDT->rootNode->updateBbox();
        KDT->split(3);

        // Accessing kd nodes and triangles as a flat structure
        // This is to help recursion removal for CUDA
        // THANK YOU NVIDIA for this...
        vector<KDN::KDnode*> nodes;
        getKDnodes(KDT->rootNode, nodes);
        std::sort(nodes.begin(), nodes.end(), nodeComparator);

        // flattened triangles data
        vector<KDN::Triangle> newTriangles;
        vector<int> offsets = cacheTriangles(nodes, newTriangles);
        numTriangles = newTriangles.size();

        vector<KDN::KDnode*> nodesLoop;
        getKDnodesLoop(KDT->rootNode, nodesLoop);
        std::sort(nodesLoop.begin(), nodesLoop.end());

        vector<KDN::KDnode> nodesLoopDeref;
        getKDnodesLoopDeref(KDT->rootNode, nodesLoopDeref);
        std::sort(nodesLoopDeref.begin(), nodesLoopDeref.end());

        
        // flattened nodes data
        numNodes = nodesLoopDeref.size();
        KDN::KDnode* newNodes = new KDN::KDnode[numNodes];
        memcpy(newNodes, nodesLoopDeref.data(), sizeof(KDN::KDnode)*numNodes);
        */

        if (enablekd == true)
        {
            /*
            printf("size of:\n\ttriangle: %d\n\ttribare: %d\n\tnode: %d\n\tnodebare: %d\n", 
                   sizeof(KDN::Triangle),
                   sizeof(KDN::TriBare),
                   sizeof(KDN::KDnode),
                   sizeof(KDN::NodeBare));
            */

            //printf("TOTAL NUM NODES = %d\n", scene->numNodes);

            // allocate KD data on the device
            //cudaMalloc((void**)&kd_nodes, scene->numNodes * sizeof(KDN::KDnode));
            //cudaMalloc((void**)&kd_triangles, scene->numTriangles * sizeof(KDN::Triangle));
            //printf("before allocating\n");
            cudaMalloc((void**)&kd_nodesBare, scene->numNodes * sizeof(KDN::NodeBare));
            cudaMalloc((void**)&kd_trianglesBare, scene->numTriangles * sizeof(KDN::TriBare));
            //printf("after allocating\n");
            
            //cudaMemcpy(kd_nodes, scene->newNodes, scene->numNodes * sizeof(KDN::KDnode), cudaMemcpyHostToDevice);
            //cudaMemcpy(kd_triangles, scene->newTriangles, scene->numTriangles * sizeof(KDN::Triangle), cudaMemcpyHostToDevice);
            //printf("before copying\n");
            cudaMemcpy(kd_nodesBare, scene->newNodesBare, scene->numNodes * sizeof(KDN::NodeBare), cudaMemcpyHostToDevice);
            cudaMemcpy(kd_trianglesBare, scene->newTrianglesBare, scene->numTriangles * sizeof(KDN::TriBare), cudaMemcpyHostToDevice);
            //printf("after copying\n");
        }
        else
        {
            cudaMemcpy(obj_numpolyverts, scene->obj_numpolyverts, scene->obj_numshapes * sizeof(int), cudaMemcpyHostToDevice);
            //printf("step 2\n");
            cudaMemcpy(obj_polyoffsets, scene->obj_polyoffsets, scene->obj_numshapes * sizeof(int), cudaMemcpyHostToDevice);
            //printf("step 3\n"); 
            cudaMemcpy(obj_polysidxflat, scene->obj_polysidxflat, scene->polyidxcount * sizeof(int), cudaMemcpyHostToDevice);
            //printf("step 4\n"); 
            cudaMemcpy(obj_verts, scene->obj_verts, scene->objmesh->attrib.vertices.size()* sizeof(float), cudaMemcpyHostToDevice);
            //printf("step 5\n"); 
            cudaMemcpy(obj_norms, scene->obj_norms, scene->objmesh->attrib.normals.size()* sizeof(float), cudaMemcpyHostToDevice);
            //printf("step 6\n"); 
            cudaMemcpy(obj_texts, scene->obj_texts, scene->objmesh->attrib.texcoords.size()* sizeof(float), cudaMemcpyHostToDevice);
            //printf("step 7\n"); 
            cudaMemcpy(obj_polysbboxes, scene->obj_bboxes, scene->obj_numshapes * 6 * sizeof(float), cudaMemcpyHostToDevice);
            //printf("step 8\n"); 
        }
        cudaMemcpy(obj_materialOffsets, scene->obj_materialOffsets, scene->obj_numshapes * sizeof(int), cudaMemcpyHostToDevice);
        //printf("step 9\n");
    }
    // shading 
    /*
    cudaMemcpy(obj_RGB, scene->obj_RGB, scene->obj_numshapes * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(obj_SPECEX, scene->obj_SPECEX, scene->obj_numshapes * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(obj_SPECRGB, scene->obj_SPECRGB, scene->obj_numshapes * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(obj_REFL, scene->obj_REFL, scene->obj_numshapes * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(obj_REFR, scene->obj_REFR, scene->obj_numshapes * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(obj_REFRIOR, scene->obj_REFRIOR, scene->obj_numshapes * sizeof(float), cudaMemcpyHostToDevice);
    */
 
    checkCUDAError("pathtraceInit");
}






























void pathtraceFree(Scene *scene, bool enablekd) {
    cudaFree(dev_image);  // no-op if dev_image is null
    cudaFree(dev_paths);
    cudaFree(dev_paths_cache);
    cudaFree(dev_geoms);
    cudaFree(dev_materials);
    cudaFree(dev_intersections);
    // TODO: clean up any extra device memory you created

    // objloader part
    if (scene->hasObj)
    {
        if (enablekd == false)
        {
            cudaFree(obj_numpolyverts);
            cudaFree(obj_polyoffsets);
            cudaFree(obj_polysidxflat);
            cudaFree(obj_verts);
            cudaFree(obj_norms);
            cudaFree(obj_texts);
        }
        cudaFree(obj_materialOffsets);

        if (enablekd == true)
        {
            // KD DATA
            //cudaFree(kd_nodes);
            //cudaFree(kd_triangles);
            cudaFree(kd_nodesBare);
            cudaFree(kd_trianglesBare);
        }
    }
    // shading
    /*
    cudaFree(obj_RGB);
    cudaFree(obj_SPECEX);
    cudaFree(obj_SPECRGB);
    cudaFree(obj_REFL);
    cudaFree(obj_REFR);
    cudaFree(obj_REFRIOR);
    */
    

    checkCUDAError("pathtraceFree");
}

/**
* Generate PathSegments with rays from the camera through the screen into the
* scene, which is the first bounce of rays.
*
* Antialiasing - add rays for sub-pixel sampling
* motion blur - jitter rays "in time"
* lens effect - jitter ray origin positions based on a lens
*/
__global__ void generateRayFromCamera(Camera cam, int iter, int traceDepth, PathSegment* pathSegments, float focalLength, float dofAngle, bool antialias)
{
    int x = (blockIdx.x * blockDim.x) + threadIdx.x;
    int y = (blockIdx.y * blockDim.y) + threadIdx.y;

    if (x < cam.resolution.x && y < cam.resolution.y) {
        int index = x + (y * cam.resolution.x);
        PathSegment & segment = pathSegments[index];

        segment.ray.origin = cam.position;
        segment.color = glm::vec3(1.0f, 1.0f, 1.0f);
        segment.ray.isinside = false;

        // store initial index
        //segment.initialidx = index;

        
        segment.ray.direction = glm::normalize(cam.view
            - cam.right * cam.pixelLength.x * ((float)x - (float)cam.resolution.x * 0.5f)
            - cam.up * cam.pixelLength.y * ((float)y - (float)cam.resolution.y * 0.5f)
            );

        // TODO: implement antialiasing by jittering the ray
        thrust::default_random_engine rng(utilhash(iter));
        thrust::uniform_real_distribution<float> unitDistrib(0, 1);
        if (antialias)
        {
            float jitterscale = 0.002;
            //thrust::uniform_real_distribution<float> unitDistrib(0, 1);

            bool fast = true;
            if (fast)
            {
                // use cheap jitter
                glm::vec3 v3((float)unitDistrib(rng), (float)unitDistrib(rng), (float)unitDistrib(rng));
                v3 = glm::normalize(v3);
                segment.ray.direction += v3*jitterscale;
                segment.ray.direction = glm::normalize(segment.ray.direction);
            }
            else
            {
                // use uniform spherical distribution
                float u = cos(PI * (float)unitDistrib(rng));
                float u2 = u*u;
                float sqrt1minusu2 = sqrt(1 - u2);
                float theta = 2 * PI * (float)unitDistrib(rng);
                glm::vec3  v3(sqrt1minusu2 * cos(theta),
                    sqrt1minusu2 * sin(theta),
                    u);
                segment.ray.direction += v3*jitterscale;
            }
        }




        // depth of field
        //thrust::uniform_real_distribution<float> unitDistrib01(-1, 1);
        
        // use uniform spherical distribution
        float u = cos(PI * (float)unitDistrib(rng));
        float u2 = u*u;
        float sqrt1minusu2 = sqrt(1 - u2);
        float theta = 2 * PI * (float)unitDistrib(rng);
        glm::vec3  v3(sqrt1minusu2 * cos(theta),
            sqrt1minusu2 * sin(theta),
            u);
        v3 = glm::normalize(v3);
        

        
        glm::vec3 center = cam.position + 8.0f * segment.ray.direction;
        //center -= cam.position;

        float R1 = (float)unitDistrib(rng);
        float R2 = (float)unitDistrib(rng);

        /*
        float angleVal = 0.25*PI;
        glm::vec3 randRotAngle(glm::cos(PI*R1 * glm::sin(angleVal*0.5f*R2)),
                               glm::sin(PI*R1 * glm::sin(angleVal*0.5f*R2)),
                               glm::cos(angleVal*0.5f*R2));
        */
        
        //v3 = glm::normalize(glm::cross(v3, segment.ray.direction));

        glm::vec3 front = glm::normalize(cam.lookAt);
        glm::vec3 up = glm::normalize(cam.up);
        glm::vec3 right = glm::normalize(cam.right);
        glm::quat Q1;
        //glm::vec3 a = glm::normalize(glm::cross(segment.ray.direction, cam.right));
        float randangle = (float)unitDistrib(rng) * PI * dofAngle;
        Q1.w = cosf(randangle / 2.0f);
        Q1.x = v3.x * sinf(randangle / 2.0f);
        Q1.y = v3.y * sinf(randangle / 2.0f);
        Q1.z = v3.z * sinf(randangle / 2.0f);
       
        glm::vec3 randrot = glm::rotate(Q1, segment.ray.direction);
        //center += cam.position;
        
        
        
        segment.ray.origin = segment.ray.origin + segment.ray.direction * focalLength - randrot*focalLength;
        segment.ray.direction = randrot;


        segment.ray.direction = glm::normalize(segment.ray.direction);


        segment.pixelIndex = index;
        segment.remainingBounces = traceDepth;
    }
}







// pathTraceOneBounce handles ray intersections, generate intersections for shading, 
// and scatter new ray. You might want to call scatterRay from interactions.h
__global__ void pathTraceOneBounce(
    int depth
    , int iter
    , int num_paths
    , PathSegment * pathSegments
    , Geom * geoms
    , int geoms_size
    , Material * materials
    , int material_size
    , ShadeableIntersection * intersections
    , float softness
    , int obj_numshapes
    , int* obj_numpolyverts
    , float* obj_verts
    , float* obj_norms
    , float* obj_texts
    , int* obj_polyoffsets
    , int* obj_polysidxflat
    , float* obj_polysbboxes
    , int polyidxcount
    /*,
    float* obj_RGB
    , float* obj_SPECEX
    , float* obj_SPECRGB
    , float* obj_REFL
    , float* obj_REFR
    , float* obj_REFRIOR
    */
    , int* obj_materialOffsets
    , int hasobj
    )
{
    int path_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (path_index < num_paths)
    {
        //path_index = pathSegments[path_index].pixelIndex;
        PathSegment pathSegment = pathSegments[path_index];
        //printf("\nO1");
        if (pathSegments[path_index].remainingBounces>0)
        {
            float t;
            glm::vec3 intersect_point;
            glm::vec3 normal;
            float t_min = FLT_MAX;
            int hit_geom_index = -1;
            bool outside = true;

            glm::vec3 tmp_intersect;
            glm::vec3 tmp_normal;

            glm::vec3 hit;
            glm::vec3 norm;
            glm::vec3 bary;
            glm::vec3 v1;
            glm::vec3 v2;
            glm::vec3 v3;
            glm::vec3 n1;
            glm::vec3 n2;
            glm::vec3 n3;
            int pidxo1 = 0;
            int pidxo2 = 0;
            int pidxo3 = 0;
            bool intersected = false;
            bool obj_intersect = false;
            // naive parse through global geoms
            //printf("\nO2");

            int objMaterialIdx = -1;
            for (int i = 0; i < geoms_size; i++)
            {
                Geom & geom = geoms[i];

                if (geom.type == CUBE)
                {
                    t = boxIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }
                else if (geom.type == SPHERE)
                {
                    t = sphereIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }
                // TODO: add more intersection tests here... triangle? metaball? CSG?

                // Compute the minimum t from the intersection tests to determine what
                // scene geometry object was hit first.
                if (t > 0.0f && t_min > t)
                {
                    t_min = t;
                    hit_geom_index = i;
                    intersect_point = tmp_intersect;
                    normal = tmp_normal;
                }
            }

            // start polygon hits
            //t_min = FLT_MAX;
            //for (int i = 0; i < obj_numshapes; i++)
            //    printf("\noffset = %d", obj_polyoffsets[i]);


            //printf("\nO3");
            //printf("\nNUMSHAPES = %d\n", obj_numshapes);
            objMaterialIdx = -1;
            int iterator = 0;
            if (hasobj)
            {
                for (int i = 0; i < obj_numshapes; i++)
                {
                    objMaterialIdx = obj_materialOffsets[i];
                    //printf("\nmaterial index = %d", objMaterialIdx);

                    // check bounding intersection first
                    float T = intersectBbox(pathSegment.ray.origin,
                        pathSegment.ray.direction,
                        glm::vec3(obj_polysbboxes[i] - 0.01, 
                                  obj_polysbboxes[i + 1] - 0.01, 
                                  obj_polysbboxes[i + 2] - 0.01),
                        glm::vec3(obj_polysbboxes[i + 3] + 0.01, 
                                  obj_polysbboxes[i + 4] + 0.01, 
                                  obj_polysbboxes[i + 5] + 0.01));

                    //if (T > -1.0f)
                    {
                        for (int j = iterator; j < iterator + obj_polyoffsets[i]; j += 3)
                        {
                            //printf("\nO5");
                            //int pidx1 = obj_polysidxflat[j];
                            //int pidx2 = obj_polysidxflat[j + 1];
                            //int pidx3 = obj_polysidxflat[j + 2];
                            pidxo1 = 3 * obj_polysidxflat[j];
                            pidxo2 = 3 * obj_polysidxflat[j + 1];
                            pidxo3 = 3 * obj_polysidxflat[j + 2];

                            v1.x = obj_verts[pidxo1];
                            v1.y = obj_verts[pidxo1 + 1];
                            v1.z = obj_verts[pidxo1 + 2];
                            v2.x = obj_verts[pidxo2];
                            v2.y = obj_verts[pidxo2 + 1];
                            v2.z = obj_verts[pidxo2 + 2];
                            v3.x = obj_verts[pidxo3];
                            v3.y = obj_verts[pidxo3 + 1];
                            v3.z = obj_verts[pidxo3 + 2];

                            n1.x = obj_norms[pidxo1];
                            n1.y = obj_norms[pidxo1 + 1];
                            n1.z = obj_norms[pidxo1 + 2];
                            n2.x = obj_norms[pidxo2];
                            n2.y = obj_norms[pidxo2 + 1];
                            n2.z = obj_norms[pidxo2 + 2];
                            n3.x = obj_norms[pidxo3];
                            n3.y = obj_norms[pidxo3 + 1];
                            n3.z = obj_norms[pidxo3 + 2];

                            //printf("\nO6");
                            //bary.x = 0.0f;
                            //bary.y = 0.0f;
                            //bary.z = 0.0f;

                            intersected = false;

                            bary.x = 0.0f; bary.y = 0.0f; bary.z = 0.0f;
                            intersected = glm::intersectRayTriangle(pathSegment.ray.origin,
                                pathSegment.ray.direction,
                                v1, v2, v3, bary);


                            glm::vec3 bary2(bary.x, bary.y, 1.0 - bary.x - bary.y);

                            if (intersected)
                            {
                                //(1 - bary.x - bary.y); bary.x; bary.y
                                //printf("\nO8");
                                hit = pathSegment.ray.origin + pathSegment.ray.direction* bary.z;// (bary2.x * v1 + bary2.y * v2 + bary2.z * v3);
                                norm = glm::normalize((1 - bary.x - bary.y) * n1 + bary.x * n2 + (bary.y) * n3);
                                //norm(glm::normalize(n1));
                                hit += norm*0.0001f;


                                t = glm::distance(pathSegment.ray.origin, hit);

                                if (t > 0.0f && t_min > t)
                                {
                                    t_min = t;
                                    hit_geom_index = obj_materialOffsets[i];
                                    intersect_point = hit;
                                    normal = norm;
                                    tmp_intersect = hit;
                                    tmp_normal = normal;
                                    obj_intersect = true;
                                    intersections[path_index].t = t;
                                }
                            }
                        }
                        iterator += obj_polyoffsets[i];
                        //printf("\nO10");
                    }
                }
            }

                
            
            //printf("\nO11");

            // TODO: scatter the ray, generate intersections for shading
            // feel free to modify the code below

            if (hit_geom_index == -1)
            {
                intersections[path_index].t = -1.0f;
            }
            else
            {
                //The ray hits something
                //intersections[path_index].t = t_min;
                //intersections[path_index].materialId = geoms[hit_geom_index].materialid;
                //intersections[path_index].surfaceNormal = normal;


                // updating rays
                //thrust::default_random_engine rng = makeSeededRandomEngine(iter, depth, depth); // WAY TOO COOL!
                thrust::default_random_engine rng = makeSeededRandomEngine(iter, path_index, depth);

                
                if (obj_intersect)
                {
                    pathSegments[path_index].materialIdHit = objMaterialIdx;

                    scatterRay(pathSegments[path_index].ray,
                        pathSegments[path_index].color,
                        intersect_point,
                        normal,
                        materials[objMaterialIdx],
                        rng,
                        softness);
                }
                else
                {
                    pathSegments[path_index].materialIdHit = geoms[hit_geom_index].materialid;

                    scatterRay(pathSegments[path_index].ray,
                        pathSegments[path_index].color,
                        intersect_point,
                        normal,
                        materials[geoms[hit_geom_index].materialid],
                        rng,
                        softness);
                }
                
                //pathSegments[path_index].ray.direction = calculateRandomDirectionInHemisphere(normal, rng);
                //pathSegments[path_index].ray.origin = intersect_point;


                if (obj_intersect)
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = objMaterialIdx; // test material
                    intersections[path_index].surfaceNormal = normal;
                }
                else
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = geoms[hit_geom_index].materialid;
                    intersections[path_index].surfaceNormal = normal;
                }
            }
        }
    }
}


// pathTraceOneBounce handles ray intersections, generate intersections for shading, 
// and scatter new ray. You might want to call scatterRay from interactions.h
__global__ void pathTraceOneBounceKDfix(
    int depth
    , int iter
    , int num_paths
    , PathSegment * pathSegments
    , Geom * geoms
    , int geoms_size
    , Material * materials
    , int material_size
    , ShadeableIntersection * intersections
    , float softness
    , KDN::Triangle* triangles
    , int numTriangles
    , KDN::KDnode* nodes
    , int numNodes
    , int* obj_materialOffsets
    , int hasobj
    )
{
    int path_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (path_index < num_paths)
    {
        //path_index = pathSegments[path_index].pixelIndex;
        PathSegment pathSegment = pathSegments[path_index];
        //printf("\nO1");
        if (pathSegments[path_index].remainingBounces>0)
        {
            float t;
            glm::vec3 intersect_point;
            glm::vec3 normal;
            float t_min = FLT_MAX;
            int hit_geom_index = -1;
            bool outside = true;

            glm::vec3 tmp_intersect;
            glm::vec3 tmp_normal;

            glm::vec3 hit;
            glm::vec3 norm;
            glm::vec3 bary;
            glm::vec3 v1;
            glm::vec3 v2;
            glm::vec3 v3;
            glm::vec3 n1;
            glm::vec3 n2;
            glm::vec3 n3;
            int pidxo1 = 0;
            int pidxo2 = 0;
            int pidxo3 = 0;
            bool intersected = false;
            bool obj_intersect = false;
            // naive parse through global geoms
            //printf("\nO2");

            int objMaterialIdx = -1;
            for (int i = 0; i < geoms_size; i++)
            {
                Geom & geom = geoms[i];

                if (geom.type == CUBE)
                {
                    t = boxIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }
                else if (geom.type == SPHERE)
                {
                    t = sphereIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }

                // Compute the minimum t from the intersection tests to determine what
                // scene geometry object was hit first.
                if (t > 0.0f && t_min > t)
                {
                    t_min = t;
                    hit_geom_index = i;
                    intersect_point = tmp_intersect;
                    normal = tmp_normal;
                }
            }

            objMaterialIdx = -1;
            int iterator = 0;
            if (hasobj)
            {
                //
                //
                if (numNodes != 0)
                {
                    bool nodeIDs[100] = { false };
                    int currID = nodeIDs[nodes[0].ID];
                    float dist = -1.0;

                    // get the root node
                    for (int i = 0; i < numNodes; i++)
                    {
                        if (nodes[i].parentID == -1)
                        {
                            currID = nodes[i].ID;
                            break;
                        }
                    }

                    KDN::KDnode* node = &(nodes[currID]);

                    bool hitGeom = false;
                    float boxdist = -1.0f;
                    bary.z = FLT_MAX;
                    while (true)
                    {
                        if (currID == -1)
                            break;

                        node = &(nodes[currID]);
                        // check if it intersects the bounds
                        //printf("1\n");
                        if (nodeIDs[currID] == true)
                        {
                            nodeIDs[node->ID] = true;
                            nodeIDs[node->leftID] = true;
                            nodeIDs[node->rightID] = true;
                            currID = node->parentID;
                            continue;
                        }
                        else
                        {
                            hitGeom = intersectAABB(pathSegment.ray, node->bbox, dist);
                            //hitGeom = intersectAABBarrays(pathSegment.ray, nodes[currID].mins, nodes[currID].maxs, dist);
                            
                            if (hitGeom == false && node->parentID == -1)
                                break;
                        }
                        //printf("2\n");


                        if (hitGeom == false && dist > bary.z)
                        {
                            nodeIDs[node->ID] = true;
                            nodeIDs[node->leftID] = true;
                            nodeIDs[node->rightID] = true;
                            currID = node->parentID;
                        }
                        else
                        {
                            if (nodes[currID].leftID != -1 && nodeIDs[nodes[currID].leftID] != true)
                                currID = node->leftID;
                            else if (nodes[currID].rightID != -1 && nodeIDs[nodes[currID].rightID] != true)
                                currID = node->rightID;
                            else if (nodeIDs[node->ID] == false)
                            {
                                //printf("NODE LOOP: %d\n", nodes[currID].ID);
                                nodeIDs[node->ID] = true;

                                int size = node->triIdSize;
                                if (size > 0)
                                {
                                    int start = node->triIdStart;
                                    int end = start + size;
                                    for (int i = start; i < end; i++)
                                    {
                                        KDN::Triangle* T = &(triangles[i]);

                                        glm::vec3 v1(T->x1, T->y1, T->z1);
                                        glm::vec3 v2(T->x2, T->y2, T->z2);
                                        glm::vec3 v3(T->x3, T->y3, T->z3);

                                        glm::vec3 n1(T->nx1, T->ny1, T->nz1);
                                        glm::vec3 n2(T->nx2, T->ny2, T->nz2);
                                        glm::vec3 n3(T->nx3, T->ny3, T->nz3);

                                        //bary(0.0f, 0.0f, 0.0f);
                                        bool intersected = glm::intersectRayTriangle(pathSegment.ray.origin,
                                                                                     pathSegment.ray.direction,
                                                                                     v1, v2, v3, bary);

                                        if (intersected)
                                        {
                                            objMaterialIdx = triangles[i].mtlIdx + material_size - 1;
                                            //(1 - bary.x - bary.y); bary.x; bary.y
                                            //printf("material id: %d\n", triangles[i].mtlIdx);
                                            hit = pathSegment.ray.origin + pathSegment.ray.direction* bary.z;// (bary2.x * v1 + bary2.y * v2 + bary2.z * v3);
                                            norm = -glm::normalize((1 - bary.x - bary.y) * n1 + bary.x * n2 + (bary.y) * n3);
                                            //norm(glm::normalize(n1));
                                            hit += norm*0.0001f;


                                            t = glm::distance(pathSegment.ray.origin, hit);

                                            if (t > 0.0f && t_min > t)
                                            {
                                                t_min = t;
                                                hit_geom_index = obj_materialOffsets[T->mtlIdx];
                                                intersect_point = hit;
                                                normal = norm;
                                                tmp_intersect = hit;
                                                tmp_normal = normal;
                                                obj_intersect = true;
                                                intersections[path_index].t = t;
                                            }
                                        }
                                    }
                                }
                            }
                            else
                                currID = node->parentID;
                        }
                    }
                }
            }



            if (hit_geom_index == -1)
            {
                intersections[path_index].t = -1.0f;
            }
            else
            {

                // updating rays
                //thrust::default_random_engine rng = makeSeededRandomEngine(iter, depth, depth); // WAY TOO COOL!
                thrust::default_random_engine rng = makeSeededRandomEngine(iter, path_index, depth);


                if (obj_intersect)
                {
                    pathSegments[path_index].materialIdHit = objMaterialIdx;

                    scatterRay(pathSegments[path_index].ray,
                               pathSegments[path_index].color,
                               intersect_point,
                               normal,
                               materials[objMaterialIdx],
                               rng,
                               softness);
                }
                else
                {
                    pathSegments[path_index].materialIdHit = geoms[hit_geom_index].materialid;

                    scatterRay(pathSegments[path_index].ray,
                               pathSegments[path_index].color,
                               intersect_point,
                               normal,
                               materials[geoms[hit_geom_index].materialid],
                               rng,
                               softness);
                }


                if (obj_intersect)
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = objMaterialIdx; // test material
                    intersections[path_index].surfaceNormal = normal;
                }
                else
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = geoms[hit_geom_index].materialid;
                    intersections[path_index].surfaceNormal = normal;
                }
            }
        }
    }
}

__host__ __device__
void traverseKDbare(KDN::NodeBare* nodes, int numNodes,
float& t,
PathSegment pathSegment,
KDN::TriBare* triangles,
glm::vec3& bary,
int& objMaterialIdx,
int& material_size,
glm::vec3& hit,
glm::vec3& norm,
float& t_min,
int& hit_geom_index,
glm::vec3& intersect_point,
glm::vec3& normal,
glm::vec3& tmp_intersect,
glm::vec3& tmp_normal,
bool& obj_intersect,
ShadeableIntersection* intersections,
int* obj_materialOffsets,
int& path_index)
{
    //
    //
    if (numNodes != 0)
    {
        bool nodeIDs[STACK_SIZE] = { false };
        int currID = nodeIDs[nodes[0].ID];
        float dist = -1.0;

        // get the root node
        for (int i = 0; i < numNodes; i++)
        {
            if (nodes[i].parentID == -1)
            {
                currID = nodes[i].ID;
                break;
            }
        }

        KDN::NodeBare* node = &(nodes[currID]);

        bool hitGeom = false;
        float boxdist = -1.0f;
        bary.z = FLT_MAX;
        while (true)
        {
            if (currID == -1)
                break;

            node = &(nodes[currID]);
            // check if it intersects the bounds
            //printf("1\n");

            if (hitGeom == false && node->parentID == -1 && nodeIDs[node->ID] == true)
                break;

            hitGeom = intersectAABBarrays(pathSegment.ray, nodes[currID].mins, nodes[currID].maxs, dist);

            if (nodeIDs[currID] == true)
            {
                nodeIDs[node->ID] = true;
                nodeIDs[node->leftID] = true;
                nodeIDs[node->rightID] = true;
                currID = node->parentID;
                continue;
            }
            else
            {
                //hitGeom = intersectAABB(pathSegment.ray, node->bbox, dist);
                //hitGeom = intersectAABBarrays(pathSegment.ray, nodes[currID].mins, nodes[currID].maxs, dist);

                if (hitGeom == false && node->parentID == -1)
                    break;
            }
            //printf("2\n");

            // if the distance is greater than the last poly hit
            if (hitGeom == false || dist > bary.z)
            {
                nodeIDs[node->ID] = true;
                nodeIDs[node->leftID] = true;
                nodeIDs[node->rightID] = true;
                currID = node->parentID;
            }
            else
            {
                if (nodes[currID].leftID != -1 && nodeIDs[nodes[currID].leftID] != true)
                    currID = node->leftID;
                else if (nodes[currID].rightID != -1 && nodeIDs[nodes[currID].rightID] != true)
                    currID = node->rightID;
                else if (nodeIDs[node->ID] == false)
                {
                    //printf("NODE LOOP: %d\n", nodes[currID].ID);
                    nodeIDs[node->ID] = true;

                    int size = node->triIdSize;
                    if (size > 0)
                    {
                        int start = node->triIdStart;
                        int end = start + size;
                        for (int i = start; i < end; i++)
                        {
                            KDN::TriBare* T = &(triangles[i]);

                            glm::vec3 v1(T->x1, T->y1, T->z1);
                            glm::vec3 v2(T->x2, T->y2, T->z2);
                            glm::vec3 v3(T->x3, T->y3, T->z3);

                            //bary(0.0f, 0.0f, 0.0f);
                            bool intersected = glm::intersectRayTriangle(pathSegment.ray.origin,
                                                                         pathSegment.ray.direction,
                                                                         v1, v2, v3, bary);

                            if (intersected)
                            {
                                glm::vec3 n1(T->nx1, T->ny1, T->nz1);
                                glm::vec3 n2(T->nx2, T->ny2, T->nz2);
                                glm::vec3 n3(T->nx3, T->ny3, T->nz3);

                                objMaterialIdx = triangles[i].mtlIdx + material_size - 1;
                                //(1 - bary.x - bary.y); bary.x; bary.y
                                //printf("material id: %d\n", triangles[i].mtlIdx);
                                hit = pathSegment.ray.origin + pathSegment.ray.direction* bary.z;// (bary2.x * v1 + bary2.y * v2 + bary2.z * v3);
                                norm = glm::normalize((1 - bary.x - bary.y) * n1 + bary.x * n2 + (bary.y) * n3);
                                //norm(glm::normalize(n1));
                                hit += norm*0.0001f;


                                t = glm::distance(pathSegment.ray.origin, hit);

                                if (t > 0.0f && t_min > t)
                                {
                                    t_min = t;
                                    hit_geom_index = obj_materialOffsets[T->mtlIdx];
                                    intersect_point = hit;
                                    normal = norm;
                                    tmp_intersect = hit;
                                    tmp_normal = normal;
                                    obj_intersect = true;
                                    intersections[path_index].t = t;
                                    return;
                                }
                            }
                        }
                    }
                }
                else
                    currID = node->parentID;
            }
        }
    }
}


__host__ __device__
void traverseKDbareShortHybrid(KDN::NodeBare* nodes, int numNodes,
float& t,
PathSegment pathSegment,
KDN::TriBare* triangles,
glm::vec3& bary,
int& objMaterialIdx,
int& material_size,
glm::vec3& hit,
glm::vec3& norm,
float& t_min,
int& hit_geom_index,
glm::vec3& intersect_point,
glm::vec3& normal,
glm::vec3& tmp_intersect,
glm::vec3& tmp_normal,
bool& obj_intersect,
ShadeableIntersection* intersections,
int* obj_materialOffsets,
int& path_index)
{
    //
    //
    if (numNodes != 0)
    {
        bool nodeIDs[STACK_SIZE] = { false };
        int currID = nodeIDs[nodes[0].ID];
        float dist = -1.0;

        // get the root node
        for (int i = 0; i < numNodes; i++)
        {
            if (nodes[i].parentID == -1)
            {
                currID = nodes[i].ID;
                break;
            }
        }

        KDN::NodeBare* node = &(nodes[currID]);

        int axis;
        float tSplit;
        bool hitGeom = false;
        float boxdist = -1.0f;
        bary.z = FLT_MAX;
        while (true)
        {
            if (currID == -1)
                break;

            node = &(nodes[currID]);
            // check if it intersects the bounds
            //printf("1\n");

            if (hitGeom == false && node->parentID == -1 && nodeIDs[node->ID] == true)
                break;

            hitGeom = intersectAABBarrays(pathSegment.ray, nodes[currID].mins, nodes[currID].maxs, dist);

            if (nodeIDs[currID] == true)
            {
                nodeIDs[node->ID] = true;
                nodeIDs[node->leftID] = true;
                nodeIDs[node->rightID] = true;
                currID = node->parentID;
                continue;
            }
            else
            {
                //hitGeom = intersectAABB(pathSegment.ray, node->bbox, dist);
                //hitGeom = intersectAABBarrays(pathSegment.ray, nodes[currID].mins, nodes[currID].maxs, dist);

                if (hitGeom == false && node->parentID == -1)
                    break;
            }
            //printf("2\n");

            // if the distance is greater than the last poly hit
            if (hitGeom == false || dist > bary.z)
            {
                nodeIDs[node->ID] = true;
                nodeIDs[node->leftID] = true;
                nodeIDs[node->rightID] = true;
                currID = node->parentID;
            }
            else
            {
                axis = node->axis;
                //tSplit = (node->splitPos - pathSegment.ray.origin[axis]) / pathSegment.ray.direction[axis];

                if (pathSegment.ray.direction[axis] > 0.0f)// && nodes[node->leftID].splitPos < tSplit)
                {
                    // left side first
                    if (nodes[currID].leftID != -1 && nodeIDs[nodes[currID].leftID] != true)
                        currID = node->leftID;
                    else if (nodes[currID].rightID != -1 && nodeIDs[nodes[currID].rightID] != true)
                        currID = node->rightID;
                    else if (nodeIDs[node->ID] == false)
                    {
                        //printf("NODE LOOP: %d\n", nodes[currID].ID);
                        nodeIDs[node->ID] = true;

                        int size = node->triIdSize;
                        if (size > 0)
                        {
                            int start = node->triIdStart;
                            int end = start + size;
                            for (int i = start; i < end; i++)
                            {
                                KDN::TriBare* T = &(triangles[i]);

                                glm::vec3 v1(T->x1, T->y1, T->z1);
                                glm::vec3 v2(T->x2, T->y2, T->z2);
                                glm::vec3 v3(T->x3, T->y3, T->z3);

                                //bary(0.0f, 0.0f, 0.0f);
                                bool intersected = glm::intersectRayTriangle(pathSegment.ray.origin,
                                                                             pathSegment.ray.direction,
                                                                             v1, v2, v3, bary);

                                if (intersected)
                                {
                                    // skip other side
                                    nodeIDs[nodes[nodeIDs[node->parentID]].rightID] = true;

                                    glm::vec3 n1(T->nx1, T->ny1, T->nz1);
                                    glm::vec3 n2(T->nx2, T->ny2, T->nz2);
                                    glm::vec3 n3(T->nx3, T->ny3, T->nz3);

                                    objMaterialIdx = triangles[i].mtlIdx + material_size - 1;
                                    //(1 - bary.x - bary.y); bary.x; bary.y
                                    //printf("material id: %d\n", triangles[i].mtlIdx);
                                    hit = pathSegment.ray.origin + pathSegment.ray.direction* bary.z;// (bary2.x * v1 + bary2.y * v2 + bary2.z * v3);
                                    norm = glm::normalize((1 - bary.x - bary.y) * n1 + bary.x * n2 + (bary.y) * n3);
                                    //norm(glm::normalize(n1));
                                    hit += norm*0.0001f;


                                    t = glm::distance(pathSegment.ray.origin, hit);

                                    if (t > 0.0f && t_min > t)
                                    {
                                        t_min = t;
                                        hit_geom_index = obj_materialOffsets[T->mtlIdx];
                                        intersect_point = hit;
                                        normal = norm;
                                        tmp_intersect = hit;
                                        tmp_normal = normal;
                                        obj_intersect = true;
                                        intersections[path_index].t = t;
                                        return;
                                    }
                                }
                            }
                        }
                    }
                    else
                        currID = node->parentID;
                }
                else
                {
                    // right side first
                    if (nodes[currID].rightID != -1 && nodeIDs[nodes[currID].rightID] != true)
                        currID = node->rightID;
                    else if (nodes[currID].leftID != -1 && nodeIDs[nodes[currID].leftID] != true)
                        currID = node->leftID;
                    else if (nodeIDs[node->ID] == false)
                    {
                        //printf("NODE LOOP: %d\n", nodes[currID].ID);
                        nodeIDs[node->ID] = true;

                        int size = node->triIdSize;
                        if (size > 0)
                        {
                            int start = node->triIdStart;
                            int end = start + size;
                            for (int i = start; i < end; i++)
                            {
                                KDN::TriBare* T = &(triangles[i]);

                                glm::vec3 v1(T->x1, T->y1, T->z1);
                                glm::vec3 v2(T->x2, T->y2, T->z2);
                                glm::vec3 v3(T->x3, T->y3, T->z3);

                                //bary(0.0f, 0.0f, 0.0f);
                                bool intersected = glm::intersectRayTriangle(pathSegment.ray.origin,
                                                                             pathSegment.ray.direction,
                                                                             v1, v2, v3, bary);

                                if (intersected)
                                {
                                    // skip other side
                                    nodeIDs[nodes[nodeIDs[node->parentID]].leftID] = true;

                                    glm::vec3 n1(T->nx1, T->ny1, T->nz1);
                                    glm::vec3 n2(T->nx2, T->ny2, T->nz2);
                                    glm::vec3 n3(T->nx3, T->ny3, T->nz3);

                                    objMaterialIdx = triangles[i].mtlIdx + material_size - 1;
                                    //(1 - bary.x - bary.y); bary.x; bary.y
                                    //printf("material id: %d\n", triangles[i].mtlIdx);
                                    hit = pathSegment.ray.origin + pathSegment.ray.direction* bary.z;// (bary2.x * v1 + bary2.y * v2 + bary2.z * v3);
                                    norm = glm::normalize((1 - bary.x - bary.y) * n1 + bary.x * n2 + (bary.y) * n3);
                                    //norm(glm::normalize(n1));
                                    hit += norm*0.0001f;


                                    t = glm::distance(pathSegment.ray.origin, hit);

                                    if (t > 0.0f && t_min > t)
                                    {
                                        t_min = t;
                                        hit_geom_index = obj_materialOffsets[T->mtlIdx];
                                        intersect_point = hit;
                                        normal = norm;
                                        tmp_intersect = hit;
                                        tmp_normal = normal;
                                        obj_intersect = true;
                                        intersections[path_index].t = t;
                                        return;
                                    }
                                }
                            }
                        }
                    }
                    else
                        currID = node->parentID;
                }
            }
        }
    }
}


__host__ __device__
void traverseKDshort(KDN::NodeBare* nodes, int numNodes,
float& t,
PathSegment pathSegment,
KDN::TriBare* triangles,
glm::vec3& bary,
int& objMaterialIdx,
int& material_size,
glm::vec3& hit,
glm::vec3& norm,
float& t_min,
int& hit_geom_index,
glm::vec3& intersect_point,
glm::vec3& normal,
glm::vec3& tmp_intersect,
glm::vec3& tmp_normal,
bool& obj_intersect,
ShadeableIntersection* intersections,
int* obj_materialOffsets,
int& path_index)
{
    //printf("numnodes = %d\n", numNodes);


    NodeStack stack[STACK_SIZE];
    int top = -1;



    //std::stack<KDN::NodeBare> nodeStack;
    //KDN::NodeStackBare nodeStack;
    //stack<KDN::NodeBare> nodeStack;
    KDN::NodeBare* node;
    KDN::NodeBare* root;
    KDN::NodeBare* first;
    KDN::NodeBare* second;
    // get the root node
    for (int i = 0; i < numNodes; i++)
    {
        if (nodes[i].parentID == -1)
        {
            node = &(nodes[i]);
            root = &(nodes[i]);
            break;
        }
    }
    //printf("1\n");
    //
    //
    float tMin, tMax, tHit, sceneMax;
    tMin = tMax = 0.0f;
    tHit = t_min;
    sceneMax = FLT_MAX;
    bool pushdown = false;
    int axis = 0;
    float tSplit = 0.0f;

    float dist = 0.0f;
    bool bboxintersect = false;

    while (tMax < sceneMax)
    {
        if (top == -1)
        {
            node = root;
            tMin = tMax;
            tMax = sceneMax;
            pushdown = true;
        }
        else
        {
            node = stack[top].node;
            tMin = node->tmin;
            tMax = node->tmax;
            top--;
            pushdown = false;
        }
        //printf("2\n");
        while (node->triIdSize != 0)
        {
            axis = node->axis;
            tSplit = (node->splitPos - pathSegment.ray.origin[axis]) / pathSegment.ray.direction[axis];

            if (pathSegment.ray.direction[axis] > 0.0f)
            {
                if (nodes[node->leftID].mins[axis] < nodes[node->rightID].mins[axis])
                {
                    first = &(nodes[node->leftID]);
                    second = &(nodes[node->rightID]);
                }
                else
                {
                    first = &nodes[node->rightID];
                    second = &nodes[node->leftID];
                }
            }
            else
            {
                if (nodes[node->leftID].maxs[axis] > nodes[node->rightID].maxs[axis])
                {
                    first = &(nodes[node->leftID]);
                    second = &(nodes[node->rightID]);
                }
                else
                {
                    first = &(nodes[node->rightID]);
                    second = &(nodes[node->leftID]);
                }
            }

            if (tSplit >= tMax || tSplit < 0.0f)
                node = first;
            else if (tSplit <= tMin)
                node = second;
            else
            {
                second->tmin = tSplit;
                second->tmax = tMax;
                top++;
                if (top <= 199)
                {
                    stack[top].node = second;
                    stack[top].tmin = tSplit;
                    stack[top].tmax = tMax;
                }
                else
                {
                    //printf("bogus");
                    break;

                }
                node = first;
                tMax = tSplit;
                pushdown = false;
            }
            if (pushdown)
                root = node;


            bboxintersect = intersectAABBarrays(pathSegment.ray, node->mins, node->maxs, dist);
            if (bboxintersect)
            {
                int start = node->triIdStart;
                int end = start + node->triIdSize;
                for (int i = start; i < end; i++)
                {
                    KDN::TriBare* T = &(triangles[i]);

                    glm::vec3 v1(T->x1, T->y1, T->z1);
                    glm::vec3 v2(T->x2, T->y2, T->z2);
                    glm::vec3 v3(T->x3, T->y3, T->z3);

                    //bary(0.0f, 0.0f, 0.0f);
                    bool intersected = glm::intersectRayTriangle(pathSegment.ray.origin,
                                                                 pathSegment.ray.direction,
                                                                 v1, v2, v3, bary);

                    if (intersected)
                    {
                        glm::vec3 n1(T->nx1, T->ny1, T->nz1);
                        glm::vec3 n2(T->nx2, T->ny2, T->nz2);
                        glm::vec3 n3(T->nx3, T->ny3, T->nz3);

                        //printf("5\n");
                        objMaterialIdx = triangles[i].mtlIdx + material_size - 1;
                        //(1 - bary.x - bary.y); bary.x; bary.y
                        //printf("material id: %d\n", triangles[i].mtlIdx);
                        hit = pathSegment.ray.origin + pathSegment.ray.direction* bary.z;// (bary2.x * v1 + bary2.y * v2 + bary2.z * v3);
                        norm = -glm::normalize((1 - bary.x - bary.y) * n1 + bary.x * n2 + (bary.y) * n3);
                        //norm(glm::normalize(n1));
                        hit += norm*0.0001f;


                        t = glm::distance(pathSegment.ray.origin, hit);

                        if (t > 0.0f || t_min > t)
                        {
                            tHit = min(tHit, t);

                            t_min = t;
                            hit_geom_index = obj_materialOffsets[T->mtlIdx];
                            intersect_point = hit;
                            normal = norm;
                            tmp_intersect = hit;
                            tmp_normal = normal;
                            obj_intersect = true;
                            intersections[path_index].t = t;
                            return;
                        }
                    }
                }
            }
        }
    }
}







__host__ __device__
void traverseKD(KDN::NodeBare* nodes, int numNodes,
float& t,
PathSegment pathSegment,
KDN::TriBare* triangles,
glm::vec3& bary,
int& objMaterialIdx,
int& material_size,
glm::vec3& hit,
glm::vec3& norm,
float& t_min,
int& hit_geom_index,
glm::vec3& intersect_point,
glm::vec3& normal,
glm::vec3& tmp_intersect,
glm::vec3& tmp_normal,
bool& obj_intersect,
ShadeableIntersection* intersections,
int* obj_materialOffsets,
int& path_index)
{
    //printf("numnodes = %d\n", numNodes);


    NodeStack stack[STACK_SIZE];
    int top = -1;



    //std::stack<KDN::NodeBare> nodeStack;
    //KDN::NodeStackBare nodeStack;
    //stack<KDN::NodeBare> nodeStack;
    KDN::NodeBare* node;
    KDN::NodeBare* root;
    KDN::NodeBare* first;
    KDN::NodeBare* second;
    // get the root node
    for (int i = 0; i < numNodes; i++)
    {
        if (nodes[i].parentID == -1)
        {
            node = &(nodes[i]);
            root = &(nodes[i]);
            break;
        }
    }
    //printf("1\n");
    //
    //
    glm::vec3 origin = pathSegment.ray.origin;
    glm::vec3 invDirection(1.0f / pathSegment.ray.direction[0],
                           1.0f / pathSegment.ray.direction[1],
                           1.0f / pathSegment.ray.direction[2]);

    float tmax = FLT_MAX;
    float tClosestIntersection = t_min;
    bool notFullyTraversed = true;


    while (notFullyTraversed)
    {
        if (node->triIdSize != 0)
        {
            //test all primitives inside the leaf
            float dist = 0.0f;
            bool bboxintersect = intersectAABBarrays(pathSegment.ray, node->mins, node->maxs, dist);
            if (bboxintersect)
            {
                int start = node->triIdStart;
                int end = start + node->triIdSize;
                for (int i = start; i < end; i++)
                {
                    KDN::TriBare* T = &(triangles[i]);

                    glm::vec3 v1(T->x1, T->y1, T->z1);
                    glm::vec3 v2(T->x2, T->y2, T->z2);
                    glm::vec3 v3(T->x3, T->y3, T->z3);

                    //bary(0.0f, 0.0f, 0.0f);
                    bool intersected = glm::intersectRayTriangle(pathSegment.ray.origin,
                                                                 pathSegment.ray.direction,
                                                                 v1, v2, v3, bary);

                    if (intersected)
                    {
                        glm::vec3 n1(T->nx1, T->ny1, T->nz1);
                        glm::vec3 n2(T->nx2, T->ny2, T->nz2);
                        glm::vec3 n3(T->nx3, T->ny3, T->nz3);

                        //printf("5\n");
                        objMaterialIdx = triangles[i].mtlIdx + material_size - 1;
                        //(1 - bary.x - bary.y); bary.x; bary.y
                        //printf("material id: %d\n", triangles[i].mtlIdx);
                        hit = pathSegment.ray.origin + pathSegment.ray.direction* bary.z;// (bary2.x * v1 + bary2.y * v2 + bary2.z * v3);
                        norm = -glm::normalize((1 - bary.x - bary.y) * n1 + bary.x * n2 + (bary.y) * n3);
                        //norm(glm::normalize(n1));
                        hit += norm*0.0001f;


                        t = glm::distance(pathSegment.ray.origin, hit);

                        if (t > 0.0f && t_min > t)
                        {
                            //tHit = min(tHit, t);

                            t_min = t;
                            hit_geom_index = obj_materialOffsets[T->mtlIdx];
                            intersect_point = hit;
                            normal = norm;
                            tmp_intersect = hit;
                            tmp_normal = normal;
                            obj_intersect = true;
                            intersections[path_index].t = t;
                            return;
                        }
                    }
                }
            }

            //test if leaf + empty stack => return
            if (top == -1)
            {
                notFullyTraversed = false;
            }
            else
            {
                //pop all stack
                origin = stack[top].origin;
                tmax = stack[top].tmax;
                node = stack[top].node;
                top--;
            }
        }
        else
        {
            //get axis of node and its split plane
            const int axis = node->axis;
            const float plane = node->splitPos;

            //test if ray is not parallel to plane
            if ((fabs(pathSegment.ray.direction[axis]) > EPSILON))
            {
                const float t = (plane - origin[axis]) * invDirection[axis];

                //case of the ray intersecting the plane, then test both childs
                if (0.0f < t && t < tmax) {
                    //traverse near first, then far. Set tmax = t for near

                    //push only far child onto stack
                    top++;
                    stack[top].origin[0] = origin[0] + pathSegment.ray.direction[0] * t;
                    stack[top].origin[1] = origin[1] + pathSegment.ray.direction[1] * t;
                    stack[top].origin[2] = origin[2] + pathSegment.ray.direction[2] * t;
                    stack[top].node = (origin[axis] > plane) ? &(nodes[node->leftID]) : &(nodes[node->rightID]);
                    stack[top].tmax = tmax - t;

                    tmax = t;
                }
            }
            //in every case: traverse near child first
            node = (origin[axis] > plane) ? &(nodes[node->rightID]) : &(nodes[node->leftID]);

        }
    }
}


// pathTraceOneBounce handles ray intersections, generate intersections for shading, 
// and scatter new ray. You might want to call scatterRay from interactions.h
__global__ void pathTraceOneBounceKDbare(
    int depth
    , int iter
    , int num_paths
    , PathSegment * pathSegments
    , Geom * geoms
    , int geoms_size
    , Material * materials
    , int material_size
    , ShadeableIntersection * intersections
    , float softness
    , KDN::TriBare* triangles
    , int numTriangles
    , KDN::NodeBare* nodes
    , int numNodes
    , int* obj_materialOffsets
    , int hasobj
    )
{
    int path_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (path_index < num_paths)
    {
        //path_index = pathSegments[path_index].pixelIndex;
        PathSegment pathSegment = pathSegments[path_index];
        //printf("\nO1");
        if (pathSegments[path_index].remainingBounces>0)
        {
            float t;
            glm::vec3 intersect_point;
            glm::vec3 normal;
            float t_min = FLT_MAX;
            int hit_geom_index = -1;
            bool outside = true;

            glm::vec3 tmp_intersect;
            glm::vec3 tmp_normal;

            glm::vec3 hit;
            glm::vec3 norm;
            glm::vec3 bary;
            glm::vec3 v1;
            glm::vec3 v2;
            glm::vec3 v3;
            glm::vec3 n1;
            glm::vec3 n2;
            glm::vec3 n3;
            int pidxo1 = 0;
            int pidxo2 = 0;
            int pidxo3 = 0;
            bool intersected = false;
            bool obj_intersect = false;
            // naive parse through global geoms
            //printf("\nO2");

            int objMaterialIdx = -1;
            for (int i = 0; i < geoms_size; i++)
            {
                Geom & geom = geoms[i];

                if (geom.type == CUBE)
                {
                    t = boxIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }
                else if (geom.type == SPHERE)
                {
                    t = sphereIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }

                // Compute the minimum t from the intersection tests to determine what
                // scene geometry object was hit first.
                if (t > 0.0f && t_min > t)
                {
                    t_min = t;
                    hit_geom_index = i;
                    intersect_point = tmp_intersect;
                    normal = tmp_normal;
                }
            }

            objMaterialIdx = -1;
            int iterator = 0;
            if (hasobj)
            {
                /*
                traverseKDbare(nodes, numNodes, t,
                                pathSegment, triangles,
                                bary, objMaterialIdx,
                                material_size, hit,
                                norm, t_min,
                                hit_geom_index, intersect_point,
                                normal, tmp_intersect,
                                tmp_normal, obj_intersect,
                                intersections, obj_materialOffsets,
                                path_index);
                */
                ///*
                traverseKDbareShortHybrid(nodes, numNodes, t,
                                pathSegment, triangles,
                                bary, objMaterialIdx,
                                material_size, hit,
                                norm, t_min,
                                hit_geom_index, intersect_point,
                                normal, tmp_intersect,
                                tmp_normal, obj_intersect,
                                intersections, obj_materialOffsets,
                                path_index);
                //*/
            }

            if (hit_geom_index == -1)
            {
                intersections[path_index].t = -1.0f;
            }
            else
            {

                // updating rays
                //thrust::default_random_engine rng = makeSeededRandomEngine(iter, depth, depth); // WAY TOO COOL!
                thrust::default_random_engine rng = makeSeededRandomEngine(iter, path_index, depth);


                if (obj_intersect)
                {
                    pathSegments[path_index].materialIdHit = objMaterialIdx;

                    scatterRay(pathSegments[path_index].ray,
                               pathSegments[path_index].color,
                               intersect_point,
                               normal,
                               materials[objMaterialIdx],
                               rng,
                               softness);
                }
                else
                {
                    pathSegments[path_index].materialIdHit = geoms[hit_geom_index].materialid;

                    scatterRay(pathSegments[path_index].ray,
                               pathSegments[path_index].color,
                               intersect_point,
                               normal,
                               materials[geoms[hit_geom_index].materialid],
                               rng,
                               softness);
                }


                if (obj_intersect)
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = objMaterialIdx; // test material
                    intersections[path_index].surfaceNormal = normal;
                }
                else
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = geoms[hit_geom_index].materialid;
                    intersections[path_index].surfaceNormal = normal;
                }
            }
        }
    }
}

// pathTraceOneBounce handles ray intersections, generate intersections for shading, 
// and scatter new ray. You might want to call scatterRay from interactions.h
__global__ void pathTraceOneBounceKDbareBoxes(
    int depth
    , int iter
    , int num_paths
    , PathSegment * pathSegments
    , Geom * geoms
    , int geoms_size
    , Material * materials
    , int material_size
    , ShadeableIntersection * intersections
    , float softness
    , KDN::TriBare* triangles
    , int numTriangles
    , KDN::NodeBare* nodes
    , int numNodes
    , int* obj_materialOffsets
    , int hasobj
    )
{
    int path_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (path_index < num_paths)
    {
        //path_index = pathSegments[path_index].pixelIndex;
        PathSegment pathSegment = pathSegments[path_index];
        //printf("\nO1");
        if (pathSegments[path_index].remainingBounces>0)
        {
            float t;
            glm::vec3 intersect_point;
            glm::vec3 normal;
            float t_min = FLT_MAX;
            int hit_geom_index = -1;
            bool outside = true;

            glm::vec3 tmp_intersect;
            glm::vec3 tmp_normal;

            glm::vec3 hit;
            glm::vec3 norm;
            glm::vec3 bary;
            glm::vec3 v1;
            glm::vec3 v2;
            glm::vec3 v3;
            glm::vec3 n1;
            glm::vec3 n2;
            glm::vec3 n3;
            int pidxo1 = 0;
            int pidxo2 = 0;
            int pidxo3 = 0;
            bool intersected = false;
            bool obj_intersect = false;
            // naive parse through global geoms
            //printf("\nO2");

            int objMaterialIdx = -1;
            for (int i = 0; i < geoms_size; i++)
            {
                Geom & geom = geoms[i];

                if (geom.type == CUBE)
                {
                    t = boxIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }
                else if (geom.type == SPHERE)
                {
                    t = sphereIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }

                // Compute the minimum t from the intersection tests to determine what
                // scene geometry object was hit first.
                if (t > 0.0f && t_min > t)
                {
                    t_min = t;
                    hit_geom_index = i;
                    intersect_point = tmp_intersect;
                    normal = tmp_normal;
                }
            }

            objMaterialIdx = -1;
            int iterator = 0;
            if (hasobj)
            {
                /*
                traverseKDbare(nodes, numNodes, t,
                pathSegment, triangles,
                bary, objMaterialIdx,
                material_size, hit,
                norm, t_min,
                hit_geom_index, intersect_point,
                normal, tmp_intersect,
                tmp_normal, obj_intersect,
                intersections, obj_materialOffsets,
                path_index);
                */

                for (int i = 0; i < numNodes; i++)
                {
                    t = boxIntersectionTestBox(nodes[i].mins, nodes[i].maxs, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                    // Compute the minimum t from the intersection tests to determine what
                    // scene geometry object was hit first.
                    if (t > 0.0f && t_min > t)
                    {
                        t_min = t;
                        hit_geom_index = geoms_size;
                        intersect_point = tmp_intersect;
                        normal = tmp_normal;
                        obj_intersect = true;
                        objMaterialIdx = material_size - 1;
                    }
                }
            }

            if (hit_geom_index == -1)
            {
                intersections[path_index].t = -1.0f;
            }
            else
            {

                // updating rays
                //thrust::default_random_engine rng = makeSeededRandomEngine(iter, depth, depth); // WAY TOO COOL!
                thrust::default_random_engine rng = makeSeededRandomEngine(iter, path_index, depth);


                if (obj_intersect)
                {
                    pathSegments[path_index].materialIdHit = objMaterialIdx;

                    scatterRay(pathSegments[path_index].ray,
                               pathSegments[path_index].color,
                               intersect_point,
                               normal,
                               materials[objMaterialIdx],
                               rng,
                               softness);
                }
                else
                {
                    pathSegments[path_index].materialIdHit = geoms[hit_geom_index].materialid;

                    scatterRay(pathSegments[path_index].ray,
                               pathSegments[path_index].color,
                               intersect_point,
                               normal,
                               materials[geoms[hit_geom_index].materialid],
                               rng,
                               softness);
                }


                if (obj_intersect)
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = objMaterialIdx; // test material
                    intersections[path_index].surfaceNormal = normal;
                }
                else
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = geoms[hit_geom_index].materialid;
                    intersections[path_index].surfaceNormal = normal;
                }
            }
        }
    }
}






// pathTraceOneBounce handles ray intersections, generate intersections for shading, 
// and scatter new ray. You might want to call scatterRay from interactions.h
__global__ void pathTraceOneBounceKDbareShortStack(
    int depth
    , int iter
    , int num_paths
    , PathSegment * pathSegments
    , Geom * geoms
    , int geoms_size
    , Material * materials
    , int material_size
    , ShadeableIntersection * intersections
    , float softness
    , KDN::TriBare* triangles
    , int numTriangles
    , KDN::NodeBare* nodes
    , int numNodes
    , int* obj_materialOffsets
    , int hasobj
    )
{

    int path_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (path_index < num_paths)
    {
        //path_index = pathSegments[path_index].pixelIndex;
        PathSegment pathSegment = pathSegments[path_index];
        //printf("\nO1");
        if (pathSegments[path_index].remainingBounces>0)
        {
            float t = 0.0;
            glm::vec3 intersect_point;
            glm::vec3 normal;
            float t_min = FLT_MAX;
            int hit_geom_index = -1;
            bool outside = true;

            glm::vec3 tmp_intersect;
            glm::vec3 tmp_normal;

            glm::vec3 hit;
            glm::vec3 norm;
            glm::vec3 bary;
            glm::vec3 v1;
            glm::vec3 v2;
            glm::vec3 v3;
            glm::vec3 n1;
            glm::vec3 n2;
            glm::vec3 n3;
            int pidxo1 = 0;
            int pidxo2 = 0;
            int pidxo3 = 0;
            bool intersected = false;
            bool obj_intersect = false;
            // naive parse through global geoms
            //printf("\nO2");

            int objMaterialIdx = -1;
            for (int i = 0; i < geoms_size; i++)
            {
                Geom & geom = geoms[i];

                if (geom.type == CUBE)
                {
                    t = boxIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }
                else if (geom.type == SPHERE)
                {
                    t = sphereIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }

                // Compute the minimum t from the intersection tests to determine what
                // scene geometry object was hit first.
                if (t > 0.0f && t_min > t)
                {
                    t_min = t;
                    hit_geom_index = i;
                    intersect_point = tmp_intersect;
                    normal = tmp_normal;
                }
            }

            objMaterialIdx = -1;
            int iterator = 0;
            if (hasobj)
            {
                //printf("before traverse\n");
                ///*
                traverseKDshort(nodes, numNodes, t,
                           pathSegment, triangles,
                           bary, objMaterialIdx,
                           material_size, hit,
                           norm, t_min,
                           hit_geom_index, intersect_point,
                           normal, tmp_intersect,
                           tmp_normal, obj_intersect,
                           intersections, obj_materialOffsets,
                           path_index);
                //*/
                /*
                traverseKD(nodes, numNodes, t,
                                pathSegment, triangles,
                                bary, objMaterialIdx,
                                material_size, hit,
                                norm, t_min,
                                hit_geom_index, intersect_point,
                                normal, tmp_intersect,
                                tmp_normal, obj_intersect,
                                intersections, obj_materialOffsets,
                                path_index);
                */
                
                //printf("after traverse\n");
                
                /*
                thrust::device_vector<KDN::NodeBare> nodeStack;
                //KDN::NodeStackBare nodeStack;
                //stack<KDN::NodeBare> nodeStack;
                KDN::NodeBare node;
                KDN::NodeBare root;
                KDN::NodeBare first;
                KDN::NodeBare second;
                // get the root node
                for (int i = 0; i < numNodes; i++)
                {
                    if (nodes[i].parentID == -1)
                    {
                        node = nodes[i];
                        root = nodes[i];
                        break;
                    }
                }
                //
                //
                float tMin, tMax, tHit;
                tMin = tMax = 0.0f;
                tHit = t;
                bool pushdown = false;
                int axis = 0;
                float tSplit = 0.0f;

                while (tMax < t)
                {
                    if (nodeStack.empty())
                    {
                        node = root;
                        tMin = tMax;
                        tMax = t;
                        pushdown = true;
                    }
                    else
                    {
                        node = nodeStack.end()[0];
                        tMin = node.tmin;
                        tMax = node.tmax;
                        nodeStack.pop_back();
                        pushdown = false;
                    }
                    while (node.triIdSize != 0)
                    {
                        axis = node.axis;
                        tSplit = (node.splitPos - pathSegment.ray.origin[axis]) / pathSegment.ray.direction[axis];

                        if (pathSegment.ray.direction[axis] > 0)
                        {
                            if (nodes[node.leftID].mins[axis] < nodes[node.rightID].mins[axis])
                            {
                                first = nodes[node.leftID];
                                second = nodes[node.rightID];
                            }
                            else
                            {
                                first = nodes[node.rightID];
                                second = nodes[node.leftID];
                            }
                        }
                        else
                        {
                            if (nodes[node.leftID].mins[axis] > nodes[node.rightID].mins[axis])
                            {
                                first = nodes[node.leftID];
                                second = nodes[node.rightID];
                            }
                            else
                            {
                                first = nodes[node.rightID];
                                second = nodes[node.leftID];
                            }
                        }

                        if (tSplit >= tMax || tSplit < 0)
                            node = first;
                        else if (tSplit <= tMin)
                            node = second;
                        else
                        {
                            second.tmin = tMin;
                            second.tmax = tMax;
                            nodeStack.push_back(second);
                            node = first;
                            tMax = tSplit;
                            pushdown = false;
                        }
                        if (pushdown)
                            root = node;



                        int start = node.triIdStart;
                        int end = start + node.triIdSize;
                        for (int i = start; i < end; i++)
                        {
                            KDN::TriBare* T = &(triangles[i]);

                            glm::vec3 v1(T->x1, T->y1, T->z1);
                            glm::vec3 v2(T->x2, T->y2, T->z2);
                            glm::vec3 v3(T->x3, T->y3, T->z3);

                            glm::vec3 n1(T->nx1, T->ny1, T->nz1);
                            glm::vec3 n2(T->nx2, T->ny2, T->nz2);
                            glm::vec3 n3(T->nx3, T->ny3, T->nz3);

                            //bary(0.0f, 0.0f, 0.0f);
                            bool intersected = glm::intersectRayTriangle(pathSegment.ray.origin,
                                                                            pathSegment.ray.direction,
                                                                            v1, v2, v3, bary);

                            if (intersected)
                            {
                                objMaterialIdx = triangles[i].mtlIdx + material_size - 1;
                                //(1 - bary.x - bary.y); bary.x; bary.y
                                //printf("material id: %d\n", triangles[i].mtlIdx);
                                hit = pathSegment.ray.origin + pathSegment.ray.direction* bary.z;// (bary2.x * v1 + bary2.y * v2 + bary2.z * v3);
                                norm = -glm::normalize((1 - bary.x - bary.y) * n1 + bary.x * n2 + (bary.y) * n3);
                                //norm(glm::normalize(n1));
                                hit += norm*0.0001f;


                                t = glm::distance(pathSegment.ray.origin, hit);

                                if (t > 0.0f && t_min > t)
                                {
                                    tHit = min(tHit, t);

                                    t_min = t;
                                    hit_geom_index = obj_materialOffsets[T->mtlIdx];
                                    intersect_point = hit;
                                    normal = norm;
                                    tmp_intersect = hit;
                                    tmp_normal = normal;
                                    obj_intersect = true;
                                    intersections[path_index].t = t;
                                }
                            }
                        }
                    }
                }*/
            }



            if (hit_geom_index == -1)
            {
                intersections[path_index].t = -1.0f;
            }
            else
            {

                // updating rays
                //thrust::default_random_engine rng = makeSeededRandomEngine(iter, depth, depth); // WAY TOO COOL!
                thrust::default_random_engine rng = makeSeededRandomEngine(iter, path_index, depth);


                if (obj_intersect)
                {
                    pathSegments[path_index].materialIdHit = objMaterialIdx;

                    scatterRay(pathSegments[path_index].ray,
                               pathSegments[path_index].color,
                               intersect_point,
                               normal,
                               materials[objMaterialIdx],
                               rng,
                               softness);
                }
                else
                {
                    pathSegments[path_index].materialIdHit = geoms[hit_geom_index].materialid;

                    scatterRay(pathSegments[path_index].ray,
                               pathSegments[path_index].color,
                               intersect_point,
                               normal,
                               materials[geoms[hit_geom_index].materialid],
                               rng,
                               softness);
                }


                if (obj_intersect)
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = objMaterialIdx; // test material
                    intersections[path_index].surfaceNormal = normal;
                }
                else
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = geoms[hit_geom_index].materialid;
                    intersections[path_index].surfaceNormal = normal;
                }
            }
        }
    }
}

// pathTraceOneBounce handles ray intersections, generate intersections for shading, 
// This is the KD-tree implementation
__global__ void pathTraceOneBounceKD(
    int depth
    , int iter
    , int num_paths
    , PathSegment * pathSegments
    , Geom * geoms
    , int geoms_size
    , Material * materials
    , int material_size
    , ShadeableIntersection * intersections
    , float softness
    , KDN::Triangle* triangles
    , int numTriangles
    , KDN::KDnode* nodes
    , int numNodes
    , int hasobj
    )
{
    int path_index = blockIdx.x * blockDim.x + threadIdx.x;

    //printf("tracekd 1\n");
    if (path_index < num_paths)
    {
        //path_index = pathSegments[path_index].pixelIndex;
        PathSegment pathSegment = pathSegments[path_index];
        //printf("\nO1");
        if (pathSegments[path_index].remainingBounces>0)
        {
            //printf("tracekd 2\n");
            float t;
            glm::vec3 intersect_point;
            glm::vec3 normal;
            float t_min = FLT_MAX;
            int hit_geom_index = -1;
            bool outside = true;

            glm::vec3 tmp_intersect;
            glm::vec3 tmp_normal;

            glm::vec3 hit;
            glm::vec3 norm;
            glm::vec3 bary;
            glm::vec3 v1;
            glm::vec3 v2;
            glm::vec3 v3;
            glm::vec3 n1;
            glm::vec3 n2;
            glm::vec3 n3;
            int pidxo1 = 0;
            int pidxo2 = 0;
            int pidxo3 = 0;
            bool intersected = false;
            bool obj_intersect = false;
            // naive parse through global geoms
            //printf("\nO2");

            //printf("tracekd 3\n");
            int objMaterialIdx = -1;
            for (int i = 0; i < geoms_size; i++)
            {
                Geom & geom = geoms[i];

                if (geom.type == CUBE)
                {
                    t = boxIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }
                else if (geom.type == SPHERE)
                {
                    t = sphereIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal, outside);
                }

                // TODO KDTREE TRAVERSAL

                // Compute the minimum t from the intersection tests to determine what
                // scene geometry object was hit first.
                
                if (t > 0.0f && t_min > t)
                {
                    t_min = t;
                    hit_geom_index = i;
                    intersect_point = tmp_intersect;
                    normal = tmp_normal;
                }
                
            }

            //printf("tracekd 4\n");

            // start polygon hits
            //t_min = FLT_MAX;
            //for (int i = 0; i < obj_numshapes; i++)
            //    printf("\noffset = %d", obj_polyoffsets[i]);


            //printf("\nO3");
            //printf("\nNUMSHAPES = %d\n", obj_numshapes);
            objMaterialIdx = -1;
            int iterator = 0;
            if (hasobj)
            {
                //printf("tracekd 5\n");
                float dist = -1.0f;
                glm::vec3 norm;

                //printf("BEFORE ALLOCATING\n");
                //bool* nodeIDs = new bool[numNodes];
                // delete [] nodeIDs;

                
                //printf("AFTER ALLOCATING\n");

                //printf("BEFORE INTERSECT\n");
                //glm::vec4 intersection = intersectKDLoopDeref(pathSegment.ray, nodes, numNodes, triangles, numTriangles);
                //norm.x = intersection.x;
                //norm.y = intersection.y;
                //norm.z = intersection.z;
                //dist = intersection.w;










                dist = -1.0f;
                norm = glm::vec3(0.0f);
                bool hitGeom = false;

                Ray r = pathSegment.ray;

                // USE AN ARRAY OF 0 NODE IDS AND SET THEM TO 1 once they're visited
                // instead of using visited to avoid conflicts when reading from
                // multiple threads
                bool nodeIDs[1000] = { false };
                //memset(nodeIDs, 0, sizeof(bool)*numNodes);



                if (numNodes != 0)
                {
                    float mindist = FLT_MAX;
                    int currID = nodeIDs[nodes[0].ID];

                    // get the root node
                    for (int i = 0; i < numNodes; i++)
                    {
                        if (nodes[i].parentID == -1)
                        {
                            currID = nodes[i].ID;
                            break;
                        }
                    }

                    float boxdist = -1.0f;
                    while (true)
                    {
                        if (currID == -1)
                            break;

                        // check if it intersects the bounds
                        //printf("1\n");
                        hitGeom = intersectAABB(r, nodes[currID].bbox, dist);
                        //printf("2\n");

                        if (hitGeom == false)
                        {
                            nodeIDs[nodes[currID].ID] = true;
                            currID = nodes[currID].parentID;
                        }
                        else
                        {
                            if (nodes[currID].leftID != -1 && nodeIDs[nodes[currID].leftID] != true)
                                currID = nodes[currID].leftID;
                            else if (nodes[currID].rightID != -1 && nodeIDs[nodes[currID].rightID] != true)
                                currID = nodes[currID].rightID;
                            else if (nodeIDs[nodes[currID].ID] == false)
                            {
                                //std::cout << "NODE LOOP: " << nodes[currID].ID << " PARENT: " << nodes[currID].parentID << std::endl;
                                nodeIDs[nodes[currID].ID] = true;

                                int size = nodes[currID].triIdSize;
                                if (size > 0)
                                {
                                    int start = nodes[currID].triIdStart;
                                    int end = start + size;
                                    for (int i = start; i < end; i++)
                                    {
                                        //KDN::Triangle t = triangles[i];

                                        glm::vec3 v1(triangles[i].x1, triangles[i].y1, triangles[i].z1);
                                        glm::vec3 v2(triangles[i].x2, triangles[i].y2, triangles[i].z2);
                                        glm::vec3 v3(triangles[i].x3, triangles[i].y3, triangles[i].z3);

                                        glm::vec3 barytemp(0.0f, 0.0f, 0.0f);
                                        bool intersected = glm::intersectRayTriangle(r.origin,
                                                                                     r.direction,
                                                                                     v1, v2, v3, barytemp);

                                        if (intersected && barytemp.z < mindist)
                                        {
                                            glm::vec3 bary(barytemp.x, barytemp.y, 1.0 - barytemp.x - barytemp.y);

                                            glm::vec3 n1(triangles[i].nx1, triangles[i].ny1, triangles[i].nz1);
                                            glm::vec3 n2(triangles[i].nx2, triangles[i].ny2, triangles[i].nz2);
                                            glm::vec3 n3(triangles[i].nx3, triangles[i].ny3, triangles[i].nz3);
                                            norm = (bary[0] * n1 + bary[1] * n2 + bary[2] * n3);

                                            dist = barytemp.z;
                                            mindist = dist;
                                            //glm::vec3 pos = r.origin + r.direction * dist;

                                            glm::vec3 intersect = r.origin + r.direction*dist;
                                            //printf("KDLOOPPTR INTERSECT POINT: P: [%f %f %f] NODEID: %d\n", intersect.x,
                                            //       intersect.y,
                                            //       intersect.z,
                                            //       currID);


                                            norm = glm::normalize((1 - bary.x - bary.y) * n1 + bary.x * n2 + (bary.y) * n3);
                                            //norm(glm::normalize(n1));
                                            //intersect += norm*0.0001f;

                                                
                                            t = dist;
                                                
                                            if (t > 0.0f && t_min > t)
                                            {
                                                t_min = t;
                                                hit_geom_index = 0;// obj_materialOffsets[i];
                                                intersect_point = intersect;
                                                tmp_intersect = intersect;
                                                tmp_normal = norm;//glm::vec3(0.0f, 1.0f, 0.0f);
                                                //obj_intersect = true;
                                                intersections[path_index].t = t;
                                            }
                                        }
                                    }
                                }
                            }
                            else
                                currID = nodes[currID].parentID;
                        }
                    }
                }

                //printf("AFTER INTERSECT\n");
                //printf("tracekd 5.1\n");
                if (hit_geom_index != -1)
                {
                    hit_geom_index = 0;
                    obj_intersect = true;
                    t_min = dist;
                    intersect_point = tmp_intersect;
                    //printf("tracekd 5.2\n");
                    // TODO add normals to Triangle class and get them
                    // testing with default normal
                    normal = tmp_normal;
                    //printf("tracekd 5.3\n");
                }
            }

            //printf("tracekd 6\n");

            //printf("\nO11");

            // TODO: scatter the ray, generate intersections for shading
            // feel free to modify the code below

            if (hit_geom_index == -1)
            {
                intersections[path_index].t = -1.0f;
            }
            else
            {
                //The ray hits something
                //intersections[path_index].t = t_min;
                //intersections[path_index].materialId = geoms[hit_geom_index].materialid;
                //intersections[path_index].surfaceNormal = normal;


                // updating rays
                //thrust::default_random_engine rng = makeSeededRandomEngine(iter, depth, depth); // WAY TOO COOL!
                thrust::default_random_engine rng = makeSeededRandomEngine(iter, path_index, depth);

                //printf("tracekd 7\n");
                if (obj_intersect)// && false)
                {
                    // testing for now;
                    objMaterialIdx = 0;

                    pathSegments[path_index].materialIdHit = objMaterialIdx;

                    scatterRay(pathSegments[path_index].ray,
                               pathSegments[path_index].color,
                               intersect_point,
                               normal,
                               materials[objMaterialIdx],
                               rng,
                               softness);
                }
                else
                {
                    pathSegments[path_index].materialIdHit = geoms[hit_geom_index].materialid;

                    scatterRay(pathSegments[path_index].ray,
                               pathSegments[path_index].color,
                               intersect_point,
                               normal,
                               materials[geoms[hit_geom_index].materialid],
                               rng,
                               softness);
                }
                //printf("tracekd 8\n");
                if (obj_intersect)// && false)
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = geoms[0].materialid; //objMaterialIdx; // test material
                    intersections[path_index].surfaceNormal = normal;
                }
                else
                {
                    intersections[path_index].t = t_min;
                    intersections[path_index].materialId = geoms[hit_geom_index].materialid;
                    intersections[path_index].surfaceNormal = normal;
                }
                //printf("tracekd 9\n");
            }
        }
    }
}


__global__ void shadeMaterial(
    int iter
    , int num_paths
    , ShadeableIntersection * shadeableIntersections
    , PathSegment * pathSegments
    , Material * materials
    , bool enablesss
    )
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < num_paths)
    {
        //idx = pathSegments[idx].initialidx;
        //idx = pathSegments[idx].pixelIndex;
        if (pathSegments[idx].remainingBounces>0)
        {
            ShadeableIntersection intersection = shadeableIntersections[idx];
            if (intersection.t > 0.0f) 
            { // if the intersection exists...
                // Set up the RNG
                thrust::default_random_engine rng = makeSeededRandomEngine(iter, idx, 0);
                thrust::uniform_real_distribution<float> u01(0, 1);

                Material material = materials[intersection.materialId];
                glm::vec3 materialColor = material.color;

                // If the material indicates that the object was a light, "light" the ray
                if (material.emittance > 0.0f) {
                    pathSegments[idx].color *= (materialColor * material.emittance);
                    pathSegments[idx].remainingBounces = 0;
                }
                // Otherwise, do some pseudo-lighting computation. This is actually more
                // like what you would expect from shading in a rasterizer like OpenGL.
                else {
                    //float lightTerm = glm::dot(intersection.surfaceNormal, glm::vec3(0.0f, 1.0f, 0.0f));

                    //if (pathSegments[idx].ray.isrefr)
                    //{
                    //    pathSegments[idx].color *= (materialColor * lightTerm) * 0.3f + materialColor * 0.7f + material.hasRefractive * materialColor;
                    //}

                    //else if (pathSegments[idx].ray.isrefl)
                    //{
                    //    pathSegments[idx].color *= (materialColor * lightTerm) * 0.3f + materialColor * 0.7f + material.hasReflective * materialColor;
                    //}

                    if (enablesss && (material.transmittance.x > 0.0f || material.transmittance.y > 0.0f || material.transmittance.z > 0.0f))
                    {
                        float scenescale = 1.0f;
                        float sss = scenescale * pathSegments[idx].ray.sdepth > 1.0 ? 1.0 : pathSegments[idx].ray.sdepth;
                        sss = 1.0f - sss < 0.0 ? 0.0 : sss;
                        sss = glm::pow(sss, 2);
                        pathSegments[idx].color *= (materialColor)* 1.0f + material.hasRefractive * material.specular.color + sss * material.transmittance;
                    }
                    else if (material.hasRefractive > 0.0f)
                    {
                        pathSegments[idx].color *= (materialColor)* 1.0f + material.hasRefractive * material.specular.color;
                    }
                    else if (material.hasReflective > 0.0f)
                    {
                        pathSegments[idx].color *= (materialColor)* 1.0f + material.hasReflective * material.specular.color;
                    }
                    else
                    {
                        pathSegments[idx].color *= (materialColor) * 1.0f;
                    }

                    pathSegments[idx].remainingBounces--;
                }
                // If there was no intersection, color the ray black.
                // Lots of renderers use 4 channel color, RGBA, where A = alpha, often
                // used for opacity, in which case they can indicate "no opacity".
                // This can be useful for post-processing and image compositing.
            }
            else {
                pathSegments[idx].color = glm::vec3(0.0f);
                pathSegments[idx].remainingBounces = 0;
            }
        }
    }
}



// LOOK: "fake" shader demonstrating what you might do with the info in
// a ShadeableIntersection, as well as how to use thrust's random number
// generator. Observe that since the thrust random number generator basically
// adds "noise" to the iteration, the image should start off noisy and get
// cleaner as more iterations are computed.
//
// Note that this shader does NOT do a BSDF evaluation!
// Your shaders should handle that - this can allow techniques such as
// bump mapping.
__global__ void shadeFakeMaterial(
    int iter
    , int num_paths
    , ShadeableIntersection * shadeableIntersections
    , PathSegment * pathSegments
    , Material * materials
    )
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < num_paths)
    {
        ShadeableIntersection intersection = shadeableIntersections[idx];
        if (intersection.t > 0.0f) { // if the intersection exists...
            // Set up the RNG
            thrust::default_random_engine rng = makeSeededRandomEngine(iter, idx, 0);
            thrust::uniform_real_distribution<float> u01(0, 1);

            Material material = materials[intersection.materialId];
            glm::vec3 materialColor = material.color;

            // If the material indicates that the object was a light, "light" the ray
            if (material.emittance > 0.0f) {
                pathSegments[idx].color *= (materialColor * material.emittance);
            }
            // Otherwise, do some pseudo-lighting computation. This is actually more
            // like what you would expect from shading in a rasterizer like OpenGL.
            else {
                float lightTerm = glm::dot(intersection.surfaceNormal, glm::vec3(0.0f, 1.0f, 0.0f));
                pathSegments[idx].color *= (materialColor * lightTerm) * 0.3f + ((1.0f - intersection.t * 0.02f) * materialColor) * 0.7f;
                pathSegments[idx].color *= u01(rng); // apply some noise because why not
            }
            // If there was no intersection, color the ray black.
            // Lots of renderers use 4 channel color, RGBA, where A = alpha, often
            // used for opacity, in which case they can indicate "no opacity".
            // This can be useful for post-processing and image compositing.
        }
        else {
            pathSegments[idx].color = glm::vec3(0.0f);
        }
    }
}

// Add the current iteration's output to the overall image
__global__ void finalGather(int nPaths, glm::vec3 * image, PathSegment * iterationPaths)
{
    int index = (blockIdx.x * blockDim.x) + threadIdx.x;

    if (index < nPaths)
    {
        index = iterationPaths[index].pixelIndex;
        PathSegment iterationPath = iterationPaths[index];
        image[iterationPath.pixelIndex] += iterationPath.color;
    }
}

// Add the current iteration's output to the current image
__global__ void partialGather(int nPaths, glm::vec3 * image, PathSegment * iterationPaths)
{
    int index = (blockIdx.x * blockDim.x) + threadIdx.x;

    if (index < nPaths)
    {
        //index = iterationPaths[index].pixelIndex;
        if (iterationPaths[index].remainingBounces == 0)
        {
            PathSegment iterationPath = iterationPaths[index];
            image[iterationPath.pixelIndex] += iterationPath.color;
        }
    }
}

/**
* Wrapper for the __global__ call that sets up the kernel calls and does a ton
* of memory management
*/
void pathtrace(uchar4 *pbo, 
               int frame, 
               int iter, 
               float focalLength, 
               float dofAngle, 
               bool cacherays, 
               bool antialias, 
               float softness, 
               bool enableSss,
               bool testingmode,
               bool compaction,
               bool enablekd,
               bool vizkd) {
    const int traceDepth = hst_scene->state.traceDepth;
    const Camera &cam = hst_scene->state.camera;
    const int pixelcount = cam.resolution.x * cam.resolution.y;

    // 2D block for generating ray from camera
    const dim3 blockSize2d(8, 8);
    const dim3 blocksPerGrid2d(
        (cam.resolution.x + blockSize2d.x - 1) / blockSize2d.x,
        (cam.resolution.y + blockSize2d.y - 1) / blockSize2d.y);

    // 1D block for path tracing
    const int blockSize1d = 32;

    ///////////////////////////////////////////////////////////////////////////

    // Recap:
    // * Initialize array of path rays (using rays that come out of the camera)
    //   * You can pass the Camera object to that kernel.
    //   * Each path ray must carry at minimum a (ray, color) pair,
    //   * where color starts as the multiplicative identity, white = (1, 1, 1).
    //   * This has already been done for you.
    // * For each depth:
    //   * Compute an intersection in the scene for each path ray.
    //     A very naive version of this has been implemented for you, but feel
    //     free to add more primitives and/or a better algorithm.
    //     Currently, intersection distance is recorded as a parametric distance,
    //     t, or a "distance along the ray." t = -1.0 indicates no intersection.
    //     * Color is attenuated (multiplied) by reflections off of any object
    //   * TODO: Stream compact away all of the terminated paths.
    //     You may use either your implementation or `thrust::remove_if` or its
    //     cousins.
    //     * Note that you can't really use a 2D kernel launch any more - switch
    //       to 1D.
    //   * TODO: Shade the rays that intersected something or didn't bottom out.
    //     That is, color the ray by performing a color computation according
    //     to the shader, then generate a new ray to continue the ray path.
    //     We recommend just updating the ray's PathSegment in place.
    //     Note that this step may come before or after stream compaction,
    //     since some shaders you write may also cause a path to terminate.
    // * Finally, add this iteration's results to the image. This has been done
    //   for you.

    // TODO: perform one iteration of path tracing
    
    cudaEvent_t startGenRayFromCam, stopGenRayFromCam;
    cudaEvent_t startPathTraceOneBounce, stopPathTraceOneBounce;
    cudaEvent_t startShadeMaterial, stopShadeMaterial;
    float millisecondsGenRayFromCam = 0.0f;
    float millisecondsPathTraceOneBounce = 0.0f;
    float millisecondsShadeMaterial = 0.0f;

    float ms1 = 0.0;
    float ms2 = 0.0;
    float ms3 = 0.0;
    /*
    if (testingmode)
    {
        cudaEventCreate(&startGenRayFromCam); cudaEventCreate(&stopGenRayFromCam); cudaEventRecord(startGenRayFromCam);
    }
    */
    // cache rays
    if (cacherays)
    {
        if (iter == 1)
        {
            generateRayFromCamera << <blocksPerGrid2d, blockSize2d >> >(cam, iter, traceDepth, dev_paths_cache, focalLength, dofAngle, antialias);
            checkCUDAError("generate camera ray");
        }
        cudaMemcpy(dev_paths, dev_paths_cache, pixelcount*sizeof(PathSegment), cudaMemcpyDeviceToDevice);
    }
    else
    {
        generateRayFromCamera << <blocksPerGrid2d, blockSize2d >> >(cam, iter, traceDepth, dev_paths, focalLength, dofAngle, antialias);
        checkCUDAError("generate camera ray");
    }
    /*
    if (testingmode)
    {
        cudaEventRecord(stopGenRayFromCam); cudaEventSynchronize(stopGenRayFromCam);
        ms1 = 0; 
        cudaEventElapsedTime(&ms1, startGenRayFromCam, stopGenRayFromCam);
        //printf("\ngenerateRayFromCamera time = %f", ms1);
        millisecondsGenRayFromCam = ms1;
        cudaEventDestroy(startGenRayFromCam);
        cudaEventDestroy(stopGenRayFromCam);
    }
    */
    int depth = 0;
    PathSegment* dev_path_end = dev_paths + pixelcount;
    int num_paths = dev_path_end - dev_paths;
    int num_paths_temp = num_paths;
    // --- PathSegment Tracing Stage ---
    // Shoot ray into scene, bounce between objects, push shading chunks

    //PathSegment* paths;
    //cudaMalloc(&paths, sizeof(PathSegment)*pixelcount);
    //cudaMemcpy(paths, dev_paths, sizeof(PathSegment)*pixelcount);
    
    bool iterationComplete = false;
    while (!iterationComplete) {
        // clean shading chunks
        cudaMemset(dev_intersections, 0, pixelcount * sizeof(ShadeableIntersection));

        // tracing
        dim3 numblocksPathSegmentTracing = (num_paths + blockSize1d - 1) / blockSize1d;
        
        if (testingmode)
        {
            cudaEventCreate(&startPathTraceOneBounce); cudaEventCreate(&stopPathTraceOneBounce); cudaEventRecord(startPathTraceOneBounce);
        }
        
        if (enablekd == false)
        {
            pathTraceOneBounce << <numblocksPathSegmentTracing, blockSize1d >> > (
                depth
                , iter
                , num_paths
                , dev_paths
                , dev_geoms
                , hst_scene->geoms.size()
                , dev_materials
                , hst_scene->materials.size()
                , dev_intersections
                , softness
                , hst_scene->obj_numshapes
                , obj_numpolyverts
                , obj_verts
                , obj_norms
                , obj_texts
                , obj_polyoffsets
                , obj_polysidxflat
                , obj_polysbboxes
                , hst_scene->polyidxcount
                //, obj_RGB
                //, obj_SPECEX
                //, obj_SPECRGB
                //, obj_REFL
                //, obj_REFR
                //, obj_REFRIOR
                , obj_materialOffsets
                , hst_scene->hasObj);
            checkCUDAError("trace one bounce");
            ///*
            //printf("numNodes = %d\n", hst_scene->numNodes);
            //printf("numTriangles = %d\n", hst_scene->numTriangles);

        }
        else
        {
            if (vizkd)
            {
                pathTraceOneBounceKDbareBoxes << <numblocksPathSegmentTracing, blockSize1d >> > (
                    depth
                    , iter
                    , num_paths
                    , dev_paths
                    , dev_geoms
                    , hst_scene->geoms.size()
                    , dev_materials
                    , hst_scene->materials.size()
                    , dev_intersections
                    , softness
                    , kd_trianglesBare
                    , hst_scene->numTriangles
                    , kd_nodesBare
                    , hst_scene->numNodes
                    , obj_materialOffsets
                    , hst_scene->hasObj);
                checkCUDAError("trace one bounce kd");
                //cudaEventQuery(0);
            }
            else
            {
                pathTraceOneBounceKDbare << <numblocksPathSegmentTracing, blockSize1d >> > (
                    depth
                    , iter
                    , num_paths
                    , dev_paths
                    , dev_geoms
                    , hst_scene->geoms.size()
                    , dev_materials
                    , hst_scene->materials.size()
                    , dev_intersections
                    , softness
                    , kd_trianglesBare
                    , hst_scene->numTriangles
                    , kd_nodesBare
                    , hst_scene->numNodes
                    , obj_materialOffsets
                    , hst_scene->hasObj);
                checkCUDAError("trace one bounce kd");
                //cudaEventQuery(0);
            }
        }

        cudaDeviceSynchronize();
        depth++;

        if (testingmode)
        {
            cudaEventRecord(stopPathTraceOneBounce); cudaEventSynchronize(stopPathTraceOneBounce);
            ms2 = 0;
            cudaEventElapsedTime(&ms2, startPathTraceOneBounce, stopPathTraceOneBounce);
            //printf("\npath trace time = %f", ms2);
            millisecondsPathTraceOneBounce += ms2;
            cudaEventDestroy(startPathTraceOneBounce);
            cudaEventDestroy(stopPathTraceOneBounce);
        }

        /*
        if (testingmode)
        {
            cudaEventCreate(&startShadeMaterial); cudaEventCreate(&stopShadeMaterial); cudaEventRecord(startShadeMaterial);
        }
        */
        shadeMaterial << <numblocksPathSegmentTracing, blockSize1d >> > (
            iter,
            num_paths,
            dev_intersections,
            dev_paths,
            dev_materials,
            enableSss
            );
        /*
        if (testingmode)
        {
            cudaEventRecord(stopShadeMaterial); cudaEventSynchronize(stopShadeMaterial);
            ms3 = 0;
            millisecondsShadeMaterial = 0;
            cudaEventElapsedTime(&ms3, startGenRayFromCam, stopShadeMaterial);
            //printf("\ngenerateRayFromCamera time = %f", ms3);
            millisecondsShadeMaterial += ms3;
            cudaEventDestroy(startGenRayFromCam);
            cudaEventDestroy(stopShadeMaterial);
        }
        */
        //if (depth > 2)
        //if (num_paths <= 0)
        //    iterationComplete = true; // TODO: should be based off stream compaction results.

        if (compaction)
        {
            dim3 numBlocksPixels = (pixelcount + blockSize1d - 1) / blockSize1d;
            partialGather << <numBlocksPixels, blockSize1d >> >(num_paths, dev_image, dev_paths);
        }

        if (compaction)
        {
            thrust::device_ptr<PathSegment> thrust_paths(dev_paths);
            thrust::device_ptr<PathSegment> P = thrust::remove_if(thrust_paths, thrust_paths + num_paths, is_zero_bounce());
            num_paths_temp = P - thrust_paths;
            num_paths = num_paths_temp;
        }
        
        // with initial sort 00:45.32 without initial sort 00:49.13 with continous sort 01:11:73
        
        // after first hit
        if (iter == 2)
        {
            //printf("\nSORTING\n");
            thrust::device_ptr<PathSegment> thrust_paths2(dev_paths);
            thrust::sort(thrust_paths2, thrust_paths2 + num_paths);
            thrust::device_ptr<ShadeableIntersection> thrust_intersections(dev_intersections);
            thrust::sort(thrust_intersections, thrust_intersections + num_paths);
        }
        
        // stop if numpaths is 0 or depth > 8 when testing without compaction
        if (num_paths <= 0 || depth > 7)
            iterationComplete = true; // TODO: should be based off stream compaction results.  
    }
    
    if (testingmode)
    {
        //printf("\n\n-------- average times --------");
        //printf("\ngenerateRayFromCamera time = %f", millisecondsGenRayFromCam);
        printf("\n   pathTraceOneBounce time = %f", millisecondsPathTraceOneBounce);// / iter);
        //printf("\n        shadeMaterial time = %f\n", millisecondsShadeMaterial / iter);

        //printf("\n[%f, %f, %f], ", millisecondsGenRayFromCam, 
        //                           millisecondsPathTraceOneBounce / iter, 
        //                           millisecondsShadeMaterial / iter);
    }
    

    if (!compaction)
    {
        // Assemble this iteration and apply it to the image
        dim3 numBlocksPixels = (pixelcount + blockSize1d - 1) / blockSize1d;
        finalGather << <numBlocksPixels, blockSize1d >> >(num_paths, dev_image, dev_paths);
    }

    /*
    //printf("\ndev_paths %d\n", dev_paths[0].color.r);
    thrust::device_ptr<PathSegment> thrust_paths(dev_paths);
    thrust::device_ptr<PathSegment> P = thrust::remove_if(thrust_paths, thrust_paths + num_paths, is_zero_bounce());
    num_paths_temp = P - thrust_paths;
    num_paths -= num_paths_temp;
    */
    ///////////////////////////////////////////////////////////////////////////

    // Send results to OpenGL buffer for rendering
    //if (iter == 25) 
    sendImageToPBO << <blocksPerGrid2d, blockSize2d >> >(pbo, cam.resolution, iter, dev_image);

    // Retrieve image from GPU
    cudaMemcpy(hst_scene->state.image.data(), dev_image,
        pixelcount * sizeof(glm::vec3), cudaMemcpyDeviceToHost);

    checkCUDAError("pathtrace");
}


