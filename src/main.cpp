#include "main.h"
#include "preview.h"
#include <cstring>

#include "objmesh.h"


#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#include <random>
#include <stdio.h>
#include <vector>

#include <string>
#include <fstream>

#include <iostream>
#include <iomanip>

#include <Shlobj.h>

#include "KDnode.h"
#include "KDtree.h"

#include <glm/gtx/intersect.hpp>

static std::string startTimeString;

// For camera controls
static bool leftMousePressed = false;
static bool rightMousePressed = false;
static bool middleMousePressed = false;
static double lastX;
static double lastY;

static bool camchanged = true;
static float dtheta = 0, dphi = 0;
static glm::vec3 cammove;

static bool rayCaching = false;
static bool antialias = true;
static float softness = 0.0f;
static bool SSS = true;

float zoom, theta, phi;
glm::vec3 cameraPosition;
glm::vec3 ogLookAt; // for recentering the camera
glm::vec3 camoffset;
Scene *scene;
RenderState *renderState;
int iteration;

int width;
int height;

static float dofAngle = 0.03f;
static float dofDistance = 6.0f;

static bool TESTINGMODE = false;
static bool COMPACTION = true;


/*
struct saxpy_functor2 { 
    const float a; 
    saxpy_functor2(float _a) : a(_a) {} 
    __host__ __device__ 
        float operator()(const float& x, const float& y) const 
    { 
        return a * x + y; 
    } 
}; 
void saxpy_fast2(float A, thrust::device_vector<float>& X, thrust::device_vector<float>& Y) 
{ 
    // Y <- A * X + Y 
    thrust::transform(X.begin(), X.end(), Y.begin(), Y.begin(), saxpy_functor2(A)); 
}

*/

// this are for testing purposes only
glm::vec3 multiplyVmat(glm::mat4 m, glm::vec4 v) {
    return glm::vec3(m * v);
}
// this is for testing purposes only
glm::vec3 getRayPoint(Ray r, float t) {
    return r.origin + (t - .0001f) * glm::normalize(r.direction);
}
// interection test for development
float intersectBox(Geom box, Ray r,
                   glm::vec3 &intersectionPoint, glm::vec3 &normal, bool &outside) {
    Ray q;
    q.origin = multiplyVmat(box.inverseTransform, glm::vec4(r.origin, 1.0f));
    q.direction = glm::normalize(multiplyVmat(box.inverseTransform, glm::vec4(r.direction, 0.0f)));

    float tmin = -1e38f;
    float tmax = 1e38f;
    glm::vec3 tmin_n;
    glm::vec3 tmax_n;
    for (int xyz = 0; xyz < 3; ++xyz) {
        float qdxyz = q.direction[xyz];
        /*if (glm::abs(qdxyz) > 0.00001f)*/ {
            float t1 = (-0.5f - q.origin[xyz]) / qdxyz;
            float t2 = (+0.5f - q.origin[xyz]) / qdxyz;
            float ta = min(t1, t2);
            float tb = max(t1, t2);
            glm::vec3 n;
            n[xyz] = t2 < t1 ? +1 : -1;
            if (ta > 0 && ta > tmin) {
                tmin = ta;
                tmin_n = n;
            }
            if (tb < tmax) {
                tmax = tb;
                tmax_n = n;
            }
        }
    }

    if (tmax >= tmin && tmax > 0) {
        outside = true;
        if (tmin <= 0) {
            tmin = tmax;
            tmin_n = tmax_n;
            outside = false;
        }
        intersectionPoint = multiplyVmat(box.transform, glm::vec4(getRayPoint(q, tmin), 1.0f));
        normal = glm::normalize(multiplyVmat(box.transform, glm::vec4(tmin_n, 0.0f)));
        return glm::length(r.origin - intersectionPoint);
    }
    return -1;
}

std::vector<KDN::Triangle*> getTrianglesFromFile(const char* path)
{
    std::vector<KDN::Triangle*>triangles;

    string line;
    ifstream file(path);

    if (file.is_open())
    {
        int iter = 0;
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


// fast AABB intersection
bool intersectAABB(Ray r, KDN::BoundingBox b, float& dist)
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
        return false;
    }

    if (dmin > dmax)
    {
        dist = dmax;
        return false;
    }

    dist = dmin;

    return true;
}


float intersectKD(Ray r, KDN::KDnode* node, float mindist=FLT_MAX)
{
    r.origin = glm::vec3(2.0f, 2.0f, 2.0f);
    r.direction = glm::vec3(-0.5f, -0.5f, -0.71f);
    glm::normalize(r.direction);

    float dist;
    bool hit = intersectAABB(r, node->bbox, dist);

    if (hit)
    {
        dist = 0.0;
        // intersect triangles if node is leaf
        int numtris = node->triangles.size();
        if ( numtris != 0)
        {
            for (int i = 0; i < numtris; i++)
            {
                KDN::Triangle* t = node->triangles[i];

                glm::vec3 v1(t->x1, t->y1, t->z1);
                glm::vec3 v2(t->x2, t->y2, t->z2);
                glm::vec3 v3(t->x3, t->y3, t->z3);

                glm::vec3 barytemp(0.0f, 0.0f, 0.0f);
                bool intersected = glm::intersectRayTriangle(r.origin,
                                                        r.direction,
                                                        v3, v2, v1, barytemp);

                //glm::vec3 bary(barytemp.x, barytemp.y, 1.0 - barytemp.x - barytemp.y);
                printf("DIST %f ", dist);
                printf("MINDIST %f ", mindist);
                printf("INTERSECTED %d\n", intersected);

                if (intersected && barytemp.z < mindist)
                {
                    dist = barytemp.z;
                    mindist = dist;
                    //glm::vec3 pos = r.origin + r.direction * dist;

                    glm::vec3 intersect = r.origin + r.direction*dist;
                    printf("INTERSECT POINT: P: [%f %f %f]\n", intersect.x, intersect.y, intersect.z);
                }
            }
        }

        /*
        if (dist < mindist || mindist == -1)
        {
            mindist = dist;

            glm::vec3 intersect = r.origin + r.direction*dist;
            printf("INTERSECT POINT: P: [%f %f %f]\n", intersect.x, intersect.y, intersect.z);
        }
        */

        if (node->left)
            intersectKD(r, node->left, mindist);

        if (node->right)
            intersectKD(r, node->right, mindist);
    }

    return dist;
}


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
        else if (currNode->visited == 0)
        {
            std::cout << "NODE LOOP: " << currNode << std::endl;
            nodes.push_back(currNode);
            currNode->visited = 1;
        }
        else
            currNode = currNode->parent;
    }
}

float intersectKDLoop(Ray r, vector<KDN::KDnode*> nodes)
{
    float dist = 0.0;
    bool hit = false;

    r.origin = glm::vec3(2.0f, 2.0f, 2.0f);
    r.direction = glm::vec3(-0.5f, -0.5f, -0.71f);
    glm::normalize(r.direction);


    for (int i = 0; i < nodes.size(); i++)
    {

    }


    //hit = intersectAABB(r, node->bbox, dist);

    return dist;
}


//-------------------------------
//-------------MAIN--------------
//-------------------------------

int main(int argc, char** argv) {
    startTimeString = currentTimeString();


    if (argc > 1)
    {
        // test mode for kdtree
        // run test geometry before exiting

        for (int i = 1; i < argc; i++)
        {
            if (strcmp(argv[i], "-t") == 0)
            {
                printf("\n running test\n");

                // read file generated from Houdini and get triangles
                char path[1024];
                if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_PROFILE, NULL, 0, path)))
                {
                    printf("path = %s\n", path);
                }
                strcat_s(path, sizeof(char) * 1024, "/git/CIS565/kdtreePathTracerOptimization/rnd/houdini/data");
                printf("path = %s\n", path);

                // read the file and save triangles into vector
                std::vector<KDN::Triangle*> triangles = getTrianglesFromFile(path);
                

                // test kdtree class generator
                //KDtree* KDT = new KDtree(path);
                KDtree* KDT = new KDtree(triangles);
                KDT->rootNode->updateBbox();


                KDT->printTree();

                cout << KDT->rootNode << endl;
                cout << KDT->rootNode->getRoot() << endl;


                KDT->rootNode->printTriangleCenters();
                KDT->printTree();

                KDT->split(1);

                printf("\nreprinting after split\n");
                KDT->printTree();

                                
                KDT->rootNode->printTriangleCenters();

                cout << KDT << endl;


                // write out data before quitting
                char pathout[1024];
                if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_PROFILE, NULL, 0, pathout)))
                {
                    printf("path = %s\n", pathout);
                }
                strcat_s(pathout, sizeof(char) * 1024, "/git/CIS565/kdtreePathTracerOptimization/rnd/houdini/dataout");
                printf("path = %s\n", pathout);

                KDT->writeKDtoFile(KDT->rootNode, pathout);



                // test collisions
                // SPHERE INTERSECTION TEST
                // INITIAL POSITION AND NORMAL:
                // POS:  2.0  2.0  2.0
                // NOR: -0.5 -0.5 -0.707
                // HIT POSITION:
                // POS: 0.695252 0.695252 0.154808



                Ray r;
                r.origin = glm::vec3(2.0f, 2.0f, 2.0f);
                r.direction = glm::vec3(-0.5f, -0.5f, -0.71f);
                glm::normalize(r.direction);


                intersectKD(r, KDT->rootNode);




                // Accessing kd nodes and triangles as a flat structure
                // This is to help recursion removal for CUDA
                // THANK YOU NVIDIA for this...
                vector<KDN::KDnode*> nodes;
                getKDnodes(KDT->rootNode, nodes);

                vector<KDN::KDnode*> nodesLoop;
                getKDnodesLoop(KDT->rootNode, nodesLoop);

                for (int i = 0; i < nodes.size(); i++)
                {
                    std::cout << "node: " << nodes[i] << " numtris: " << nodes[i]->triangles.size() << std::endl;
                }

                for (int i = 0; i < nodes.size(); i++)
                {
                    std::cout << "nodeLoop: " << nodes[i] 
                              << " id: " << nodes[i]->ID 
                              << " parent id: " << nodes[i]->parentID
                              << " left id: " << nodes[i]->leftID
                              << " right id: " << nodes[i]->rightID
                              << " numtris: " << nodes[i]->triangles.size() 
                              << std::endl;
                }

                printf("SIZEOF Triangle*: %d\n", sizeof(KDN::Triangle*));
                printf("SIZEOF KDnode*: %d\n", sizeof(KDN::KDnode*));
                printf("SIZEOF KDnode: %d\n", sizeof((nodes[0])[0]));
                

                /*
                for (int i = 0; i < triangles.size(); i++)
                {
                    printf("triangle: [%f %f %f]\n", triangles[i]->center[0]
                                                   , triangles[i]->center[1]
                                                   , triangles[i]->center[2]);
                }
                */
                /*
                float dist;
                intersectAABB(r, KDT->rootNode->bbox, dist);

                glm::vec3 intersect = r.origin + r.direction*dist;
                printf("INTERSECT POINT: P: [%f %f %f]\n", intersect.x, intersect.y, intersect.z);
                */

                delete KDT;
                //deleteTree(KD->getRoot());


                _CrtDumpMemoryLeaks();
                return 0;
            }
        }
    }


    if (argc < 2) {
        printf("Usage: %s SCENEFILE.txt\n       %s SCENEFILE.txt SCENEFILE.obj\n", argv[0], argv[0]);
        return 1;
    }

    char *sceneFile;
    std::string objPath;

    if (argc == 2) 
    {
        sceneFile = argv[1];
        // Load scene file
        scene = new Scene(sceneFile);
    }
    else if (argc == 3)
    {
        sceneFile = argv[1];
        // Load scene file
        scene = new Scene(sceneFile);
        objPath = argv[2];
        scene->loadObj(objPath, objPath.substr(0, objPath.find_last_of("/\\") + 1));
    }
    else{
        printf("Usage: %s SCENEFILE.txt\n       %s SCENEFILE.txt SCENEFILE.obj\n", argv[0], argv[0]);
        return 1;
    }
    

    // Load scene file
    //scene = new Scene(sceneFile);
    
    //ObjMesh* objmesh = new ObjMesh("C:/Users/moi/Desktop/chair/chair.obj", "C:/Users/moi/Desktop/chair/");
    //objPath = "C:/Users/moi/Desktop/chair/werewolf.obj";
    //std::string objPath = "C:/Users/moi/Desktop/chair/broccoli.obj";
    //std::string objPath = "C:/Users/moi/Desktop/chair/test.obj";
    //std::string objPath = "C:/Users/moi/Desktop/chair/hazelnut.obj";
    //scene->loadObj(objPath, objPath.substr(0, objPath.find_last_of("/\\")+1));

    
    // Set up camera stuff from loaded path tracer settings
    iteration = 0;
    renderState = &scene->state;
    Camera &cam = renderState->camera;
    width = cam.resolution.x;
    height = cam.resolution.y;

    glm::vec3 view = cam.view;
    glm::vec3 up = cam.up;
    glm::vec3 right = glm::cross(view, up);
    up = glm::cross(right, view);

    cameraPosition = cam.position;

    // compute phi (horizontal) and theta (vertical) relative 3D axis
    // so, (0 0 1) is forward, (0 1 0) is up
    glm::vec3 viewXZ = glm::vec3(view.x, 0.0f, view.z);
    glm::vec3 viewZY = glm::vec3(0.0f, view.y, view.z);
    phi = glm::acos(glm::dot(glm::normalize(viewXZ), glm::vec3(0, 0, -1)));
    theta = glm::acos(glm::dot(glm::normalize(viewZY), glm::vec3(0, 1, 0)));
    ogLookAt = cam.lookAt;
    zoom = glm::length(cam.position - ogLookAt);


    // Initialize CUDA and GL components
    init();

    // GLFW main loop
    mainLoop();

    //delete objmesh;

    _CrtDumpMemoryLeaks();

    return 0;
}

void saveImage() {
    float samples = iteration;
    // output image file
    image img(width, height);

    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int index = x + (y * width);
            glm::vec3 pix = renderState->image[index];
            img.setPixel(width - 1 - x, y, glm::vec3(pix) / samples);
        }
    }

    std::string filename = renderState->imageName;
    std::ostringstream ss;
    ss << filename << "." << startTimeString << "." << samples << "samp";
    filename = ss.str();

    // CHECKITOUT
    img.savePNG(filename);
    //img.saveHDR(filename);  // Save a Radiance HDR file
}

void runCuda() {
    if (camchanged) {
        iteration = 0;
        Camera &cam = renderState->camera;
        cameraPosition.x = zoom * sin(phi) * sin(theta)+camoffset.x;
        cameraPosition.y = zoom * cos(theta) + camoffset.y;
        cameraPosition.z = zoom * cos(phi) * sin(theta) + camoffset.z;

        cam.view = -glm::normalize(cameraPosition);
        glm::vec3 v = cam.view;
        glm::vec3 u = glm::vec3(0, 1, 0);//glm::normalize(cam.up);
        glm::vec3 r = glm::cross(v, u);
        cam.up = glm::cross(r, v);
        cam.right = r;

        cam.position = cameraPosition;
        cameraPosition += cam.lookAt + camoffset;
        cam.position = cameraPosition;
        camchanged = false;
      }

    // Map OpenGL buffer object for writing from CUDA on a single GPU
    // No data is moved (Win & Linux). When mapped to CUDA, OpenGL should not use this buffer

    if (iteration == 0) {
        pathtraceFree(scene);
        pathtraceInit(scene);
    }

    if (iteration < renderState->iterations) {
        uchar4 *pbo_dptr = NULL;
        iteration++;
        cudaGLMapBufferObject((void**)&pbo_dptr, pbo);

        // execute the kernel
        int frame = 0;
        pathtrace(pbo_dptr, 
                  frame, 
                  iteration, 
                  dofDistance, 
                  dofAngle, 
                  rayCaching, 
                  antialias, 
                  softness,
                  SSS,
                  TESTINGMODE,
                  COMPACTION);

        // unmap buffer object
        cudaGLUnmapBufferObject(pbo);
    } else {
        saveImage();
        pathtraceFree(scene);
        cudaDeviceReset();
        exit(EXIT_SUCCESS);
    }
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_REPEAT || action == GLFW_PRESS) {
        if (key == GLFW_KEY_ESCAPE){
            saveImage();
            glfwSetWindowShouldClose(window, GL_TRUE);
        } else if (key == GLFW_KEY_S){
            saveImage();
        } else if (key == GLFW_KEY_SPACE){
            camchanged = true;
            renderState = &scene->state;
            Camera &cam = renderState->camera;
            cam.lookAt = ogLookAt;
            camoffset.x = 0.0f;
            camoffset.y = 0.0f;
            camoffset.z = 0.0f;
        } else if (key == GLFW_KEY_UP){
          camoffset.y += 1.0f;
          camchanged = true;
        } else if (key == GLFW_KEY_DOWN){
            camoffset.y -= 1.0f;
            camchanged = true;
        } else if (key == GLFW_KEY_LEFT){
            camoffset.x -= 1.0f;
            camchanged = true;
        } else if (key == GLFW_KEY_RIGHT){
            camoffset.x += 1.0f;
            camchanged = true;
        }
        else if (key == GLFW_KEY_EQUAL){
            dofAngle += 0.0004f;
            if (dofAngle > 0.03f)
                dofAngle == 0.03f;
            printf("\ndof blur = %f", dofAngle);
            camchanged = true;
        }
        else if (key == GLFW_KEY_MINUS){
            dofAngle -= 0.0004f;
            if (dofAngle < 0.0f)
                dofAngle == 0.0f;
            printf("\ndof blur = %f", dofAngle);
            camchanged = true;
        }
        else if (key == GLFW_KEY_LEFT_BRACKET){
            dofDistance -= 0.1f;
            if (dofDistance < 0.0f)
                dofDistance == 0.0f;
            printf("\nfocal point = %f", dofDistance);
            camchanged = true;
        }
        else if (key == GLFW_KEY_RIGHT_BRACKET){
            dofDistance += 0.1f;
            printf("\nfocal point = %f", dofDistance);
            camchanged = true;
        }
        else if (key == GLFW_KEY_0){
            dofDistance = 0.0f;
            dofAngle = 0.0f;
            camchanged = true;
        }
        else if (key == GLFW_KEY_C){
            rayCaching = !rayCaching;
            printf("\ncaching = %d", rayCaching);
            camchanged = true;
        }
        else if (key == GLFW_KEY_A){
            antialias = !antialias;
            printf("\nantialias = %d", antialias);
            camchanged = true;
        }
        else if (key == GLFW_KEY_2){
            softness += 0.0000001f;
            if (softness > 1.0f)
                softness = 1.0f;
            printf("\nsoftness = %f", softness);
            camchanged = true;
        }
        else if (key == GLFW_KEY_1){
            softness -= 0.0000001f;
            if (softness < 0.0f)
                softness = 0.0f;
            printf("\nsoftness = %f", softness);
            camchanged = true;
        }
        else if (key == GLFW_KEY_X){
            SSS = !SSS;
            printf("\nsubsurface scattering = %s", SSS == 0 ? "disabled" : "enabled");
            camchanged = true;
        }
        else if (key == GLFW_KEY_F){
            COMPACTION = !COMPACTION;
            printf("\stream compaction = %s", COMPACTION == 0 ? "disabled" : "enabled");
            camchanged = true;
        }
        else if (key == GLFW_KEY_T){
            TESTINGMODE = !TESTINGMODE;
            printf("\nTESTING = %s", TESTINGMODE == 0 ? "disabled" : "enabled");
            camchanged = true;
        }
    }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  leftMousePressed = (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS);
  rightMousePressed = (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS);
  middleMousePressed = (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS);
}

void mousePositionCallback(GLFWwindow* window, double xpos, double ypos) {
  if (leftMousePressed) {
    // compute new camera parameters
    phi -= (xpos - lastX) / width;
    theta -= (ypos - lastY) / height;
    theta = std::fmax(0.001f, std::fmin(theta, PI));
    camchanged = true;
  }
  else if (rightMousePressed) {
    zoom += 10.0f*(ypos - lastY) / height;
    zoom = std::fmax(0.1f, zoom);
    camchanged = true;
  }
  else if (middleMousePressed) {
    renderState = &scene->state;
    Camera &cam = renderState->camera;
    glm::vec3 forward = cam.view;
    forward.y = 0.0f;
    forward = glm::normalize(forward);
    glm::vec3 right = cam.right;
    right.y = 0.0f;
    right = glm::normalize(right);

    cam.lookAt -= 5.0f*(float) (xpos - lastX) * right * 0.01f;
    cam.lookAt += 5.0f*(float)(ypos - lastY) * forward * 0.01f;
    camchanged = true;
  }
  lastX = xpos;
  lastY = ypos;
}
