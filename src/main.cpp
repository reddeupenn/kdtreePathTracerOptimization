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

                //std::vector<KDN::Triangle*> triangles = getTrianglesFromFile(path);

                // test kdtree class generator
                KDtree* KDT = new KDtree(path);
                KDT->rootNode->updateBbox();


                KDT->printTree();

                cout << KDT->rootNode << endl;
                cout << KDT->rootNode->getRoot() << endl;


                KDT->rootNode->printTriangleCenters();
                KDT->printTree();

                KDT->split(30);

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
