#include <cstdio>
#include <cuda.h>
#include <cmath>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/remove.h>
#include <thrust/device_ptr.h>
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
// TODO: static variables for device memory, any extra info you need, etc
// ...
static int iter2 = 0;

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


int obj_numshapes = 0;
int* obj_numpolyverts = NULL;
//int** obj_polysidx = NULL;
float* obj_verts = NULL;
float* obj_norms = NULL;
float* obj_texts = NULL;
int* obj_polyoffsets = NULL;
int* obj_polysidxflat = NULL;
float* obj_polysbboxes = NULL;
/*
float* obj_RGB = NULL;
float* obj_SPECEX = NULL;
float* obj_SPECRGB = NULL;
float* obj_REFL = NULL;
float* obj_REFR = NULL;
float* obj_REFRIOR = NULL;
*/
int* obj_materialOffsets = NULL;

void pathtraceInit(Scene *scene) {
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
        cudaMalloc((void**)&obj_numpolyverts, scene->obj_numshapes * sizeof(int));
        cudaMalloc((void**)&obj_polyoffsets, scene->obj_numshapes * sizeof(int));
        cudaMalloc((void**)&obj_polysidxflat, scene->polyidxcount * sizeof(int));
        cudaMalloc((void**)&obj_verts, scene->objmesh->attrib.vertices.size()* sizeof(float));
        cudaMalloc((void**)&obj_norms, scene->objmesh->attrib.normals.size()* sizeof(float));
        cudaMalloc((void**)&obj_texts, scene->objmesh->attrib.texcoords.size()* sizeof(float));
        cudaMalloc((void**)&obj_polysbboxes, scene->obj_numshapes * 6 * sizeof(float));
        cudaMalloc((void**)&obj_materialOffsets, scene->obj_numshapes * sizeof(int));
   
        // shading
        /*
        cudaMalloc((void**)&obj_RGB, scene->obj_numshapes * 3 * sizeof(float));
        cudaMalloc((void**)&obj_SPECEX, scene->obj_numshapes * sizeof(float));
        cudaMalloc((void**)&obj_SPECRGB, scene->obj_numshapes * 3 * sizeof(float));
        cudaMalloc((void**)&obj_REFL, scene->obj_numshapes * sizeof(float));
        cudaMalloc((void**)&obj_REFR, scene->obj_numshapes * sizeof(float));
        cudaMalloc((void**)&obj_REFRIOR, scene->obj_numshapes * sizeof(float));
        */

        cudaMemcpy(obj_numpolyverts, scene->obj_numpolyverts, scene->obj_numshapes * sizeof(int), cudaMemcpyHostToDevice);
        cudaMemcpy(obj_polyoffsets, scene->obj_polyoffsets, scene->obj_numshapes * sizeof(int), cudaMemcpyHostToDevice);
        cudaMemcpy(obj_polysidxflat, scene->obj_polysidxflat, scene->polyidxcount * sizeof(int), cudaMemcpyHostToDevice);
        cudaMemcpy(obj_verts, scene->obj_verts, scene->objmesh->attrib.vertices.size()* sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(obj_norms, scene->obj_norms, scene->objmesh->attrib.normals.size()* sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(obj_texts, scene->obj_texts, scene->objmesh->attrib.texcoords.size()* sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(obj_polysbboxes, scene->obj_bboxes, scene->obj_numshapes * 6 * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(obj_materialOffsets, scene->obj_materialOffsets, scene->obj_numshapes * sizeof(int), cudaMemcpyHostToDevice);
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

void pathtraceFree(Scene *scene) {
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
        cudaFree(obj_numpolyverts);
        cudaFree(obj_polyoffsets);
        cudaFree(obj_polysidxflat);
        cudaFree(obj_verts);
        cudaFree(obj_norms);
        cudaFree(obj_texts);
        cudaFree(obj_materialOffsets);
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







// TODO: 
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

                    if (T > -1.0f)
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
               bool compaction) {
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

    if (testingmode)
    {
        cudaEventCreate(&startGenRayFromCam); cudaEventCreate(&stopGenRayFromCam); cudaEventRecord(startGenRayFromCam);
    }

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
            /*
            , obj_RGB
            , obj_SPECEX
            , obj_SPECRGB
            , obj_REFL
            , obj_REFR
            , obj_REFRIOR
            */
            , obj_materialOffsets
            , hst_scene->hasObj);
        checkCUDAError("trace one bounce");

        /*
        pathTraceOneBounce2 << <numblocksPathSegmentTracing, blockSize1d >> > (
            depth
            , iter
            , num_paths
            , dev_paths
            , dev_geoms
            , hst_scene->geoms.size()
            , dev_materials
            , hst_scene->materials.size()
            , dev_intersections);
        checkCUDAError("trace one bounce2");
        */
        cudaDeviceSynchronize();
        depth++;

        if (testingmode)
        {
            cudaEventRecord(stopPathTraceOneBounce); cudaEventSynchronize(stopPathTraceOneBounce);
            ms2 = 0;
            cudaEventElapsedTime(&ms2, startPathTraceOneBounce, stopPathTraceOneBounce);
            //printf("\ngenerateRayFromCamera time = %f", ms2);
            millisecondsPathTraceOneBounce += ms2;
            cudaEventDestroy(startPathTraceOneBounce);
            cudaEventDestroy(stopPathTraceOneBounce);
        }

        // TODO:
        // --- Shading Stage ---
        // Shade path segments based on intersections and generate new rays by
        // evaluating the BSDF.
        // Start off with just a big kernel that handles all the different
        // materials you have in the scenefile.
        // TODO: compare between directly shading the path segments and shading
        // path segments that have been reshuffled to be contiguous in memory.

        if (testingmode)
        {
            cudaEventCreate(&startShadeMaterial); cudaEventCreate(&stopShadeMaterial); cudaEventRecord(startShadeMaterial);
        }
        shadeMaterial << <numblocksPathSegmentTracing, blockSize1d >> > (
            iter,
            num_paths,
            dev_intersections,
            dev_paths,
            dev_materials,
            enableSss
            );
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
        //printf("\n   pathTraceOneBounce time = %f", millisecondsPathTraceOneBounce / iter);
        //printf("\n        shadeMaterial time = %f\n", millisecondsShadeMaterial / iter);

        printf("\n[%f, %f, %f], ", millisecondsGenRayFromCam, 
                                   millisecondsPathTraceOneBounce / iter, 
                                   millisecondsShadeMaterial / iter);
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


