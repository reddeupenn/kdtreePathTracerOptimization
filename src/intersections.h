#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/intersect.hpp>

#include "sceneStructs.h"
#include "utilities.h"

#include "KDnode.h"
#include "KDtree.h"

/**
 * Handy-dandy hash function that provides seeds for random number generation.
 */
__host__ __device__ inline unsigned int utilhash(unsigned int a) {
    a = (a + 0x7ed55d16) + (a << 12);
    a = (a ^ 0xc761c23c) ^ (a >> 19);
    a = (a + 0x165667b1) + (a << 5);
    a = (a + 0xd3a2646c) ^ (a << 9);
    a = (a + 0xfd7046c5) + (a << 3);
    a = (a ^ 0xb55a4f09) ^ (a >> 16);
    return a;
}

// CHECKITOUT
/**
 * Compute a point at parameter value `t` on ray `r`.
 * Falls slightly short so that it doesn't intersect the object it's hitting.
 */
__host__ __device__ glm::vec3 getPointOnRay(Ray r, float t) {
    return r.origin + (t - .0001f) * glm::normalize(r.direction);
}

/**
 * Multiplies a mat4 and a vec4 and returns a vec3 clipped from the vec4.
 */
__host__ __device__ glm::vec3 multiplyMV(glm::mat4 m, glm::vec4 v) {
    return glm::vec3(m * v);
}

// CHECKITOUT
/**
 * Test intersection between a ray and a transformed cube. Untransformed,
 * the cube ranges from -0.5 to 0.5 in each axis and is centered at the origin.
 *
 * @param intersectionPoint  Output parameter for point of intersection.
 * @param normal             Output parameter for surface normal.
 * @param outside            Output param for whether the ray came from outside.
 * @return                   Ray parameter `t` value. -1 if no intersection.
 */
__host__ __device__ float boxIntersectionTest(Geom box, Ray r,
        glm::vec3 &intersectionPoint, glm::vec3 &normal, bool &outside) {
    Ray q;
    q.origin    =                multiplyMV(box.inverseTransform, glm::vec4(r.origin   , 1.0f));
    q.direction = glm::normalize(multiplyMV(box.inverseTransform, glm::vec4(r.direction, 0.0f)));

    float tmin = -1e38f;
    float tmax = 1e38f;
    glm::vec3 tmin_n;
    glm::vec3 tmax_n;
    for (int xyz = 0; xyz < 3; ++xyz) {
        float qdxyz = q.direction[xyz];
        /*if (glm::abs(qdxyz) > 0.00001f)*/ {
            float t1 = (-0.5f - q.origin[xyz]) / qdxyz;
            float t2 = (+0.5f - q.origin[xyz]) / qdxyz;
            float ta = glm::min(t1, t2);
            float tb = glm::max(t1, t2);
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
        intersectionPoint = multiplyMV(box.transform, glm::vec4(getPointOnRay(q, tmin), 1.0f));
        normal = glm::normalize(multiplyMV(box.transform, glm::vec4(tmin_n, 0.0f)));
        return glm::length(r.origin - intersectionPoint);
    }
    return -1;
}

// CHECKITOUT
/**
 * Test intersection between a ray and a transformed sphere. Untransformed,
 * the sphere always has radius 0.5 and is centered at the origin.
 *
 * @param intersectionPoint  Output parameter for point of intersection.
 * @param normal             Output parameter for surface normal.
 * @param outside            Output param for whether the ray came from outside.
 * @return                   Ray parameter `t` value. -1 if no intersection.
 */
__host__ __device__ float sphereIntersectionTest(Geom sphere, Ray r,
        glm::vec3 &intersectionPoint, glm::vec3 &normal, bool &outside) {
    float radius = .5;

    glm::vec3 ro = multiplyMV(sphere.inverseTransform, glm::vec4(r.origin, 1.0f));
    glm::vec3 rd = glm::normalize(multiplyMV(sphere.inverseTransform, glm::vec4(r.direction, 0.0f)));

    Ray rt;
    rt.origin = ro;
    rt.direction = rd;

    float vDotDirection = glm::dot(rt.origin, rt.direction);
    float radicand = vDotDirection * vDotDirection - (glm::dot(rt.origin, rt.origin) - powf(radius, 2));
    if (radicand < 0) {
        return -1;
    }

    float squareRoot = sqrt(radicand);
    float firstTerm = -vDotDirection;
    float t1 = firstTerm + squareRoot;
    float t2 = firstTerm - squareRoot;

    float t = 0;
    if (t1 < 0 && t2 < 0) {
        return -1;
    } else if (t1 > 0 && t2 > 0) {
        t = min(t1, t2);
        outside = true;
    } else {
        t = max(t1, t2);
        outside = false;
    }

    glm::vec3 objspaceIntersection = getPointOnRay(rt, t);

    intersectionPoint = multiplyMV(sphere.transform, glm::vec4(objspaceIntersection, 1.f));
    normal = glm::normalize(multiplyMV(sphere.invTranspose, glm::vec4(objspaceIntersection, 0.f)));
    if (!outside) {
        normal = -normal;
    }

    return glm::length(r.origin - intersectionPoint);
}



__host__ __device__ bool intersectAABB(Ray r, KDN::BoundingBox b, float& dist)
{

    bool result = false;
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
    /*
    if (dmin < 0 || dmax > dmin)
    {
    dist = -1;
    return false;
    }
    else
    {
    dist = dmax;
    return true;
    }
    */
    if (dmax < 0)
    {
        dist = dmax;
        result = false;
        return result;
    }
    if (dmin > dmax)
    {
        dist = dmax;
        result = false;
        return result;
    }
    dist = dmin;
    result = true;
    return result;
}

// loop version of copies tree traversal
__host__ __device__ //__global__
glm::vec4 intersectKDLoopDeref(Ray r, KDN::KDnode* nodes, int numNodes, KDN::Triangle* triangles, int numTriangles)
{
    float dist = -1.0;
    glm::vec3 normal(0.0f);
    bool hit = false;

    // USE AN ARRAY OF 0 NODE IDS AND SET THEM TO 1 once they're visited
    // instead of using visited to avoid conflicts when reading from
    // multiple threads
    bool nodeIDs[100] = { false };
    //memset(nodeIDs, 0, sizeof(bool)*numNodes);


    if (numNodes == 0)
        return glm::vec4(normal, dist);
    else
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

        float boxdist = -1.0;
        while (true)
        {
            if (currID == -1)
                break;

            // check if it intersects the bounds
            bool hit = intersectAABB(r, nodes[currID].bbox, dist);

            if (hit == false)
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
                                normal = (bary[0] * n1 + bary[1] * n2 + bary[2] * n3);

                                dist = barytemp.z;
                                mindist = dist;
                                //glm::vec3 pos = r.origin + r.direction * dist;

                                glm::vec3 intersect = r.origin + r.direction*dist;
                                //printf("KDLOOPPTR INTERSECT POINT: P: [%f %f %f] NODEID: %d\n", intersect.x,
                                //       intersect.y,
                                //       intersect.z,
                                //       currID);
                            }
                        }
                    }
                }
                else
                    currID = nodes[currID].parentID;
            }
        }
    }

    return glm::vec4(normal, dist);
}

