#pragma once

#include "intersections.h"

/**
 * Computes a cosine-weighted random direction in a hemisphere.
 * Used for diffuse lighting.
 */
__host__ __device__
glm::vec3 calculateRandomDirectionInHemisphere(
        glm::vec3 normal, thrust::default_random_engine &rng) {
    thrust::uniform_real_distribution<float> u01(0, 1);

    float up = sqrt(u01(rng)); // cos(theta)
    float over = sqrt(1 - up * up); // sin(theta)
    float around = u01(rng) * TWO_PI;

    // Find a direction that is not the normal based off of whether or not the
    // normal's components are all equal to sqrt(1/3) or whether or not at
    // least one component is less than sqrt(1/3). Learned this trick from
    // Peter Kutz.

    glm::vec3 directionNotNormal;
    if (abs(normal.x) < SQRT_OF_ONE_THIRD) {
        directionNotNormal = glm::vec3(1, 0, 0);
    } else if (abs(normal.y) < SQRT_OF_ONE_THIRD) {
        directionNotNormal = glm::vec3(0, 1, 0);
    } else {
        directionNotNormal = glm::vec3(0, 0, 1);
    }

    // Use not-normal direction to generate two perpendicular directions
    glm::vec3 perpendicularDirection1 =
        glm::normalize(glm::cross(normal, directionNotNormal));
    glm::vec3 perpendicularDirection2 =
        glm::normalize(glm::cross(normal, perpendicularDirection1));

    return up * normal
        + cos(around) * over * perpendicularDirection1
        + sin(around) * over * perpendicularDirection2;
}


__host__ __device__
glm::vec3 rotateVector(glm::vec3 n1, glm::vec3 axis, float angle)
{
    axis = glm::normalize(axis);
    float u = axis[0];
    float v = axis[1];
    float w = axis[2];

    float x = n1[0];
    float y = n1[1];
    float z = n1[2];

    float cosalpha = cos(angle);
    float sinalpha = sin(angle);

    glm::vec3 vec(
        (-u * (-u*x - v*y - w*z)) * (1 - cosalpha) + x * cosalpha + (-w*y + v*z) * sinalpha,
        (-v * (-u*x - v*y - w*z)) * (1 - cosalpha) + y * cosalpha + (w*x - u*z) * sinalpha,
        (-w * (-u*x - v*y - w*z)) * (1 - cosalpha) + z * cosalpha + (-v*x + u*y) * sinalpha);

    return vec;
}

__host__ __device__
glm::vec3 randSphericalVec(float angle, thrust::default_random_engine &rng)
{
    thrust::uniform_real_distribution<float> u01(0, 1);

    //srand(SEED++);
    double theta = 2 * PI * u01(rng);
    //srand(SEED++);
    double phi = glm::acos((angle * PI * u01(rng) - 1.0f));

    glm::vec3 V(glm::cos(theta)*glm::sin(phi),
        glm::sin(theta)*glm::sin(phi),
                glm::cos(phi));
    V = glm::normalize(V);

    return V;
}


__host__ __device__
float getFresnelValOld(const glm::vec3& I, const glm::vec3& N, const float& ior)
{
    float refl = 0.0f;

    float cosi = glm::dot(I, N);
    if (cosi < -1.0f)
        cosi = -1.0f;
    else if (cosi>1.0f)
        cosi = 1.0f;

    float etai = 1.0f;
    float etat = ior;

    if (cosi > 0.0f)
    {
        float tmp = etai;
        etai = etat;
        etat = tmp;
    }

    float sint = etai / etat * glm::sqrt(max(0.0f, 1.0f - cosi * cosi));
    
    // check if reflection
    if (sint >= 1.0f) 
        return 1.0f;
    else 
    {
        float cost = glm::sqrt(max(0.0f, 1.0f - sint*sint));
        cosi = glm::abs(cosi);

        float s = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float p = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (s*s + p*p) / 2.0f;
    }

    return 0.0;
}


// reimplementing using wikipedia's Shlick's 5th power approximation equation
__host__ __device__
float getFresnelVal(glm::vec3 I, glm::vec3 N, float ior) 
{
    float R0 = glm::pow((1.0f - ior) / (1.0f + ior), 2.0f);
    float F = R0 + (1.0f - R0) * glm::pow(1.0f - glm::dot(N, -I), 5);
    return F;
}


// fast simple bbox intersection by direction vector positioning
__host__ __device__
float intersectBbox(glm::vec3 origin, glm::vec3 direction, glm::vec3 mincoords, glm::vec3 maxcoords)
{

    bool result = false;
    glm::vec3 invdir(1.0f / direction.x,
                     1.0f / direction.y,
                     1.0f / direction.z);

    float v1 = (mincoords[0] - origin.x)*invdir.x;
    float v2 = (maxcoords[0] - origin.x)*invdir.x;
    float v3 = (mincoords[1] - origin.y)*invdir.y;
    float v4 = (maxcoords[1] - origin.y)*invdir.y;
    float v5 = (mincoords[2] - origin.z)*invdir.z;
    float v6 = (maxcoords[2] - origin.z)*invdir.z;

    float dmin = max(max(min(v1, v2), min(v3, v4)), min(v5, v6));
    float dmax = min(min(max(v1, v2), max(v3, v4)), max(v5, v6));

    if (dmax < 0)
    {
        return dmax;
    }
    if (dmin > dmax)
    {
        return dmax;
    }
    return dmin;
}





/**
 * Scatter a ray with some probabilities according to the material properties.
 * For example, a diffuse surface scatters in a cosine-weighted hemisphere.
 * A perfect specular surface scatters in the reflected ray direction.
 * In order to apply multiple effects to one surface, probabilistically choose
 * between them.
 * 
 * The visual effect you want is to straight-up add the diffuse and specular
 * components. You can do this in a few ways. This logic also applies to
 * combining other types of materias (such as refractive).
 * 
 * - Always take an even (50/50) split between a each effect (a diffuse bounce
 *   and a specular bounce), but divide the resulting color of either branch
 *   by its probability (0.5), to counteract the chance (0.5) of the branch
 *   being taken.
 *   - This way is inefficient, but serves as a good starting point - it
 *     converges slowly, especially for pure-diffuse or pure-specular.
 * - Pick the split based on the intensity of each material color, and divide
 *   branch result by that branch's probability (whatever probability you use).
 *
 * This method applies its changes to the Ray parameter `ray` in place.
 * It also modifies the color `color` of the ray in place.
 *
 */
__host__ __device__
void scatterRay(
        Ray &ray,
        glm::vec3 &color,
        glm::vec3 intersect,
        glm::vec3 normal,
        const Material &m,
        thrust::default_random_engine &rng,
        float softness) {

    if (m.transmittance.x > 0.0f || m.transmittance.y > 0.0f || m.transmittance.z > 0.0f)
    {
        thrust::uniform_real_distribution<float> u01(0, 1);
        float randval = u01(rng);

        if (randval < 0.5f && !ray.isinside) // reflect inside
        {

            glm::vec3 v = randSphericalVec(0.0001, rng);
            float angle = acos(glm::dot(glm::vec3(0.0f, 0.0f, -1.0f), ray.direction));
            glm::vec3 axis(0.0f, 0.0f, -1.0f);
            axis = glm::normalize(glm::cross(axis, ray.direction));
            ray.direction = rotateVector(v, axis, angle);

            ray.origin += ray.direction * 0.0001f;
            ray.sdepth = glm::distance(ray.origin, intersect);
            ray.isinside = true;
        }
        else // default
        {
            ray.direction = calculateRandomDirectionInHemisphere(normal, rng);
            ray.origin = intersect + normal*0.00001f;
            ray.sdepth = 0.0; // stop computing depth since ray is outside
        }
    }
    else if (m.hasRefractive != 0.0f)
    {
        thrust::uniform_real_distribution<float> u01(0, 1);
        float randval = u01(rng);

        ray.direction = glm::normalize(ray.direction);
        normal = glm::normalize(normal);

        // use fresnel to split the ray
        float fresn = getFresnelVal(ray.direction, normal, m.indexOfRefraction);
        if (randval < 1.0f - fresn) 
        {
            // setup index of refraction and invert if outside
            float ior = m.indexOfRefraction;
            if (!ray.isinside)
                ior = 1.0f / m.indexOfRefraction;

            // determine if we're reflecting or refracting to avoid untracked reflect/refract modes
            float angle = 1.0f - glm::pow(ior, 2) * (1.0f - glm::pow(glm::dot(normal, ray.direction), 2));

            if (angle < 0.0f) // the ray will be reflected if we can refract so the ray is no inside
            {
                // check reflection value and decide
                float val = u01(rng);
                if (val < m.hasReflective) // reflect
                {
                    ray.direction = glm::reflect(ray.direction, normal);

                    // enable soft reflections
                    if (softness > 0.0f)
                    {
                        glm::vec3 v = randSphericalVec(0.02, rng);
                        float angle = acos(glm::dot(glm::vec3(0.0f, 0.0f, -1.0f), ray.direction));
                        glm::vec3 axis(0.0f, 0.0f, -1.0f);
                        axis = glm::normalize(glm::cross(axis, ray.direction));
                        ray.direction = rotateVector(v, axis, angle);
                    }

                    ray.origin = intersect + normal * 0.00001f;
                }
                else
                {
                    ray.direction = calculateRandomDirectionInHemisphere(normal, rng);
                    ray.origin = intersect + normal*0.00001f;
                }
                //ray.isinside = !ray.isinside;
            }
            else // now we know the ray won't get reflected when we refract 
            {
                float val = u01(rng);
                if (val < m.hasRefractive) // refract
                {
                    ray.direction = glm::refract(ray.direction, normal, ior);
                    
                    // enable soft reflections
                    if (softness > 0.0f)
                    {
                        glm::vec3 v = randSphericalVec(0.02, rng);
                        float angle = acos(glm::dot(glm::vec3(0.0f, 0.0f, -1.0f), ray.direction));
                        glm::vec3 axis(0.0f, 0.0f, -1.0f);
                        axis = glm::normalize(glm::cross(axis, ray.direction));
                        ray.direction = rotateVector(v, axis, angle);
                    }

                    ray.origin = intersect - normal * 0.001f;
                    ray.isinside = !ray.isinside;
                }
                else
                {
                    ray.direction = calculateRandomDirectionInHemisphere(normal, rng);
                    ray.origin = intersect + normal*0.00001f;
                }
            }
        }
        else {
            ray.direction = glm::reflect(ray.direction, normal);
            ray.origin = intersect + normal * 0.00001f;
            ray.isinside = false;
        }

    }
    
    else if (m.hasReflective != 0.0f)
    {
        thrust::uniform_real_distribution<float> u01(0, 1);
        float randval = u01(rng);

        if (randval < m.hasReflective) // reflect
        {
            ray.direction = glm::reflect(ray.direction, normal);
        
            // enable soft reflections
            if (softness > 0.0f)
            {
                glm::vec3 v = randSphericalVec(0.02, rng);
                float angle = acos(glm::dot(glm::vec3(0.0f, 0.0f, -1.0f), ray.direction));
                glm::vec3 axis(0.0f, 0.0f, -1.0f);
                axis = glm::normalize(glm::cross(axis, ray.direction));
                ray.direction = rotateVector(v, axis, angle);
            }

            ray.origin = intersect + normal * 0.0001f;
            ray.isinside = false;
        }
        else // default
        {
            ray.direction = calculateRandomDirectionInHemisphere(normal, rng);
            ray.origin = intersect + normal*0.00001f;
        }
    }
    else
    { 
        // use uniform spherical distribution
        ray.direction = calculateRandomDirectionInHemisphere(normal, rng);

        /*
        glm::vec3 v = randSphericalVec(0.1f, rng);
        float angle = acos(glm::dot(glm::vec3(0.0f, 0.0f, 1.0f), normal));
        glm::vec3 axis(0.0f, 0.0f, 1.0f);
        axis = glm::normalize(glm::cross(axis, normal));
        ray.direction = rotateVector(v, axis, angle);
        */
        //if (glm::dot(ray.direction, normal) <= 0.0f)
        //    ray.direction = -ray.direction;

        ray.origin = intersect+normal*0.00001f;
        ray.isinside = false;  
    }  
}
