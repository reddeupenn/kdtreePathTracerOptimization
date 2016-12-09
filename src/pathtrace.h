#pragma once

#include <vector>
#include "scene.h"

void pathtraceInit(Scene *scene,bool enablekd);
void pathtraceFree(Scene *scene, bool enablekd);
void pathtrace(uchar4 *pbo, int frame, 
               int iteration, 
               float focalLength, 
               float dofAngle, 
               bool cacherays, 
               bool antialias, 
               float softness, 
               bool enableSss,
               bool testingmode,
               bool compaction,
               bool enablekd,
               bool vizkd,
               bool USEBBOX,
               bool SHORTSTACK);
