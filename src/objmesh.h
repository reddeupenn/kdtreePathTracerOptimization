#pragma once

#ifndef __OBJMESH__
#define __OBJMESH__

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

class ObjMesh {
private:
    std::string filename;
    std::string mtlbase;

public:
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    ObjMesh(std::string filename, std::string mtlbase);
    ~ObjMesh();
};


#endif