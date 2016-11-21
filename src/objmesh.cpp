#include "objmesh.h"


ObjMesh::ObjMesh(std::string filename, std::string mtlbase)
{
    this->filename = filename;
    this->mtlbase = mtlbase;

    bool triangulate = true;

    std::cout << "Loading " << filename.c_str() << std::endl;

    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename.c_str(),
        mtlbase.c_str(), triangulate);

    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        printf("Failed to load/parse .obj.\n");
        return;
    }
}

ObjMesh::~ObjMesh()
{
}

