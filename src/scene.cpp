#include <iostream>
#include "scene.h"
#include <cstring>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtx/string_cast.hpp>

Scene::Scene(string filename) {
    obj_numshapes = 0;
    obj_numpolyverts = NULL;
    obj_polysidx = NULL;
    obj_verts = NULL;
    obj_norms = NULL;
    obj_texts = NULL;
    obj_bboxes = NULL;
    obj_polyoffsets = NULL;
    obj_polysidxflat = NULL;
    objmesh = NULL;
    // shading
    obj_RGB = NULL;
    obj_SPECEX = NULL;
    obj_SPECRGB = NULL;
    obj_REFL = NULL;
    obj_REFR = NULL;
    obj_REFRIOR = NULL;
    obj_materialOffsets = NULL;
    hasObj = false;

    // kd
    newNodes = NULL;
    newTriangles = NULL;
    newNodesBare = NULL;
    newTrianglesBare = NULL;

    polyidxcount = 0;

    cout << "Reading scene from " << filename << " ..." << endl;
    cout << " " << endl;
    char* fname = (char*)filename.c_str();
    fp_in.open(fname);
    if (!fp_in.is_open()) {
        cout << "Error reading from file - aborting!" << endl;
        throw;
    }
    while (fp_in.good()) {
        string line;
        utilityCore::safeGetline(fp_in, line);
        if (!line.empty()) {
            std::vector<string> tokens = utilityCore::tokenizeString(line);
            if (strcmp(tokens[0].c_str(), "MATERIAL") == 0) {
                loadMaterial(tokens[1]);
                cout << " " << endl;
            } else if (strcmp(tokens[0].c_str(), "OBJECT") == 0) {
                loadGeom(tokens[1]);
                cout << " " << endl;
            } else if (strcmp(tokens[0].c_str(), "CAMERA") == 0) {
                loadCamera();
                cout << " " << endl;
            }
        }
    }
}

Scene::~Scene()
{

    if (obj_polysidx != NULL)
    {
        for (int i = 0; i < obj_numshapes; i++)
            delete[] obj_polysidx[i];
        delete[] obj_polysidx;
    }
    if (obj_verts != NULL)
        delete[] obj_verts;
    if (obj_norms != NULL)
        delete[] obj_norms;
    if (obj_texts != NULL)
        delete[] obj_texts;
    if (obj_bboxes != NULL)
        delete[] obj_bboxes;
    if (obj_numpolyverts != NULL)
        delete[] obj_numpolyverts;
    if (objmesh != NULL)
        delete objmesh;

    // shading
    if (obj_RGB != NULL)
        delete[] obj_RGB;
    if (obj_SPECEX != NULL)
        delete[] obj_SPECEX;
    if (obj_SPECRGB != NULL)
        delete[] obj_SPECRGB;
    if (obj_REFL != NULL)
        delete[] obj_REFL;
    if (obj_REFR != NULL)
        delete[] obj_REFR;
    if (obj_REFRIOR != NULL)
        delete[] obj_REFRIOR;
    if (obj_materialOffsets != NULL)
        delete[] obj_materialOffsets;

    if(obj_polyoffsets != NULL)
        delete[] obj_polyoffsets;
    if(obj_polysidxflat != NULL)
        delete[] obj_polysidxflat;

    polyidxcount = 0;

    if (newNodes != NULL)
        delete[] newNodes;
    if (newTriangles != NULL)
        delete[] newTriangles;
    if (newNodesBare != NULL)
        delete[] newNodesBare;
    if (newTrianglesBare != NULL)
        delete[] newTrianglesBare;
}

int Scene::loadGeom(string objectid) {
    int id = atoi(objectid.c_str());
    if (id != geoms.size()) {
        cout << "ERROR: OBJECT ID does not match expected number of geoms" << endl;
        return -1;
    } else {
        cout << "Loading Geom " << id << "..." << endl;
        Geom newGeom;
        string line;

        //load object type
        utilityCore::safeGetline(fp_in, line);
        if (!line.empty() && fp_in.good()) {
            if (strcmp(line.c_str(), "sphere") == 0) {
                cout << "Creating new sphere..." << endl;
                newGeom.type = SPHERE;
            } else if (strcmp(line.c_str(), "cube") == 0) {
                cout << "Creating new cube..." << endl;
                newGeom.type = CUBE;
            }
        }

        //link material
        utilityCore::safeGetline(fp_in, line);
        if (!line.empty() && fp_in.good()) {
            std::vector<string> tokens = utilityCore::tokenizeString(line);
            newGeom.materialid = atoi(tokens[1].c_str());
            cout << "Connecting Geom " << objectid << " to Material " << newGeom.materialid << "..." << endl;
        }

        //load transformations
        utilityCore::safeGetline(fp_in, line);
        while (!line.empty() && fp_in.good()) {
            std::vector<string> tokens = utilityCore::tokenizeString(line);

            //load tranformations
            if (strcmp(tokens[0].c_str(), "TRANS") == 0) {
                newGeom.translation = glm::vec3(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()));
            } else if (strcmp(tokens[0].c_str(), "ROTAT") == 0) {
                newGeom.rotation = glm::vec3(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()));
            } else if (strcmp(tokens[0].c_str(), "SCALE") == 0) {
                newGeom.scale = glm::vec3(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()));
            }

            utilityCore::safeGetline(fp_in, line);
        }

        newGeom.transform = utilityCore::buildTransformationMatrix(
                newGeom.translation, newGeom.rotation, newGeom.scale);
        newGeom.inverseTransform = glm::inverse(newGeom.transform);
        newGeom.invTranspose = glm::inverseTranspose(newGeom.transform);

        geoms.push_back(newGeom);
        return 1;
    }
}

int Scene::loadCamera() {
    cout << "Loading Camera ..." << endl;
    RenderState &state = this->state;
    Camera &camera = state.camera;
    float fovy;

    //load static properties
    for (int i = 0; i < 5; i++) {
        string line;
        utilityCore::safeGetline(fp_in, line);
        std::vector<string> tokens = utilityCore::tokenizeString(line);
        if (strcmp(tokens[0].c_str(), "RES") == 0) {
            camera.resolution.x = atoi(tokens[1].c_str());
            camera.resolution.y = atoi(tokens[2].c_str());
        } else if (strcmp(tokens[0].c_str(), "FOVY") == 0) {
            fovy = atof(tokens[1].c_str());
        } else if (strcmp(tokens[0].c_str(), "ITERATIONS") == 0) {
            state.iterations = atoi(tokens[1].c_str());
        } else if (strcmp(tokens[0].c_str(), "DEPTH") == 0) {
            state.traceDepth = atoi(tokens[1].c_str());
        } else if (strcmp(tokens[0].c_str(), "FILE") == 0) {
            state.imageName = tokens[1];
        }
    }

    string line;
    utilityCore::safeGetline(fp_in, line);
    while (!line.empty() && fp_in.good()) {
        std::vector<string> tokens = utilityCore::tokenizeString(line);
        if (strcmp(tokens[0].c_str(), "EYE") == 0) {
            camera.position = glm::vec3(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()));
        } else if (strcmp(tokens[0].c_str(), "LOOKAT") == 0) {
            camera.lookAt = glm::vec3(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()));
        } else if (strcmp(tokens[0].c_str(), "UP") == 0) {
            camera.up = glm::vec3(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()));
        }

        utilityCore::safeGetline(fp_in, line);
    }

    //calculate fov based on resolution
    float yscaled = tan(fovy * (PI / 180));
    float xscaled = (yscaled * camera.resolution.x) / camera.resolution.y;
    float fovx = (atan(xscaled) * 180) / PI;
    camera.fov = glm::vec2(fovx, fovy);

	camera.right = glm::normalize(glm::cross(camera.view, camera.up));
	camera.pixelLength = glm::vec2(2 * xscaled / (float)camera.resolution.x
							, 2 * yscaled / (float)camera.resolution.y);

    camera.view = glm::normalize(camera.lookAt - camera.position);

    //set up render camera stuff
    int arraylen = camera.resolution.x * camera.resolution.y;
    state.image.resize(arraylen);
    std::fill(state.image.begin(), state.image.end(), glm::vec3());

    cout << "Loaded camera!" << endl;
    return 1;
}

int Scene::loadMaterial(string materialid) {
    int id = atoi(materialid.c_str());
    if (id != materials.size()) {
        cout << "ERROR: MATERIAL ID does not match expected number of materials" << endl;
        return -1;
    } else {
        cout << "Loading Material " << id << "..." << endl;
        Material newMaterial;

        //load static properties
        for (int i = 0; i < 7; i++) {
            string line;
            utilityCore::safeGetline(fp_in, line);
            std::vector<string> tokens = utilityCore::tokenizeString(line);
            if (strcmp(tokens[0].c_str(), "RGB") == 0) {
                glm::vec3 color( atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()) );
                newMaterial.color = color;
            } else if (strcmp(tokens[0].c_str(), "SPECEX") == 0) {
                newMaterial.specular.exponent = atof(tokens[1].c_str());
            } else if (strcmp(tokens[0].c_str(), "SPECRGB") == 0) {
                glm::vec3 specColor(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()));
                newMaterial.specular.color = specColor;
            } else if (strcmp(tokens[0].c_str(), "REFL") == 0) {
                newMaterial.hasReflective = atof(tokens[1].c_str());
            } else if (strcmp(tokens[0].c_str(), "REFR") == 0) {
                newMaterial.hasRefractive = atof(tokens[1].c_str());
            } else if (strcmp(tokens[0].c_str(), "REFRIOR") == 0) {
                newMaterial.indexOfRefraction = atof(tokens[1].c_str());
            } else if (strcmp(tokens[0].c_str(), "EMITTANCE") == 0) {
                newMaterial.emittance = atof(tokens[1].c_str());
            }
        }
        materials.push_back(newMaterial);
        return 1;
    }
}



void Scene::getKDnodes_(KDN::KDnode* root, std::vector<KDN::KDnode*>& nodes)
{
    if (root != NULL)
    {
        nodes.push_back(root);
        getKDnodes_(root->left, nodes);
        getKDnodes_(root->right, nodes);
    }
}

void Scene::getKDnodesLoop_(KDN::KDnode* root, std::vector<KDN::KDnode*>& nodes)
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

void Scene::getKDnodesLoopDeref_(KDN::KDnode* root, std::vector<KDN::KDnode>& nodes)
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

std::vector<int> Scene::cacheTriangles_(KDN::KDnode* nodes, int numNodes, std::vector<KDN::Triangle>& newTriangles)
{

    int triCount = 0;
    std::vector<int> offsets;

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

        /*
        std::cout << "node: " << nodes[i].ID << " numtris: " << numTriangles 
            << " bbox: [" << nodes[i].bbox.mins[0] << " "
                         << nodes[i].bbox.mins[1] << " "
                         << nodes[i].bbox.mins[2] << "] ["
                         << nodes[i].bbox.maxs[0] << " "
                         << nodes[i].bbox.maxs[1] << " "
                         << nodes[i].bbox.maxs[2] << "]"
            << std::endl;
        */
    }


    return offsets;
}

std::vector<int> Scene::cacheTriangles_(std::vector<KDN::KDnode*> nodes, std::vector<KDN::Triangle>& newTriangles)
{

    int triCount = 0;
    std::vector<int> offsets;

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

        /*
        std::cout << "node: " << nodes[i]->ID << " numtris: " << numTriangles
                              << " bbox: [" << nodes[i]->bbox.mins[0] << " "
                              << nodes[i]->bbox.mins[1] << " "
                              << nodes[i]->bbox.mins[2] << "] ["
                              << nodes[i]->bbox.maxs[0] << " "
                              << nodes[i]->bbox.maxs[1] << " "
                              << nodes[i]->bbox.maxs[2] << "]"
                              << std::endl;
        */
        /*
        std::cout << nodes[i]->bbox.mins[0] << " "
            << nodes[i]->bbox.mins[1] << " "
            << nodes[i]->bbox.mins[2] << " "
            << nodes[i]->bbox.maxs[0] << " "
            << nodes[i]->bbox.maxs[1] << " "
            << nodes[i]->bbox.maxs[2]
            << std::endl;
        */
    }


    return offsets;
}

std::vector<int> Scene::cacheTriangles_(std::vector<KDN::KDnode> nodes, std::vector<KDN::Triangle>& newTriangles)
{
    int triCount = 0;
    std::vector<int> offsets;

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
        /*
        std::cout << "node: " << nodes[i].ID << " numtris: " << numTriangles
            << " bbox: [" << nodes[i].bbox.mins[0] << " "
            << nodes[i].bbox.mins[1] << " "
            << nodes[i].bbox.mins[2] << "] ["
            << nodes[i].bbox.maxs[0] << " "
            << nodes[i].bbox.maxs[1] << " "
            << nodes[i].bbox.maxs[2] << "]"
            << std::endl;
        */
    }


    return offsets;
}

void Scene::deleteTree_(KDN::KDnode* root)
{
    if (root != NULL)
    {
        deleteTree_(root->left);
        deleteTree_(root->right);
        //delete root;

        if (root->left != NULL)
            root->left = NULL;
        if (root->right != NULL)
            root->right = NULL;

        delete root;
        root = NULL;
    }
}

bool nodeComparator_(const void* a, const void* b)
{
    int ida = (*(KDN::KDnode*)a).ID;
    int idb = (*(KDN::KDnode*)b).ID;

    if (ida <= idb)
        return true;
    else if (ida > idb)
        return false;
}

std::vector<KDN::Triangle*> Scene::getTrianglesFromScene_(void)
{
    std::vector<KDN::Triangle*> triangles;

    int iterator = 0;

    int pidxo1 = 0;
    int pidxo2 = 0;
    int pidxo3 = 0;

    for (int i = 0; i < obj_numshapes; i++)
    {
        for (int j = iterator; j < iterator + obj_polyoffsets[i]; j += 3)
        {
            pidxo1 = 3 * obj_polysidxflat[j];
            pidxo2 = 3 * obj_polysidxflat[j + 1];
            pidxo3 = 3 * obj_polysidxflat[j + 2];

            KDN::Triangle* t = new KDN::Triangle(
                obj_verts[pidxo1],
                obj_verts[pidxo1 + 1],
                obj_verts[pidxo1 + 2],
                obj_verts[pidxo2],
                obj_verts[pidxo2 + 1],
                obj_verts[pidxo2 + 2],
                obj_verts[pidxo3],
                obj_verts[pidxo3 + 1],
                obj_verts[pidxo3 + 2],
                obj_norms[pidxo1],
                obj_norms[pidxo1 + 1],
                obj_norms[pidxo1 + 2],
                obj_norms[pidxo2],
                obj_norms[pidxo2 + 1],
                obj_norms[pidxo2 + 2],
                obj_norms[pidxo3],
                obj_norms[pidxo3 + 1],
                obj_norms[pidxo3 + 2]);

            t->mtlIdx = i;
            triangles.push_back(t);
        }

        iterator += obj_polyoffsets[i];
    }

    return triangles;
}

void Scene::loadObj(string filepath, string mtlpath)
{
    if (objmesh != NULL)
    {
        delete[] obj_polyoffsets;
        delete[] obj_polysidxflat;
        delete[] obj_numpolyverts;
        delete[] obj_polysidx;
        delete[] obj_verts;
        delete[] obj_norms;
        delete[] obj_texts;
        delete[] obj_bboxes;
        delete[] obj_RGB;
        delete[] obj_SPECEX;
        delete[] obj_SPECRGB;
        delete[] obj_REFL;
        delete[] obj_REFR;
        delete[] obj_REFRIOR;
        delete[] obj_materialOffsets;
        delete objmesh;
    }

    objmesh = new ObjMesh(filepath, mtlpath);

    obj_numshapes = objmesh->shapes.size();
    obj_numpolyverts = new int[obj_numshapes];
    obj_polysidx = new int*[obj_numshapes];
    obj_verts = new float[objmesh->attrib.vertices.size()];
    obj_norms = new float[objmesh->attrib.normals.size()];
    obj_texts = new float[objmesh->attrib.texcoords.size()];
    obj_bboxes = new float[obj_numshapes*6];
    // shading
    obj_RGB = new float[obj_numshapes * 3];
    obj_SPECEX = new float[obj_numshapes];
    obj_SPECRGB = new float[obj_numshapes * 3];
    obj_REFL = new float[obj_numshapes];
    obj_REFR = new float[obj_numshapes];
    obj_REFRIOR = new float[obj_numshapes];
    obj_materialOffsets = new int[obj_numshapes];


    // vertices
    // get the vertex indices

    for (int i = 0; i < objmesh->attrib.vertices.size(); i += 3){
        obj_verts[i] = objmesh->attrib.vertices[i];
        obj_verts[i + 1] = objmesh->attrib.vertices[i + 1];
        obj_verts[i + 2] = objmesh->attrib.vertices[i + 2];
    }

    for (int i = 0; i < objmesh->attrib.normals.size(); i++) {
        obj_norms[i] = objmesh->attrib.normals[i];
    }

    for (int i = 0; i < objmesh->attrib.texcoords.size(); i++) {
        obj_texts[i] = objmesh->attrib.texcoords[i];
    }


    // polygon idx
    for (int i = 0; i < obj_numshapes; i++)
    {
        obj_numpolyverts[i] = objmesh->shapes[i].mesh.indices.size();
        obj_polysidx[i] = new int[objmesh->shapes[i].mesh.indices.size()];

        // get the polygon indices
        for (int j = 0; j < obj_numpolyverts[i]; j++)
        {
            obj_polysidx[i][j] = objmesh->shapes[i].mesh.indices[j].vertex_index;
        }
    }


    obj_polyoffsets = new int[obj_numshapes];

    polyidxcount = 0;
    for (int i = 0; i < obj_numshapes; i++) {
        polyidxcount += objmesh->shapes[i].mesh.indices.size();
        obj_polyoffsets[i] = objmesh->shapes[i].mesh.indices.size();
    }

    obj_polysidxflat = new int[polyidxcount];


    int iter = 0;
    for (int i = 0; i < obj_numshapes; i++){
        for (int j = 0; j < obj_numpolyverts[i]; j++){
            obj_polysidxflat[iter] = objmesh->shapes[i].mesh.indices[j].vertex_index;
            iter++;
        }
    }


    // bboxes
    int iterator = 0;
    for (int i = 0; i < obj_numshapes; i++)
    {
        float minx = FLT_MAX;
        float maxx = 0.0f;
        float miny = FLT_MAX;
        float maxy = 0.0f;
        float minz = FLT_MAX;
        float maxz = 0.0f;
        for (int j = iterator; j < iterator + obj_polyoffsets[i]; j += 3)
        {
            int idx1 = obj_polysidxflat[j];
            int idx2 = obj_polysidxflat[j + 1];
            int idx3 = obj_polysidxflat[j + 2];
            int idxo1 = 3 * idx1;
            int idxo2 = 3 * idx2;
            int idxo3 = 3 * idx3;

            if (obj_verts[idxo1] < minx)
                minx = obj_verts[idxo1];
            if (obj_verts[idxo1] > maxx)
                maxx = obj_verts[idxo1];
            if (obj_verts[idxo1 + 1] < miny)
                miny = obj_verts[idxo1 + 1];
            if (obj_verts[idxo1 + 1] > maxy)
                maxy = obj_verts[idxo1 + 1];
            if (obj_verts[idxo1 + 2] < minz)
                minz = obj_verts[idxo1 + 2];
            if (obj_verts[idxo1 + 2] > maxz)
                maxz = obj_verts[idxo1 + 2];
        }
        obj_bboxes[iterator] = minx;
        obj_bboxes[iterator+1] = miny;
        obj_bboxes[iterator+2] = minz;
        obj_bboxes[iterator+3] = maxx;
        obj_bboxes[iterator+4] = maxy;
        obj_bboxes[iterator+5] = maxz;

        iterator += obj_polyoffsets[i];
    }


    // shaders
    for (int i = 0; i < obj_numshapes; i++)
    {
        if (objmesh->materials.size() > i)
        {
            int illum = objmesh->materials[i].illum;

            if (illum <= 2)
            {
                obj_RGB[3 * i] = objmesh->materials[i].ambient[0] > objmesh->materials[i].diffuse[0] ? objmesh->materials[i].ambient[0] : objmesh->materials[i].diffuse[0];
                obj_RGB[3 * i + 1] = objmesh->materials[i].ambient[1] > objmesh->materials[i].diffuse[1] ? objmesh->materials[i].ambient[1] : objmesh->materials[i].diffuse[1];
                obj_RGB[3 * i + 2] = objmesh->materials[i].ambient[2] > objmesh->materials[i].diffuse[2] ? objmesh->materials[i].ambient[2] : objmesh->materials[i].diffuse[2];
                obj_SPECEX[i] = 0.0f;
                obj_SPECRGB[3 * i] = 0.0f;
                obj_SPECRGB[3 * i + 1] = 0.0f;
                obj_SPECRGB[3 * i + 2] = 0.0f;
                obj_REFL[i] = 0.0f;
                obj_REFR[i] = 0.0f;
                obj_REFRIOR[i] = 0.0f;
            }
            else if (illum == 3)
            {
                obj_RGB[3 * i] = objmesh->materials[i].ambient[0] > objmesh->materials[i].diffuse[0] ? objmesh->materials[i].ambient[0] : objmesh->materials[i].diffuse[0];
                obj_RGB[3 * i + 1] = objmesh->materials[i].ambient[1] > objmesh->materials[i].diffuse[1] ? objmesh->materials[i].ambient[1] : objmesh->materials[i].diffuse[1];
                obj_RGB[3 * i + 2] = objmesh->materials[i].ambient[2]> objmesh->materials[i].diffuse[2] ? objmesh->materials[i].ambient[2] : objmesh->materials[i].diffuse[2];
                obj_SPECEX[i] = 1.0f;
                obj_SPECRGB[3 * i] = objmesh->materials[i].specular[0];
                obj_SPECRGB[3 * i + 1] = objmesh->materials[i].specular[1];
                obj_SPECRGB[3 * i + 2] = objmesh->materials[i].specular[2];
                obj_REFL[i] = 1.0f;
                obj_REFR[i] = 0.0f;
                obj_REFRIOR[i] = 0.0f;
            }
            else
            {
                obj_RGB[3 * i] = objmesh->materials[i].ambient[0] > objmesh->materials[i].diffuse[0] ? objmesh->materials[i].ambient[0] : objmesh->materials[i].diffuse[0];
                obj_RGB[3 * i + 1] = objmesh->materials[i].ambient[1] > objmesh->materials[i].diffuse[1] ? objmesh->materials[i].ambient[1] : objmesh->materials[i].diffuse[1];
                obj_RGB[3 * i + 2] = objmesh->materials[i].ambient[2] > objmesh->materials[i].diffuse[2] ? objmesh->materials[i].ambient[2] : objmesh->materials[i].diffuse[2];
                obj_SPECEX[i] = 1.0f;
                obj_SPECRGB[3 * i] = objmesh->materials[i].specular[0];
                obj_SPECRGB[3 * i + 1] = objmesh->materials[i].specular[1];
                obj_SPECRGB[3 * i + 2] = objmesh->materials[i].specular[2];
                obj_REFL[i] = 1.0f;
                obj_REFR[i] = 1.0f;
                obj_REFRIOR[i] = objmesh->materials[i].ior;
            }
            //printf("\nmaterial RGB = %f \n", objmesh->materials[i].ambient[0]);
            //printf("\nmaterial REFRIOR = %f \n", objmesh->materials[i].ior);
            //printf("\nmaterial SPEC = %f %f %f\n", objmesh->materials[i].specular[0], objmesh->materials[i].specular[1], objmesh->materials[i].specular[2]);
        }
        else
        {
            // default shading
            obj_RGB[3 * i] = 1.0f;
            obj_RGB[3 * i + 1] = 1.0f;
            obj_RGB[3 * i + 2] = 1.0f;
            obj_SPECEX[i] = 0.0f;
            obj_SPECRGB[3 * i] = 0.0f;
            obj_SPECRGB[3 * i + 1] = 0.0f;
            obj_SPECRGB[3 * i + 2] = 0.0f;
            obj_REFL[i] = 0.0f;
            obj_REFR[i] = 0.0f;
            obj_REFRIOR[i] = 0.0f;
        }

        Material objMaterial;
        objMaterial.color = glm::vec3(obj_RGB[3 * i], obj_RGB[3 * i + 1], obj_RGB[3 * i + 2]);
        objMaterial.specular.exponent = obj_SPECEX[i];
        objMaterial.specular.color = glm::vec3(obj_SPECRGB[3 * i], obj_SPECRGB[3 * i + 1], obj_SPECRGB[3 * i + 2]);
        objMaterial.hasReflective = obj_REFL[i];
        objMaterial.hasRefractive = obj_REFR[i];
        objMaterial.indexOfRefraction = obj_REFRIOR[i];
        objMaterial.emittance = 0.0f;
        objMaterial.transmittance = glm::vec3(objmesh->materials[i].transmittance[0], objmesh->materials[i].transmittance[1], objmesh->materials[i].transmittance[2]);

        /*
        objMaterial.color = glm::vec3((float)obj_RGB[3 * i], (float)obj_RGB[3 * i + 1], (float)obj_RGB[3 * i + 2]);
        objMaterial.color = glm::vec3(1.0f, 1.0f, 1.0f);
        objMaterial.specular.exponent = 0.0f;
        objMaterial.specular.color = glm::vec3(0.0f, 0.0f, 0.0f);
        objMaterial.hasReflective = 0.0f;
        objMaterial.hasRefractive = 0.0f;
        objMaterial.indexOfRefraction = 0.0f;
        */

        
        //printf("\ndiffuse = %f %f %f", obj_RGB[3 * i], obj_RGB[3 * i + 1], obj_RGB[3 * i + 2]);
        //printf("\ntransmittance = %f %f %f", objMaterial.transmittance.x, objMaterial.transmittance.y, objMaterial.transmittance.z);
        
        
        /*
        printf("\ndiffuse = %f %f %f", objmesh->materials[i].diffuse[0], objmesh->materials[i].diffuse[1], objmesh->materials[i].diffuse[2]);
        objMaterial.color.x = objmesh->materials[i].diffuse[0];// > objmesh->materials[i].diffuse[0] ? objmesh->materials[i].ambient[0] : objmesh->materials[i].diffuse[0];
        objMaterial.color.y = objmesh->materials[i].diffuse[1];// > objmesh->materials[i].diffuse[1] ? objmesh->materials[i].ambient[1] : objmesh->materials[i].diffuse[1];
        objMaterial.color.z = objmesh->materials[i].diffuse[2];// > objmesh->materials[i].diffuse[2] ? objmesh->materials[i].ambient[2] : objmesh->materials[i].diffuse[2];
        objMaterial.specular.exponent = 0.0f;
        objMaterial.specular.color.x = 0.f;
        objMaterial.specular.color.y = 0.0f;
        objMaterial.specular.color.z = 0.0f;
        objMaterial.hasReflective = 0.0f;
        objMaterial.hasRefractive = 0.0f;
        objMaterial.indexOfRefraction = objmesh->materials[i].ior;
        */

        obj_materialOffsets[i] = materials.size();
        materials.push_back(objMaterial);
        
    }

    /*
    // sanity check print
    int iterator = 0;
    for (int i = 0; i < obj_numshapes; i++)
    {
        printf("loop1\n");
        for (int j = iterator; j < iterator + obj_polyoffsets[i]; j += 3)
        {
            int idx1 = obj_polysidxflat[j];
            int idx2 = obj_polysidxflat[j + 1];
            int idx3 = obj_polysidxflat[j + 2];
            int idxo1 = 3 * idx1;
            int idxo2 = 3 * idx2;
            int idxo3 = 3 * idx3;
            printf("i: %d %d %d : [%.1f %.1f %.1f] [%.1f %.1f %.1f] [%.1f %.1f %.1f]\n",
                idx1,
                idx2,
                idx3,
                obj_verts[idxo1],
                obj_verts[idxo1 + 1],
                obj_verts[idxo1 + 2],
                obj_verts[idxo2],
                obj_verts[idxo2 + 1],
                obj_verts[idxo2 + 2],
                obj_verts[idxo3],
                obj_verts[idxo3 + 1],
                obj_verts[idxo3 + 2]);
        }

        iterator += obj_polyoffsets[i];
    }
    */

    hasObj = true;


    printf("loading triangles\n");

    // This section assembles a KD tree and flattens out the entire tree
    // as an array of KDnodes, an array of Triangles and an array of 
    // indices with the triangle offsets so that we can traverse the tree
    // non recursively.
    std::vector<KDN::Triangle*> triangles = getTrianglesFromScene_();

    KDT = new KDtree(triangles);
    KDT->rootNode->updateBbox();

    int splitDepth = round(log10((double)triangles.size())*1);
    KDT->split(/*splitDepth*/13);

    // Accessing kd nodes and triangles as a flat structure
    // This is to help recursion removal for CUDA
    // THANK YOU NVIDIA for this...
    getKDnodes_(KDT->rootNode, nodes);
    std::sort(nodes.begin(), nodes.end(), nodeComparator_);

    // flattened triangles data
    offsets = cacheTriangles_(nodes, Triangles);
    numTriangles = Triangles.size();
    newTriangles = new KDN::Triangle[numTriangles];
    memcpy(newTriangles, Triangles.data(), sizeof(KDN::Triangle)*numTriangles);

    getKDnodesLoop_(KDT->rootNode, nodesLoop);
    std::sort(nodesLoop.begin(), nodesLoop.end());

    getKDnodesLoopDeref_(KDT->rootNode, nodesLoopDeref);
    std::sort(nodesLoopDeref.begin(), nodesLoopDeref.end());

    // flattened nodes data
    numNodes = nodesLoopDeref.size();
    newNodes = new KDN::KDnode[numNodes];
    memcpy(newNodes, nodesLoopDeref.data(), sizeof(KDN::KDnode)*numNodes);

    printf("num nodes = %d\n", numNodes);

    // cache reduced data
    cacheNodesBare();
    cacheTrianglesBare();
    
}

void Scene::cacheNodesBare()
{
    if (newNodes == NULL)
        return;

    if (newNodesBare != NULL)
        delete[] newNodesBare;
    
    newNodesBare = new KDN::NodeBare[numNodes];
    
    for (int i = 0; i < numNodes; i++)
    {
        newNodesBare[i].axis = newNodes[i].axis;
        newNodesBare[i].ID = newNodes[i].ID;
        newNodesBare[i].parentID = newNodes[i].parentID;
        newNodesBare[i].leftID = newNodes[i].leftID;
        newNodesBare[i].rightID = newNodes[i].rightID;
        newNodesBare[i].mins[0] = newNodes[i].bbox.mins[0];
        newNodesBare[i].mins[1] = newNodes[i].bbox.mins[1];
        newNodesBare[i].mins[2] = newNodes[i].bbox.mins[2];
        newNodesBare[i].maxs[0] = newNodes[i].bbox.maxs[0];
        newNodesBare[i].maxs[1] = newNodes[i].bbox.maxs[1];
        newNodesBare[i].maxs[2] = newNodes[i].bbox.maxs[2];
        newNodesBare[i].triIdSize = newNodes[i].triIdSize;
        newNodesBare[i].triIdStart = newNodes[i].triIdStart;
        newNodesBare[i].splitPos = newNodes[i].splitPos;
    }
}

void Scene::cacheTrianglesBare()
{
    if (newTriangles == NULL)
        return;

    if (newTrianglesBare != NULL)
        delete[] newTrianglesBare;

    newTrianglesBare = new KDN::TriBare[numTriangles];

    for (int i = 0; i < numTriangles; i++)
    {
        newTrianglesBare[i].x1 = newTriangles[i].x1;
        newTrianglesBare[i].x2 = newTriangles[i].x2;
        newTrianglesBare[i].x3 = newTriangles[i].x3;
        newTrianglesBare[i].y1 = newTriangles[i].y1;
        newTrianglesBare[i].y2 = newTriangles[i].y2;
        newTrianglesBare[i].y3 = newTriangles[i].y3;
        newTrianglesBare[i].z1 = newTriangles[i].z1;
        newTrianglesBare[i].z2 = newTriangles[i].z2;
        newTrianglesBare[i].z3 = newTriangles[i].z3;

        newTrianglesBare[i].nx1 = newTriangles[i].nx1;
        newTrianglesBare[i].nx2 = newTriangles[i].nx2;
        newTrianglesBare[i].nx3 = newTriangles[i].nx3;
        newTrianglesBare[i].ny1 = newTriangles[i].ny1;
        newTrianglesBare[i].ny2 = newTriangles[i].ny2;
        newTrianglesBare[i].ny3 = newTriangles[i].ny3;
        newTrianglesBare[i].nz1 = newTriangles[i].nz1;
        newTrianglesBare[i].nz2 = newTriangles[i].nz2;
        newTrianglesBare[i].nz3 = newTriangles[i].nz3;

        newTrianglesBare[i].mtlIdx = newTriangles[i].mtlIdx;
    }
}