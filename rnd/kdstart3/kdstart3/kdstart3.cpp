// kdstart3.cpp : Defines the entry point for the console application.
//

#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#include "stdafx.h"
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

using namespace std;




float frand(float scale)
{
    return scale * ((rand() % 10000 + 1) / 10000.0);
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

void printToFile(KDN::KDnode* root, ofstream& fileout)
{
    if (root != NULL)
    {
        fileout << root->bbox.mins[0] << " " << root->bbox.mins[1] << " " << root->bbox.mins[2] << " "
            << root->bbox.maxs[0] << " " << root->bbox.maxs[1] << " " << root->bbox.maxs[2] << "\n";

        printToFile(root->left, fileout);
        printToFile(root->right, fileout);
    }
}

void writeKDtoFile(KDN::KDnode* root, const char* path)
{
    ofstream fileout;
    fileout.open(path);

    printToFile(root, fileout);

    fileout.close();
}

int main()
{
    /*
    KDN::Point a[] = { KDN::Point(1.5, 20, 0),
        KDN::Point(1, 8, 2),
        KDN::Point(2, 5, 4),
        KDN::Point(7, 4, 4) };


    
    //Triangle t[] = { Triangle(10, 10, 11, 13, 12, 8, 9, 10, 8),
    //Triangle(1, 8, 2, -1, 3, 2, 5, 8, 10),
    //Triangle(12, 15, 14, 20, 15, 26, 14, 26, 36),
    //Triangle(-3, -4, -4, -1, -5, -6, -7, -3, -10),
    //Triangle(20, 18, 12, 15, 30, 20, 50, 90, 31) };
    

    int TRICOUNT = 5;
    KDN::Triangle** tempT = new KDN::Triangle*[TRICOUNT];
    tempT[0] = new KDN::Triangle(10, 10, 11, 13, 12, 8, 9, 10, 8);
    tempT[1] = new KDN::Triangle(1, 8, 2, -1, 3, 2, 5, 8, 10);
    tempT[2] = new KDN::Triangle(12, 15, 14, 20, 15, 26, 14, 26, 36);
    tempT[3] = new KDN::Triangle(-3, -4, -4, -1, -5, -6, -7, -3, -10);
    tempT[4] = new KDN::Triangle(20, 18, 12, 15, 30, 20, 50, 90, 31);


    // read file generated from Houdini and get triangles
    char path[1024];
    if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_PROFILE, NULL, 0, path)))
    {
        printf("path = %s\n", path);
    }
    strcat_s(path, sizeof(char) * 1024, "/git/CIS565/kdtreePathTracerOptimization/rnd/houdini/data");
    printf("path = %s\n", path);

    std::vector<KDN::Triangle*> triangles = getTrianglesFromFile(path);



    KDN::KDnode* KD = new KDN::KDnode(triangles.data(), triangles.size());
    //KDN::KDnode* KD = new KDN::KDnode(tempT, 5);
    //KDN::KDnode* KD = new KDN::KDnode(&first, 0);
    //KDN::KDN::KDnode* KD1 = new KDN::KDnode(1.0, 3.0, 1, 0);
    //KDN::KDnode* KD2 = new KDN::KDnode(1.0, 3.0, 1, 0);
    //KDN::KDnode* KD3 = new KDN::KDnode(1.0, 3.0, 1, 0);
    //KDN::KDnode* KD4 = new KDN::KDnode(1.0, 3.0, 1, 0);

    
    //for (int i = 0; i < 5; i++)
    //{
    //KDN::KDnode* x = new KDN::KDnode(&(t[i]));
    //KD->insert(x);
    //}
    

    KD->updateBbox();



    cout << a[0].x << " " << a[0].y << endl;

    KD->printTree(KD->getRoot());

    cout << KD << endl;
    cout << KD->getRoot() << endl;


    //deleteTree(KD1->getRoot());
    //deleteTree(KD2->getRoot());
    //deleteTree(KD3->getRoot());
    //deleteTree(KD4->getRoot());

    KDN::Triangle t0(0.0, 0.0, 0.0,
                1.0, 6.0, 1.0,
                2.0, 2.0, -2.0);

    KDN::BoundingBox b(&(t0));

    printf("\nbbox vals min: [%.1f %.1f %.1f]\n          max: [%.1f, %.1f, %.1f]\n",
           b.mins[0], b.mins[1], b.mins[2], b.maxs[0], b.maxs[1], b.maxs[2]);


    int depth = KD->getRoot()->getDepth(KD->getRoot(), 0);
    printf("\ndepth = %d\n", depth);

    KD->printTriangleCenters();
    KD->printTree(KD);

    KD->split(30);

    printf("\nreprinting after split\n");
    KD->printTree(KD);

    //for (int i=0; i<8; i++)
    //    KD->split();

    //printf("\nreprinting after split\n");
    //KD->printTree(KD);

    //KD->split();
    //KD->updateBbox();

    //printf("\nreprinting after split\n");
    //KD->printTree(KD);

    KD->printTriangleCenters();





    cout << KD << endl;


    // write out data before quitting
    char pathout[1024];
    if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_PROFILE, NULL, 0, pathout)))
    {
        printf("path = %s\n", pathout);
    }
    strcat_s(pathout, sizeof(char) * 1024, "/git/CIS565/kdtreePathTracerOptimization/rnd/houdini/dataout");
    printf("path = %s\n", pathout);

    writeKDtoFile(KD, pathout);



    deleteTree(KD->getRoot());

    for (int i = 0; i < TRICOUNT; i++)
        delete tempT[i];

    delete[] tempT;

    //int* cc = new int[4];
    */




    // read file generated from Houdini and get triangles
    char path[1024];
    if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_PROFILE, NULL, 0, path)))
    {
        printf("path = %s\n", path);
    }
    strcat_s(path, sizeof(char) * 1024, "/git/CIS565/kdtreePathTracerOptimization/rnd/houdini/data");
    printf("path = %s\n", path);

    // test kdtree class generator
    //std::vector<KDN::Triangle*> triangles = getTrianglesFromFile(path);
    //KDtree* KDT = new KDtree(triangles);

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

    //for (int i=0; i<8; i++)
    //    KD->split();

    //printf("\nreprinting after split\n");
    //KD->printTree(KD);

    //KD->split();
    //KD->updateBbox();

    //printf("\nreprinting after split\n");
    //KD->printTree(KD);

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

    writeKDtoFile(KDT->rootNode, pathout);


    delete KDT;
    //deleteTree(KD->getRoot());



    _CrtDumpMemoryLeaks();
    return 0;
}



