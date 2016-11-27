// kdstart3.cpp : Defines the entry point for the console application.
//

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


#include "KDtree.h"

using namespace std;


KDtree::KDtree()
{
    //if (rootNode != NULL)
    //    delete rootNode;

    rootNode = new KDN::KDnode();
}

KDtree::KDtree(std::vector<KDN::Triangle*> t)
{
    //if (rootNode != NULL)
    //    delete rootNode;

    rootNode = new KDN::KDnode();
    rootNode->triangles = t;
}

KDtree::KDtree(const char* path)
{
    //if (rootNode != NULL)
    //    delete rootNode;

    rootNode = new KDN::KDnode();

    vector<KDN::Triangle*> t = getTrianglesFromFile(path);
    rootNode->triangles = t;
}


KDtree::~KDtree()
{
    deleteTree(rootNode);
}

//  for testing
float KDtree::frand(float scale)
{
    return scale * ((rand() % 10000 + 1) / 10000.0);
}


void KDtree::deleteTree(KDN::KDnode* root)
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

        //for (int i = 0; i < root->triangles.size(); i++)
        //{
        //    delete root->triangles[i];
        //    root->triangles[i] = NULL;
        //}

        delete root;
        root = NULL;
    }
}


std::vector<KDN::Triangle*> KDtree::getTrianglesFromFile(const char* path)
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

void KDtree::printToFile(KDN::KDnode* root, ofstream& fileout)
{
    if (root != NULL)
    {
        fileout << root->bbox.mins[0] << " " << root->bbox.mins[1] << " " << root->bbox.mins[2] << " "
            << root->bbox.maxs[0] << " " << root->bbox.maxs[1] << " " << root->bbox.maxs[2] << "\n";

        printToFile(root->left, fileout);
        printToFile(root->right, fileout);
    }
}

void KDtree::writeKDtoFile(KDN::KDnode* root, const char* path)
{
    ofstream fileout;
    fileout.open(path);

    printToFile(root, fileout);

    fileout.close();
}

void KDtree::printTree()
{
    if (rootNode)
    {
        rootNode->printTree(rootNode->getRoot());
    }
}

void KDtree::split(int maxdepth)
{
    rootNode->split(maxdepth);
}