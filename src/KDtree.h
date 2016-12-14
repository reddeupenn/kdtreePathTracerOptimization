// kdstart3.cpp : Defines the entry point for the console application.
//

#pragma once
#include <stdlib.h>
#include <random>
#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "KDnode.h"

using namespace std;

class KDtree
{
public:
    KDN::KDnode* rootNode;

    KDtree();
    KDtree(std::vector<KDN::Triangle*> t);
    KDtree(const char* path);
    ~KDtree();
    float frand(float scale);
    void deleteTree(KDN::KDnode* root);
    std::vector<KDN::Triangle*> getTrianglesFromFile(const char* path);
    void printToFile(KDN::KDnode* root, ofstream& fileout);
    void writeKDtoFile(KDN::KDnode* root, const char* path);
    void printTree();
    void split(int maxdepth);
};

