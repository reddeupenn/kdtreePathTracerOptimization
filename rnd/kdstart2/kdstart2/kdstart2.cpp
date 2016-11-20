// kdstart2.cpp : Defines the entry point for the console application.
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

using namespace std;


class Point
{
public:
    float x;
    float y;
    float z;

    Point()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    Point(float x, float y, float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Point(const Point& p)
    {
        x = p.x;
        y = p.y;
        z = p.z;
    }

    ~Point() {}
};

class Triangle
{
public:
    float x1, x2, x3,
          y1, y2, y3,
          z1, z2, z3;

    float center[3];
    float mins[3];
    float maxs[3];

    Triangle()
    {
        x1 = 0.0; y1 = 0.0; z1 = 0.0;
        x2 = 0.0; y2 = 0.0; z2 = 0.0;
        x3 = 0.0; y3 = 0.0; z3 = 0.0;

        center[0] = 0.0;
        center[1] = 0.0;
        center[2] = 0.0;
        computeBounds();
    }

    Triangle(float X1, float Y1, float Z1,
             float X2, float Y2, float Z2,
             float X3, float Y3, float Z3)
    {
        x1 = X1; y1 = Y1; z1 = Z1;
        x2 = X2; y2 = Y2; z2 = Z2;
        x3 = X3; y3 = Y3; z3 = Z3;

        computeCentroid();
        computeBounds();
    }

    ~Triangle() {}

    void setValues(float val)
    {
        x1 = val; y1 = val; z1 = val;
        x2 = val; y2 = val; z2 = val;
        x3 = val; y3 = val; z3 = val;

        center[0] = val; center[1] = val; center[2] = val;
        computeBounds();
    }

    void setValues(float X1, float Y1, float Z1,
                   float X2, float Y2, float Z2,
                   float X3, float Y3, float Z3)
    {
        x1 = X1; y1 = Y1; z1 = Z1;
        x2 = X2; y2 = Y2; z2 = Z2;
        x3 = X3; y3 = Y3; z3 = Z3;

        computeCentroid();
        computeBounds();
    }

    void computeCentroid()
    {
        center[0] = (x1 + x2 + x3) / 3.0;
        center[1] = (y1 + y2 + y3) / 3.0;
        center[2] = (z1 + z2 + z3) / 3.0;
    }

    void computeBounds()
    {
        mins[0] = x1 < x2 ? (x1 < x3 ? x1 : x3) : (x2 < x3 ? x2 : x3);
        mins[1] = y1 < y2 ? (y1 < y3 ? y1 : y3) : (y2 < y3 ? y2 : y3);
        mins[2] = z1 < z2 ? (z1 < z3 ? z1 : z3) : (z2 < z3 ? z2 : z3);

        maxs[0] = x1 > x2 ? (x1 > x3 ? x1 : x3) : (x2 > x3 ? x2 : x3);
        maxs[1] = y1 > y2 ? (y1 > y3 ? y1 : y3) : (y2 > y3 ? y2 : y3);
        maxs[2] = z1 > z2 ? (z1 > z3 ? z1 : z3) : (z2 > z3 ? z2 : z3);
    }

};

class BoundingBox
{
public:
    float mins[3];
    float maxs[3];
    //float centerx, centery, centerz;
    float center[3]{};

    BoundingBox()
    {
        setBounds(0.0);
    }

    BoundingBox(float xMin, float yMin, float zMin,
                float xMax, float yMax, float zMax)
    {
        setBounds(mins[0], mins[1], mins[2],
                  maxs[0], maxs[1], maxs[2]);
        updateCentroid();

    }

    BoundingBox(Triangle* t)
    {
        setBounds(t);
        updateCentroid();
    }

    void setBounds(Triangle* t)
    {
        mins[0] = t->x1 < t->x2 ? (t->x1 < t->x3 ? t->x1 : t->x3) : (t->x2 < t->x3 ? t->x2 : t->x3);
        mins[1] = t->y1 < t->y2 ? (t->y1 < t->y3 ? t->y1 : t->y3) : (t->y2 < t->y3 ? t->y2 : t->y3);
        mins[2] = t->z1 < t->z2 ? (t->z1 < t->z3 ? t->z1 : t->z3) : (t->z2 < t->z3 ? t->z2 : t->z3);

        maxs[0] = t->x1 > t->x2 ? (t->x1 > t->x3 ? t->x1 : t->x3) : (t->x2 > t->x3 ? t->x2 : t->x3);
        maxs[1] = t->y1 > t->y2 ? (t->y1 > t->y3 ? t->y1 : t->y3) : (t->y2 > t->y3 ? t->y2 : t->y3);
        maxs[2] = t->z1 > t->z2 ? (t->z1 > t->z3 ? t->z1 : t->z3) : (t->z2 > t->z3 ? t->z2 : t->z3);
        updateCentroid();
    }

    void updateCentroid()
    {
        center[0] = (mins[0] + maxs[0]) / 2.0;
        center[1] = (mins[1] + maxs[1]) / 2.0;
        center[2] = (mins[2] + maxs[2]) / 2.0;
    }

    void setBounds(float xMin, float yMin, float zMin,
                   float xMax, float yMax, float zMax)
    {
        mins[0] = xMin; mins[1] = yMin; mins[2] = zMin;
        maxs[0] = xMax; maxs[1] = yMax; maxs[2] = zMax;
        updateCentroid();
    }

    void setBounds(float val)
    {
        mins[0] = val; mins[1] = val; mins[2] = val;
        maxs[0] = val; maxs[1] = val; maxs[2] = val;
        center[0] = val; center[1] = val; center[2] = val;
        // centery = val; centerz = val;
    }
};

class KDnode
{
public:
    int axis;
    KDnode* parent;
    KDnode* left;
    KDnode* right;
    BoundingBox bbox;
    std::vector< Triangle* > triangles;

    KDnode()
    {
        parent = NULL;
        left = NULL;
        right = NULL;
        axis = 0;
    }

    KDnode(Triangle* t)
    {
        parent = NULL;
        left = NULL;
        right = NULL;
        triangles.push_back(t);
        axis = 0;
    }

    KDnode(Triangle* t, int axis=0)
    {
        parent = NULL;
        left = NULL;
        right = NULL;
        triangles.push_back(t);
        this->axis = axis;
    }

    KDnode(Triangle** t, int size, int axis=0)
    {
        // copy data pointers
        triangles.clear();
        triangles.resize(size);
        memcpy(triangles.data(), t, sizeof(Triangle*) * size);
        this->axis = axis;
        updateBbox();
    }

    ~KDnode() {}

    KDnode* getRoot()
    {
        KDnode* p = this;

        while (p->parent != NULL)
        {
            p = p->parent;
        }

        return p;
    }

    void updateTriangleBbox(Triangle* t)
    {
        BoundingBox b(t);
        mergeBbox(b);
    }

    void mergeBbox(BoundingBox b)
    {
        bbox.mins[0] = bbox.mins[0] > b.mins[0] ? b.mins[0] : bbox.mins[0];
        bbox.mins[1] = bbox.mins[1] > b.mins[1] ? b.mins[1] : bbox.mins[1];
        bbox.mins[2] = bbox.mins[2] > b.mins[2] ? b.mins[2] : bbox.mins[2];

        bbox.maxs[0] = bbox.maxs[0] < b.maxs[0] ? b.maxs[0] : bbox.maxs[0];
        bbox.maxs[1] = bbox.maxs[1] < b.maxs[1] ? b.maxs[1] : bbox.maxs[1];
        bbox.maxs[2] = bbox.maxs[2] < b.maxs[2] ? b.maxs[2] : bbox.maxs[2];

        bbox.updateCentroid();
    }

    BoundingBox updateBbox()
    {
        int numTris = triangles.size();

        // set the bounds to the first triangle to avoid 0 bounds
        // when the bbox is at 0 0 0 0 0 0
        if (numTris > 0)
            bbox.setBounds(triangles[0]);

        for (int i = 1; i < numTris; i++)
        {
            updateTriangleBbox(triangles[i]);
        }
        
        if (left)
        {
            mergeBbox(left->updateBbox());
        }

        if (right)
        {
            mergeBbox(right->updateBbox());
        }
        
        //bbox.updateCentroid();
        
        // pad bounds
        float pad = 0.001;
        bbox.mins[0] -= pad;
        bbox.mins[1] -= pad;
        bbox.mins[2] -= pad;
        
        bbox.maxs[0] += pad;
        bbox.maxs[1] += pad;
        bbox.maxs[2] += pad;

        return bbox;
    }

    void split(int maxdepth)
    {
        int num = triangles.size();

        if (num == 0)
        {
            if (left)
                left->split(maxdepth);

            if (right)
                right->split(maxdepth);
        }
        // don't split if we have less than 2 triangles
        else if (num > 1)
        {
            int level = getLevel(this);

            if (level > maxdepth)
                return;

            level = level % 3;

            std::vector<Triangle*> leftSide;
            std::vector<Triangle*> rightSide;
            
            //printf("\nlevel = %d", level);
            for (int i = 0; i < num; i++)
            {
                if (triangles[i]->mins[level] < bbox.center[level]+0.0001)
                {
                    leftSide.push_back(triangles[i]);
                }
                if (triangles[i]->maxs[level] >= bbox.center[level]-0.0001)
                {
                    rightSide.push_back(triangles[i]);
                }
            }

            // no split possible so we return
            if (leftSide.size() == triangles.size() || rightSide.size() == triangles.size())
                return;

            if (leftSide.size() != 0)
            {
                if (left == NULL)
                {
                    left = new KDnode(leftSide.data(), leftSide.size(), (level + 1)%3);
                    left->updateBbox();
                    left->parent = this;
                    left->split(maxdepth);
                }
                else
                {
                    left->updateBbox();
                    left->split(maxdepth);
                }
            }

            if (rightSide.size() != 0)
            {
                if (right == NULL)
                {
                    right = new KDnode(rightSide.data(), rightSide.size(), (level + 1)%3);
                    right->updateBbox();
                    right->parent = this;
                    right->split(maxdepth);
                }
                else
                {
                    right->updateBbox();
                    right->split(maxdepth);
                }
            }

            // split was successful so we remove the current triangles from the node
            if (leftSide.size() != triangles.size() || rightSide.size() != triangles.size())
                triangles.erase(triangles.begin(), triangles.end());
        }
    }

    void deleteTree(KDnode* root)
    {
        if (root != NULL)
        {
            deleteTree(root->left);
            deleteTree(root->right);
            delete root;

            if (root->left != NULL)
                root->left = NULL;
            if (root->right != NULL)
                root->right = NULL;
            root = NULL;
        }
    }

    void printTriangleCenters()
    {
        printf("\ntriangle centers:\n");
        for (int i = 0; i < triangles.size(); i++)
        {
            printf("%0.1f %0.1f %0.1f\n", triangles[i]->center[0],
                                          triangles[i]->center[1],
                                          triangles[i]->center[2]);
        }
    }
    
    void printTree(KDnode* root)
    {
        if (root != NULL)
        {
            printf("lvl:%d sz:%d ", root->getLevel(root), root->triangles.size());

            if (root->parent)
            {
                if (root->parent->left == root)
                    printf("node left:");
                else
                    printf("node right:");
                printf(" xyz: [%0.1f %0.1f %0.1f] axis: %d parent[%0.1f %0.1f %0.1f] bb[%0.1f %0.1f %0.1f] [%0.1f %0.1f %0.1f]\n",
                        root->bbox.center[0],
                        root->bbox.center[1],
                        root->bbox.center[2],
                        root->axis,
                        root->parent->bbox.center[0],
                        root->parent->bbox.center[1],
                        root->parent->bbox.center[2],
                        root->bbox.mins[0], root->bbox.mins[1], root->bbox.mins[2],
                        root->bbox.maxs[0], root->bbox.maxs[1], root->bbox.maxs[2]);
            }
            else
            {
                printf(" xyz: [%0.1f %0.1f %0.1f] axis: %d bb[%0.1f %0.1f %0.1f] [%0.1f %0.1f %0.1f]\n",
                        root->bbox.center[0],
                        root->bbox.center[1],
                        root->bbox.center[2],
                        root->axis,
                        root->bbox.mins[0], root->bbox.mins[1], root->bbox.mins[2],
                        root->bbox.maxs[0], root->bbox.maxs[1], root->bbox.maxs[2]);
            }


            printTree(root->left);
            printTree(root->right);
        }
    }
    

    int getDepth(KDnode* n, int depth = 0)
    {
        if (n == NULL)
            return depth;

        int depth1 = depth;
        int depth2 = depth;

        if (n->left)
        {
            depth1 += getDepth(n->left, depth1++);
        }
        if (n->right)
        {
            depth2 += getDepth(n->right, depth2++);
        }

        return (depth1 > depth2 ? depth1 : depth2);
    }

    int getLevel(KDnode* n)
    {
        if (n == NULL)
            return 0;

        int level = 0;

        KDnode* node = n;

        while (node->parent != NULL)
        {
            level++;
            node = node->parent;
        }

        return level;
    }

/*
    void pprint(KDnode* n, int offset, int depth = 0)
    {
        if (n != NULL)
        {
            int d = n->getDepth(n, 0);
            int h = pow(2, d);

            printf("\n");
            if (depth == 0)
            {
                std::cout << std::setw(offset + h); std::cout << " ";
                printf("%0.1f\n", n->triangle.centerx);
            }

            std::cout << std::setw(offset + h - 1); std::cout << " ";
            printf(" /  \\\n");

            std::cout << std::setw(offset + h - 1); std::cout << " ";
            if (n->left)
            {
                printf("%0.1f", n->left->triangle.centerx);
            }
            std::cout << std::setw(2); std::cout << " ";
            if (n->right)
                printf("%0.1f", n->right->triangle.centerx);

            if (d > 1)
            {
                pprint(n->left, offset - 2, d - 1);
                pprint(n->right, offset + 2, d - 1);
            }
        }
    }

    void prettyPrint(KDnode* n, std::vector< std::vector<KDnode*> >* nodeVecs, int offset = 0)
    {
        if (n != NULL)
        {
            int depth = n->getDepth(n) + 1;
            int level = n->getLevel(n);
            int maxDepth = (*nodeVecs).size();


            if (depth == maxDepth)
            {
                (*nodeVecs)[0][pow(2, depth) / 2] = n;
                //std::cout << nodeVecs[0][pow(2, depth) / 2]->triangle.centerx << std::endl;
                offset = pow(2, depth - 1) / 2;
            }
            else
                (*nodeVecs)[maxDepth - depth][offset + pow(2, maxDepth - level) / 2] = n;

            prettyPrint(n->right, nodeVecs, offset + pow(2, maxDepth - level - 1) / 2);
            prettyPrint(n->left, nodeVecs, offset - pow(2, maxDepth - level - 1) / 2);
        }
    }

    void pprint2(KDnode* n)
    {
        if (n)
        {
            // assemble 2d vector of nodes to fill in later
            int depth = n->getDepth(n);

            printf("\ndepth = %d\n", depth);

            int width = pow(2, depth + 1);
            std::vector< std::vector<KDnode*> > nodeVecs;

            for (int i = 0; i <= depth; i++)
            {
                std::vector<KDnode*> nodeVec;
                for (int j = 0; j < width; j++)
                    nodeVec.push_back(NULL);

                nodeVecs.push_back(nodeVec);
            }

            prettyPrint(n, &nodeVecs);

            for (int i = 0; i <= depth; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    KDnode* node = nodeVecs[i][j];
                    if (node != NULL)
                        printf("%0.1f ", node->triangle.centerx);
                    else
                        printf("x ");
                }
                printf("\n");
            }
        }
    }
*/
    /*
    void insert(KDnode* n)
    {
        n->axis = ++(n->axis) % 3;
        printf("n->axis = %d\n", n->axis);
        if (axis == 0)
        {
            if (n->bbox.centerx < this->bbox.centerx)
            {
                if (this->left == NULL)
                {
                    this->left = n;
                    n->parent = this;
                }
                else
                {
                    this->left->insert(n);
                }
            }
            else if (n->bbox.centerx >= this->bbox.centerx)
            {
                if (this->right == NULL)
                {
                    this->right = n;
                    n->parent = this;
                }
                else
                {
                    this->right->insert(n);
                }
            }
        }
        else if (axis == 1)
        {
            if (n->bbox.centery < this->bbox.centery)
            {
                if (this->left == NULL)
                {
                    this->left = n;
                    n->parent = this;
                }
                else
                {
                    this->left->insert(n);
                }
            }
            else if (n->bbox.centery >= this->bbox.centery)
            {
                if (this->right == NULL)
                {
                    this->right = n;
                    n->parent = this;
                }
                else
                {
                    this->right->insert(n);
                }
            }
        }
        else if (axis == 2)
        {
            if (n->bbox.centerz < this->bbox.centerz)
            {
                if (this->left == NULL)
                {
                    this->left = n;
                    n->parent = this;
                }
                else
                {
                    this->left->insert(n);
                }
            }
            else if (n->bbox.centerz >= this->bbox.centerz)
            {
                if (this->right == NULL)
                {
                    this->right = n;
                    n->parent = this;
                }
                else
                {
                    this->right->insert(n);
                }
            }
        }
    }
    */

    void add(Triangle* t)
    {
        triangles.push_back(t);
    }

    void updateBbox(KDnode* n)
    {
        if (n)
        {
            if (n->left == NULL && n->right == NULL)
            {
                n->bbox.setBounds(n->triangles[0]);
                return;
            }

            BoundingBox current = n->bbox;

            if (n->left)
            {
                n->left->updateBbox(n->left);
                n->left->bbox.updateCentroid();
            }
            if (n->right)
            {
                n->right->updateBbox(n->right);
                n->right->bbox.updateCentroid();
            }
        }
    }
};



float frand(float scale)
{
    return scale * ((rand() % 10000 + 1) / 10000.0);
}


void deleteTree(KDnode* root)
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


std::vector<Triangle*> getTrianglesFromFile(const char* path)
{
    std::vector<Triangle*>triangles;

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

            Triangle* t = new Triangle(x1, y1, z1,
                                       x2, y2, z2, 
                                       x3, y3, z3);
            triangles.push_back(t);
        }
    }
    return triangles;
}

void printToFile(KDnode* root, ofstream& fileout)
{
    if (root != NULL)
    {
        fileout << root->bbox.mins[0] << " " << root->bbox.mins[1] << " " << root->bbox.mins[2] << " "
                << root->bbox.maxs[0] << " " << root->bbox.maxs[1] << " " << root->bbox.maxs[2] << "\n";

        printToFile(root->left, fileout);
        printToFile(root->right, fileout);
    }
}

void writeKDtoFile(KDnode* root, const char* path)
{
    ofstream fileout;
    fileout.open(path);

    printToFile(root, fileout);

    fileout.close();
}

int main()
{

    Point a[] = { Point(1.5, 20, 0),
        Point(1, 8, 2),
        Point(2, 5, 4),
        Point(7, 4, 4) };


    /*
    Triangle t[] = { Triangle(10, 10, 11, 13, 12, 8, 9, 10, 8),
                     Triangle(1, 8, 2, -1, 3, 2, 5, 8, 10),
                     Triangle(12, 15, 14, 20, 15, 26, 14, 26, 36),
                     Triangle(-3, -4, -4, -1, -5, -6, -7, -3, -10),
                     Triangle(20, 18, 12, 15, 30, 20, 50, 90, 31) };
    */

    int TRICOUNT = 5;
    Triangle** tempT = new Triangle*[TRICOUNT];
    tempT[0] = new Triangle(10, 10, 11, 13, 12, 8, 9, 10, 8);
    tempT[1] = new Triangle(1, 8, 2, -1, 3, 2, 5, 8, 10);
    tempT[2] = new Triangle(12, 15, 14, 20, 15, 26, 14, 26, 36);
    tempT[3] = new Triangle(-3, -4, -4, -1, -5, -6, -7, -3, -10);
    tempT[4] = new Triangle(20, 18, 12, 15, 30, 20, 50, 90, 31);


    // read file generated from Houdini and get triangles
    char path[1024];
    if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_PROFILE, NULL, 0, path)))
    {
        printf("path = %s\n", path);
    }
    strcat_s(path, sizeof(char) * 1024, "/git/CIS565/kdtreePathTracerOptimization/rnd/houdini/data");
    printf("path = %s\n", path);

    std::vector<Triangle*> triangles = getTrianglesFromFile(path);


     
    KDnode* KD = new KDnode(triangles.data(), triangles.size());
    //KDnode* KD = new KDnode(tempT, 5);
    //KDnode* KD = new KDnode(&first, 0);
    //KDnode* KD1 = new KDnode(1.0, 3.0, 1, 0);
    //KDnode* KD2 = new KDnode(1.0, 3.0, 1, 0);
    //KDnode* KD3 = new KDnode(1.0, 3.0, 1, 0);
    //KDnode* KD4 = new KDnode(1.0, 3.0, 1, 0);

    /*
    for (int i = 0; i < 5; i++)
    {
        KDnode* x = new KDnode(&(t[i]));
        KD->insert(x);
    }
    */
    
    KD->updateBbox();



    cout << a[0].x << " " << a[0].y << endl;

    KD->printTree(KD->getRoot());

    cout << KD << endl;
    cout << KD->getRoot() << endl;


    //deleteTree(KD1->getRoot());
    //deleteTree(KD2->getRoot());
    //deleteTree(KD3->getRoot());
    //deleteTree(KD4->getRoot());

    Triangle t0(0.0, 0.0, 0.0,
                1.0, 6.0, 1.0,
                2.0, 2.0, -2.0);

    BoundingBox b(&(t0));

    printf("\nbbox vals min: [%.1f %.1f %.1f]\n          max: [%.1f, %.1f, %.1f]\n",
           b.mins[0], b.mins[1], b.mins[2], b.maxs[0], b.maxs[1], b.maxs[2]);


    int depth = KD->getRoot()->getDepth(KD->getRoot(), 0);
    printf("\ndepth = %d\n", depth);

    KD->printTriangleCenters();
    KD->printTree(KD);
    
    KD->split(3);

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

    _CrtDumpMemoryLeaks();
    return 0;
}



