// kdstart2.cpp : Defines the entry point for the console application.
//

#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#include "stdafx.h"
#include <random>
#include <stdio.h>
#include <vector>

#include <iostream>
#include <iomanip>

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

    Triangle()
    {
        x1 = 0.0; y1 = 0.0; z1 = 0.0;
        x2 = 0.0; y2 = 0.0; z2 = 0.0;
        x3 = 0.0; y3 = 0.0; z3 = 0.0;

        center[0] = 0.0;
        center[1] = 0.0;
        center[2] = 0.0;
    }

    Triangle(float X1, float Y1, float Z1,
             float X2, float Y2, float Z2,
             float X3, float Y3, float Z3)
    {
        x1 = X1; y1 = Y1; z1 = Z1;
        x2 = X2; y2 = Y2; z2 = Z2;
        x3 = X3; y3 = Y3; z3 = Z3;

        computeCentroid();
    }

    ~Triangle() {}

    void setValues(float val)
    {
        x1 = val; y1 = val; z1 = val;
        x2 = val; y2 = val; z2 = val;
        x3 = val; y3 = val; z3 = val;

        center[0] = val; center[1] = val; center[2] = val;
    }

    void setValues(float X1, float Y1, float Z1,
                   float X2, float Y2, float Z2,
                   float X3, float Y3, float Z3)
    {
        x1 = X1; y1 = Y1; z1 = Z1;
        x2 = X2; y2 = Y2; z2 = Z2;
        x3 = X3; y3 = Y3; z3 = Z3;

        computeCentroid();
    }

    void computeCentroid()
    {
        center[0] = (x1 + x2 + x3) / 3.0;
        center[1] = (y1 + y2 + y3) / 3.0;
        center[2] = (z1 + z2 + z3) / 3.0;
    }

};

class BoundingBox
{
public:
    float xmin, ymin, zmin;
    float xmax, ymax, zmax;
    //float centerx, centery, centerz;
    float center[3]{};

    BoundingBox()
    {
        setBounds(0.0);
    }

    BoundingBox(float xMin, float yMin, float zMin,
                float xMax, float yMax, float zMax)
    {
        setBounds(xMin, yMin, zMin,
                  xMax, yMax, zMax);
        updateCentroid();

    }

    BoundingBox(Triangle* t)
    {
        setBounds(t);
        updateCentroid();
    }

    void setBounds(Triangle* t)
    {
        xmin = t->x1 < t->x2 ? (t->x1 < t->x3 ? t->x1 : t->x3) : (t->x2 < t->x3 ? t->x2 : t->x3);
        ymin = t->y1 < t->y2 ? (t->y1 < t->y3 ? t->y1 : t->y3) : (t->y2 < t->y3 ? t->y2 : t->y3);
        zmin = t->z1 < t->z2 ? (t->z1 < t->z3 ? t->z1 : t->z3) : (t->z2 < t->z3 ? t->z2 : t->z3);

        xmax = t->x1 > t->x2 ? (t->x1 > t->x3 ? t->x1 : t->x3) : (t->x2 > t->x3 ? t->x2 : t->x3);
        ymax = t->y1 > t->y2 ? (t->y1 > t->y3 ? t->y1 : t->y3) : (t->y2 > t->y3 ? t->y2 : t->y3);
        zmax = t->z1 > t->z2 ? (t->z1 > t->z3 ? t->z1 : t->z3) : (t->z2 > t->z3 ? t->z2 : t->z3);
    }

    void updateCentroid()
    {
        center[0] = (xmin + xmax) / 2.0;
        center[1] = (ymin + ymax) / 2.0;
        center[2] = (zmin + zmax) / 2.0;
    }

    void setBounds(float xMin, float yMin, float zMin,
                   float xMax, float yMax, float zMax)
    {
        xmin = xMin; ymin = yMin; zmin = zMin;
        xmax = xMax; ymax = yMax; zmax = zMax;
        updateCentroid();
    }

    void setBounds(float val)
    {
        xmin = val; ymin = val; zmin = val;
        xmax = val; ymax = val; zmax = val;
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
        bbox.xmin = bbox.xmin > b.xmin ? b.xmin : bbox.xmin;
        bbox.ymin = bbox.ymin > b.ymin ? b.ymin : bbox.ymin;
        bbox.zmin = bbox.zmin > b.zmin ? b.zmin : bbox.zmin;

        bbox.xmax = bbox.xmax < b.xmax ? b.xmax : bbox.xmax;
        bbox.ymax = bbox.ymax < b.ymax ? b.ymax : bbox.ymax;
        bbox.zmax = bbox.zmax < b.zmax ? b.zmax : bbox.zmax;
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

        return bbox;
    }

    void split()
    {
        int num = triangles.size();

        // don't split if we have less than 2 triangles
        if (num > 1)
        {

            int level = getLevel(this);

            std::vector<Triangle*> leftSide;
            std::vector<Triangle*> rightSide;

            for (int i = 0; i < num; i++)
            {
                if (triangles[i]->center[level] < bbox.center[level])
                {
                    leftSide.push_back(triangles[i]);
                }
                else
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
                    left = new KDnode(leftSide.data(), leftSide.size(), level + 1);
                    left->parent = this;
                }
                else
                {
                    left->split();
                }
            }

            if (rightSide.size() != 0)
            {
                if (right == NULL)
                {
                    right = new KDnode(rightSide.data(), rightSide.size(), level + 1);
                    right->parent = this;
                }
                else
                {
                    right->split();
                }
            }

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
                        root->bbox.xmin, root->bbox.ymin, root->bbox.zmin,
                        root->bbox.xmax, root->bbox.ymax, root->bbox.zmax);
            }
            else
            {
                printf(" xyz: [%0.1f %0.1f %0.1f] axis: %d bb[%0.1f %0.1f %0.1f] [%0.1f %0.1f %0.1f]\n",
                        root->bbox.center[0],
                        root->bbox.center[1],
                        root->bbox.center[2],
                        root->axis,
                        root->bbox.xmin, root->bbox.ymin, root->bbox.zmin,
                        root->bbox.xmax, root->bbox.ymax, root->bbox.zmax);
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
                /*
                BoundingBox leftbb = n->left->bbox;
                if (leftbb.xmin < current.xmin)
                    n->bbox.xmin = leftbb.xmin;
                if (leftbb.ymin < current.ymin)
                    n->bbox.ymin = leftbb.ymin;
                if (leftbb.zmin < current.zmin)
                    n->bbox.zmin = leftbb.zmin;
                if (leftbb.xmax > current.xmax)
                    n->bbox.xmax = leftbb.xmax;
                if (leftbb.ymax > current.ymax)
                    n->bbox.ymax = leftbb.ymax;
                if (leftbb.zmax > current.zmax)
                    n->bbox.zmax = leftbb.zmax;
                */
            }
            if (n->right)
            {
                n->right->updateBbox(n->right);
                /*
                BoundingBox rightbb = n->right->bbox;
                if (rightbb.xmin < current.xmin)
                    n->bbox.xmin = rightbb.xmin;
                if (rightbb.ymin < current.ymin)
                    n->bbox.ymin = rightbb.ymin;
                if (rightbb.zmin < current.zmin)
                    n->bbox.zmin = rightbb.zmin;
                if (rightbb.xmax > current.xmax)
                    n->bbox.xmax = rightbb.xmax;
                if (rightbb.ymax > current.ymax)
                    n->bbox.ymax = rightbb.ymax;
                if (rightbb.zmax > current.zmax)
                    n->bbox.zmax = rightbb.zmax;
                */
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
        delete root;

        if (root->left != NULL)
            root->left = NULL;
        if (root->right != NULL)
            root->right = NULL;
        root = NULL;
    }
}

int main()
{

    Point a[] = { Point(1.5, 20, 0),
        Point(1, 8, 2),
        Point(2, 5, 4),
        Point(7, 4, 4) };


    Triangle t[] = { Triangle(10, 10, 11, 13, 12, 8, 9, 10, 8),
                     Triangle(1, 8, 2, -1, 3, 2, 5, 8, 10),
                     Triangle(12, 15, 14, 20, 15, 26, 14, 26, 36),
                     Triangle(-3, -4, -4, -1, -5, -6, -7, -3, -10),
                     Triangle(20, 18, 12, 15, 30, 20, 50, 90, 31) };

    int TRICOUNT = 5;
    Triangle** tempT = new Triangle*[TRICOUNT];
    tempT[0] = new Triangle(10, 10, 11, 13, 12, 8, 9, 10, 8);
    tempT[1] = new Triangle(1, 8, 2, -1, 3, 2, 5, 8, 10);
    tempT[2] = new Triangle(12, 15, 14, 20, 15, 26, 14, 26, 36);
    tempT[3] = new Triangle(-3, -4, -4, -1, -5, -6, -7, -3, -10);
    tempT[4] = new Triangle(20, 18, 12, 15, 30, 20, 50, 90, 31);



    //Triangle first(-30, -10.0, -28, -6, -50, -48, -38, -70, -49);
    
    KDnode* KD = new KDnode(tempT, 5);
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
           b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax);


    int depth = KD->getRoot()->getDepth(KD->getRoot(), 0);
    printf("\ndepth = %d\n", depth);

    KD->printTriangleCenters();
    KD->printTree(KD);
    KD->split();
    printf("\nreprinting after split\n");
    KD->updateBbox();
    KD->printTree(KD);

    KD->printTriangleCenters();

    //for (int i=0; i<1000*3; i++)
    //    cout << frand(1.0)*10 << endl;



    //Point* points = {Point()}

    //int * x =  (int*)malloc(3 * sizeof(int));

    //KD->pprint(KD, 0, 0);
    //printf("\n");





    cout << KD << endl;



    //KD->pprint2(KD);

/*
    // vector copying
    
    int NUM = 15;
    std::vector<int>v;
    v.resize(NUM);

    int* x = new int[NUM];
    for (int i = 0; i < NUM; i++)
        x[i] = i;

    memcpy(v.data(), x, sizeof(int) * NUM);
    delete[] x;

    printf("\n");
    for (int i = 0; i < v.size(); i++)
        printf("%d ", v[i]);
    printf("\n");

    // vector copying end
*/
    

    deleteTree(KD->getRoot());

    for (int i = 0; i < TRICOUNT; i++)
        delete tempT[i];

    delete[] tempT;

    //int* cc = new int[4];

    _CrtDumpMemoryLeaks();
    return 0;
}



