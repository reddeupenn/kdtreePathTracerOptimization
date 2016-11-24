#pragma once


#include <vector>

namespace KDN
{
    // Used to prototype.  Not used for triangle KD tree.
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

    // triangle storage for maintaining triangle information
    // should probably expand to include normals information
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

    // bounding box class for creating bounds and setting node bounds
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

    // kn node class where the bounds and triangles are defined
    // this is where the logic happens
    class KDnode
    {
    public:
        int axis;
        KDnode* parent;
        KDnode* left;
        KDnode* right;
        BoundingBox bbox;
        std::vector< Triangle* > triangles;

        KDnode();
        KDnode(Triangle* t);
        KDnode(Triangle* t, int axis = 0);
        KDnode(Triangle** t, int size, int axis = 0);
        ~KDnode();

        KDnode* getRoot();
        void updateTriangleBbox(Triangle* t);
        void mergeBbox(BoundingBox b);
        BoundingBox updateBbox();
        void split(int maxdepth);
        void deleteTree(KDnode* root);
        void printTriangleCenters();
        void printTree(KDnode* root);
        int getDepth(KDnode* n, int depth = 0);
        int getLevel(KDnode* n);
        void add(Triangle* t);
        void updateBbox(KDnode* n);
    };
}