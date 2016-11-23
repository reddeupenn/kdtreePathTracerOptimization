#include "stdafx.h"
#include "KDnode.h"

KDN::KDnode::KDnode()
{
    parent = NULL;
    left = NULL;
    right = NULL;
    axis = 0;
}

KDN::KDnode::KDnode(Triangle* t)
{
    parent = NULL;
    left = NULL;
    right = NULL;
    triangles.push_back(t);
    axis = 0;
}

KDN::KDnode::KDnode(Triangle* t, int axis)
{
    parent = NULL;
    left = NULL;
    right = NULL;
    triangles.push_back(t);
    this->axis = axis;
}

KDN::KDnode::KDnode(Triangle** t, int size, int axis)
{
    // copy data pointers
    triangles.clear();
    triangles.resize(size);
    memcpy(triangles.data(), t, sizeof(Triangle*) * size);
    this->axis = axis;
    updateBbox();
}

KDN::KDnode::~KDnode() {}

KDN::KDnode* KDN::KDnode::getRoot()
{
    KDnode* p = this;

    while (p->parent != NULL)
    {
        p = p->parent;
    }

    return p;
}

void KDN::KDnode::updateTriangleBbox(Triangle* t)
{
    BoundingBox b(t);
    mergeBbox(b);
}

void KDN::KDnode::mergeBbox(BoundingBox b)
{
    bbox.mins[0] = bbox.mins[0] > b.mins[0] ? b.mins[0] : bbox.mins[0];
    bbox.mins[1] = bbox.mins[1] > b.mins[1] ? b.mins[1] : bbox.mins[1];
    bbox.mins[2] = bbox.mins[2] > b.mins[2] ? b.mins[2] : bbox.mins[2];

    bbox.maxs[0] = bbox.maxs[0] < b.maxs[0] ? b.maxs[0] : bbox.maxs[0];
    bbox.maxs[1] = bbox.maxs[1] < b.maxs[1] ? b.maxs[1] : bbox.maxs[1];
    bbox.maxs[2] = bbox.maxs[2] < b.maxs[2] ? b.maxs[2] : bbox.maxs[2];

    bbox.updateCentroid();
}

KDN::BoundingBox KDN::KDnode::updateBbox()
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

void KDN::KDnode::split(int maxdepth)
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
            if (triangles[i]->mins[level] < bbox.center[level] + 0.0001)
            {
                leftSide.push_back(triangles[i]);
            }
            if (triangles[i]->maxs[level] >= bbox.center[level] - 0.0001)
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
                left = new KDnode(leftSide.data(), leftSide.size(), (level + 1) % 3);
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
                right = new KDnode(rightSide.data(), rightSide.size(), (level + 1) % 3);
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

void KDN::KDnode::deleteTree(KDnode* root)
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

void KDN::KDnode::printTriangleCenters()
{
    printf("\ntriangle centers:\n");
    for (int i = 0; i < triangles.size(); i++)
    {
        printf("%0.1f %0.1f %0.1f\n", triangles[i]->center[0],
                triangles[i]->center[1],
                triangles[i]->center[2]);
    }
}

void KDN::KDnode::printTree(KDnode* root)
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


int KDN::KDnode::getDepth(KDnode* n, int depth)
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

int KDN::KDnode::getLevel(KDnode* n)
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

void KDN::KDnode::add(Triangle* t)
{
    triangles.push_back(t);
}

void KDN::KDnode::updateBbox(KDnode* n)
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
