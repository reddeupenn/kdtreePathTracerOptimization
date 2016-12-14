#include "KDnode.h"
static int currentID = 0;

KDN::KDnode::KDnode()
{
    parent = NULL;
    left = NULL;
    right = NULL;
    axis = 0;
    splitPos = 0.0;
    visited = false;
    ID = 0;
    parentID = -1;
    leftID = -1;
    rightID = -1;

    triIdStart = -1;
    triIdSize = -1;
}

KDN::KDnode::KDnode(Triangle* t)
{
    parent = NULL;
    left = NULL;
    right = NULL;
    triangles.push_back(t);
    axis = 0;
    splitPos = 0.0;
    visited = false;
    ID = 0;
    parentID = -1;
    leftID = -1;
    rightID = -1;

    triIdStart = -1;
    triIdSize = -1;
}

KDN::KDnode::KDnode(Triangle* t, int axis)
{
    parent = NULL;
    left = NULL;
    right = NULL;
    triangles.push_back(t);
    this->axis = axis;
    splitPos = 0.0;
    visited = false;
    ID = 0;
    parentID = -1;
    leftID = -1;
    rightID = -1;

    triIdStart = -1;
    triIdSize = -1;
}

KDN::KDnode::KDnode(Triangle** t, int size, int axis)
{
    parent = NULL;
    left = NULL;
    right = NULL;
    // copy data pointers
    triangles.clear();
    triangles.resize(size);
    memcpy(triangles.data(), t, sizeof(Triangle*) * size);
    this->axis = axis;
    splitPos = 0.0;
    visited = false;
    ID = 0;
    parentID = -1;
    leftID = -1;
    rightID = -1;
    updateBbox();

    triIdStart = -1;
    triIdSize = -1;
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
    else if (num > 2)
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

        //int currentID = this->ID;
        if (leftSide.size() != 0)
        {
            if (left == NULL)
            {
                left = new KDnode(leftSide.data(), leftSide.size(), (level + 1) % 3);
                // update IDs
                currentID++;
                left->ID = currentID;
                left->parentID = this->ID;
                this->leftID = currentID;
                left->parent = this;
            }
            //left->updateBbox();
            // avoid overfitting
            //left->splitPos = bbox.maxs[level];
            left->bbox.setBounds(bbox.mins[0], bbox.mins[1], bbox.mins[2],
                                 bbox.maxs[0], bbox.maxs[1], bbox.maxs[2]);
            left->bbox.maxs[level] = bbox.center[level];
            left->bbox.updateCentroid();
            left->bbox.updateSize();
            left->splitPos = bbox.maxs[level];

            left->split(maxdepth);
        }

        if (rightSide.size() != 0)
        {
            if (right == NULL)
            {
                right = new KDnode(rightSide.data(), rightSide.size(), (level + 1) % 3);
                // update IDs
                currentID++;
                right->ID = currentID;
                right->parentID = this->ID;
                this->rightID = currentID;
                right->parent = this;

            }
            //right->updateBbox();
            // avoid overfitting
            //right->splitPos = bbox.mins[level];
            right->bbox.setBounds(bbox.mins[0], bbox.mins[1], bbox.mins[2],
                                  bbox.maxs[0], bbox.maxs[1], bbox.maxs[2]);
            right->bbox.mins[level] = bbox.center[level];
            right->bbox.updateCentroid();
            right->bbox.updateSize();
            right->splitPos = bbox.mins[level];

            right->split(maxdepth);
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

bool KDN::KDnode::operator<(const KDnode& rhs)
{
    return this->ID < rhs.ID;
}


void KDN::NodeStack::push(KDnode node)
{
    nodes.push(node);
}

void KDN::NodeStack::pop()
{
    nodes.pop();
}

KDN::KDnode KDN::NodeStack::top()
{
    return nodes.top();
}

void KDN::NodeStackBare::push(NodeBare node)
{
    nodes.push(node);
}

void KDN::NodeStackBare::pop()
{
    nodes.pop();
}

KDN::NodeBare KDN::NodeStackBare::top()
{
    return nodes.top();
}