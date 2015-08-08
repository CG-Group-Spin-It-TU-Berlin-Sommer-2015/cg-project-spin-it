#ifndef EXTENDEDOCTREE_H
#define EXTENDEDOCTREE_H

#include "basicoctree.h"

namespace octree {

struct cubeObject{

    Mesh* mesh;
    float beta;
    GLint index;

};

}

class ExtendedOctree : public BasicOctree
{
public:
    ExtendedOctree();

    void setBetasForCubes(QVector<octree::cubeObject>* vec);
    void getInnerCubes(QVector<octree::cubeObject>* vec);

    void getNodesOfDepth(GLint depth,QVector<GLint>* indices);
    void getInnerNodesForNode(GLint index,QVector<GLint>* indices);
    void getInnerNodesOfLeafSet(QVector<GLint>* indices);
    void getInnerLeaves(QVector<GLint>* indices);

    void splitAndMerge(GLfloat epsilon);

    void split(octree::octreeNode* nodePointer);
    void split(GLint nodeIndex);
    void merge(octree::octreeNode* nodePointer);
    void merge(GLint nodeIndex);

    void increaseShell(GLint loopAmount);

    bool allChildrenAreLeaves(GLint nodeIndex);

    void setVoids();

private:

    void handleShellNeighbor(GLint x, GLint y, GLint z, QVector<GLint>* backVec);

};

#endif // EXTENDEDOCTREE_H
