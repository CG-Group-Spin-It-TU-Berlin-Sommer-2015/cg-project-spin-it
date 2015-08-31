#ifndef EXTENDEDOCTREE_H
#define EXTENDEDOCTREE_H

#include "basicoctree.h"

namespace octree {

/**
 * @brief The cubeObject struct Needed for the optimization.
 */
struct cubeObject{

public:
    Mesh* mesh;
    float beta;
    GLint index;

};

}

class ExtendedOctree : public BasicOctree
{
public:
    ExtendedOctree();

    void updateBetaValues();
    QVector<octree::cubeObject>* getInnerCubes();

    void getNodesOfDepth(GLint depth,QVector<GLint>* indices);
    void getInnerLeavesForNode(GLint index,QVector<GLint>* indices);

    bool splitAndMerge(GLfloat epsilon);

    void split(octree::octreeNode* nodePointer);
    void split(GLint nodeIndex);

    void increaseShell(GLint loopAmount);

    bool allChildrenAreLeaves(GLint nodeIndex);

    void setVoids();
    void setMergeNodes();

    void deleteNodeMeshes();

private:

    QVector<octree::cubeObject> cubeVector;

    void handleShellNeighbor(GLint x, GLint y, GLint z, QVector<GLint>* backVec);
    void setMergeChild(GLint index, GLfloat beta);
    void addCubeMesh(GLint index,QVector<GLfloat>* geometry, QVector<GLint>* indices, GLint offset);


};

#endif // EXTENDEDOCTREE_H
