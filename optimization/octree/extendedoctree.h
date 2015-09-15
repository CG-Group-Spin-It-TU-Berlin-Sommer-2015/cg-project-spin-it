#ifndef EXTENDEDOCTREE_H
#define EXTENDEDOCTREE_H

#include "optimization/octree/basicoctree.h"

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
    void updateBetaValuesWithPropagation();

    void createMeshForNode(GLint index);

    QVector<octree::cubeObject>* getInnerCubes();
    QVector<octree::cubeObject>* getCubesOfLowerDepth(int depth);

    bool splitAndMerge(GLfloat epsilon);

    void split(octree::octreeNode* nodePointer, GLint maxDepht);
    void split(GLint nodeIndex, GLint maxDepht);

    void increaseShell(GLint loopAmount);

    bool allChildrenAreLeaves(GLint nodeIndex);

    void setVoids();
    void setMergeNodes();

    void deleteNodeMeshes();

private:

    QVector<octree::cubeObject> cubeVector;

private:


    void handleShellNeighbor(GLint x, GLint y, GLint z, QVector<GLint>* backVec);
    void setMergeChild(GLint index, GLfloat beta);
    void addCubeMesh(GLint index,QVector<GLfloat>* geometry, QVector<GLint>* indices, GLint offset);

    void propagateBeta(GLint index, GLfloat beta);

};

#endif // EXTENDEDOCTREE_H
