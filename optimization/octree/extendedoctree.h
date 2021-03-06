#ifndef EXTENDEDOCTREE_H
#define EXTENDEDOCTREE_H

#include "optimization/octree/basicoctree.h"
#include "eigen3/Eigen/Sparse"

typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> T;

const int GEOMETRY_DATA_SIZE = 3;

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
    ~ExtendedOctree();

    void updateBetaValues();
    void updateBetaValuesWithPropagation();

    void createMeshForNode(GLint index);

    QVector<octree::cubeObject>* getInnerCubes();
    QVector<octree::cubeObject>* getMergedCubes();
    QVector<octree::cubeObject>* getCubesOfLowerDepth(int depth);

    bool splitAndMerge(GLfloat epsilon,GLint depth);
    bool splitStep(GLfloat epsilon,GLint depth);
    void mergeStep(GLfloat epsilon,GLint depth);

    void split(octree::octreeNode* nodePointer, GLint maxDepht);
    void split(GLint nodeIndex, GLint maxDepht);

    void increaseShell(GLint loopAmount);

    bool allChildrenAreLeaves(GLint nodeIndex);

    void setVoids();

    void deleteNodeMeshes();

    // methods for visualization
    void renderBetaGrid(QGLShaderProgram* shader);
    void makeDirty();

    SpMat getUniformLaplace(QVector<octree::cubeObject>* cubes);

    GLint getMergedCubesNumber();

    void transformOctree(QMatrix4x4 mat);

    void resetBetaOctreeState();

private:

    QVector<octree::cubeObject> cubeVector;

    // for rendering octree
    bool isDirty;
    QOpenGLBuffer* vbo;
    QOpenGLBuffer* cbo;
    QOpenGLBuffer* ibo;
    QOpenGLBuffer* vbo_line;
    QOpenGLBuffer* ibo_line;
    QVector<int> indexBuffer;
    QVector<int> indexBuffer_line;

    bool deactivateBetaGrid;

private:

    GLint getNeighborhoodValue(GLint index1, GLint index2);
    GLint getSurfaceIndex(GLint index1, GLint index2);

    void handleShellNeighbor(GLint x, GLint y, GLint z, QVector<GLint>* backVec);
    void setMergeChild(GLint index, GLfloat beta);
    void addCubeMesh(GLint index,QVector<GLfloat>* geometry, QVector<GLint>* indices, GLint offset);

    void propagateBeta(GLint index, GLfloat beta);

    void getInnerLeavesForNodeForSurface(GLint index,QVector<GLint>* indices,GLint surfaceIndex);

};

#endif // EXTENDEDOCTREE_H
