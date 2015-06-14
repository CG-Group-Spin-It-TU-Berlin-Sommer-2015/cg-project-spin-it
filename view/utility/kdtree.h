#ifndef KDTREE_H
#define KDTREE_H

#include <QtOpenGL>
#include "mesh.h"

struct kdNode {
  GLint index;
  GLint leftChildIndex;
  GLint rightChildIndex;
  GLint depth;
} ;

class KDTree
{
public:
    KDTree();

    void setKDTree();

    GLint getNearestNeighborHelper(GLint nodeIndex, float x, float y, float z);
    GLint getNearestNeighbor(float x, float y, float z);

private:

    GLint setKDTreeHelper(GLint start, GLint end, GLint searchIndex, QVector<GLint>* searchIndices, QVector<kdNode>* nodes, GLint depth);

    GLfloat getDistanceToPlane(GLint axisIndex, GLint index, GLfloat x, GLfloat y, GLfloat z);
    GLfloat getLength(GLint index, GLfloat x, GLfloat y, GLfloat z);

    QVector<kdNode>* nodes;
    QVector<GLint>* searchIndices;
    QVector<GLfloat>* searchBorders;

    Mesh* mesh;
};

#endif // KDTREE_H
