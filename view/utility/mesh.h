#ifndef MESH_H
#define MESH_H

#include <iostream>

#include <QtOpenGL>
#include <QGLFunctions>
#include <QOpenGLBuffer>


struct kdNode {
  GLint index;
  GLint leftChildIndex;
  GLint rightChildIndex;
  GLint depth;
} ;

struct octreeNode {

  GLfloat x;
  GLfloat y;
  GLfloat z;

  GLint childIndex0;
  GLint childIndex1;
  GLint childIndex2;
  GLint childIndex3;

  GLint childIndex4;
  GLint childIndex5;
  GLint childIndex6;
  GLint childIndex7;

  GLint depth;
  bool cut;
} ;

struct triObject {
  GLint index;

  GLfloat x;
  GLfloat y;
  GLfloat z;
  GLfloat halflength;
} ;

class Mesh
{
private:
    bool isDirty;

    QOpenGLBuffer* vbo;
    QOpenGLBuffer* ibo;

    QVector<GLfloat>* geometry;
    QVector<GLfloat>* normals;
    QVector<GLshort>* indices;

    QVector<GLint>* searchIndices;
    QVector<GLfloat>* searchBorders;

    QVector<kdNode>* nodes;
    QVector<octreeNode>* interiorNodes;
    GLfloat length;

    GLfloat getDistanceToPlane(GLint axisIndex, GLint index, GLfloat x, GLfloat y, GLfloat z);
    GLfloat getLength(GLint index, GLfloat x, GLfloat y, GLfloat z);

    GLint setKDTreeHelper(GLint start, GLint end, GLint searchIndex, QVector<GLint>* searchIndices, QVector<kdNode>* nodes, GLint depth);
    GLint setOctreeInteriorsHelper(GLfloat length,GLfloat x,GLfloat y,GLfloat z,QVector<octreeNode>* nodes,QVector<triObject>* triObjects,QVector<GLint>* triObjectIndices, GLint depth, GLint maxDepth);
    void testIntersection(QVector<triObject>* triObjects,QVector<GLint>* newTriObjectIndices,QVector<GLint>* oldTriObjectIndices,GLfloat x,GLfloat y,GLfloat z,GLfloat length);
    bool testIntersection2(QVector<triObject>* triObjects,QVector<GLint>* triObjectIndices,GLfloat x,GLfloat y,GLfloat z,GLfloat length);

public:
    Mesh(QVector<GLfloat>* geometry, QVector<GLshort>* indices);
    Mesh(QVector<GLfloat>* geometry, QVector<GLfloat>* normals, QVector<GLshort>* indices);
    ~Mesh();
    QVector<GLfloat>* getGeometry();
    QVector<GLfloat>* getNormals();
    QVector<GLshort>* getIndices();


    void render(QGLShaderProgram* shader, GLenum primitive);

    void setKDTree();
    void setOctreeInteriors(GLfloat x,GLfloat y,GLfloat z, GLint maxDepth);

    GLint getNearestNeighborHelper1(GLint nodeIndex, float x, float y, float z);
    GLint getNearestNeighborHelper2(GLint nodeIndex, float x, float y, float z);
    GLint getNearestNeighbor(float x, float y, float z);

    QVector3D getMean();
};

#endif // MESH_H
