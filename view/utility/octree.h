#ifndef Octree_H
#define Octree_H

#include <QtOpenGL>
#include <QGLFunctions>
#include <QOpenGLBuffer>
#include <QVector3D>
#include <QMatrix4x4>
#include "mesh.h"

struct octreeNode {

  GLint index;

  bool leaf;

  /* indices of the children according to vector 'interiorNodes' */
  GLint childIndex0;
  GLint childIndex1;
  GLint childIndex2;
  GLint childIndex3;

  GLint childIndex4;
  GLint childIndex5;
  GLint childIndex6;
  GLint childIndex7;

  GLint parentIndex;

  /* positions of the corners (comments is position compared to the middle point*/

  QVector3D p0; //(-x,-y,-z)
  QVector3D p1; //(-x,-y,+z)
  QVector3D p2; //(-x,+y,-z)
  QVector3D p3; //(-x,+y,+z)

  QVector3D p4; //(+x,-y,-z)
  QVector3D p5; //(+x,-y,+z)
  QVector3D p6; //(+x,+y,-z)
  QVector3D p7; //(+x,+y,+z)

  bool cut;
  bool interior;
  bool set;
} ;

struct triObject {
  GLint index;

  QVector3D p;
  GLfloat halflength;
} ;

class Octree
{
public:
    Octree();

    void setOctreeInteriors(GLfloat x,GLfloat y,GLfloat z, GLint maxDepth,Mesh* m);
    void setOctreeInteriors(GLint maxDepth,Mesh* m);

    octreeNode* getOctreeRoot();

    QVector<octreeNode>* allNodes;
    QVector<GLint>* interiorNodesIndices;
    QVector<GLint>* leafVector;

    Mesh* mesh;

    bool test();
    bool test_tc();
    bool test_tp();
    bool test_ls();

    void merge(octreeNode* parent);
    void split(octreeNode* node);

    void render(QGLShaderProgram* shader);

private:

    bool isDirty;

    QOpenGLBuffer* vbo;
    QOpenGLBuffer* ibo;

    QMatrix4x4 rotx;
    QMatrix4x4 roty;

    QVector3D xvec;
    QVector3D yvec;
    QVector3D zvec;

    QVector<GLint>* freeIndices;

    void setLeafVectorHelper(GLint maxDepth, GLint currentDepth, GLint index, GLint x, GLint y, GLint z);
    void setLeafVector(GLint maxDepth);
    void setBorder(GLint maxDepth);
    void traverseLeafVector(GLint maxDepth);

    GLint setOctreeInteriorsHelper(
            GLfloat length,
            GLfloat x,
            GLfloat y,
            GLfloat z,
            QVector<triObject>* triObjects,
            QVector<GLint>* triObjectIndices,
            GLint depth,
            GLint maxDepth);

    void testIntersection(
            QVector<triObject>* triObjects,
            QVector<GLint>* newTriObjectIndices,
            QVector<GLint>* oldTriObjectIndices,
            GLfloat x,
            GLfloat y,
            GLfloat z,
            GLfloat length);

    bool testIntersection2(
            QVector<triObject>* triObjects,
            QVector<GLint>* triObjectIndices,
            GLfloat x,
            GLfloat y,
            GLfloat z,
            GLfloat length);

    bool lineCutsSquare(GLfloat halfLength, QVector3D p1, QVector3D p2);
    bool triangleCutsPlane(GLfloat halfLength, QVector3D p, QVector3D n0, QVector3D n1);
    bool triangleCutsCube(QVector3D q,GLfloat halfLength, QVector3D p, QVector3D n0, QVector3D n1);

    GLint createNode(GLfloat length, GLfloat x, GLfloat y, GLfloat z);

    QVector<int> cubeLineIndices;

};

#endif // Octree_H
