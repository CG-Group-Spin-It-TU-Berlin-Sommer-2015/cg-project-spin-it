#ifndef OCTREE2_H
#define OCTREE2_H

#include <QtOpenGL>
#include <QGLFunctions>
#include <QOpenGLBuffer>

#include <QVector>
#include <QVector3D>

#include "mesh.h"

/*
#include <pcl/point_cloud.h>

#include <Eigen/StdVector>
#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>
#include <pcl/octree/octree_base.h>
#include <pcl/octree/octree.h>
*/

struct octreeNode2 {

  octreeNode2()
  {

      childIndex0 = -1;
      childIndex1 = -1;
      childIndex2 = -1;
      childIndex3 = -1;
      childIndex4 = -1;
      childIndex5 = -1;
      childIndex6 = -1;
      childIndex7 = -1;

      parentIndex = -1;

      typeVectorIndex = -1;

      isSet = false;
      invalid = false;
  }

  GLint index;
  GLint typeVectorIndex;

  GLint x;
  GLint y;
  GLint z;
  GLint cell_length;

  GLint nodeDepth;

  /* indices of the children according to vector 'interiorNodes' */
  GLint childIndex0;
  GLint childIndex1;
  GLint childIndex2;
  GLint childIndex3;

  GLint childIndex4;
  GLint childIndex5;
  GLint childIndex6;
  GLint childIndex7;

  /* index of the parent (if negative there is no parent => root node) */
  GLint parentIndex;

  bool shellNode;
  bool leaf;

  QVector3D p0; //(-x,-y,-z)
  QVector3D p1; //(-x,-y,+z)
  QVector3D p2; //(-x,+y,-z)
  QVector3D p3; //(-x,+y,+z)

  QVector3D p4; //(+x,-y,-z)
  QVector3D p5; //(+x,-y,+z)
  QVector3D p6; //(+x,+y,-z)
  QVector3D p7; //(+x,+y,+z)

  bool isSet;
  bool inside;

  bool invalid;

} ;

struct hashItem2{
    QVector3D vertex;
    GLint index;
};

class Octree2
{
public:
    Octree2();

    bool hasMesh();
    bool hasQantizedSurface();

    void setStartDepth(GLint depth);
    void setMaxDepth(GLint depth);

    void adjustMaxDepth();

    void setMesh(Mesh* mesh);
    void quantizeSurface();
    void setupVectors();

    //void setupOctree_pcl();

    void setupOctree();

    void setOuterNodes();
    void setInnerNodes();

    void render(QGLShaderProgram* shader);

    octreeNode2* getLeafNodeByCoordinate(GLint x, GLint y, GLint z);
    octreeNode2* getLeafNodeByCoordinate(GLint x, GLint y, GLint z, GLint startNodeIndex);

    void split(octreeNode2* nodePointer);
    void split(GLint nodeIndex);
    void merge(octreeNode2* nodePointer);
    void merge(GLint nodeIndex);

    void increaseShell(GLint loopAmount);
    void createInnerSurface();
    Mesh* getMesh();
    Mesh* getPointMesh();  

    void setInnerNodeIndices();
    void setShellNodeIndices();

private:

    bool allChildrenAreLeaves(GLint nodeIndex);
    GLint createNode(GLint x,GLint y,GLint z,GLint depth,bool addToInteriors,GLint parentIndex);

    bool addTriangle(GLint x, GLint y, GLint z);
    void createTriangle(GLint x, GLint y, GLint z, GLint code);

    QVector<GLint> innerNodeIndices;
    QVector<GLint> shellNodeIndices;

    QVector<GLfloat> geometry;
    QHash<GLint, hashItem2> geometryMap;
    QVector<GLint> indices;

    octreeNode2* getLeafNodeByCoordinateHelper(GLint x, GLint y, GLint z, GLint startNodeIndex);

    GLint setupOctreeHelper(GLint depth,GLint start, GLint end, GLint x, GLint y, GLint z);

    void addTriangle(QVector3D* p1,QVector3D* p2,QVector3D* p3,QVector<GLfloat>* buffer);

    bool testNeighborNode(GLint x, GLint y, GLint z, octreeNode2* nodePointer);

    GLint handleHashItem(GLint vertexIndex,QVector3D point);

    void handleShellNeighbor(GLint x, GLint y, GLint z, QVector<GLint>* backVec);

    QVector<GLfloat> raw_voxels;
    QVector<QVector3D> voxels;
    QVector<octreeNode2> octreeNodes;
    Mesh* mesh;
    GLint startDepth;
    GLint maxDepth;
    GLint rootNodeIndex;

    QVector3D mean;
    GLfloat cell_length;
    GLint axis_length;
    GLint plane_length;
    GLdouble max_g;

    QVector<GLint> freeIndicesForAllNodes;
    QVector<GLint> freeIndicesForInnerNodes;

    GLint sortHalf(GLint start,GLint end,GLint coor, GLint prior);

    /* */
    bool isDirty;
    QOpenGLBuffer* vbo;
    QOpenGLBuffer* ibo;
    QVector<int> cubeLineIndices;

};

#endif // OCTREE2_H
