#ifndef OCTREE_H
#define OCTREE_H

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

namespace octree
{

struct octreeNode {

  octreeNode()
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

      // random float between 0 and 1
      beta = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

      isVoid = true;

  }

  void setPoints(GLfloat x,GLfloat y,GLfloat z,GLfloat cell_length)
  {

      p0.setX(x);               p0.setY(y);               p0.setZ(z);
      p1.setX(x);               p1.setY(y);               p1.setZ(z+cell_length);
      p2.setX(x);               p2.setY(y+cell_length);   p2.setZ(z);
      p3.setX(x);               p3.setY(y+cell_length);   p3.setZ(z+cell_length);
      p4.setX(x+cell_length);   p4.setY(y);               p4.setZ(z);
      p5.setX(x+cell_length);   p5.setY(y);               p5.setZ(z+cell_length);
      p6.setX(x+cell_length);   p6.setY(y+cell_length);   p6.setZ(z);
      p7.setX(x+cell_length);   p7.setY(y+cell_length);   p7.setZ(z+cell_length);

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

  float beta;

  bool isVoid;
} ;

struct hashItem{
    QVector3D vertex;
    GLint index;
};

}

class BasicOctree
{
public:
    BasicOctree();

    void setMesh(Mesh* mesh);
    bool hasMesh();

    void setStartDepth(GLint depth);
    void setMaxDepth(GLint depth);

    void adjustMaxDepth();

    void quantizeSurface();
    bool hasQantizedSurface();

    void setupVectors();

    //void setupOctree_pcl();

    void setupOctree();

    void setOuterNodes();
    void setInnerNodes();

    void render(QGLShaderProgram* shader);

    octree::octreeNode* getLeafNodeByCoordinate(GLint x, GLint y, GLint z);
    octree::octreeNode* getLeafNodeByCoordinate(GLint x, GLint y, GLint z, GLint startNodeIndex);

    void createInnerSurface();

    Mesh* getMesh(bool flip = false);
    Mesh* getPointMesh();

    void setInnerNodeIndices();
    void setShellNodeIndices();

private:

    bool addTriangle(GLint x, GLint y, GLint z);
    void createTriangle(GLint x, GLint y, GLint z, GLint code);

    QVector<GLfloat> geometry;
    QVector<GLint> indices;
    QHash<GLint, octree::hashItem> geometryMap;

    octree::octreeNode* getLeafNodeByCoordinateHelper(GLint x, GLint y, GLint z, GLint startNodeIndex);

    GLint setupOctreeHelper(GLint depth,GLint start, GLint end, GLint x, GLint y, GLint z);

    void addTriangle(QVector3D* p1,QVector3D* p2,QVector3D* p3,QVector<GLfloat>* buffer);

    bool testNeighborNode(GLint x, GLint y, GLint z, octree::octreeNode* nodePointer);

    GLint handleHashItem(GLint vertexIndex,QVector3D point);

    GLint sortHalf(GLint start,GLint end,GLint coor, GLint prior);

    bool isDirty;
    QOpenGLBuffer* vbo;
    QOpenGLBuffer* ibo;
    QVector<int> cubeLineIndices;

    QVector<GLfloat> raw_voxels;
    QVector<QVector3D> voxels;
    Mesh* mesh;

protected:

    QVector<octree::octreeNode> octreeNodes;

    GLint createNode(GLint x,GLint y,GLint z,GLint depth,bool addToInteriors,GLint parentIndex);

    QVector<GLint> innerNodeIndices;
    QVector<GLint> shellNodeIndices;

    QVector<GLint> freeIndicesForAllNodes;
    QVector<GLint> freeIndicesForInnerNodes;

    GLint startDepth;
    GLint maxDepth;
    GLint rootNodeIndex;

    QVector3D mean;
    GLfloat cell_length;
    GLint axis_length;
    GLint plane_length;
    GLdouble max_g;

};

#endif // OCTREE_H
