#ifndef OCTREE_H
#define OCTREE_H

#include <QtOpenGL>
#include <QGLFunctions>
#include <QOpenGLBuffer>

#include <QVector>
#include <QVector3D>

#include "mesh/mesh.h"

#define START_DENSITY 0.9f

#define IS_SHELL_NODE (nodePointer->isShell)
#define IS_INNER_NODE (!nodePointer->isShell && nodePointer->isInside)
#define IS_OUTER_NODE (!nodePointer->isShell && !nodePointer->isInside)

namespace octree
{

/**
 * @brief The octreeNode struct
 */
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

      isSet = false;
      isInside = false;
      isShell = false;
      isLeaf = false;
      isVoid = true;

      isMergeRoot = false;
      isMergeChild = false;
      isIgnored = false;

      beta = START_DENSITY;

      mesh = NULL;

  }

  /**
   * @brief setPoints
   * @param x
   * @param y
   * @param z
   * @param cell_length
   */
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

  bool isSet;
  bool isInside;
  bool isShell;
  bool isLeaf;
  bool isMergeRoot;
  bool isMergeChild;
  bool isIgnored;
  bool isVoid;

  QVector3D p0; //(-x,-y,-z)
  QVector3D p1; //(-x,-y,+z)
  QVector3D p2; //(-x,+y,-z)
  QVector3D p3; //(-x,+y,+z)

  QVector3D p4; //(+x,-y,-z)
  QVector3D p5; //(+x,-y,+z)
  QVector3D p6; //(+x,+y,-z)
  QVector3D p7; //(+x,+y,+z)

  float beta;

  Mesh* mesh;
} ;

/**
 * @brief The hashItem struct
 */
struct hashItem{
    QVector3D vertex;
    GLint index;
};

}

class BasicOctree
{  
public:

    BasicOctree();
    ~BasicOctree();

    // setters and getters
    void setMesh(Mesh* mesh);

    void setStartMaxDepth(GLint depth);
    void setOptimizationMaxDepth(GLint depth);

    GLint getStartMaxDepth();
    GLint getOptimizationMaxDepth();

    // methods for creating the octree
    void quantizeSurface();
    void setupVectors();
    void setupOctree();
    void setupOuterNodes();
    void setupInnerNodes();

    // check out methods
    bool hasMesh();
    bool hasQantizedSurface();

    void adjustToBasicMaxDepth();
    void initiateMergeRoots();

    octree::octreeNode* getLeafNodeByCoordinate(GLint x, GLint y, GLint z);
    octree::octreeNode* getLeafNodeByCoordinate(GLint x, GLint y, GLint z, GLint startNodeIndex);

    void createInnerSurface();
    Mesh* getShellMesh(bool flip = false);

    void printNumberOfLeafTypes();

private:

    Mesh* mesh;

    QVector<GLfloat> geometry;
    QVector<GLint> indices;
    QHash<GLint, octree::hashItem> geometryMap;

    QVector<GLfloat> rawVoxels;
    QVector<QVector3D> voxels;

private:

    void setRawVoxel(GLfloat x,GLfloat y,GLfloat z);

    bool addTriangle(GLint x, GLint y, GLint z);

    void createTriangle(GLint x, GLint y, GLint z, GLint code);

    octree::octreeNode* getLeafNodeByCoordinateHelper(GLint x, GLint y, GLint z, GLint startNodeIndex);

    GLint setupOctreeHelper(GLint depth,GLint start, GLint end, GLint x, GLint y, GLint z);

    void addTriangle(QVector3D* p1,QVector3D* p2,QVector3D* p3,QVector<GLfloat>* buffer);

    GLint handleHashItem(GLint vertexIndex,QVector3D point);

    GLint sortHalf(GLint start,GLint end,GLint coor, GLint prior);

    bool mergeChildExists(GLint index);

protected:

    QVector<octree::octreeNode> octreeNodes;

    GLint startMaxDepth;
    GLint optimizationMaxDepth;
    GLint basicMaxDepth;

    GLint rootNodeIndex;

    QVector3D mean;
    GLdouble max_g;

    GLfloat cell_length;
    GLint axis_length;
    GLint plane_length;


protected:

    GLint createNode(
            GLint x,
            GLint y,
            GLint z,
            GLint depth,
            bool isInside,
            GLint parentIndex,
            QVector3D p0,
            QVector3D p1,
            QVector3D p2,
            QVector3D p3,
            QVector3D p4,
            QVector3D p5,
            QVector3D p6,
            QVector3D p7);

    void getInnerLeaves(QVector<GLint>* indices);
    void getShellLeaves(QVector<GLint>* indices);
    void getOuterLeaves(QVector<GLint>* indices);

    void getMergeRoots(QVector<GLint>* indices);
    void getMergeChilds(QVector<GLint>* indices);
    void getIgnoredNodes(QVector<GLint>*indices);

    void getMergeRootCandidates(QVector<GLint>* indices);

    void getNodesOfDepth(GLint depth,QVector<GLint>* indices);
    void getInnerLeavesForNode(GLint index,QVector<GLint>* indices);

};

#endif // OCTREE_H
