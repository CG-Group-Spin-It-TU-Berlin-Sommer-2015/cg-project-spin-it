#ifndef Octree_H
#define Octree_H

#include <QtOpenGL>
#include <QGLFunctions>
#include <QOpenGLBuffer>
#include <QVector3D>
#include <QMatrix4x4>
#include "mesh.h"

struct hashItem{
    QVector3D vertex;
    GLint index;
};

struct octreeNode {

  GLint index;
  GLint interiorIndex;

  GLint nodeDepth;

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

  /* index of the parent (if negative there is no parent => root node) */
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

  /* intersects the surface */
  bool intersect;

  /* is interior */
  bool interior;

  /* needed for internal calculation of the octree */
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

    /**
     * @brief setOctreeInteriors set octree interior at provided start point
     * @param x x-coordinate of start point
     * @param y y- ...
     * @param z z- ...
     * @param maxDepth maximum depth for octree construction
     * @param outerMesh mesh which is needed for octree size
     * @param innerMesh mesh which is needed for intersection test
     */
    void setOctreeInteriors(
            GLfloat x,
            GLfloat y,
            GLfloat z,
            GLint maxDepth,
            Mesh* outerMesh,
            Mesh* innerMesh);

    /**
     * @brief setOctreeInteriors set octree interior at point (0,0,0)
     * @param maxDepth maximum depth for octree construction
     * @param outerMesh mesh which is needed for octree size
     * @param innerMesh mesh which is needed for intersection test
     */
    void setOctreeInteriors(GLint maxDepth,Mesh* outerMesh,Mesh* innerMesh);

    /**
     * @brief merge The parent node (no leaf) become a leaf (Assumption: All children are leaves!)
     * @param parent the parent octreeNode
     */
    void merge(octreeNode* parent);

    /**
     * @brief merge The parent node (no leaf) become a leaf (Assumption: All children are leaves!)
     * @param parentIndex the index of the octree node which will be merged
     */
    void merge(GLint parentIndex);

    /**
     * @brief split The node is split (Assumption: Node is a leaf.)
     * @param node the octree node which will be split
     */
    void split(octreeNode* node);

    /**
     * @brief split The node is split (Assumption: Node is a leaf!)
     * @param nodeIndex the index of the octree node which will be split
     */
    void split(GLint nodeIndex);

    /**
     * @brief render draw the octree
     * @param shader the shader
     */
    void render(QGLShaderProgram* shader);

    /**
     * @brief setVoidSurface set surface of the voids
     */
    void setVoidSurface();

    /**
     * @brief getRootID get the index of the root node
     * @return the index of the root node
     */
    GLint getRootID();

    /**
     * @brief getAllInnerNodeIDsOfDepth all indices of nodes will be added to the vector
     * which are at the provided depth, and intersect or are an interiors
     * @param vec container for collected indices
     * @param depth searching depth
     */
    void getAllInnerNodeIDsOfDepth(QVector<GLint>* vec,GLint depth);

    /**
     * @brief getAllInnerNodeLeafIDsOfNode get all indices of the child leaves for the provided node
     * @param vec container for collected indices
     * @param nodeIndex index of the provided node
     */
    void getAllInnerNodeLeafIDsOfNode(QVector<GLint>* vec,GLint nodeIndex);

    /**
     * @brief getAllInnerNodeLeafIDs get all indices of the child leaves for the root node
     * @param vec container for collected indices
     */
    void getAllInnerNodeLeafIDs(QVector<GLint>* vec);

    /**
     * @brief allChildrenAreLeaves are all children of the provided node leaves
     * @param nodeIndex index of the provided node
     * @return true if all children of the provided nodes are leaves else false
     */
    bool allChildrenAreLeaves(GLint nodeIndex);

    /**
     * @brief isLeaf is the provided node a leaf
     * @param nodeIndex index of the provided node
     * @return true if the provided node is a leaf
     */
    bool isLeaf(GLint nodeIndex);

    /**
     * @brief getOctreeNode get the pointer to the octree node for the provided index
     * @param index the index for which the pointer is returned
     * @return pointer to the octree node for the provided index
     */
    octreeNode* getOctreeNode(GLint index);

private:

    void getAllInnerNodeIDsOfDepthHelper(QVector<GLint>* vec,GLint depth,GLint nodeIndex);

    QVector<octreeNode>* allNodes;
    QVector<GLint>* interiorLeafIndices;

    QVector<GLint>* leafVector;

    Mesh* outerMesh;
    Mesh* innerMesh;
    GLint maxDepth;

    GLint rootIndex;

    /* */
    bool isDirty;
    QOpenGLBuffer* vbo;
    QOpenGLBuffer* ibo;
    QVector<int> cubeLineIndices;

    /* */
    QMatrix4x4 rotx;
    QMatrix4x4 roty;
    QVector3D xvec;
    QVector3D yvec;
    QVector3D zvec;

    /* */
    GLint handleHashItem(GLint vertexIndex,QVector3D point);

    void addSurface(
            octreeNode inner,
            GLint code,
            GLint i,
            GLint j,
            GLint k,
            GLint axisSize,
            GLint planeSize);

    QVector<GLfloat>* geometry;
    QHash<GLint, hashItem>* geometryMap;
    QVector<GLshort>* indices;

    /* */
    void setLeafVectorHelper(
            GLint maxDepth,
            GLint currentDepth,
            GLint index, GLint x,
            GLint y,
            GLint z);

    void setLeafVector(GLint maxDepth);
    void setBorder(GLint maxDepth);
    void traverseLeafVector(GLint maxDepth);

    /*
     * vector which contains the indices of voids in the vector allNodes
     * purpose: reduce the amount of storage if many nodes are deleted
     */
    QVector<GLint>* freeIndicesBufferForAllNodes;
    QVector<GLint>* freeIndicesBufferForInteriorLeafIndices;

    /* */
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

    bool lineIntersectsSquare(
            GLfloat halfLength,
            QVector3D p1,
            QVector3D p2);

    bool triangleIntersectsPlane(
            GLfloat halfLength,
            QVector3D p,
            QVector3D n0,
            QVector3D n1);

    bool triangleIntersectsCube(
            QVector3D q,
            GLfloat halfLength,
            QVector3D p,
            QVector3D n0,
            QVector3D n1);

    GLint createNode(
            GLfloat length,
            GLfloat x,
            GLfloat y,
            GLfloat z,
            GLint nodeDepth,
            bool addToInteriors);

};

#endif // Octree_H
