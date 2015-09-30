#include "extendedoctree.h"

using namespace std;
using namespace octree;
using namespace Eigen;


ExtendedOctree::ExtendedOctree():
isDirty(true),
vbo(NULL),
cbo(NULL),
ibo(NULL),
vbo_line(NULL),
ibo_line(NULL)
{

}

ExtendedOctree::~ExtendedOctree()
{

    this->indexBuffer.clear();
    this->indexBuffer_line.clear();

    if(vbo != NULL)
    {
        vbo->destroy();
    }

    if(cbo != NULL)
    {
        cbo->destroy();
    }

    if(ibo != NULL)
    {
        ibo->destroy();
    }

    if(vbo_line != NULL)
    {
        vbo_line->destroy();
    }

    if(ibo_line != NULL)
    {
        ibo_line->destroy();
    }

}

/**
 * @brief ExtendedOctree::updateBetas Update all beta values of the inner leaf nodes with the values of the cube vector.
 */
void ExtendedOctree::updateBetaValues()
{

    // iterate about all cubes
    for(int i=0;i<cubeVector.size();i++)
    {
        cubeObject* obj = &cubeVector.data()[i];
        (&octreeNodes.data()[obj->index])->beta = obj->beta;
    }

}

#define CASE1(v1,v1_,v2,v2_) ((v1<=v2 && v2_<=v1_)||(v2<=v1 && v1_<=v2_))
#define CASE2(v1,v1_,v2,v2_) ((v1<=v2 && v2<v1_ && v1_<=v2_)||(v2<=v1 && v1<v2_ && v2_<=v1_))

#define MAX(v1,v2) (v1>v2?v1:v2);
#define MIN(v1,v2) (v1<v2?v1:v2);

/**
 * @brief ExtendedOctree::getNeighborhoodValue
 * @param index1
 * @param index2
 * @return
 */
GLint ExtendedOctree::getNeighborhoodValue(GLint index1, GLint index2)
{

    octreeNode *nodePointer1,*nodePointer2;

    nodePointer1 = &this->octreeNodes.data()[index1];
    nodePointer2 = &this->octreeNodes.data()[index2];

    GLint len1 = nodePointer1->cell_length;
    GLint len2 = nodePointer2->cell_length;

    GLint x1 = nodePointer1->x;
    GLint y1 = nodePointer1->y;
    GLint z1 = nodePointer1->z;
    GLint x1_ = x1+len1-1;
    GLint y1_ = y1+len1-1;
    GLint z1_ = z1+len1-1;

    GLint x2 = nodePointer2->x;
    GLint y2 = nodePointer2->y;
    GLint z2 = nodePointer2->z;
    GLint x2_ = x2+len2-1;
    GLint y2_ = y2+len2-1;
    GLint z2_ = z2+len2-1;

    GLint vx,vy,vz;

    if(CASE1(x1,x1_,x2,x2_))
    {
       vx = MIN(len1,len2);
    }
    else
    if(CASE2(x1,x1_,x2,x2_))
    {
        vx= MAX(x1_-x2,x2_-x1);
        vx++;
    }
    else
    {
        vx= -MAX(x1-x2_,x2-x1_);
        vx++;
    }

    if(CASE1(y1,y1_,y2,y2_))
    {
       vy = MIN(len1,len2);
    }
    else
    if(CASE2(y1,y1_,y2,y2_))
    {
        vy= MAX(y1_-y2,y2_-y1);
        vy++;
    }
    else
    {
        vy= -MAX(y1-y2_,y2-y1_)
        vy++;
    }

    if(CASE1(z1,z1_,z2,z2_))
    {
       vz = MIN(len1,len2);
    }
    else
    if(CASE2(z1,z1_,z2,z2_))
    {
        vz= MAX(z1_-z2,z2_-z1);
        vz++;
    }
    else
    {
        vz= -MAX(z1-z2_,z2-z1_);
        vz++;
    }

    GLint value = -1;

    value = vx>0&&vy>0&&vz==0?vx*vx:value;
    value = vx>0&&vy==0&&vz>0?vx*vx:value;
    value = vx==0&&vy>0&&vz>0?vz*vz:value;

    return value;
}

#define TOP_SURFACE 0
#define BOTTOM_SURFACE 1
#define LEFT_SURFACE 2
#define RIGHT_SURFACE 3
#define FRONT_SURFACE 4
#define BACK_SURFACE 5

/**
 * @brief ExtendedOctree::getSurfaceIndex
 * @param index1
 * @param index2
 * @return
 */
GLint ExtendedOctree::getSurfaceIndex(GLint index1, GLint index2)
{

    octreeNode *nodePointer1,*nodePointer2;

    nodePointer1 = &this->octreeNodes.data()[index1];
    nodePointer2 = &this->octreeNodes.data()[index2];

    GLint len1 = nodePointer1->cell_length;
    GLint len2 = nodePointer2->cell_length;

    GLint x1 = nodePointer1->x;
    GLint y1 = nodePointer1->y;
    GLint z1 = nodePointer1->z;
    GLint x1_ = x1+len1-1;
    GLint y1_ = y1+len1-1;
    GLint z1_ = z1+len1-1;

    GLint x2 = nodePointer2->x;
    GLint y2 = nodePointer2->y;
    GLint z2 = nodePointer2->z;
    GLint x2_ = x2+len2-1;
    GLint y2_ = y2+len2-1;
    GLint z2_ = z2+len2-1;

    bool isOnXAxis = !CASE1(x1,x1_,x2,x2_) && !CASE2(x1,x1_,x2,x2_);
    bool isOnYAxis = !CASE1(y1,y1_,y2,y2_) && !CASE2(y1,y1_,y2,y2_);
    bool isOnZAxis = !CASE1(z1,z1_,z2,z2_) && !CASE2(z1,z1_,z2,z2_);


    GLint value = TOP_SURFACE;
    value = isOnXAxis?(x1-x2_ < x2-x1_?FRONT_SURFACE:BACK_SURFACE):value;
    value = isOnYAxis?(y1_-y2 < y2_-y1?RIGHT_SURFACE:LEFT_SURFACE):value;
    value = isOnZAxis?(z1-z2_ < z2-z1_?TOP_SURFACE:BOTTOM_SURFACE):value;

    return value;
}

/**
 * @brief ExtendedOctree::getUniformLaplace
 * @param cubes
 * @return
 */
SpMat ExtendedOctree::getUniformLaplace(QVector<cubeObject>* cubes)
{

    SpMat L(cubes->size(),cubes->size());
    std::vector<T> coefficients;
    std::vector<int> diagonalValues;

    QVector<GLint> vec1;
    QVector<GLint> vec2;

    for(int i=0;i<cubes->size();i++)
    {
        diagonalValues.push_back(0);
    }

    for(int i=0;i<cubes->size();i++)
    {

        for(int j=i+1;j<cubes->size();j++)
        {

            if(i==j)
            {
                continue;
            }

            GLint valueMain = getNeighborhoodValue(cubes->data()[i].index,cubes->data()[j].index);

            if(valueMain > 0){

                GLint surfaceIndex1 = getSurfaceIndex(cubes->data()[i].index,cubes->data()[j].index);
                getInnerLeavesForNodeForSurface(cubes->data()[i].index,&vec1,surfaceIndex1);

                GLint surfaceIndex2 = getSurfaceIndex(cubes->data()[j].index,cubes->data()[i].index);
                getInnerLeavesForNodeForSurface(cubes->data()[j].index,&vec2,surfaceIndex2);

                for(int k=0;k<vec1.size();k++)
                {
                    for(int l=0;l<vec2.size();l++)
                    {

                        GLint value = getNeighborhoodValue(vec1.data()[k],vec2.data()[l]);


                        if(value > 0)
                        {
                            diagonalValues.data()[i] +=value;
                            diagonalValues.data()[j] +=value;

                            coefficients.push_back(T(i,j,-value));
                            coefficients.push_back(T(j,i,-value));
                        }

                    }
                }

                vec1.clear();
                vec2.clear();

            }
        }
    }

    double maxValue = -1.0;

    for(int i=0;i<cubes->size();i++)
    {
        coefficients.push_back(T(i,i,diagonalValues.data()[i]));
        if(maxValue<diagonalValues.data()[i])
        {
           maxValue = diagonalValues.data()[i];
        }
    }

    L.setFromTriplets(coefficients.begin(), coefficients.end());
    if(maxValue>0)
    {
        L /= maxValue;
    }

    cout << "----------------------------------------" << endl;
    cout << "finished laplace operator!" << endl;

    return L;
}

/**
 * @brief ExtendedOctree::getInnerLeavesForNodeForSurface
 * @param index
 * @param indices
 * @param surfaceIndex
 */
void ExtendedOctree::getInnerLeavesForNodeForSurface(GLint index,QVector<GLint>* indices,GLint surfaceIndex)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    // break recursion because current node is a leaf
    if(nodePointer->isLeaf)
    {

        if(IS_INNER_NODE)
        {
            indices->push_back(nodePointer->index);
        }

        return;
    }

    // check every child of the current node

    switch(surfaceIndex)
    {
    case TOP_SURFACE:
        getInnerLeavesForNode(nodePointer->childIndex1,indices);
        getInnerLeavesForNode(nodePointer->childIndex3,indices);
        getInnerLeavesForNode(nodePointer->childIndex5,indices);
        getInnerLeavesForNode(nodePointer->childIndex7,indices);
        break;
    case BOTTOM_SURFACE:
        getInnerLeavesForNode(nodePointer->childIndex0,indices);
        getInnerLeavesForNode(nodePointer->childIndex2,indices);
        getInnerLeavesForNode(nodePointer->childIndex4,indices);
        getInnerLeavesForNode(nodePointer->childIndex6,indices);
        break;
    case LEFT_SURFACE:
        getInnerLeavesForNode(nodePointer->childIndex0,indices);
        getInnerLeavesForNode(nodePointer->childIndex1,indices);
        getInnerLeavesForNode(nodePointer->childIndex4,indices);
        getInnerLeavesForNode(nodePointer->childIndex5,indices);
        break;
    case RIGHT_SURFACE:
        getInnerLeavesForNode(nodePointer->childIndex2,indices);
        getInnerLeavesForNode(nodePointer->childIndex3,indices);
        getInnerLeavesForNode(nodePointer->childIndex6,indices);
        getInnerLeavesForNode(nodePointer->childIndex7,indices);
        break;
    case FRONT_SURFACE:
        getInnerLeavesForNode(nodePointer->childIndex4,indices);
        getInnerLeavesForNode(nodePointer->childIndex5,indices);
        getInnerLeavesForNode(nodePointer->childIndex6,indices);
        getInnerLeavesForNode(nodePointer->childIndex7,indices);
        break;
    case BACK_SURFACE:
        getInnerLeavesForNode(nodePointer->childIndex0,indices);
        getInnerLeavesForNode(nodePointer->childIndex1,indices);
        getInnerLeavesForNode(nodePointer->childIndex2,indices);
        getInnerLeavesForNode(nodePointer->childIndex3,indices);
        break;

    }

}

/**
 * @brief ExtendedOctree::propagateBeta
 * @param index
 * @param beta
 */
void ExtendedOctree::propagateBeta(GLint index, GLfloat beta)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    nodePointer->beta = beta;

    if(!nodePointer->isLeaf)
    {
        propagateBeta(nodePointer->childIndex0,beta);
        propagateBeta(nodePointer->childIndex1,beta);
        propagateBeta(nodePointer->childIndex2,beta);
        propagateBeta(nodePointer->childIndex3,beta);

        propagateBeta(nodePointer->childIndex4,beta);
        propagateBeta(nodePointer->childIndex5,beta);
        propagateBeta(nodePointer->childIndex6,beta);
        propagateBeta(nodePointer->childIndex7,beta);

    }

}

/**
 * @brief ExtendedOctree::updateBetaValuesWithPropagation
 */
void ExtendedOctree::updateBetaValuesWithPropagation()
{

    // iterate about all cubes
    for(int i=0;i<cubeVector.size();i++)
    {
        cubeObject* obj = &cubeVector.data()[i];
        (&octreeNodes.data()[obj->index])->beta = obj->beta;

        propagateBeta(obj->index,obj->beta);

    }

}

/**
 * @brief ExtendedOctree::addCubeMesh Add the vertices and indices to the mesh
 * @param index index of the node
 * @param geometry the vector of vertices
 * @param indices the vector of indieces
 * @param offset the offset for the indices
 */
void ExtendedOctree::addCubeMesh(GLint index,QVector<GLfloat>* geometry, QVector<GLint>* indices, GLint offset)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    geometry->push_back(nodePointer->p0.x());
    geometry->push_back(nodePointer->p0.z());
    geometry->push_back(nodePointer->p0.y());

    geometry->push_back(nodePointer->p1.x());
    geometry->push_back(nodePointer->p1.z());
    geometry->push_back(nodePointer->p1.y());

    geometry->push_back(nodePointer->p2.x());
    geometry->push_back(nodePointer->p2.z());
    geometry->push_back(nodePointer->p2.y());

    geometry->push_back(nodePointer->p3.x());
    geometry->push_back(nodePointer->p3.z());
    geometry->push_back(nodePointer->p3.y());

    geometry->push_back(nodePointer->p4.x());
    geometry->push_back(nodePointer->p4.z());
    geometry->push_back(nodePointer->p4.y());

    geometry->push_back(nodePointer->p5.x());
    geometry->push_back(nodePointer->p5.z());
    geometry->push_back(nodePointer->p5.y());

    geometry->push_back(nodePointer->p6.x());
    geometry->push_back(nodePointer->p6.z());
    geometry->push_back(nodePointer->p6.y());

    geometry->push_back(nodePointer->p7.x());
    geometry->push_back(nodePointer->p7.z());
    geometry->push_back(nodePointer->p7.y());

    // set the indices

    // bottom
    indices->push_back(offset+2);
    indices->push_back(offset+1);
    indices->push_back(offset+0);
    indices->push_back(offset+2);
    indices->push_back(offset+3);
    indices->push_back(offset+1);

    // top
    indices->push_back(offset+4);
    indices->push_back(offset+5);
    indices->push_back(offset+6);
    indices->push_back(offset+5);
    indices->push_back(offset+7);
    indices->push_back(offset+6);

    // left
    indices->push_back(offset+1);
    indices->push_back(offset+5);
    indices->push_back(offset+4);
    indices->push_back(offset+1);
    indices->push_back(offset+4);
    indices->push_back(offset+0);

    // right
    indices->push_back(offset+2);
    indices->push_back(offset+6);
    indices->push_back(offset+7);
    indices->push_back(offset+2);
    indices->push_back(offset+7);
    indices->push_back(offset+3);

    // back
    indices->push_back(offset+0);
    indices->push_back(offset+4);
    indices->push_back(offset+6);
    indices->push_back(offset+0);
    indices->push_back(offset+6);
    indices->push_back(offset+2);

    // front
    indices->push_back(offset+3);
    indices->push_back(offset+7);
    indices->push_back(offset+1);
    indices->push_back(offset+7);
    indices->push_back(offset+5);
    indices->push_back(offset+1);

}

/**
 * @brief ExtendedOctree::createMeshForNode
 * @param index
 */
void ExtendedOctree::createMeshForNode(GLint index)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    QVector<GLfloat>* geometry;
    QVector<GLint>* indices;

    QVector<GLint> childIndices;

    if(nodePointer->isLeaf)
    {

        // allocate and reserve storage for the geometry of a cube
        geometry = new QVector<GLfloat>();
        indices = new QVector<GLint>();
        geometry->reserve(8*3);
        indices->reserve(12*3);

        addCubeMesh(nodePointer->index,geometry,indices,0);
        nodePointer->mesh = new Mesh(geometry,indices);

    }
    else
    {
        childIndices.clear();
        this->getInnerLeavesForNode(nodePointer->index,&childIndices);

        geometry = new QVector<GLfloat>();
        indices = new QVector<GLint>();
        geometry->reserve(8*3*childIndices.size());
        indices->reserve(12*3*childIndices.size());

        for(int i=0;i<childIndices.size();i++)
        {
            addCubeMesh(childIndices.data()[i],geometry,indices,8*i);
        }

        nodePointer->mesh = new Mesh(geometry,indices);
    }

    childIndices.clear();

}

/**
 * @brief ExtendedOctree::getInnerCubes Get a pointer of a vector of cubes which represent the geometry of all inner leaf nodes.
 * @return pointer of vector
 */
QVector<cubeObject>* ExtendedOctree::getInnerCubes()
{

    // reset the default state
    cubeVector.clear();

    // get a vector of the indices of all inner nodes
    QVector<GLint> innerIndices;
    this->getInnerLeaves(&innerIndices);

    octreeNode* nodePointer;

    // iterate about indices of all inner nodes and create a cube
    for(int i=0;i<innerIndices.size();i++)
    {

        nodePointer = &this->octreeNodes.data()[innerIndices.data()[i]];

        if(nodePointer->mesh == NULL)
        {
            createMeshForNode(innerIndices.data()[i]);
        }

        if(nodePointer->mesh->getGeometry()->size()<1)
        {
            continue;
        }

        // set the values of the cube
        cubeObject obj;
        obj.mesh = nodePointer->mesh;
        obj.beta = nodePointer->beta;
        obj.index = nodePointer->index;

        // add the cube to the vector
        cubeVector.push_back(obj);
    }

    innerIndices.clear();

    return &cubeVector;
}

/**
 * @brief ExtendedOctree::getMergedCubes
 * @return
 */
GLint ExtendedOctree::getMergedCubesNumber()
{
    QVector<GLint> mergeIndices;
    this->getMergeRoots(&mergeIndices);

    GLint number = mergeIndices.size();

    mergeIndices.clear();

    return number;
}

/**
 * @brief ExtendedOctree::getMergedCubes Get a pointer of a vector of cubes which represent the geometry of all inner leaf nodes.
 * @return pointer of vector
 */
QVector<cubeObject>* ExtendedOctree::getMergedCubes()
{

    // reset the default state
    cubeVector.clear();

    // get a vector of the indices of all inner nodes
    QVector<GLint> mergeIndices;
    this->getMergeRoots(&mergeIndices);

    octreeNode* nodePointer;

    // iterate about indices of all inner nodes and create a cube
    for(int i=0;i<mergeIndices.size();i++)
    {

        nodePointer = &this->octreeNodes.data()[mergeIndices.data()[i]];

        if(nodePointer->mesh == NULL)
        {
            createMeshForNode(mergeIndices.data()[i]);
        }

        if(nodePointer->mesh->getGeometry()->size()<1)
        {
            continue;
        }

        // set the values of the cube
        cubeObject obj;
        obj.mesh = nodePointer->mesh;
        obj.beta = nodePointer->beta;
        obj.index = nodePointer->index;

        // add the cube to the vector
        cubeVector.push_back(obj);
    }

    mergeIndices.clear();

    return &cubeVector;
}

/**
 * @brief ExtendedOctree::getCubesOfLowerDepth
 * @param depth
 * @return
 */
QVector<octree::cubeObject>* ExtendedOctree::getCubesOfLowerDepth(int depth)
{

    // TO DO: test

    // reset the default state
    cubeVector.clear();

    QVector<GLint> nodeIndices;
    octreeNode* nodePointer;

    int currentMaxDepth = depth<this->startMaxDepth?depth:this->startMaxDepth;

    // get inner leaf nodes for depht smaller currentMaxDepth (depth)
    for(int i=0;i<currentMaxDepth;i++)
    {

        getNodesOfDepth(i,&nodeIndices);

        for(int j=0;j<nodeIndices.size();j++)
        {
            nodePointer = &this->octreeNodes.data()[nodeIndices.data()[j]];

            if(nodePointer->isMergeRoot)
            {

                if(nodePointer->mesh == NULL)
                {
                    createMeshForNode(nodeIndices.data()[j]);
                }

                if(nodePointer->mesh->getGeometry()->size()<1)
                {
                    continue;
                }

                // set the values of the cube
                cubeObject obj;
                obj.mesh = nodePointer->mesh;
                obj.beta = nodePointer->beta;
                obj.index = nodePointer->index;

                // add the cube to the vector
                cubeVector.push_back(obj);

            }

        }

        nodeIndices.clear();

    }

    getNodesOfDepth(currentMaxDepth,&nodeIndices);

    for(int j=0;j<nodeIndices.size();j++)
    {
        nodePointer = &this->octreeNodes.data()[nodeIndices.data()[j]];

        if(nodePointer->isMergeRoot || (!nodePointer->isMergeChild && !nodePointer->isIgnored))
        {

            if(nodePointer->mesh == NULL)
            {
                createMeshForNode(nodeIndices.data()[j]);
            }

            if(nodePointer->mesh->getGeometry()->size()<1)
            {
                continue;
            }

            // set the values of the cube
            cubeObject obj;
            obj.mesh = nodePointer->mesh;
            obj.beta = nodePointer->beta;
            obj.index = nodePointer->index;

            // add the cube to the vector
            cubeVector.push_back(obj);

        }

    }

    nodeIndices.clear();

    return &cubeVector;
}

/**
 * @brief ExtendedOctree::splitStep
 * @param epsilon
 * @param depth
 * @return
 */
bool ExtendedOctree::splitStep(GLfloat epsilon,GLint depth)
{

    octreeNode* nodePointer;

    GLfloat oneMinusEpsilon = 1-epsilon;
    bool isSplited = false;

    GLint splitNumber = 0;

    // get a vector of the indices of all inner nodes
    QVector<GLint> innerLeafIndices;
    getInnerLeaves(&innerLeafIndices);

    // split step
    for(int i=0;i<innerLeafIndices.size();i++)
    {

        nodePointer = &this->octreeNodes.data()[innerLeafIndices.data()[i]];

        if(nodePointer->nodeDepth>depth)
        {
            continue;
        }

        if(!nodePointer->isMergeRoot)
        {
            continue;
        }

        // if the beta value is in the set (0,epsilon)^(1-epsilon,1)
        if( epsilon < nodePointer->beta && nodePointer->beta < oneMinusEpsilon )
        {
            this->split(innerLeafIndices.data()[i],this->optimizationMaxDepth);

            nodePointer = &this->octreeNodes.data()[innerLeafIndices.data()[i]];

            // check whether the node has been splitted
            if(!nodePointer->isLeaf)
            {

                // all children get the beta value of their parent
                this->octreeNodes.data()[nodePointer->childIndex0].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex1].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex2].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex3].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex4].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex5].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex6].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex7].beta = nodePointer->beta;

                this->octreeNodes.data()[nodePointer->childIndex0].isMergeRoot = true;
                this->octreeNodes.data()[nodePointer->childIndex1].isMergeRoot = true;
                this->octreeNodes.data()[nodePointer->childIndex2].isMergeRoot = true;
                this->octreeNodes.data()[nodePointer->childIndex3].isMergeRoot = true;
                this->octreeNodes.data()[nodePointer->childIndex4].isMergeRoot = true;
                this->octreeNodes.data()[nodePointer->childIndex5].isMergeRoot = true;
                this->octreeNodes.data()[nodePointer->childIndex6].isMergeRoot = true;
                this->octreeNodes.data()[nodePointer->childIndex7].isMergeRoot = true;

                // reset node
                nodePointer->isMergeRoot = false;
                if(nodePointer->mesh != NULL)
                {
                    delete nodePointer->mesh;
                    nodePointer->mesh = NULL;
                }

                // convergece is not reached
                isSplited = true;

                splitNumber++;
            }
        }
    }

    innerLeafIndices.clear();

    cout << "----------------------------------------" << endl;
    cout << "Number of Splits: " << splitNumber << endl;

    return isSplited;
}

/**
 * @brief ExtendedOctree::mergeStep
 * @param epsilon
 * @param depth
 */
void ExtendedOctree::mergeStep(GLfloat epsilon,GLint depth)
{

    octreeNode* nodePointer;

    GLfloat oneMinusEpsilon = 1-epsilon;
    GLint mergeNumber = 0;
    GLint mergeLoops = -1;

    bool merged = true;

    while(merged)
    {

        merged = false;

        // get a vector of the indices of all merge candidates
        QVector<GLint> mergeCandidates;
        getMergeRootCandidates(&mergeCandidates);

        octreeNode *c0p,*c1p,*c2p,*c3p,*c4p,*c5p,*c6p,*c7p;

        // merge step
        for(int i=0;i<mergeCandidates.size();i++)
        {

            nodePointer = &this->octreeNodes.data()[mergeCandidates.data()[i]];

            if(nodePointer->nodeDepth>depth)
            {
                continue;
            }

            c0p = &this->octreeNodes.data()[nodePointer->childIndex0];
            c1p = &this->octreeNodes.data()[nodePointer->childIndex1];
            c2p = &this->octreeNodes.data()[nodePointer->childIndex2];
            c3p = &this->octreeNodes.data()[nodePointer->childIndex3];
            c4p = &this->octreeNodes.data()[nodePointer->childIndex4];
            c5p = &this->octreeNodes.data()[nodePointer->childIndex5];
            c6p = &this->octreeNodes.data()[nodePointer->childIndex6];
            c7p = &this->octreeNodes.data()[nodePointer->childIndex7];

            bool ignoreC0 = c0p->isIgnored;
            bool ignoreC1 = c1p->isIgnored;
            bool ignoreC2 = c2p->isIgnored;
            bool ignoreC3 = c3p->isIgnored;
            bool ignoreC4 = c4p->isIgnored;
            bool ignoreC5 = c5p->isIgnored;
            bool ignoreC6 = c6p->isIgnored;
            bool ignoreC7 = c7p->isIgnored;


            bool allSmallerEpsilon =
                    (ignoreC0 || (c0p->isMergeRoot && c0p->beta < epsilon)) &&
                    (ignoreC1 || (c1p->isMergeRoot && c1p->beta < epsilon)) &&
                    (ignoreC2 || (c2p->isMergeRoot && c2p->beta < epsilon)) &&
                    (ignoreC3 || (c3p->isMergeRoot && c3p->beta < epsilon)) &&
                    (ignoreC4 || (c4p->isMergeRoot && c4p->beta < epsilon)) &&
                    (ignoreC5 || (c5p->isMergeRoot && c5p->beta < epsilon)) &&
                    (ignoreC6 || (c6p->isMergeRoot && c6p->beta < epsilon)) &&
                    (ignoreC7 || (c7p->isMergeRoot && c7p->beta < epsilon));

            bool allGreaterEpsilon =
                    (ignoreC0 || (c0p->isMergeRoot && c0p->beta > oneMinusEpsilon)) &&
                    (ignoreC1 || (c1p->isMergeRoot && c1p->beta > oneMinusEpsilon)) &&
                    (ignoreC2 || (c2p->isMergeRoot && c2p->beta > oneMinusEpsilon)) &&
                    (ignoreC3 || (c3p->isMergeRoot && c3p->beta > oneMinusEpsilon)) &&
                    (ignoreC4 || (c4p->isMergeRoot && c4p->beta > oneMinusEpsilon)) &&
                    (ignoreC5 || (c5p->isMergeRoot && c5p->beta > oneMinusEpsilon)) &&
                    (ignoreC6 || (c6p->isMergeRoot && c6p->beta > oneMinusEpsilon)) &&
                    (ignoreC7 || (c7p->isMergeRoot && c7p->beta > oneMinusEpsilon));


            if(allSmallerEpsilon || allGreaterEpsilon)
            {
                // merge
                float beta = allGreaterEpsilon?1.f:0.f;


                nodePointer->isIgnored = allSmallerEpsilon && allGreaterEpsilon;
                nodePointer->isMergeRoot = !nodePointer->isIgnored;
                nodePointer->isMergeChild = false;

                nodePointer->beta = beta;

                setMergeChild(c0p->index,beta);
                setMergeChild(c1p->index,beta);
                setMergeChild(c2p->index,beta);
                setMergeChild(c3p->index,beta);
                setMergeChild(c4p->index,beta);
                setMergeChild(c5p->index,beta);
                setMergeChild(c6p->index,beta);
                setMergeChild(c7p->index,beta);

                mergeNumber++;

                merged = true;
            }

        }

        mergeLoops++;

        mergeCandidates.clear();

    }

    cout << "----------------------------------------" << endl;
    cout << "Number of Merges: " << mergeNumber << endl;
    cout << "Number of Merges Loops: " << mergeLoops << endl;

}

/**
 * @brief ExtendedOctree::splitAndMerge Split and merge inner nodes.
 * @param epsilon range for the merge area (0,epsilon) and (1-epsilon,1)
 * @param depth
 * @return true, if there was a split, false, otherwise
 */
bool ExtendedOctree::splitAndMerge(GLfloat epsilon,GLint depth)
{

    bool isSplited = splitStep(epsilon,depth);

    mergeStep(epsilon,depth);

    return isSplited;

}

/**
 * @brief ExtendedOctree::setVoids Mark every inner leaf nodes as void or not.
 */
void ExtendedOctree::setVoids()
{

    octreeNode* nodePointer;

    for(int i=0;i<this->octreeNodes.size();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];

        nodePointer->isVoid = nodePointer->beta>0.5f;

    }

}

/**
 * @brief ExtendedOctree::setMergeChild Set a merge root to a merge child
 * @param index index of the merge root
 * @param beta new beta value
 */
void ExtendedOctree::setMergeChild(GLint index, GLfloat beta)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    nodePointer->beta = beta;

    if(nodePointer->isMergeRoot)
    {
        nodePointer->isMergeRoot = false;
        nodePointer->isMergeChild = true;
        nodePointer->isIgnored = false;
    }

    if(nodePointer->mesh != NULL)
    {
        delete nodePointer->mesh;
        nodePointer->mesh = NULL;
    }
}

/**
 * @brief ExtendedOctree::split Split a specific node.
 * @param nodePointer pointer of the specific node
 * @param maxDepth maximal depht
 */
void ExtendedOctree::split(octreeNode* nodePointer, GLint maxDepth)
{

    // the node has to be a leaf and
    if(!nodePointer->isLeaf)
    {
        return;
    }

    // the node has not to have the maximal depth
    if( nodePointer->nodeDepth >= maxDepth)
    {
        return;
    }

    GLint x = nodePointer->x;
    GLint y = nodePointer->y;
    GLint z = nodePointer->z;

    GLint newDepth = nodePointer->nodeDepth+1;

    nodePointer->isLeaf = false;

    bool isInside = nodePointer->isInside;

    GLint newLength = nodePointer->cell_length>>1;
    GLint index = nodePointer->index;

    // create all new nodes
    nodePointer->childIndex0=createNode(x,y,z,newDepth,isInside,index);
    nodePointer->childIndex1=createNode(x,y,z+newLength,newDepth,isInside,index);
    nodePointer->childIndex2=createNode(x,y+newLength,z,newDepth,isInside,index);
    nodePointer->childIndex3=createNode(x,y+newLength,z+newLength,newDepth,isInside,index);
    nodePointer->childIndex4=createNode(x+newLength,y,z,newDepth,isInside,index);
    nodePointer->childIndex5=createNode(x+newLength,y,z+newLength,newDepth,isInside,index);
    nodePointer->childIndex6=createNode(x+newLength,y+newLength,z,newDepth,isInside,index);
    nodePointer->childIndex7=createNode(x+newLength,y+newLength,z+newLength,newDepth,isInside,index);

}

/**
 * @brief ExtendedOctree::split Split a specific node.
 * @param nodeIndex index of the specific node
 * @param maxDepht maximal depht
 */
void ExtendedOctree::split(GLint nodeIndex, GLint maxDepth)
{
    octreeNode node = this->octreeNodes.data()[nodeIndex];
    split(&node,maxDepth);
    this->octreeNodes.data()[nodeIndex] = node;
}

/**
 * @brief ExtendedOctree::allChildrenAreLeaves Check whether a specific node has only leaves as children
 * @param nodeIndex index of the specific node
 * @return true if all children are leafes, false otherwise
 */
bool ExtendedOctree::allChildrenAreLeaves(GLint nodeIndex)
{
    if(nodeIndex<0)
    {
        return false;
    }

    octreeNode* nodePointer = &this->octreeNodes.data()[nodeIndex];

    if(nodePointer->isLeaf)
    {
        return false;
    }

    return
        this->octreeNodes.data()[nodePointer->childIndex0].isLeaf &&
        this->octreeNodes.data()[nodePointer->childIndex1].isLeaf &&
        this->octreeNodes.data()[nodePointer->childIndex2].isLeaf &&
        this->octreeNodes.data()[nodePointer->childIndex3].isLeaf &&
        this->octreeNodes.data()[nodePointer->childIndex4].isLeaf &&
        this->octreeNodes.data()[nodePointer->childIndex5].isLeaf &&
        this->octreeNodes.data()[nodePointer->childIndex6].isLeaf &&
        this->octreeNodes.data()[nodePointer->childIndex7].isLeaf;

}

/**
 * @brief ExtendedOctree::handleShellNeighbor Make the node with the specific coordinate to shell node
 * @param x x-coordinate in octree
 * @param y y-coordinate in octree
 * @param z z-coordinate in octree
 * @param backVec pointer of the back buffer vector
 */
void inline ExtendedOctree::handleShellNeighbor(GLint x, GLint y, GLint z, QVector<GLint>* backVec)
{

    octreeNode* nodePointer = this->getLeafNodeByCoordinate(x,y,z);

    if(nodePointer == NULL)
    {
        return;
    }
    if(!IS_INNER_NODE)
    {
        return;
    }

    // split node as long as the maximal depth is not reached
    while(nodePointer->cell_length>1)
    {
        GLint index = nodePointer->index;

        this->split(index,basicMaxDepth);

        nodePointer = &this->octreeNodes.data()[index];

        nodePointer->isInside = false;
        nodePointer->isShell = true;

        nodePointer = this->getLeafNodeByCoordinate(x,y,z,index);
    }

    nodePointer->isInside = false;
    nodePointer->isShell = true;
    backVec->push_back(nodePointer->index);

}

/**
 * @brief ExtendedOctree::increaseShell Increase the shell torward the inner
 * @param loopNumber number of loops for increasing the shell
 */
void ExtendedOctree::increaseShell(GLint loopNumber)
{

    // get a vector of the indices of all inner nodes
    QVector<GLint> shellLeafIndices;
    this->getShellLeaves(&shellLeafIndices);
    octreeNode* nodePointer;

    // buffers
    QVector<GLint> tempVec1;
    QVector<GLint> tempVec2;
    QVector<GLint>* frontVec = &tempVec1;
    QVector<GLint>* backVec = &tempVec2;

    // set front vector
    for(int i=0;i<shellLeafIndices.size();i++)
    {
        nodePointer = &this->octreeNodes.data()[shellLeafIndices.data()[i]];
        frontVec->push_back(nodePointer->index);
    }

    for(int i=0;i<loopNumber;i++)
    {

       for (int j=0;j<frontVec->size();j++)
       {

            // current shell node
            nodePointer = &this->octreeNodes.data()[frontVec->data()[j]];
            GLint x = nodePointer->x;
            GLint y = nodePointer->y;
            GLint z = nodePointer->z;
            GLint cell_length = nodePointer->cell_length;

            // check neigbors

            for(int k=0;k<cell_length;k++)
            {
                for(int l=0;l<cell_length;l++)
                {

                    handleShellNeighbor(x+k,            y+l,            z+cell_length,  backVec);
                    handleShellNeighbor(x+k,            y+l,            z-1,            backVec);
                    handleShellNeighbor(x+k,            y+cell_length,  z+l,            backVec);
                    handleShellNeighbor(x+k,            y-1,            z+l,            backVec);
                    handleShellNeighbor(x+cell_length,  y+k,            z+l,            backVec);
                    handleShellNeighbor(x-1,            y+k,            z+l,            backVec);

                }
            }

       }

       // change front and back buffer
       frontVec->clear();

       if(tempVec1.size()>0)
       {
           frontVec = &tempVec1;
           backVec = &tempVec2;
       }
       else
       if(tempVec2.size()>0){
           frontVec = &tempVec2;
           backVec = &tempVec1;
       }

    }

    shellLeafIndices.clear();
}


/**
 * @brief ExtendedOctree::deleteNodeMeshes Delete the meshes of all merge nodes
 */
void ExtendedOctree::deleteNodeMeshes()
{

    octreeNode* nodePointer;

    for(int i=0;i<this->octreeNodes.size();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];
        if(nodePointer->mesh != NULL)
        {
            delete nodePointer->mesh;
            nodePointer->mesh = NULL;
        }

    }

}

void ExtendedOctree::transformOctree(QMatrix4x4 mat)
{

    octreeNode* nodePointer;

    QVector4D temp1,temp2;

    temp1.setW(1);

    for(int i=0;i<this->octreeNodes.size();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];
        QVector3D vec;

        vec = nodePointer->p0;
        temp1.setX(vec.x());
        temp1.setY(vec.y());
        temp1.setZ(vec.z());
        temp2 = mat*temp1;
        vec.setX(temp2.x());
        vec.setY(temp2.y());
        vec.setZ(temp2.z());
        nodePointer->p0 = vec;

        vec = nodePointer->p1;
        temp1.setX(vec.x());
        temp1.setY(vec.y());
        temp1.setZ(vec.z());
        temp2 = mat*temp1;
        vec.setX(temp2.x());
        vec.setY(temp2.y());
        vec.setZ(temp2.z());
        nodePointer->p1 = vec;

        vec = nodePointer->p2;
        temp1.setX(vec.x());
        temp1.setY(vec.y());
        temp1.setZ(vec.z());
        temp2 = mat*temp1;
        vec.setX(temp2.x());
        vec.setY(temp2.y());
        vec.setZ(temp2.z());
        nodePointer->p2 = vec;

        vec = nodePointer->p3;
        temp1.setX(vec.x());
        temp1.setY(vec.y());
        temp1.setZ(vec.z());
        temp2 = mat*temp1;
        vec.setX(temp2.x());
        vec.setY(temp2.y());
        vec.setZ(temp2.z());
        nodePointer->p3 = vec;

        vec = nodePointer->p4;
        temp1.setX(vec.x());
        temp1.setY(vec.y());
        temp1.setZ(vec.z());
        temp2 = mat*temp1;
        vec.setX(temp2.x());
        vec.setY(temp2.y());
        vec.setZ(temp2.z());
        nodePointer->p4 = vec;

        vec = nodePointer->p5;
        temp1.setX(vec.x());
        temp1.setY(vec.y());
        temp1.setZ(vec.z());
        temp2 = mat*temp1;
        vec.setX(temp2.x());
        vec.setY(temp2.y());
        vec.setZ(temp2.z());
        nodePointer->p5 = vec;

        vec = nodePointer->p6;
        temp1.setX(vec.x());
        temp1.setY(vec.y());
        temp1.setZ(vec.z());
        temp2 = mat*temp1;
        vec.setX(temp2.x());
        vec.setY(temp2.y());
        vec.setZ(temp2.z());
        nodePointer->p6 = vec;

        vec = nodePointer->p7;
        temp1.setX(vec.x());
        temp1.setY(vec.y());
        temp1.setZ(vec.z());
        temp2 = mat*temp1;
        vec.setX(temp2.x());
        vec.setY(temp2.y());
        vec.setZ(temp2.z());
        nodePointer->p7 = vec;

    }

}

//-------------------------------------------------- draw octree grid

/**
 * @brief BasicOctree::setDirty
 */
void ExtendedOctree::makeDirty()
{
    this->isDirty = true;
}

/**
 * @brief BasicOctree::renderOctreeGrid Render the grid of the octree
 * @param shader the shader
 */
void ExtendedOctree::renderOctreeGrid(QGLShaderProgram* shader)
{

    if(this->octreeNodes.size()<1){
        return;
    }

    GLenum primitive = GL_LINES;

    /* if needed needed variables are set for painting */
    if (isDirty) {
        QVector<GLfloat> buffer_vertices,buffer_colors,buffer_vertices_line;

        buffer_vertices.reserve( this->octreeNodes.size() * 8 * 3 * sizeof(GLfloat));
        buffer_colors.reserve( this->octreeNodes.size() * 8 * 3 * sizeof(GLfloat));
        buffer_vertices_line.reserve( this->octreeNodes.size() * 8 * 3 * sizeof(GLfloat));

        indexBuffer.clear();
        indexBuffer_line.clear();

        GLint index = 0;
        GLint index_line = 0;

        GLint viewIndex = (this->axis_length>>1);

        for (int i = 0; i < this->octreeNodes.size(); i++) {

            octreeNode* nodePointer = &this->octreeNodes.data()[i];

            if( !IS_INNER_NODE || !nodePointer->isLeaf)
            {
                continue;
            }

            if(nodePointer->x>=viewIndex)
            {
                continue;
            }

            /* add vertices*/
            buffer_vertices.push_back(nodePointer->p0.x());buffer_vertices.push_back(nodePointer->p0.y());buffer_vertices.push_back(nodePointer->p0.z());
            buffer_vertices.push_back(nodePointer->p1.x());buffer_vertices.push_back(nodePointer->p1.y());buffer_vertices.push_back(nodePointer->p1.z());
            buffer_vertices.push_back(nodePointer->p2.x());buffer_vertices.push_back(nodePointer->p2.y());buffer_vertices.push_back(nodePointer->p2.z());
            buffer_vertices.push_back(nodePointer->p3.x());buffer_vertices.push_back(nodePointer->p3.y());buffer_vertices.push_back(nodePointer->p3.z());

            buffer_vertices.push_back(nodePointer->p4.x());buffer_vertices.push_back(nodePointer->p4.y());buffer_vertices.push_back(nodePointer->p4.z());
            buffer_vertices.push_back(nodePointer->p5.x());buffer_vertices.push_back(nodePointer->p5.y());buffer_vertices.push_back(nodePointer->p5.z());
            buffer_vertices.push_back(nodePointer->p6.x());buffer_vertices.push_back(nodePointer->p6.y());buffer_vertices.push_back(nodePointer->p6.z());
            buffer_vertices.push_back(nodePointer->p7.x());buffer_vertices.push_back(nodePointer->p7.y());buffer_vertices.push_back(nodePointer->p7.z());

            QColor color;

            if(nodePointer->beta>1)
            {
                color.setRedF(0);
                color.setGreenF(0);
                color.setBlueF(0);
            }
            else
            if(nodePointer->beta<0)
            {
                color.setRedF(1);
                color.setGreenF(1);
                color.setBlueF(1);
            }
            else
            {
                QVector3D fillColor(255/(float)255,255/(float)255,224/(float)255);
                QVector3D voidColor(128/(float)255,128/(float)255,0/(float)255);

                QVector3D col = fillColor+(voidColor-fillColor)*nodePointer->beta;

                color.setRedF(col.x());
                color.setGreenF(col.y());
                color.setBlueF(col.z());
            }

            /* add colors*/
            buffer_colors.push_back(color.redF());buffer_colors.push_back(color.greenF());buffer_colors.push_back(color.blueF());
            buffer_colors.push_back(color.redF());buffer_colors.push_back(color.greenF());buffer_colors.push_back(color.blueF());
            buffer_colors.push_back(color.redF());buffer_colors.push_back(color.greenF());buffer_colors.push_back(color.blueF());
            buffer_colors.push_back(color.redF());buffer_colors.push_back(color.greenF());buffer_colors.push_back(color.blueF());

            buffer_colors.push_back(color.redF());buffer_colors.push_back(color.greenF());buffer_colors.push_back(color.blueF());
            buffer_colors.push_back(color.redF());buffer_colors.push_back(color.greenF());buffer_colors.push_back(color.blueF());
            buffer_colors.push_back(color.redF());buffer_colors.push_back(color.greenF());buffer_colors.push_back(color.blueF());
            buffer_colors.push_back(color.redF());buffer_colors.push_back(color.greenF());buffer_colors.push_back(color.blueF());

            /* add indices*/
            indexBuffer.push_back(index+0);
            indexBuffer.push_back(index+1);
            indexBuffer.push_back(index+2);
            indexBuffer.push_back(index+1);
            indexBuffer.push_back(index+3);
            indexBuffer.push_back(index+2);

            indexBuffer.push_back(index+6);
            indexBuffer.push_back(index+7);
            indexBuffer.push_back(index+4);
            indexBuffer.push_back(index+7);
            indexBuffer.push_back(index+5);
            indexBuffer.push_back(index+4);

            indexBuffer.push_back(index+2);
            indexBuffer.push_back(index+3);
            indexBuffer.push_back(index+6);
            indexBuffer.push_back(index+3);
            indexBuffer.push_back(index+7);
            indexBuffer.push_back(index+6);

            indexBuffer.push_back(index+4);
            indexBuffer.push_back(index+5);
            indexBuffer.push_back(index+0);
            indexBuffer.push_back(index+5);
            indexBuffer.push_back(index+1);
            indexBuffer.push_back(index+0);

            indexBuffer.push_back(index+1);
            indexBuffer.push_back(index+5);
            indexBuffer.push_back(index+3);
            indexBuffer.push_back(index+5);
            indexBuffer.push_back(index+7);
            indexBuffer.push_back(index+3);

            indexBuffer.push_back(index+2);
            indexBuffer.push_back(index+6);
            indexBuffer.push_back(index+0);
            indexBuffer.push_back(index+6);
            indexBuffer.push_back(index+4);
            indexBuffer.push_back(index+0);

            index +=8;

            if(nodePointer->nodeDepth<=6)
            {
                buffer_vertices_line.push_back(nodePointer->p0.x());buffer_vertices_line.push_back(nodePointer->p0.y());buffer_vertices_line.push_back(nodePointer->p0.z());
                buffer_vertices_line.push_back(nodePointer->p1.x());buffer_vertices_line.push_back(nodePointer->p1.y());buffer_vertices_line.push_back(nodePointer->p1.z());
                buffer_vertices_line.push_back(nodePointer->p2.x());buffer_vertices_line.push_back(nodePointer->p2.y());buffer_vertices_line.push_back(nodePointer->p2.z());
                buffer_vertices_line.push_back(nodePointer->p3.x());buffer_vertices_line.push_back(nodePointer->p3.y());buffer_vertices_line.push_back(nodePointer->p3.z());

                buffer_vertices_line.push_back(nodePointer->p4.x());buffer_vertices_line.push_back(nodePointer->p4.y());buffer_vertices_line.push_back(nodePointer->p4.z());
                buffer_vertices_line.push_back(nodePointer->p5.x());buffer_vertices_line.push_back(nodePointer->p5.y());buffer_vertices_line.push_back(nodePointer->p5.z());
                buffer_vertices_line.push_back(nodePointer->p6.x());buffer_vertices_line.push_back(nodePointer->p6.y());buffer_vertices_line.push_back(nodePointer->p6.z());
                buffer_vertices_line.push_back(nodePointer->p7.x());buffer_vertices_line.push_back(nodePointer->p7.y());buffer_vertices_line.push_back(nodePointer->p7.z());

                indexBuffer_line.push_back(index_line+0);indexBuffer_line.push_back(index_line+1);
                indexBuffer_line.push_back(index_line+2);indexBuffer_line.push_back(index_line+3);
                indexBuffer_line.push_back(index_line+4);indexBuffer_line.push_back(index_line+5);
                indexBuffer_line.push_back(index_line+6);indexBuffer_line.push_back(index_line+7);

                indexBuffer_line.push_back(index_line+0);indexBuffer_line.push_back(index_line+2);
                indexBuffer_line.push_back(index_line+4);indexBuffer_line.push_back(index_line+6);
                indexBuffer_line.push_back(index_line+0);indexBuffer_line.push_back(index_line+4);
                indexBuffer_line.push_back(index_line+2);indexBuffer_line.push_back(index_line+6);

                indexBuffer_line.push_back(index_line+1);indexBuffer_line.push_back(index_line+3);
                indexBuffer_line.push_back(index_line+5);indexBuffer_line.push_back(index_line+7);
                indexBuffer_line.push_back(index_line+1);indexBuffer_line.push_back(index_line+5);
                indexBuffer_line.push_back(index_line+3);indexBuffer_line.push_back(index_line+7);

                index_line +=8;
            }
        }

        if(vbo != NULL)
        {
            vbo->destroy();
            vbo = NULL;
        }

        vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        vbo->create();
        vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        vbo->bind();
        vbo->allocate(buffer_vertices.size() * sizeof(GLfloat));
        vbo->write(0, buffer_vertices.constData(), buffer_vertices.size() * sizeof(GLfloat));
        vbo->release();

        if(cbo != NULL)
        {
            cbo->destroy();
            cbo = NULL;
        }

        cbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        cbo->create();
        cbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        cbo->bind();
        cbo->allocate(buffer_colors.size() * 3 * sizeof(GLfloat));
        cbo->write(0, buffer_colors.constData(), buffer_colors.size() * sizeof(GLfloat));
        cbo->release();

        if(ibo != NULL)
        {
            ibo->destroy();
            ibo = NULL;
        }

        ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        ibo->create();
        ibo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        ibo->bind();
        ibo->allocate(indexBuffer.size() * sizeof(GLint));
        ibo->write(0, indexBuffer.constData(), indexBuffer.size() * sizeof(GLint));
        ibo->release();

        if(vbo_line != NULL)
        {
            vbo_line->destroy();
            vbo_line = NULL;
        }

        vbo_line = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        vbo_line->create();
        vbo_line->setUsagePattern(QOpenGLBuffer::StaticDraw);
        vbo_line->bind();
        vbo_line->allocate(buffer_vertices_line.size() * 3 * sizeof(GLfloat));
        vbo_line->write(0, buffer_vertices_line.constData(), buffer_vertices_line.size() * sizeof(GLfloat));
        vbo_line->release();

        if(ibo_line != NULL)
        {
            ibo_line->destroy();
            ibo_line = NULL;
        }

        ibo_line = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        ibo_line->create();
        ibo_line->setUsagePattern(QOpenGLBuffer::StaticDraw);
        ibo_line->bind();
        ibo_line->allocate(indexBuffer_line.size() * sizeof(GLint));
        ibo_line->write(0, indexBuffer_line.constData(), indexBuffer_line.size() * sizeof(GLint));
        ibo_line->release();

        isDirty = false;

    }

    GLint stride = sizeof(GLfloat) * (GEOMETRY_DATA_SIZE);

    vbo->bind();
    shader->enableAttributeArray("geometry");
    shader->setAttributeBuffer("geometry", GL_FLOAT, 0, 3, stride);
    vbo->release();

    cbo->bind();
    shader->enableAttributeArray("color");
    shader->setAttributeBuffer("color", GL_FLOAT, 0, 3, stride);
    cbo->release();

    ibo->bind();
    glDrawElements(GL_TRIANGLES, indexBuffer.size(), GL_UNSIGNED_INT, (void*) 0);
    ibo->release();

    shader->disableAttributeArray("geometry");
    shader->disableAttributeArray("color");

    shader->setAttributeValue("color",QColor(Qt::black));

    glLineWidth(2);

    vbo_line->bind();
    shader->setAttributeBuffer("geometry", GL_FLOAT, 0, 3, stride);
    shader->enableAttributeArray("geometry");
    vbo_line->release();

    ibo_line->bind();
    glDrawElements(primitive, indexBuffer_line.size(), GL_UNSIGNED_INT, (void*) 0);
    ibo_line->release();

    glLineWidth(1);

    shader->disableAttributeArray("geometry");

}
