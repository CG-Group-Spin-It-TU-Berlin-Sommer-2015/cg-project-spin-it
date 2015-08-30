#include "extendedoctree.h"

using namespace std;
using namespace octree;

ExtendedOctree::ExtendedOctree()
{

}

/**
 * @brief ExtendedOctree::updateBetas Update all beta values of the inner leaf nodes with the values of the cube vector.
 */
void ExtendedOctree::updateBetaValues()
{

    // iterate about all cubes
    for(int i=0;i<cubeVector.length();i++)
    {
        cubeObject* obj = &cubeVector.data()[i];
        (&octreeNodes.data()[obj->index])->beta = obj->beta;
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

    // set the vertices
    geometry->push_back(nodePointer->p0.x());
    geometry->push_back(nodePointer->p0.y());
    geometry->push_back(nodePointer->p0.z());
    geometry->push_back(nodePointer->p1.x());
    geometry->push_back(nodePointer->p1.y());
    geometry->push_back(nodePointer->p1.z());

    geometry->push_back(nodePointer->p2.x());
    geometry->push_back(nodePointer->p2.y());
    geometry->push_back(nodePointer->p2.z());
    geometry->push_back(nodePointer->p3.x());
    geometry->push_back(nodePointer->p3.y());
    geometry->push_back(nodePointer->p3.z());

    geometry->push_back(nodePointer->p4.x());
    geometry->push_back(nodePointer->p4.y());
    geometry->push_back(nodePointer->p4.z());
    geometry->push_back(nodePointer->p5.x());
    geometry->push_back(nodePointer->p5.y());
    geometry->push_back(nodePointer->p5.z());

    geometry->push_back(nodePointer->p6.x());
    geometry->push_back(nodePointer->p6.y());
    geometry->push_back(nodePointer->p6.z());
    geometry->push_back(nodePointer->p7.x());
    geometry->push_back(nodePointer->p7.y());
    geometry->push_back(nodePointer->p7.z());

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
 * @brief ExtendedOctree::getInnerCubes Get a pointer of a vector of cubes which represent the geometry of all inner leaf nodes.
 * @return pointer of vector
 */
QVector<cubeObject>* ExtendedOctree::getInnerCubes()
{
    // reset the default state
    cubeVector.clear();


    // get a vector of the indices of all inner nodes
    QVector<GLint> mergeIndices;
    this->getMergeRoots(&mergeIndices);
    octreeNode* nodePointer;


    QVector<GLfloat>* geometry;
    QVector<GLint>* indices;

    QVector<GLint> childIndices;

    // iterate about indices of all inner nodes and create a cube
    for(int i=0;i<mergeIndices.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[mergeIndices.data()[i]];

        if(nodePointer->mesh == NULL)
        {

            if(nodePointer->leaf)
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
                geometry->reserve(8*3*childIndices.length());
                indices->reserve(12*3*childIndices.length());

                for(int i=0;i<childIndices.length();i++)
                {
                    addCubeMesh(childIndices.at(i),geometry,indices,8*i);
                }

                nodePointer->mesh = new Mesh(geometry,indices);
            }

        }

        if(nodePointer->mesh->getGeometry()->length()<1)
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
 * @brief ExtendedOctree::splitAndMerge Split and merge inner nodes.
 * @param epsilon range for the merge area (0,epsilon) and (1-epsilon,1)
 * @return true, if there was a split, false, otherwise
 */
bool ExtendedOctree::splitAndMerge(GLfloat epsilon)
{

    octreeNode* nodePointer;

    GLfloat oneMinusEpsilon = 1-epsilon;
    bool isSplited = false;


    // get a vector of the indices of all inner nodes
    QVector<GLint> innerLeafIndices;
    getInnerLeaves(&innerLeafIndices);

    // split step
    for(int i=0;i<innerLeafIndices.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[innerLeafIndices.at(i)];

        if(!nodePointer->isMergeRoot)
        {
            continue;
        }

        // if the beta value is in the set (0,epsilon)^(1-epsilon,1)
        if( epsilon < nodePointer->beta && nodePointer->beta < oneMinusEpsilon )
        {
            this->split(innerLeafIndices.at(i));

            nodePointer = &this->octreeNodes.data()[innerLeafIndices.at(i)];

            // check whether the node has been splitted
            if(!nodePointer->leaf)
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

                // konvergece is not reached
                isSplited = true;
            }
        }
    }

    innerLeafIndices.clear();

    // get a vector of the indices of all merge candidates
    QVector<GLint> mergeCandidates;
    getMergeRootCandidates(&mergeCandidates);

    octreeNode *c0p,*c1p,*c2p,*c3p,*c4p,*c5p,*c6p,*c7p;

    // merge step
    for(int i=0;i<mergeCandidates.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[mergeCandidates.at(i)];

        c0p = &this->octreeNodes.data()[nodePointer->childIndex0];
        c1p = &this->octreeNodes.data()[nodePointer->childIndex1];
        c2p = &this->octreeNodes.data()[nodePointer->childIndex2];
        c3p = &this->octreeNodes.data()[nodePointer->childIndex3];
        c4p = &this->octreeNodes.data()[nodePointer->childIndex4];
        c5p = &this->octreeNodes.data()[nodePointer->childIndex5];
        c6p = &this->octreeNodes.data()[nodePointer->childIndex6];
        c7p = &this->octreeNodes.data()[nodePointer->childIndex7];

        bool ignoreC0 =(c0p->isShell && c0p->leaf) || (!c0p->isShell && !c0p->isInside);
        bool ignoreC1 =(c1p->isShell && c1p->leaf) || (!c1p->isShell && !c1p->isInside);
        bool ignoreC2 =(c2p->isShell && c2p->leaf) || (!c2p->isShell && !c2p->isInside);
        bool ignoreC3 =(c3p->isShell && c3p->leaf) || (!c3p->isShell && !c3p->isInside);
        bool ignoreC4 =(c4p->isShell && c4p->leaf) || (!c4p->isShell && !c4p->isInside);
        bool ignoreC5 =(c5p->isShell && c5p->leaf) || (!c5p->isShell && !c5p->isInside);
        bool ignoreC6 =(c6p->isShell && c6p->leaf) || (!c6p->isShell && !c6p->isInside);
        bool ignoreC7 =(c7p->isShell && c7p->leaf) || (!c7p->isShell && !c7p->isInside);


        bool allSmallerEpsilon =
                (ignoreC0 || (c0p->isMergeRoot && c0p->beta < epsilon)) &&
                (ignoreC1 || (c1p->isMergeRoot && c1p->beta < epsilon)) &&
                (ignoreC2 || (c2p->isMergeRoot && c2p->beta < epsilon)) &&
                (ignoreC3 || (c3p->isMergeRoot && c3p->beta < epsilon)) &&
                (ignoreC4 || (c4p->isMergeRoot && c4p->beta < epsilon)) &&
                (ignoreC5 || (c5p->isMergeRoot && c5p->beta < epsilon)) &&
                (ignoreC6 || (c6p->isMergeRoot && c6p->beta < epsilon)) &&
                (ignoreC7 || (c7p->isMergeRoot && c7p->beta < epsilon));

        bool allGreaterEpsilon = false;

        if(!allSmallerEpsilon)
        {

            allGreaterEpsilon =
                    (ignoreC0 || (c0p->isMergeRoot && c0p->beta > oneMinusEpsilon)) &&
                    (ignoreC1 || (c1p->isMergeRoot && c1p->beta > oneMinusEpsilon)) &&
                    (ignoreC2 || (c2p->isMergeRoot && c2p->beta > oneMinusEpsilon)) &&
                    (ignoreC3 || (c3p->isMergeRoot && c3p->beta > oneMinusEpsilon)) &&
                    (ignoreC4 || (c4p->isMergeRoot && c4p->beta > oneMinusEpsilon)) &&
                    (ignoreC5 || (c5p->isMergeRoot && c5p->beta > oneMinusEpsilon)) &&
                    (ignoreC6 || (c6p->isMergeRoot && c6p->beta > oneMinusEpsilon)) &&
                    (ignoreC7 || (c7p->isMergeRoot && c7p->beta > oneMinusEpsilon));
        }

        if(allSmallerEpsilon || allGreaterEpsilon)
        {
            // merge
            float beta = allGreaterEpsilon?1.f:0.f;

            nodePointer->isMergeRoot = true;
            nodePointer->beta = beta;

            setMergeChild(c0p->index,beta);
            setMergeChild(c1p->index,beta);
            setMergeChild(c2p->index,beta);
            setMergeChild(c3p->index,beta);
            setMergeChild(c4p->index,beta);
            setMergeChild(c5p->index,beta);
            setMergeChild(c6p->index,beta);
            setMergeChild(c7p->index,beta);

        }

    }

    mergeCandidates.clear();

    return isSplited;

}

/**
 * @brief ExtendedOctree::setMergeChild Set a merge root to a merge child
 * @param index index of the merge root
 * @param beta new beta value
 */
void ExtendedOctree::setMergeChild(GLint index, GLfloat beta)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    if(!nodePointer->isMergeRoot)
    {
        return;
    }

    nodePointer->beta = beta;
    nodePointer->isMergeRoot = false;
    nodePointer->isMergeChild = true;

    if(nodePointer->mesh != NULL)
    {
        delete nodePointer->mesh;
        nodePointer->mesh = NULL;
    }
}


/**
 * @brief ExtendedOctree::getNodesOfDepth Get the indices of all nodes which lay a specific depth.
 * @param depth the depth of the searched nodes
 * @param indices a pointer of th vector which get the indices of the found nodes
 */
void ExtendedOctree::getNodesOfDepth(GLint depth,QVector<GLint>* indices)
{

    octreeNode* nodePointer;

    // iterate about all nodes
    for(int i=0;i<this->octreeNodes.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];

        if(nodePointer->nodeDepth>=depth)
        {
            indices->push_back(nodePointer->index);
        }

    }

}

/**
 * @brief ExtendedOctree::getInnerNodesForNode Get the indices of all inner leaf nodes which are predecessors of a specific node.
 * @param index index of the specific node
 * @param indices a pointer of th vector which get the indices of the found nodes
 */
void ExtendedOctree::getInnerLeavesForNode(GLint index,QVector<GLint>* indices)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    // break recursion because current node is a leaf
    if(nodePointer->leaf)
    {

        if(nodePointer->isInside && !nodePointer->isShell)
        {
            indices->push_back(nodePointer->index);
        }

        return;
    }

    // check every child of the current node

    getInnerLeavesForNode(nodePointer->childIndex0,indices);
    getInnerLeavesForNode(nodePointer->childIndex1,indices);
    getInnerLeavesForNode(nodePointer->childIndex2,indices);
    getInnerLeavesForNode(nodePointer->childIndex3,indices);

    getInnerLeavesForNode(nodePointer->childIndex4,indices);
    getInnerLeavesForNode(nodePointer->childIndex5,indices);
    getInnerLeavesForNode(nodePointer->childIndex6,indices);
    getInnerLeavesForNode(nodePointer->childIndex7,indices);

}

/**
 * @brief ExtendedOctree::split Split a specific node.
 * @param nodePointer pointer of the specific node
 */
void ExtendedOctree::split(octreeNode* nodePointer){

    // the node has to be a leaf and
    if(!nodePointer->leaf)
    {
        return;
    }

    // the node has not to have the maximal depth
    if( (nodePointer->nodeDepth + 1) > this->optimationMaxDepth)
    {
        return;
    }

    GLint x = nodePointer->x;
    GLint y = nodePointer->y;
    GLint z = nodePointer->z;

    GLint newDepth = nodePointer->nodeDepth+1;

    nodePointer->leaf = false;

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
 */
void ExtendedOctree::split(GLint nodeIndex)
{
    octreeNode node = this->octreeNodes.data()[nodeIndex];
    split(&node);
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

    if(nodePointer->leaf)
    {
        return false;
    }

    return
        this->octreeNodes.data()[nodePointer->childIndex0].leaf &&
        this->octreeNodes.data()[nodePointer->childIndex1].leaf &&
        this->octreeNodes.data()[nodePointer->childIndex2].leaf &&
        this->octreeNodes.data()[nodePointer->childIndex3].leaf &&
        this->octreeNodes.data()[nodePointer->childIndex4].leaf &&
        this->octreeNodes.data()[nodePointer->childIndex5].leaf &&
        this->octreeNodes.data()[nodePointer->childIndex6].leaf &&
        this->octreeNodes.data()[nodePointer->childIndex7].leaf;

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

    if(!nodePointer->isInside || nodePointer->isShell)
    {
        return;
    }

    // split node as long as the maximal depth is not reached
    while(nodePointer->cell_length>1)
    {
        GLint index = nodePointer->index;

        this->split(index);
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
    for(int i=0;i<shellLeafIndices.length();i++)
    {
        nodePointer = &this->octreeNodes.data()[shellLeafIndices.at(i)];
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
 * @brief ExtendedOctree::setVoids Mark every inner leaf nodes as void or not.
 */
void ExtendedOctree::setVoids()
{

    // get a vector of the indices of all inner nodes
    QVector<GLint> innerLeafIndices;
    this->getInnerLeaves(&innerLeafIndices);
    octreeNode* nodePointer;

    // iterate about all inner leaves
    for(int i=0;i<innerLeafIndices.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[innerLeafIndices.data()[i]];
        nodePointer->isVoid = nodePointer->beta>0.5;

    }

    innerLeafIndices.clear();
}

/**
 * @brief ExtendedOctree::setVoids Mark every inner leaf nodes as void or not.
 */
void ExtendedOctree::setMergeNodes()
{

    // get a vector of the indices of all inner nodes
    QVector<GLint> innerLeafIndices;
    this->getInnerLeaves(&innerLeafIndices);
    octreeNode* nodePointer;

    // iterate about all inner leaves
    for(int i=0;i<innerLeafIndices.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[innerLeafIndices.data()[i]];
        nodePointer->isMergeRoot = true;

    }

    innerLeafIndices.clear();
}

/**
 * @brief ExtendedOctree::deleteNodeMeshes Delete the meshes of all merge nodes
 */
void ExtendedOctree::deleteNodeMeshes()
{

    // get a vector of the indices of all inner nodes
    QVector<GLint> mergeIndices;
    this->getMergeRoots(&mergeIndices);
    octreeNode* nodePointer;

    // iterate about indices of all inner nodes and create a cube
    for(int i=0;i<mergeIndices.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[mergeIndices.data()[i]];
        if(nodePointer->mesh != NULL)
        {
            delete nodePointer->mesh;
        }

    }

    mergeIndices.clear();

}
