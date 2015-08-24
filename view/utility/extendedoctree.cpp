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
 * @brief ExtendedOctree::getInnerCubes Get a pointer of a vector of cubes which represent the geometry of all inner leaf nodes.
 * @return pointer of vector
 */
QVector<cubeObject>* ExtendedOctree::getInnerCubes()
{
    // reset the default state
    // deallocate the geometry of the previous cubes
    for(int i=0;i<cubeVector.length();i++)
    {
        delete cubeVector.data()[i].mesh;
    }
    cubeVector.clear();


    // get a vector of the indices of all inner nodes
    QVector<GLint> innerNodeIndices;
    this->getInnerLeaves(&innerNodeIndices);


    QVector<GLfloat>* geometry;
    QVector<GLint>* indices;
    octreeNode* nodePointer;


    // iterate about indices of all inner nodes and create a cube
    for(int i=0;i<innerNodeIndices.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[innerNodeIndices.data()[i]];

        // allocate and reserve storage for the geometry of a cube
        geometry = new QVector<GLfloat>();
        geometry->reserve(8*3);
        indices = new QVector<GLint>();
        indices->reserve(12*3);

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
        indices->push_back(2);
        indices->push_back(1);
        indices->push_back(0);
        indices->push_back(2);
        indices->push_back(3);
        indices->push_back(1);

        // top
        indices->push_back(4);
        indices->push_back(5);
        indices->push_back(6);
        indices->push_back(5);
        indices->push_back(7);
        indices->push_back(6);

        // left
        indices->push_back(1);
        indices->push_back(5);
        indices->push_back(4);
        indices->push_back(1);
        indices->push_back(4);
        indices->push_back(0);

        // right
        indices->push_back(2);
        indices->push_back(6);
        indices->push_back(7);
        indices->push_back(2);
        indices->push_back(7);
        indices->push_back(3);

        // back
        indices->push_back(0);
        indices->push_back(4);
        indices->push_back(6);
        indices->push_back(0);
        indices->push_back(6);
        indices->push_back(2);

        // front
        indices->push_back(3);
        indices->push_back(7);
        indices->push_back(1);
        indices->push_back(7);
        indices->push_back(5);
        indices->push_back(1);

        // set the values of the cube
        cubeObject obj;
        obj.mesh = new Mesh(geometry,indices);
        obj.beta = nodePointer->beta;
        obj.index = nodePointer->index;

        // add the cube to the vector
        cubeVector.push_back(obj);

    }

    return &cubeVector;
}

/**
 * @brief ExtendedOctree::splitAndMerge Split and merge inner nodes.
 * @param epsilon range for the merge area (0,epsilon) and (1-epsilon,1)
 * @return true, if there was a split, false, otherwise
 */
bool ExtendedOctree::splitAndMerge(GLfloat epsilon)
{

    // get a vector of the indices of all inner nodes
    QVector<GLint> innerLeafIndices;
    getInnerLeaves(&innerLeafIndices);


    GLfloat oneMinusEpsilon = 1-epsilon;
    octreeNode* nodePointer;
    bool isSplited = false;

    // split step
    // iterate about the indices of all inner "leaf" nodes
    for(int i=0;i<innerLeafIndices.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[innerLeafIndices.at(i)];

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

                isSplited = true;
            }
        }
    }

    // get a vector of the indices of all inner nodes
    QVector<GLint> innerLeafSetNodeIndices;
    getInnerLeafSets(&innerLeafSetNodeIndices);


    bool hasToBeMerged = false;
    GLfloat beta = 0.f;
    GLfloat b0,b1,b2,b3,b4,b5,b6,b7;

    for(int i=0;i<innerLeafSetNodeIndices.length();i++)
    {

        hasToBeMerged = false;

        nodePointer = &this->octreeNodes.data()[innerLeafSetNodeIndices.at(i)];

        b0 = this->octreeNodes.data()[nodePointer->childIndex0].beta;
        b1 = this->octreeNodes.data()[nodePointer->childIndex1].beta;
        b2 = this->octreeNodes.data()[nodePointer->childIndex2].beta;
        b3 = this->octreeNodes.data()[nodePointer->childIndex3].beta;
        b4 = this->octreeNodes.data()[nodePointer->childIndex4].beta;
        b5 = this->octreeNodes.data()[nodePointer->childIndex5].beta;
        b6 = this->octreeNodes.data()[nodePointer->childIndex6].beta;
        b7 = this->octreeNodes.data()[nodePointer->childIndex7].beta;

        // all beta values of inner leaves are greater than 1-epsilon
        if(
            (b0 >= oneMinusEpsilon) &&
            (b1 >= oneMinusEpsilon) &&
            (b2 >= oneMinusEpsilon) &&
            (b3 >= oneMinusEpsilon) &&
            (b4 >= oneMinusEpsilon) &&
            (b5 >= oneMinusEpsilon) &&
            (b6 >= oneMinusEpsilon) &&
            (b7 >= oneMinusEpsilon))
        {

            beta = 1.f;
            hasToBeMerged = true;
        }
        else
        // all beta values of inner leaves are small than epsilon
        if(
            (b0 <= oneMinusEpsilon) &&
            (b1 <= oneMinusEpsilon) &&
            (b2 <= oneMinusEpsilon) &&
            (b3 <= oneMinusEpsilon) &&
            (b4 <= oneMinusEpsilon) &&
            (b5 <= oneMinusEpsilon) &&
            (b6 <= oneMinusEpsilon) &&
            (b7 <= oneMinusEpsilon))
        {

            beta = 0.f;
            hasToBeMerged = true;
        }

        // the node has to be merged
        if( hasToBeMerged )
        {
            this->merge(innerLeafSetNodeIndices.at(i));

            nodePointer = &this->octreeNodes.data()[innerLeafSetNodeIndices.at(i)];
            nodePointer->beta = beta;
        }

    }

    return isSplited;

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

        if(nodePointer->inside && nodePointer->isSet)
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
 * @brief ExtendedOctree::getInnerNodesOfLeafSet Get the indices of all inner nodes which have only leaves as children.
 * @param indices a pointer of th vector which get the indices of the found nodes
 */
void ExtendedOctree::getInnerLeafSets(QVector<GLint>* indices)
{

    octreeNode* nodePointer;

    for(int i=0;i<this->octreeNodes.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];

        if(nodePointer->invalid){
            continue;
        }

        if(nodePointer->inside && nodePointer->isSet && !nodePointer->leaf)
        {
            if(allChildrenAreLeaves(nodePointer->index))
            {
                indices->push_back(nodePointer->index);
            }
        }
    }
}

/**
 * @brief ExtendedOctree::getInnerLeaves Get the indices of all inner leaf nodes
 * @param indices a pointer of th vector which get the indices of the found nodes
 */
void ExtendedOctree::getInnerLeaves(QVector<GLint>* indices)
{

    octreeNode* nodePointer;

    for(int i=0;i<this->octreeNodes.length();i++)
    {

        if(nodePointer->invalid){
            continue;
        }

        nodePointer = &this->octreeNodes.data()[i];

        if(nodePointer->inside && nodePointer->isSet && nodePointer->leaf)
        {
           indices->push_back(nodePointer->index);
        }

    }

}

/**
 * @brief ExtendedOctree::split Split a specific node.
 * @param nodePointer pointer of the specific node
 */
void ExtendedOctree::split(octreeNode* nodePointer){

    // the node has to be a leaf and
    if(!nodePointer->leaf || !nodePointer->isSet){
        return;
    }

    // the node has not to have the maximal depth
    if( (nodePointer->nodeDepth + 1) > this->maxDepth)
    {
        return;
    }

    GLint x = nodePointer->x;
    GLint y = nodePointer->y;
    GLint z = nodePointer->z;

    GLint newDepth = nodePointer->nodeDepth+1;
    bool addToInterios = nodePointer->inside;

    GLint newLength = nodePointer->cell_length>>1;
    GLint index = nodePointer->index;
    GLint index0 = createNode(x,y,z,newDepth,addToInterios,index);

    nodePointer->childIndex0 = index0;

    // create all new nodes
    nodePointer->childIndex1=createNode(x,y,z+newLength,newDepth,addToInterios,index);
    nodePointer->childIndex2=createNode(x,y+newLength,z,newDepth,addToInterios,index);
    nodePointer->childIndex3=createNode(x,y+newLength,z+newLength,newDepth,addToInterios,index);
    nodePointer->childIndex4=createNode(x+newLength,y,z,newDepth,addToInterios,index);
    nodePointer->childIndex5=createNode(x+newLength,y,z+newLength,newDepth,addToInterios,index);
    nodePointer->childIndex6=createNode(x+newLength,y+newLength,z,newDepth,addToInterios,index);
    nodePointer->childIndex7=createNode(x+newLength,y+newLength,z+newLength,newDepth,addToInterios,index);

    nodePointer->leaf = false;

    // remove the node from the list as inner
    if(nodePointer->inside)
    {
        this->innerLeafIndices.data()[nodePointer->typeVectorIndex] = -1;
        this->freeIndicesForInnerLeaves.push_back(nodePointer->typeVectorIndex);
    }

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
 * @brief ExtendedOctree::merge Merge a specific node.
 * @param nodePointer pointer of the node
 */
void ExtendedOctree::merge(octreeNode* nodePointer)
{

    // all children must be leaves
    if(!this->allChildrenAreLeaves(nodePointer->index))
    {
        return;
    }

    // mark the children as invalid
    this->octreeNodes.data()[nodePointer->childIndex0].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex1].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex2].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex3].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex4].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex5].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex6].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex7].invalid = true;

    // set the indices as free
    this->freeIndicesForAllNodes.push_back(nodePointer->childIndex0);
    this->freeIndicesForAllNodes.push_back(nodePointer->childIndex1);
    this->freeIndicesForAllNodes.push_back(nodePointer->childIndex2);
    this->freeIndicesForAllNodes.push_back(nodePointer->childIndex3);
    this->freeIndicesForAllNodes.push_back(nodePointer->childIndex4);
    this->freeIndicesForAllNodes.push_back(nodePointer->childIndex5);
    this->freeIndicesForAllNodes.push_back(nodePointer->childIndex6);
    this->freeIndicesForAllNodes.push_back(nodePointer->childIndex7);

    octreeNode node;

    /* the children will be removed from interiorNodeLeafIndices*/
    node = this->octreeNodes.data()[nodePointer->childIndex0];
    this->innerLeafIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerLeaves.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex1];
    this->innerLeafIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerLeaves.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex2];
    this->innerLeafIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerLeaves.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex3];
    this->innerLeafIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerLeaves.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex4];
    this->innerLeafIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerLeaves.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex5];
    this->innerLeafIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerLeaves.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex6];
    this->innerLeafIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerLeaves.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex7];
    this->innerLeafIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerLeaves.push_back(node.typeVectorIndex);

    // make the indices of the children invalid
    nodePointer->childIndex0=-1;
    nodePointer->childIndex1=-1;
    nodePointer->childIndex2=-1;
    nodePointer->childIndex3=-1;
    nodePointer->childIndex4=-1;
    nodePointer->childIndex5=-1;
    nodePointer->childIndex6=-1;
    nodePointer->childIndex7=-1;

    // get an index for the merged node
    if(!this->freeIndicesForInnerLeaves.empty())
    {
        nodePointer->typeVectorIndex = this->freeIndicesForInnerLeaves.last();
        this->freeIndicesForInnerLeaves.pop_back();
        this->innerLeafIndices.data()[nodePointer->typeVectorIndex] = nodePointer->index;
    }
    else
    {
        nodePointer->typeVectorIndex = this->freeIndicesForInnerLeaves.length();
        this->innerLeafIndices.push_back(nodePointer->index);
    }

    nodePointer->leaf = true;

}

/**
 * @brief ExtendedOctree::merge Merge a specific node.
 * @param nodeIndex index of the specific node
 */
void ExtendedOctree::merge(GLint nodeIndex)
{
    octreeNode node = this->octreeNodes.data()[nodeIndex];
    merge(&node);
    this->octreeNodes.data()[nodeIndex] = node;
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

    if(!nodePointer->isSet || !nodePointer->inside || nodePointer->shellNode)
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

    nodePointer->shellNode = true;
    backVec->push_back(nodePointer->index);

}

/**
 * @brief ExtendedOctree::increaseShell Increase the shell torward the inner
 * @param loopNumber number of loops for increasing the shell
 */
void ExtendedOctree::increaseShell(GLint loopNumber)
{

    octreeNode node;

    // buffers
    QVector<GLint> tempVec1;
    QVector<GLint> tempVec2;
    QVector<GLint>* frontVec = &tempVec1;
    QVector<GLint>* backVec = &tempVec2;

    // set front vector
    for(int i=0;i<this->shellNodeIndices.length();i++)
    {
        node = this->octreeNodes.data()[this->shellNodeIndices.data()[i]];
        frontVec->push_back(node.index);
    }

    for(int i=0;i<loopNumber;i++)
    {

       for (int j=0;j<frontVec->size();j++)
       {

            // current shell node
            node = this->octreeNodes.data()[frontVec->data()[j]];
            GLint x = node.x;
            GLint y = node.y;
            GLint z = node.z;
            GLint cell_length = node.cell_length;

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

       if(tempVec1.size()>0){
           frontVec = &tempVec1;
           backVec = &tempVec2;
       }
       else
       if(tempVec2.size()>0){
           frontVec = &tempVec2;
           backVec = &tempVec1;
       }

    }

    return;
}

/**
 * @brief ExtendedOctree::setVoids Mark every inner leaf nodes as void or not.
 */
void ExtendedOctree::setVoids()
{

    QVector<GLint> innerLeafIndices;

    octreeNode* nodePointer;

    this->getInnerLeaves(&innerLeafIndices);

    // iterate about all inner leaves
    for(int i=0;i<innerLeafIndices.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[innerLeafIndices.data()[i]];

        nodePointer->beta = nodePointer->beta>0.5? 1.f : 0.f;

    }

}
