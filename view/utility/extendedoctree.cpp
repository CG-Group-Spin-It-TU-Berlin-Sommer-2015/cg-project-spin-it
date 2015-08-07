#include "extendedoctree.h"

using namespace std;
using namespace octree;

ExtendedOctree::ExtendedOctree()
{

}

void ExtendedOctree::setBetasForCubes(QVector<cubeObject>* vec)
{

    for(int i=0;i<vec->length();i++)
    {
        cubeObject* obj = &vec->data()[i];

        (&octreeNodes.data()[obj->index])->beta = obj->beta;

    }

}

void ExtendedOctree::getInnerCubes(QVector<cubeObject>* vec)
{

    QVector<GLint> innerNodeIndices;

    octreeNode* nodePointer;

    this->getInnerLeaves(&innerNodeIndices);

    QVector<GLfloat>* geometry;
    QVector<GLint>* indices;

    for(int i=0;i<innerNodeIndices.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[innerNodeIndices.data()[i]];

        geometry = new QVector<GLfloat>();
        geometry->reserve(8*3);
        indices = new QVector<GLint>();
        indices->reserve(12*3);

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

        cubeObject obj;
        obj.mesh = new Mesh(geometry,indices);
        obj.beta = nodePointer->beta;
        obj.index = nodePointer->index;

        vec->push_back(obj);

    }

}



void ExtendedOctree::splitAndMerge(GLfloat epsilon)
{

    GLfloat oneMinusEpsilon = 1-epsilon;

    octreeNode* nodePointer;

    QVector<GLint> innerLeafIndices;
    getInnerLeaves(&innerLeafIndices);

    for(int i=0;i<innerLeafIndices.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[innerLeafIndices.at(i)];

        if( false /* epsilon < nodePointer->beta && nodePointer < oneMinusEpsilon */)
        {
            this->split(innerLeafIndices.at(i));

            nodePointer = &this->octreeNodes.data()[innerLeafIndices.at(i)];

            if(!nodePointer->leaf)
            {
                this->octreeNodes.data()[nodePointer->childIndex0].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex1].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex2].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex3].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex4].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex5].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex6].beta = nodePointer->beta;
                this->octreeNodes.data()[nodePointer->childIndex7].beta = nodePointer->beta;
            }
        }

    }

    QVector<GLint> innerLeafSetNodeIndices;
    getInnerNodesOfLeafSet(&innerLeafSetNodeIndices);

    bool merge = false;
    GLfloat beta = 0.f;

    GLfloat b0,b1,b2,b3,b4,b5,b6,b7;

    for(int i=0;i<innerLeafSetNodeIndices.length();i++)
    {

        merge = false;

        nodePointer = &this->octreeNodes.data()[innerLeafSetNodeIndices.at(i)];

        b0 = this->octreeNodes.data()[nodePointer->childIndex0].beta;
        b1 = this->octreeNodes.data()[nodePointer->childIndex1].beta;
        b2 = this->octreeNodes.data()[nodePointer->childIndex2].beta;
        b3 = this->octreeNodes.data()[nodePointer->childIndex3].beta;
        b4 = this->octreeNodes.data()[nodePointer->childIndex4].beta;
        b5 = this->octreeNodes.data()[nodePointer->childIndex5].beta;
        b6 = this->octreeNodes.data()[nodePointer->childIndex6].beta;
        b7 = this->octreeNodes.data()[nodePointer->childIndex7].beta;

        // all betas of inner leaves are greater than 1-epsilon
        if( false
        /*
        (b0 > oneMinusEpsilon) &&
        (b1 > oneMinusEpsilon) &&
        (b2 > oneMinusEpsilon) &&
        (b3 > oneMinusEpsilon) &&
        (b4 > oneMinusEpsilon) &&
        (b5 > oneMinusEpsilon) &&
        (b6 > oneMinusEpsilon) &&
        (b7 > oneMinusEpsilon)
        */ ){

            beta = 1.f;
            merge = true;
        }
        else
        // all betas of inner leaves are small than epsilon
        if( false
        /*
        (b0 < oneMinusEpsilon) &&
        (b1 < oneMinusEpsilon) &&
        (b2 < oneMinusEpsilon) &&
        (b3 < oneMinusEpsilon) &&
        (b4 < oneMinusEpsilon) &&
        (b5 < oneMinusEpsilon) &&
        (b6 < oneMinusEpsilon) &&
        (b7 < oneMinusEpsilon)
        */ ){

            beta = 0.f;
            merge = true;
        }

        if( merge )
        {
            this->merge(innerLeafSetNodeIndices.at(i));

            nodePointer = &this->octreeNodes.data()[innerLeafSetNodeIndices.at(i)];
            nodePointer->beta = beta;
        }

    }

}

void ExtendedOctree::getNodesOfDepth(GLint depth,QVector<GLint>* indices)
{

    octreeNode* nodePointer;

    for(int i=0;i<this->octreeNodes.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];
        if(nodePointer->nodeDepth>=depth)
        {
            indices->push_back(nodePointer->index);
        }

    }

}

void ExtendedOctree::getInnerNodesForNode(GLint index,QVector<GLint>* indices)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    if(nodePointer->leaf)
    {

        if(nodePointer->inside && nodePointer->isSet)
        {
            indices->push_back(nodePointer->index);
        }

        return;
    }

    getInnerNodesForNode(nodePointer->childIndex0,indices);
    getInnerNodesForNode(nodePointer->childIndex1,indices);
    getInnerNodesForNode(nodePointer->childIndex2,indices);
    getInnerNodesForNode(nodePointer->childIndex3,indices);

    getInnerNodesForNode(nodePointer->childIndex4,indices);
    getInnerNodesForNode(nodePointer->childIndex5,indices);
    getInnerNodesForNode(nodePointer->childIndex6,indices);
    getInnerNodesForNode(nodePointer->childIndex7,indices);

}

void ExtendedOctree::getInnerNodesOfLeafSet(QVector<GLint>* indices)
{

    octreeNode* nodePointer;

    for(int i=0;i<this->octreeNodes.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];
        if(nodePointer->inside && nodePointer->isSet && !nodePointer->leaf)
        {
            if(allChildrenAreLeaves(nodePointer->index))
            {
                indices->push_back(nodePointer->index);
            }

        }

    }

}

void ExtendedOctree::getInnerLeaves(QVector<GLint>* indices)
{

    octreeNode* nodePointer;

    for(int i=0;i<this->octreeNodes.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];
        if(nodePointer->inside && nodePointer->isSet && nodePointer->leaf)
        {
           indices->push_back(nodePointer->index);
        }

    }

}

void ExtendedOctree::split(octreeNode* nodePointer){

    if(!nodePointer->leaf || !nodePointer->isSet){
        return;
    }

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

    nodePointer->childIndex1=createNode(x,y,z+newLength,newDepth,addToInterios,index);
    nodePointer->childIndex2=createNode(x,y+newLength,z,newDepth,addToInterios,index);
    nodePointer->childIndex3=createNode(x,y+newLength,z+newLength,newDepth,addToInterios,index);
    nodePointer->childIndex4=createNode(x+newLength,y,z,newDepth,addToInterios,index);
    nodePointer->childIndex5=createNode(x+newLength,y,z+newLength,newDepth,addToInterios,index);
    nodePointer->childIndex6=createNode(x+newLength,y+newLength,z,newDepth,addToInterios,index);
    nodePointer->childIndex7=createNode(x+newLength,y+newLength,z+newLength,newDepth,addToInterios,index);

    nodePointer->leaf = false;

    if(nodePointer->inside)
    {
        this->innerNodeIndices.data()[nodePointer->typeVectorIndex] = -1;
        this->freeIndicesForInnerNodes.push_back(nodePointer->typeVectorIndex);
    }

}

void ExtendedOctree::split(GLint nodeIndex)
{
    octreeNode node = this->octreeNodes.data()[nodeIndex];
    split(&node);
    this->octreeNodes.data()[nodeIndex] = node;
}

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

void ExtendedOctree::merge(octreeNode* nodePointer)
{

    if(!this->allChildrenAreLeaves(nodePointer->index))
    {
        return;
    }

    this->octreeNodes.data()[nodePointer->childIndex0].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex1].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex2].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex3].invalid = true;

    this->octreeNodes.data()[nodePointer->childIndex4].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex5].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex6].invalid = true;
    this->octreeNodes.data()[nodePointer->childIndex7].invalid = true;

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
    this->innerNodeIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerNodes.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex1];
    this->innerNodeIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerNodes.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex2];
    this->innerNodeIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerNodes.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex3];
    this->innerNodeIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerNodes.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex4];
    this->innerNodeIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerNodes.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex5];
    this->innerNodeIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerNodes.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex6];
    this->innerNodeIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerNodes.push_back(node.typeVectorIndex);

    node = this->octreeNodes.data()[nodePointer->childIndex7];
    this->innerNodeIndices.data()[node.typeVectorIndex] = -1;
    this->freeIndicesForInnerNodes.push_back(node.typeVectorIndex);

    nodePointer->childIndex0=-1;
    nodePointer->childIndex1=-1;
    nodePointer->childIndex2=-1;
    nodePointer->childIndex3=-1;
    nodePointer->childIndex4=-1;
    nodePointer->childIndex5=-1;
    nodePointer->childIndex6=-1;
    nodePointer->childIndex7=-1;

    if(!this->freeIndicesForInnerNodes.empty())
    {
        nodePointer->typeVectorIndex = this->freeIndicesForInnerNodes.last();
        this->freeIndicesForInnerNodes.pop_back();
        this->innerNodeIndices.data()[nodePointer->typeVectorIndex] = nodePointer->index;
    }
    else
    {
        nodePointer->typeVectorIndex = this->freeIndicesForInnerNodes.length();
        this->innerNodeIndices.push_back(nodePointer->index);
    }

    nodePointer->leaf = true;

}

void ExtendedOctree::merge(GLint nodeIndex)
{
    octreeNode node = this->octreeNodes.data()[nodeIndex];
    merge(&node);
    this->octreeNodes.data()[nodeIndex] = node;
}

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

    while(nodePointer->cell_length>1)
    {
        GLint index = nodePointer->index;

        this->split(index);
        nodePointer = this->getLeafNodeByCoordinate(x,y,z,index);
    }

    nodePointer->shellNode = true;
    backVec->push_back(nodePointer->index);

}

void ExtendedOctree::increaseShell(GLint loopAmount)
{

    octreeNode node;

    QVector<GLint> tempVec1;
    QVector<GLint> tempVec2;

    QVector<GLint>* frontVec = &tempVec1;
    QVector<GLint>* backVec = &tempVec2;

    for(int i=0;i<this->shellNodeIndices.length();i++)
    {
        node = this->octreeNodes.data()[this->shellNodeIndices.data()[i]];
        frontVec->push_back(node.index);
    }

    for(int i=0;i<loopAmount;i++)
    {

       for (int i=0;i<frontVec->size();i++)
       {

            node = this->octreeNodes.data()[frontVec->data()[i]];
            GLint x = node.x;
            GLint y = node.y;
            GLint z = node.z;

            handleShellNeighbor(x,y,z+1,backVec);
            handleShellNeighbor(x,y,z-1,backVec);
            handleShellNeighbor(x,y+1,z,backVec);
            handleShellNeighbor(x,y-1,z,backVec);
            handleShellNeighbor(x+1,y,z,backVec);
            handleShellNeighbor(x-1,y,z,backVec);

       }

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
