#include "kdtree.h"

using namespace std;

KDTree::KDTree()
{

}

GLint KDTree::setKDTreeHelper(GLint start, GLint end, GLint searchIndex, QVector<GLint>* searchIndices, QVector<kdNode>* nodes, GLint depth)
{

    GLfloat* gvd = this->mesh->getGeometry()->data();
    GLint* svd = searchIndices->data();

    if(end-start<1)
    {
        return -1;
    }

    if(end-start<2)
    {

        kdNode node;
        node.index = svd[start];
        node.leftChildIndex = -1;
        node.rightChildIndex = -1;
        node.depth = depth;
        nodes->push_back(node);
        return nodes->length()-1;

    }

    int diff = end-start;
    int halfLength = (diff+(diff%2))/2+start;
    int halfLengthP1 = halfLength+1;
    int halfLengthM1 = halfLength-1;
    int nextSearchIndex = (searchIndex+1)%3;

    /* sort */

    GLfloat minValue;
    GLint minIndex;

    for (GLint i=start;i<halfLengthP1;i++)
    {

        GLint iv = svd[i]*3+searchIndex;

        minValue = gvd[iv];
        minIndex = i;

        for (GLint j=i+1;j<end;j++)
        {

            GLint jv = svd[j]*3+searchIndex;

            if(minValue>gvd[jv])
            {
                minValue = gvd[jv];
                minIndex = j;
            }

        }

        GLint tempIndex = svd[minIndex];
        svd[minIndex] =  svd[i];
        svd[i] = tempIndex;

    }

    /* */

    kdNode node;
    node.index = svd[halfLengthM1];
    node.leftChildIndex = this->setKDTreeHelper(start,halfLengthM1,nextSearchIndex,searchIndices,nodes,depth+1);
    node.rightChildIndex = this->setKDTreeHelper(halfLength,end,nextSearchIndex,searchIndices,nodes,depth+1);
    node.depth = depth;
    nodes->push_back(node);
    return nodes->length()-1;

}

void KDTree::setKDTree()
{

    QVector<GLfloat>* geometry = this->mesh->getGeometry();
    QVector<GLint>* searchIndices = new QVector<GLint>();
    QVector<kdNode>* nodes = new QVector<kdNode>();

    int length = geometry->length()/3;
    for (GLint i=0;i<length;i++)
    {
        searchIndices->push_back(i);
    }

    this->setKDTreeHelper(0,length,0,searchIndices,nodes,0);

    this->nodes = nodes;

}

inline GLfloat KDTree::getDistanceToPlane(GLint axisIndex, GLint index, GLfloat x, GLfloat y, GLfloat z)
{

    GLfloat* gvd = this->mesh->getGeometry()->data();

    GLfloat value = (axisIndex==0?x:(axisIndex==1?y:z));

    GLint index2 = nodes->at(index).index;

    return fabs(gvd[index2*3+axisIndex]-value);
}

inline GLfloat KDTree::getLength(GLint index, GLfloat x, GLfloat y, GLfloat z)
{

    GLfloat* gvd = this->mesh->getGeometry()->data();

    GLint index2 = nodes->at(index).index;

    GLfloat xv = pow(gvd[index2*3+0]-x,2);
    GLfloat yv = pow(gvd[index2*3+1]-y,2);
    GLfloat zv = pow(gvd[index2*3+2]-z,2);

    return sqrt(xv+yv+zv);
}

GLint KDTree::getNearestNeighborHelper(GLint nodeIndex, float x, float y, float z)
{

    kdNode node = nodes->at(nodeIndex);

    kdNode left;
    if(node.leftChildIndex!=-1){left = nodes->at(node.leftChildIndex);}

    kdNode right;
    if(node.rightChildIndex!=-1){right = nodes->at(node.rightChildIndex);}

    GLfloat nodeDistance = getLength(node.index,x,y,z);
    GLfloat leftDistance = node.leftChildIndex==-1?std::numeric_limits<GLfloat>::max():getLength(left.index,x,y,z);
    GLfloat rightDistance = node.rightChildIndex==-1?std::numeric_limits<GLfloat>::max():getLength(right.index,x,y,z);

    GLfloat dist = getDistanceToPlane(node.depth%3,node.index,x,y,z);

    GLint newLeftIndex = node.leftChildIndex;
    GLint newRightIndex = node.rightChildIndex;

    if(leftDistance<dist)
    {
        newLeftIndex = getNearestNeighborHelper(newLeftIndex,x,y,z);
        leftDistance = getLength(newLeftIndex,x,y,z);
    }
    else
    if(rightDistance<dist)
    {
        newRightIndex = getNearestNeighborHelper(newRightIndex,x,y,z);
        rightDistance = getLength(newRightIndex,x,y,z);
    }
    else
    {

        if(node.leftChildIndex!=-1)
        {
            newLeftIndex = getNearestNeighborHelper(newLeftIndex,x,y,z);
            leftDistance = getLength(newLeftIndex,x,y,z);
        }
        if(node.rightChildIndex!=-1)
        {
            newRightIndex = getNearestNeighborHelper(newRightIndex,x,y,z);
            rightDistance = getLength(newRightIndex,x,y,z);
        }
    }

    if(nodeDistance<leftDistance){

        if(nodeDistance<rightDistance){return nodeIndex;}
        else{return newRightIndex;}
    }
    else{

        if(leftDistance<rightDistance){return newLeftIndex;}
        else{return newRightIndex;}
    }

}

GLint KDTree::getNearestNeighbor(float x, float y, float z)
{

    GLint nodeIndex = getNearestNeighborHelper(nodes->length()-1,x,y,z);
    return nodes->at(nodeIndex).index;
}
