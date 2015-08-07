#include "octree2.h"

using namespace std;

Octree2::Octree2():
mesh(NULL),
startDepth(6),
maxDepth(6),
isDirty(true),
rootNodeIndex(-1)
{

}

/**
 * @brief Octree2::setMesh
 * @param mesh
 */
void Octree2::setMesh(Mesh* mesh)
{
    this->mesh = mesh;
}

void inline Octree2::addTriangle(QVector3D* p1,QVector3D* p2,QVector3D* p3,QVector<GLfloat>* buffer)
{

    buffer->push_back(p1->x());
    buffer->push_back(p1->y());
    buffer->push_back(p1->z());
    buffer->push_back(p2->x());
    buffer->push_back(p2->y());
    buffer->push_back(p2->z());
    buffer->push_back(p3->x());
    buffer->push_back(p3->y());
    buffer->push_back(p3->z());

}

/**
 * @brief Octree2::quantizeSurface
 */
void Octree2::quantizeSurface()
{

    mean = mesh->getMean();

    GLfloat x = mean.x();
    GLfloat y = mean.y();
    GLfloat z = mean.z();

    if(this->mesh == NULL)
    {
        return;
    }

    GLfloat* gvd = this->mesh->getGeometry()->data();
    GLint* ivd = this->mesh->getIndices()->data();
    int length = this->mesh->getIndices()->length();

    GLdouble max_g = 0;

    for (GLint i=0;i<length;i+=3)
    {
        max_g = std::max(max_g,(double)(fabs(gvd[ivd[i+0]*3+0]-x) ));
        max_g = std::max(max_g,(double)(fabs(gvd[ivd[i+0]*3+1]-y) ));
        max_g = std::max(max_g,(double)(fabs(gvd[ivd[i+0]*3+2]-z) ));
        max_g = std::max(max_g,(double)(fabs(gvd[ivd[i+1]*3+0]-x) ));
        max_g = std::max(max_g,(double)(fabs(gvd[ivd[i+1]*3+1]-y) ));
        max_g = std::max(max_g,(double)(fabs(gvd[ivd[i+1]*3+2]-z) ));
        max_g = std::max(max_g,(double)(fabs(gvd[ivd[i+2]*3+0]-x) ));
        max_g = std::max(max_g,(double)(fabs(gvd[ivd[i+2]*3+1]-y) ));
        max_g = std::max(max_g,(double)(fabs(gvd[ivd[i+2]*3+2]-z) ));
    }

    this->max_g = max_g;

    this->raw_voxels.clear();
    this->raw_voxels.reserve( length*pow(4,2) );

    QVector<GLfloat> triangleBuffer1;
    QVector<GLfloat> triangleBuffer2;
    triangleBuffer1.reserve(length*3);
    triangleBuffer2.reserve(length*3);

    QVector<float>* frontBuffer = &triangleBuffer1;
    QVector<float>* backBuffer = &triangleBuffer2;

    GLfloat px = max_g-x;
    GLfloat py = max_g-y;
    GLfloat pz = max_g-z;

    axis_length = pow(2,this->startDepth);
    plane_length = axis_length*axis_length;
    cell_length = (max_g*2)/axis_length;

    for (GLint i=0;i<length;i+=3)
    {
        GLfloat x1 = (gvd[ivd[i+0]*3+0]+px)/cell_length;
        GLfloat y1 = (gvd[ivd[i+0]*3+1]+py)/cell_length;
        GLfloat z1 = (gvd[ivd[i+0]*3+2]+pz)/cell_length;
        GLfloat x2 = (gvd[ivd[i+1]*3+0]+px)/cell_length;
        GLfloat y2 = (gvd[ivd[i+1]*3+1]+py)/cell_length;
        GLfloat z2 = (gvd[ivd[i+1]*3+2]+pz)/cell_length;
        GLfloat x3 = (gvd[ivd[i+2]*3+0]+px)/cell_length;
        GLfloat y3 = (gvd[ivd[i+2]*3+1]+py)/cell_length;
        GLfloat z3 = (gvd[ivd[i+2]*3+2]+pz)/cell_length;

        frontBuffer->push_back(x1);
        frontBuffer->push_back(y1);
        frontBuffer->push_back(z1);
        frontBuffer->push_back(x2);
        frontBuffer->push_back(y2);
        frontBuffer->push_back(z2);
        frontBuffer->push_back(x3);
        frontBuffer->push_back(y3);
        frontBuffer->push_back(z3);
    }

    QVector3D p1,p2,p3;
    GLdouble max_v = 0;

    while(frontBuffer->size()>0)
    {

        while(frontBuffer->size()>0)
        {

            p3.setZ(frontBuffer->last());
            frontBuffer->pop_back();
            p3.setY(frontBuffer->last());
            frontBuffer->pop_back();
            p3.setX(frontBuffer->last());
            frontBuffer->pop_back();

            p2.setZ(frontBuffer->last());
            frontBuffer->pop_back();
            p2.setY(frontBuffer->last());
            frontBuffer->pop_back();
            p2.setX(frontBuffer->last());
            frontBuffer->pop_back();

            p1.setZ(frontBuffer->last());
            frontBuffer->pop_back();
            p1.setY(frontBuffer->last());
            frontBuffer->pop_back();
            p1.setX(frontBuffer->last());
            frontBuffer->pop_back();

            QVector3D mean = (p1+p2+p3)/3;

            max_v = 0.0;

            max_v = std::max(max_v,(double)(fabs(p1.x()-mean.x()) ));
            max_v = std::max(max_v,(double)(fabs(p1.y()-mean.y()) ));
            max_v = std::max(max_v,(double)(fabs(p1.z()-mean.z()) ));
            max_v = std::max(max_v,(double)(fabs(p2.x()-mean.x()) ));
            max_v = std::max(max_v,(double)(fabs(p2.y()-mean.y()) ));
            max_v = std::max(max_v,(double)(fabs(p2.z()-mean.z()) ));
            max_v = std::max(max_v,(double)(fabs(p3.x()-mean.x()) ));
            max_v = std::max(max_v,(double)(fabs(p3.y()-mean.y()) ));
            max_v = std::max(max_v,(double)(fabs(p3.z()-mean.z()) ));

            if(max_v<0.5f)
            {
                this->raw_voxels.push_back(p1.x());
                this->raw_voxels.push_back(p1.y());
                this->raw_voxels.push_back(p1.z());
                this->raw_voxels.push_back(p2.x());
                this->raw_voxels.push_back(p2.y());
                this->raw_voxels.push_back(p2.z());
                this->raw_voxels.push_back(p3.x());
                this->raw_voxels.push_back(p3.y());
                this->raw_voxels.push_back(p3.z());

            }
            else{

                QVector3D newp1 = p1+(p2-p1)/2;
                QVector3D newp2 = p1+(p3-p1)/2;
                QVector3D newp3 = p2+(p3-p2)/2;

                this->addTriangle(&p1,&newp2,&newp1,backBuffer);
                this->addTriangle(&p2,&newp1,&newp3,backBuffer);
                this->addTriangle(&p3,&newp3,&newp2,backBuffer);
                this->addTriangle(&newp1,&newp2,&newp3,backBuffer);

            }
        }

        if(triangleBuffer1.size()>0)
        {
            frontBuffer = &triangleBuffer1;
            backBuffer = &triangleBuffer2;
        }
        else
        {
            frontBuffer = &triangleBuffer2;
            backBuffer = &triangleBuffer1;
        }

    }
}

/*
void Octree2::setupOctree_pcl()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    GLint length = this->raw_voxels.length();

    cloud->width = length;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (GLint i = 0; i < length ; i+=3)
    {

      cloud->points[i].x = this->raw_voxels.at(i+0);
      cloud->points[i].y = this->raw_voxels.at(i+1);
      cloud->points[i].z = this->raw_voxels.at(i+2);
    }

    float resolution_octree = 0.5f;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution_octree);


    octree.setInputCloud (cloud);
    octree.defineBoundingBox ();
    octree.addPointsFromInputCloud ();

}
*/

octreeNode2* Octree2::getLeafNodeByCoordinate(GLint x, GLint y, GLint z)
{
    return getLeafNodeByCoordinate(x,y,z,this->rootNodeIndex);
}

octreeNode2* Octree2::getLeafNodeByCoordinate(GLint x, GLint y, GLint z, GLint startNodeIndex)
{
    if(x<0 || y<0 || z<0)
    {
        return NULL;
    }
    if(x>=this->axis_length || y>=this->axis_length || z>=this->axis_length)
    {
        return NULL;
    }

    return getLeafNodeByCoordinateHelper(x,y,z,startNodeIndex);
}

octreeNode2* Octree2::getLeafNodeByCoordinateHelper(GLint x, GLint y, GLint z, GLint nodeIndex)
{
    octreeNode2* nodePointer = &this->octreeNodes.data()[nodeIndex];

    if(nodePointer->leaf)
    {
        return nodePointer;
    }

    GLint xarea = (nodePointer->x+(nodePointer->cell_length>>1))<=x?1:0;
    GLint yarea = (nodePointer->y+(nodePointer->cell_length>>1))<=y?1:0;
    GLint zarea = (nodePointer->z+(nodePointer->cell_length>>1))<=z?1:0;

    GLint code = zarea*1+yarea*2+xarea*4;

    switch(code){
    case 0:
        return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex0);
    case 1:
        return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex1);
    case 2:
        return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex2);
    case 3:
        return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex3);
    case 4:
        return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex4);
    case 5:
        return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex5);
    case 6:
        return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex6);
    case 7:
        return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex7);
    }

    return NULL;
}

bool inline Octree2::testNeighborNode(GLint x, GLint y, GLint z, octreeNode2* nodePointer)
{

    octreeNode2* neighborNodePointer = this->getLeafNodeByCoordinate(x,y,z);
    if(neighborNodePointer->isSet && !neighborNodePointer->inside)
    {
        nodePointer->isSet = true;
        nodePointer->inside = false;

        return true;
    }

    return false;
}

GLint Octree2::createNode(GLint x,GLint y,GLint z,GLint depth,bool addToInteriors,GLint parentIndex){

    octreeNode2 node;

    node.shellNode = false;
    node.leaf = true;

    node.isSet = true;
    node.inside = addToInteriors;

    node.parentIndex = parentIndex;

    GLfloat xf = x*this->cell_length-(max_g-this->mean.x());
    GLfloat yf = y*this->cell_length-(max_g-this->mean.y());
    GLfloat zf = z*this->cell_length-(max_g-this->mean.z());

    GLfloat cell_length = this->cell_length*(pow(2,this->maxDepth-depth));

    node.p0.setX(xf);               node.p0.setY(yf);               node.p0.setZ(zf);
    node.p1.setX(xf);               node.p1.setY(yf);               node.p1.setZ(zf+cell_length);
    node.p2.setX(xf);               node.p2.setY(yf+cell_length);   node.p2.setZ(zf);
    node.p3.setX(xf);               node.p3.setY(yf+cell_length);   node.p3.setZ(zf+cell_length);
    node.p4.setX(xf+cell_length);   node.p4.setY(yf);               node.p4.setZ(zf);
    node.p5.setX(xf+cell_length);   node.p5.setY(yf);               node.p5.setZ(zf+cell_length);
    node.p6.setX(xf+cell_length);   node.p6.setY(yf+cell_length);   node.p6.setZ(zf);
    node.p7.setX(xf+cell_length);   node.p7.setY(yf+cell_length);   node.p7.setZ(zf+cell_length);

    node.x = x;
    node.y = y;
    node.z = z;
    node.cell_length = pow(2,this->maxDepth-depth);

    node.nodeDepth = depth;

    /* add node in the data structures (avoid waste of storage by using voids in the vectors) */
    if(!this->freeIndicesForAllNodes.empty())
    {
        node.index = this->freeIndicesForAllNodes.last();
        this->freeIndicesForAllNodes.pop_back();
        this->octreeNodes.data()[node.index] = node;
    }
    else
    {
        node.index = this->octreeNodes.length();
        this->octreeNodes.push_back(node);
    }

    if(addToInteriors)
    {
        if(!this->freeIndicesForInnerNodes.empty())
        {
            node.typeVectorIndex = this->freeIndicesForInnerNodes.last();
            this->freeIndicesForInnerNodes.pop_back();
            this->innerNodeIndices.data()[node.typeVectorIndex] = node.index;
        }
        else
        {
            node.typeVectorIndex = this->freeIndicesForInnerNodes.length();
            this->innerNodeIndices.push_back(node.index);
        }
    }

    this->octreeNodes.data()[node.index] = node;

    return node.index;
}

void Octree2::split(octreeNode2* nodePointer){

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

void Octree2::split(GLint nodeIndex)
{
    octreeNode2 node = this->octreeNodes.data()[nodeIndex];
    split(&node);
    this->octreeNodes.data()[nodeIndex] = node;
}

bool Octree2::allChildrenAreLeaves(GLint nodeIndex)
{
    if(nodeIndex<0)
    {
        return false;
    }

    octreeNode2* nodePointer = &this->octreeNodes.data()[nodeIndex];

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

void Octree2::merge(octreeNode2* nodePointer)
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

    octreeNode2 node;

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

void Octree2::merge(GLint nodeIndex)
{
    octreeNode2 node = this->octreeNodes.data()[nodeIndex];
    merge(&node);
    this->octreeNodes.data()[nodeIndex] = node;
}

void inline Octree2::handleShellNeighbor(GLint x, GLint y, GLint z, QVector<GLint>* backVec)
{

    octreeNode2* nodePointer = this->getLeafNodeByCoordinate(x,y,z);

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

void Octree2::increaseShell(GLint loopAmount)
{

    octreeNode2 node;

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

GLint inline Octree2::handleHashItem(GLint vertexIndex,QVector3D point)
{

    if (geometryMap.contains(vertexIndex))
    {
        return geometryMap.value(vertexIndex).index;
    }

    point.setX(point.x()*this->cell_length-(max_g-this->mean.x()));
    point.setY(point.y()*this->cell_length-(max_g-this->mean.y()));
    point.setZ(point.z()*this->cell_length-(max_g-this->mean.z()));

    hashItem2 item;

    item.vertex = point;
    item.index = geometryMap.size();

    geometryMap.insert(vertexIndex, item);

    return item.index;

}

//QVector<Mesh*> getQ

bool inline Octree2::addTriangle(GLint x, GLint y, GLint z)
{
    return this->getLeafNodeByCoordinate(x,y,z)->shellNode;
}

void Octree2::createTriangle(GLint x, GLint y, GLint z, GLint code)
{

    GLint pl = this->plane_length;
    GLint al = this->axis_length;

    GLint index0 = -1;
    GLint index1 = -1;
    GLint index2 = -1;
    GLint index3 = -1;

    QVector3D vec;

    bool flip = false;

    switch(code){

    case 0:

        vec.setX(x);
        vec.setY(y);
        vec.setZ(z);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x+1);
        vec.setY(y);
        vec.setZ(z);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x+1);
        vec.setY(y+1);
        vec.setZ(z);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x);
        vec.setY(y+1);
        vec.setZ(z);
        index3 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        flip = true;

        break;

    case 1:

        vec.setX(x);
        vec.setY(y);
        vec.setZ(z+1);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x+1);
        vec.setY(y);
        vec.setZ(z+1);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x+1);
        vec.setY(y+1);
        vec.setZ(z+1);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x);
        vec.setY(y+1);
        vec.setZ(z+1);
        index3 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        break;

    case 2:

        vec.setX(x);
        vec.setY(y);
        vec.setZ(z);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x);
        vec.setY(y);
        vec.setZ(z+1);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x+1);
        vec.setY(y);
        vec.setZ(z+1);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x+1);
        vec.setY(y);
        vec.setZ(z);
        index3 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        flip = true;

        break;

    case 3:

        vec.setX(x);
        vec.setY(y+1);
        vec.setZ(z);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x);
        vec.setY(y+1);
        vec.setZ(z+1);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x+1);
        vec.setY(y+1);
        vec.setZ(z+1);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x+1);
        vec.setY(y+1);
        vec.setZ(z);
        index3 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        break;

    case 4:

        vec.setX(x);
        vec.setY(y);
        vec.setZ(z);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x);
        vec.setY(y);
        vec.setZ(z+1);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x);
        vec.setY(y+1);
        vec.setZ(z+1);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x);
        vec.setY(y+1);
        vec.setZ(z);
        index3 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        break;

    case 5:

        vec.setX(x+1);
        vec.setY(y);
        vec.setZ(z);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x+1);
        vec.setY(y);
        vec.setZ(z+1);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x+1);
        vec.setY(y+1);
        vec.setZ(z+1);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        vec.setX(x+1);
        vec.setY(y+1);
        vec.setZ(z);
        index3 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        flip = true;

        break;
    }

    if(flip)
    {

        indices.push_back(index2);
        indices.push_back(index1);
        indices.push_back(index0);

        indices.push_back(index3);
        indices.push_back(index2);
        indices.push_back(index0);

    }
    else
    {

        indices.push_back(index0);
        indices.push_back(index1);
        indices.push_back(index2);

        indices.push_back(index0);
        indices.push_back(index2);
        indices.push_back(index3);

    }


    return;
}

void Octree2::setInnerNodeIndices()
{

    octreeNode2* nodePointer;

    GLint length = this->octreeNodes.length();

    this->innerNodeIndices.clear();
    this->freeIndicesForInnerNodes.clear();

    for(int i=1;i<length;i++)
    {

         nodePointer = &this->octreeNodes.data()[i];
         if(!nodePointer->leaf || !nodePointer->isSet || !nodePointer->inside || nodePointer->shellNode)
         {
           continue;
         }

         innerNodeIndices.push_back(i);
    }

    return;

}

void Octree2::setShellNodeIndices()
{

    octreeNode2* nodePointer;

    GLint length = this->octreeNodes.length();

    this->shellNodeIndices.clear();

    for(int i=1;i<length;i++)
    {

         nodePointer = &this->octreeNodes.data()[i];
         if(!nodePointer->leaf || !nodePointer->shellNode)
         {
           continue;
         }

         shellNodeIndices.push_back(i);
    }

    return;

}

void Octree2::createInnerSurface()
{

    geometry.clear();
    indices.clear();
    geometryMap.clear();

    GLint length = innerNodeIndices.length();

    for(int i=0;i<length;i++)
    {
        octreeNode2* nodePointer = &this->octreeNodes.data()[innerNodeIndices.at(i)];

        GLint length2 = nodePointer->cell_length;

        GLint x = nodePointer->x;
        GLint y = nodePointer->y;
        GLint z = nodePointer->z;

        for(int j=0;j<length2;j++)
        {
            for(int k=0;k<length2;k++)
            {                

                 if(addTriangle(x+j,y+k,z-1)){
                    createTriangle(x+j,y+k,z,0);
                 }
                 if(addTriangle(x+j,y+k,z+length2)){
                    createTriangle(x+j,y+k,z+length2-1,1);
                 }

                 if(addTriangle(x+j,y-1,z+k)){
                    createTriangle(x+j,y,z+k,2);
                 }
                 if(addTriangle(x+j,y+length2,z+k)){
                    createTriangle(x+j,y+length2-1,z+k,3);
                 }

                 if(addTriangle(x-1,y+j,z+k)){
                    createTriangle(x,y+j,z+k,4);
                 }
                 if(addTriangle(x+length2,y+j,z+k)){
                    createTriangle(x+length2-1,y+j,z+k,5);
                 }

            }
        }
    }

    QHash<GLint, hashItem2>::iterator i;

    for (i = geometryMap.begin(); i != geometryMap.end(); ++i)
    {

        QVector3D vec = i.value().vertex;
        geometry.push_back(vec.x());
        geometry.push_back(vec.y());
        geometry.push_back(vec.z());

    }

    GLfloat* data = geometry.data();

    for (i = geometryMap.begin(); i != geometryMap.end(); ++i)
    {

        QVector3D vec = i.value().vertex;
        data[i.value().index*3+0] = vec.x();
        data[i.value().index*3+1] = vec.y();
        data[i.value().index*3+2] = vec.z();

    }

    return;
}

void Octree2::setOuterNodes()
{
    octreeNode2* nodePointer;


    QHash<GLint, octreeNode2*> tempMap1;
    QHash<GLint, octreeNode2*> tempMap2;

    QHash<GLint, octreeNode2*>* frontMap = &tempMap1;
    QHash<GLint, octreeNode2*>* backMap = &tempMap2;

    for(int i=0;i<this->axis_length;i++)
    {
        for(int j=0;j<this->axis_length;j++)
        {


            nodePointer = this->getLeafNodeByCoordinate(i,j,0);
            if(nodePointer !=NULL && !nodePointer->shellNode)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }
            nodePointer = this->getLeafNodeByCoordinate(i,j,this->axis_length-1);
            if(nodePointer !=NULL && !nodePointer->shellNode)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }
            nodePointer = this->getLeafNodeByCoordinate(i,0,j);
            if(nodePointer !=NULL && !nodePointer->shellNode)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }
            nodePointer = this->getLeafNodeByCoordinate(i,this->axis_length-1,j);
            if(nodePointer !=NULL && !nodePointer->shellNode)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }
            nodePointer = this->getLeafNodeByCoordinate(0,i,j);
            if(nodePointer !=NULL && !nodePointer->shellNode)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }
            nodePointer = this->getLeafNodeByCoordinate(this->axis_length-1,i,j);
            if(nodePointer !=NULL && !nodePointer->shellNode)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }

        }
    }

    while(frontMap->size()>0)
    {

        QHash<GLint, octreeNode2*>::iterator i;

        for (i = frontMap->begin(); i != frontMap->end(); ++i)
        {

            octreeNode2* nodePointer = i.value();
            nodePointer->isSet = true;
            nodePointer->inside = false;

        }

        for (i = frontMap->begin(); i != frontMap->end(); ++i)
        {

            GLint length2 = i.value()->cell_length;

            GLint x = i.value()->x;
            GLint y = i.value()->y;
            GLint z = i.value()->z;

            for(int j=0;j<length2;j++)
            {
                for(int k=0;k<length2;k++)
                {

                    nodePointer = this->getLeafNodeByCoordinate(x+j,y+k,z-1);
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->shellNode)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }
                    nodePointer = this->getLeafNodeByCoordinate(x+j,y+k,z+length2);
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->shellNode)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }


                    nodePointer = this->getLeafNodeByCoordinate(x+j,y-1,z+k);
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->shellNode)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }
                    nodePointer = this->getLeafNodeByCoordinate(x+j,y+length2,z+k);
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->shellNode)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }


                    nodePointer = this->getLeafNodeByCoordinate(x-1,y+j,z+k);
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->shellNode)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }
                    nodePointer = this->getLeafNodeByCoordinate(x+length2,y+j,z+k);
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->shellNode)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }

                }
            }
        }

        frontMap->clear();

        if(tempMap1.size()>0){
            frontMap = &tempMap1;
            backMap = &tempMap2;
        }
        else
        if(tempMap2.size()>0){
            frontMap = &tempMap2;
            backMap = &tempMap1;
        }

    }

    return;
}

void Octree2::setInnerNodes()
{

    octreeNode2* nodePointer;

    GLint length = this->octreeNodes.length();

    for(int i=1;i<length;i++)
    {

         nodePointer = &this->octreeNodes.data()[i];
         if(!nodePointer->leaf || nodePointer->isSet || nodePointer->shellNode)
         {
           continue;
         }

         nodePointer->isSet = true;
         nodePointer->inside = true;
         nodePointer->typeVectorIndex = innerNodeIndices.size();

    }

}

void Octree2::getInnerCubes(QVector<cubeObject>* vec)
{

    QVector<GLint> innerNodeIndices;

    octreeNode2* nodePointer;

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

        Mesh* mesh = new Mesh(geometry,indices);
        GLfloat beta = nodePointer->beta;

        cubeObject obj;
        obj.mesh = mesh;
        obj.beta = beta;

        vec->push_back(obj);

    }

}

void Octree2::splitAndMerge(GLfloat epsilon)
{

    GLfloat oneMinusEpsilon = 1-epsilon;

    octreeNode2* nodePointer;

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
    GLfloat beta;

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

void Octree2::getNodesOfDepth(GLint depth,QVector<GLint>* indices)
{

    octreeNode2* nodePointer;

    for(int i=0;i<this->octreeNodes.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];
        if(nodePointer->nodeDepth>=depth)
        {
            indices->push_back(nodePointer->index);
        }

    }

}

void Octree2::getInnerNodesForNode(GLint index,QVector<GLint>* indices)
{

    octreeNode2* nodePointer = &this->octreeNodes.data()[index];

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

void Octree2::getInnerNodesOfLeafSet(QVector<GLint>* indices)
{

    octreeNode2* nodePointer;

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

void Octree2::getInnerLeaves(QVector<GLint>* indices)
{

    octreeNode2* nodePointer;

    for(int i=0;i<this->octreeNodes.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];
        if(nodePointer->inside && nodePointer->isSet && nodePointer->leaf)
        {
           indices->push_back(nodePointer->index);
        }

    }

}

bool compare (const QVector3D & a, const QVector3D & b)
{
  if(a.x()<b.x()){return true;}
  if(a.x()>b.x()){return false;}

  if(a.y()<b.y()){return true;}
  if(a.y()>b.y()){return false;}

  if(a.z()<b.z()){return true;}
  if(a.z()>b.z()){return false;}

  return false;
}

void Octree2::setupVectors()
{

    QVector<QVector3D> tempVoxels;
    tempVoxels.reserve(this->raw_voxels.length()/3);

    this->voxels.clear();
    this->voxels.reserve(this->raw_voxels.length()/3);

    GLint length;

    length = this->raw_voxels.length();

    for (GLint i = 0; i < length ; i+=3)
    {

      QVector3D point;

      point.setX(this->raw_voxels.at(i+0));
      point.setY(this->raw_voxels.at(i+1));
      point.setZ(this->raw_voxels.at(i+2));

      voxels.push_back(point);
    }

}

GLint Octree2::sortHalf(GLint start,GLint end,GLint coor, GLint prior)
{

    GLint tempEnd = end;

    QVector3D* voxelData = this->voxels.data();

    for (GLint i = start; i < tempEnd ; i++)
    {
        QVector3D p = voxelData[i];

        bool swap = false;
        swap = (coor == 0) ? p.x()>=prior:swap;
        swap = (coor == 1) ? p.y()>=prior:swap;
        swap = (coor == 2) ? p.z()>=prior:swap;

        if(swap)
        {

            std::swap(voxelData[i],voxelData[tempEnd-1]);

            i -= 1;
            tempEnd -= 1;
        }
    }

    return tempEnd;
}

void Octree2::setupOctree(){

    freeIndicesForAllNodes.clear();
    freeIndicesForInnerNodes.clear();

    this->shellNodeIndices.clear();

    this->rootNodeIndex = setupOctreeHelper(0,0,this->voxels.length(),0,0,0);

    this->voxels.clear();

}

Mesh* Octree2::getPointMesh()
{

    if(this->voxels.length()>10000)
    {
        return NULL;
    }

    QVector<GLfloat>* geometry = new QVector<GLfloat>();
    QVector<GLint>* indices = new QVector<GLint>();

    geometry->reserve(this->voxels.length()*3);
    indices->reserve(this->voxels.length());

    for (int i=0;i<this->voxels.length();++i)
    {
        geometry->push_back(this->voxels.at(i).x()*this->cell_length-(max_g-this->mean.x()));
        geometry->push_back(this->voxels.at(i).y()*this->cell_length-(max_g-this->mean.y()));
        geometry->push_back(this->voxels.at(i).z()*this->cell_length-(max_g-this->mean.z()));
        indices->push_back(i*3+0);

        geometry->push_back((this->voxels.at(i).x()+0.1)*this->cell_length-(max_g-this->mean.x()));
        geometry->push_back(this->voxels.at(i).y()*this->cell_length-(max_g-this->mean.y()));
        geometry->push_back(this->voxels.at(i).z()*this->cell_length-(max_g-this->mean.z()));
        indices->push_back(i*3+1);

        geometry->push_back(this->voxels.at(i).x()*this->cell_length-(max_g-this->mean.x()));
        geometry->push_back((this->voxels.at(i).y()+0.1)*this->cell_length-(max_g-this->mean.y()));
        geometry->push_back(this->voxels.at(i).z()*this->cell_length-(max_g-this->mean.z()));
        indices->push_back(i*3+2);
    }

    Mesh* mesh = new Mesh(geometry,indices);

    return mesh;

}


GLint Octree2::setupOctreeHelper(GLint depth,GLint start, GLint end, GLint x, GLint y, GLint z){

    if(this->startDepth<=depth || end-start<1 ){

        octreeNode2 node;

        node.shellNode = this->startDepth<=depth && end-start>0;
        node.leaf = true;

        GLfloat xf = x*this->cell_length-(max_g-this->mean.x());
        GLfloat yf = y*this->cell_length-(max_g-this->mean.y());
        GLfloat zf = z*this->cell_length-(max_g-this->mean.z());

        GLfloat cell_length = this->cell_length*(pow(2,this->startDepth-depth));

        node.p0.setX(xf);               node.p0.setY(yf);               node.p0.setZ(zf);
        node.p1.setX(xf);               node.p1.setY(yf);               node.p1.setZ(zf+cell_length);
        node.p2.setX(xf);               node.p2.setY(yf+cell_length);   node.p2.setZ(zf);
        node.p3.setX(xf);               node.p3.setY(yf+cell_length);   node.p3.setZ(zf+cell_length);
        node.p4.setX(xf+cell_length);   node.p4.setY(yf);               node.p4.setZ(zf);
        node.p5.setX(xf+cell_length);   node.p5.setY(yf);               node.p5.setZ(zf+cell_length);
        node.p6.setX(xf+cell_length);   node.p6.setY(yf+cell_length);   node.p6.setZ(zf);
        node.p7.setX(xf+cell_length);   node.p7.setY(yf+cell_length);   node.p7.setZ(zf+cell_length);

        node.x = x;
        node.y = y;
        node.z = z;

        node.cell_length = pow(2,this->startDepth-depth);

        node.index = octreeNodes.length();
        node.nodeDepth = depth;

        octreeNodes.push_back(node);

        return node.index;
    }

    GLint cell_length_int = pow(2,this->startDepth-depth);
    GLint prior = cell_length_int >> 1;

    GLint startEndY = sortHalf(start,end,0,x+prior);

    GLint startEndZ1 = sortHalf(start,      startEndY,  1,y+prior);
    GLint startEndZ2 = sortHalf(startEndY,  end,        1,y+prior);

    GLint startEndTemp1 = sortHalf(start,       startEndZ1  ,2,z+prior);
    GLint startEndTemp2 = sortHalf(startEndZ1,  startEndY   ,2,z+prior);
    GLint startEndTemp3 = sortHalf(startEndY,   startEndZ2  ,2,z+prior);
    GLint startEndTemp4 = sortHalf(startEndZ2,  end         ,2,z+prior);

    GLint newDepth = depth+1;

    octreeNode2 node;
    node.shellNode = true;
    node.leaf = false;

    GLint half_cell_length = cell_length_int >> 1;

    node.childIndex0 = setupOctreeHelper(newDepth,start         ,startEndTemp1 ,x,                 y,                 z                  );
    node.childIndex1 = setupOctreeHelper(newDepth,startEndTemp1 ,startEndZ1    ,x,                 y,                 z+half_cell_length );
    node.childIndex2 = setupOctreeHelper(newDepth,startEndZ1    ,startEndTemp2 ,x,                 y+half_cell_length,z                  );
    node.childIndex3 = setupOctreeHelper(newDepth,startEndTemp2 ,startEndY     ,x,                 y+half_cell_length,z+half_cell_length );

    node.childIndex4 = setupOctreeHelper(newDepth,startEndY     ,startEndTemp3 ,x+half_cell_length,y,                 z                  );
    node.childIndex5 = setupOctreeHelper(newDepth,startEndTemp3 ,startEndZ2    ,x+half_cell_length,y,                 z+half_cell_length );
    node.childIndex6 = setupOctreeHelper(newDepth,startEndZ2    ,startEndTemp4 ,x+half_cell_length,y+half_cell_length,z                  );
    node.childIndex7 = setupOctreeHelper(newDepth,startEndTemp4 ,end           ,x+half_cell_length,y+half_cell_length,z+half_cell_length );

    GLfloat xf = x*this->cell_length-(max_g-this->mean.x());
    GLfloat yf = y*this->cell_length-(max_g-this->mean.y());
    GLfloat zf = z*this->cell_length-(max_g-this->mean.z());

    GLfloat cell_length = this->cell_length*(pow(2,this->startDepth-depth));

    node.p0.setX(xf);               node.p0.setY(yf);               node.p0.setZ(zf);
    node.p1.setX(xf);               node.p1.setY(yf);               node.p1.setZ(zf+cell_length);
    node.p2.setX(xf);               node.p2.setY(yf+cell_length);   node.p2.setZ(zf);
    node.p3.setX(xf);               node.p3.setY(yf+cell_length);   node.p3.setZ(zf+cell_length);

    node.p4.setX(xf+cell_length);   node.p4.setY(yf);               node.p4.setZ(zf);
    node.p5.setX(xf+cell_length);   node.p5.setY(yf);               node.p5.setZ(zf+cell_length);
    node.p6.setX(xf+cell_length);   node.p6.setY(yf+cell_length);   node.p6.setZ(zf);
    node.p7.setX(xf+cell_length);   node.p7.setY(yf+cell_length);   node.p7.setZ(zf+cell_length);

    node.x = x;
    node.y = y;
    node.z = z;
    node.cell_length = cell_length_int;

    node.index = octreeNodes.length();
    node.nodeDepth = depth;

    octreeNodes.data()[node.childIndex0].parentIndex = node.index;
    octreeNodes.data()[node.childIndex1].parentIndex = node.index;
    octreeNodes.data()[node.childIndex2].parentIndex = node.index;
    octreeNodes.data()[node.childIndex3].parentIndex = node.index;
    octreeNodes.data()[node.childIndex4].parentIndex = node.index;
    octreeNodes.data()[node.childIndex5].parentIndex = node.index;
    octreeNodes.data()[node.childIndex6].parentIndex = node.index;
    octreeNodes.data()[node.childIndex7].parentIndex = node.index;

    octreeNodes.push_back(node);

    return node.index;
}

void Octree2::setStartDepth(GLint depth){

    GLint minDepthLimit = 3;
    GLint maxDepthLimit = 9;

    depth = depth<maxDepthLimit?depth:maxDepthLimit;
    depth = depth>minDepthLimit?depth:minDepthLimit;
    depth = depth<=maxDepth?depth:maxDepth;

    this->startDepth = depth;
}

void Octree2::setMaxDepth(GLint depth){

    GLint minDepthLimit = 3;
    GLint maxDepthLimit = 9;

    depth = depth<maxDepthLimit?depth:maxDepthLimit;
    depth = depth>minDepthLimit?depth:minDepthLimit;
    depth = depth>=startDepth?depth:startDepth;

    this->maxDepth = depth;
}

void Octree2::adjustMaxDepth()
{
    GLint diff = this->maxDepth-startDepth;

    this->cell_length = this->cell_length/pow(2,diff);

    this->axis_length = this->axis_length<<diff;
    this->plane_length = this->axis_length*this->axis_length;

    octreeNode2* nodePointer;
    for(int i=0;i<this->octreeNodes.length();i++){
        nodePointer = &this->octreeNodes.data()[i];

        nodePointer->x = nodePointer->x<<diff;
        nodePointer->y = nodePointer->y<<diff;
        nodePointer->z = nodePointer->z<<diff;

        nodePointer->cell_length = nodePointer->cell_length<<diff;
    }

}

Mesh* Octree2::getMesh()
{

    QVector<GLfloat>* geometry = new QVector<GLfloat>();
    QVector<GLint>* indices = new QVector<GLint>();

    geometry->reserve(this->geometry.length());
    indices->reserve(this->indices.length());

    for (int i=0;i<this->geometry.length();++i)
    {
        geometry->push_back(this->geometry.at(i));
    }
    for (int i=0;i<this->indices.length();++i)
    {
        indices->push_back(this->indices.at(i));
    }

    Mesh* mesh = new Mesh(geometry,indices);

    return mesh;
}

void Octree2::render(QGLShaderProgram* shader)
{


    if(this->octreeNodes.length()<0){
        return;
    }


    GLenum primitive = GL_LINES;

    /* if needed needed variables are set for painting */
    if (isDirty) {
        QVector<GLfloat> buffer;

        buffer.reserve( this->octreeNodes.length() * 8 * 3 * sizeof(GLfloat));

        cubeLineIndices.clear();
        GLint index = 0;

        GLint length = this->octreeNodes.length();

        GLint viewIndex1 = (this->axis_length>>1);
        GLint viewIndex2 = (this->axis_length>>1)+1;

        for (int i = 0; i < length; i++) {

            octreeNode2 node = this->octreeNodes.at(i);

            if(!node.leaf || node.invalid)
            {
                continue;
            }


            if( !node.inside || node.shellNode )
            {
                continue;
            }

            /*
            if(node.x<viewIndex1 || node.x>viewIndex2)
            {
                continue;
            }
            */

            /* add the vertices the lines */
            buffer.push_back(node.p0.x());buffer.push_back(node.p0.y());buffer.push_back(node.p0.z());
            buffer.push_back(node.p1.x());buffer.push_back(node.p1.y());buffer.push_back(node.p1.z());
            buffer.push_back(node.p2.x());buffer.push_back(node.p2.y());buffer.push_back(node.p2.z());
            buffer.push_back(node.p3.x());buffer.push_back(node.p3.y());buffer.push_back(node.p3.z());

            buffer.push_back(node.p4.x());buffer.push_back(node.p4.y());buffer.push_back(node.p4.z());
            buffer.push_back(node.p5.x());buffer.push_back(node.p5.y());buffer.push_back(node.p5.z());
            buffer.push_back(node.p6.x());buffer.push_back(node.p6.y());buffer.push_back(node.p6.z());
            buffer.push_back(node.p7.x());buffer.push_back(node.p7.y());buffer.push_back(node.p7.z());

            /* add the indices for the lines */
            cubeLineIndices.push_back(index+0);cubeLineIndices.push_back(index+1);
            cubeLineIndices.push_back(index+2);cubeLineIndices.push_back(index+3);
            cubeLineIndices.push_back(index+4);cubeLineIndices.push_back(index+5);
            cubeLineIndices.push_back(index+6);cubeLineIndices.push_back(index+7);

            cubeLineIndices.push_back(index+0);cubeLineIndices.push_back(index+2);
            cubeLineIndices.push_back(index+4);cubeLineIndices.push_back(index+6);
            cubeLineIndices.push_back(index+0);cubeLineIndices.push_back(index+4);
            cubeLineIndices.push_back(index+2);cubeLineIndices.push_back(index+6);

            cubeLineIndices.push_back(index+1);cubeLineIndices.push_back(index+3);
            cubeLineIndices.push_back(index+5);cubeLineIndices.push_back(index+7);
            cubeLineIndices.push_back(index+1);cubeLineIndices.push_back(index+5);
            cubeLineIndices.push_back(index+3);cubeLineIndices.push_back(index+7);

            index +=8;
        }

        /* create vertex buffer */
        vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        vbo->create();
        vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        vbo->bind();
        vbo->allocate(index * 3 * sizeof(GLfloat));
        vbo->write(0, buffer.constData(), buffer.size() * sizeof(GLfloat));
        vbo->release();

        /* create index buffer */
        ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        ibo->create();
        ibo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        ibo->bind();
        ibo->allocate(cubeLineIndices.size() * sizeof(GLint));
        ibo->write(0, cubeLineIndices.constData(), cubeLineIndices.size() * sizeof(GLint));
        ibo->release();

        isDirty = false;

    }

    GLint stride = sizeof(GLfloat) * (3);

    /* use vertex buffer */
    vbo->bind();
    shader->setAttributeBuffer("geometry", GL_FLOAT, 0, 3, stride);
    shader->enableAttributeArray("geometry");
    vbo->release();

    /* use index buffer */
    ibo->bind();
    glDrawElements(primitive, cubeLineIndices.length(), GL_UNSIGNED_INT, (void*) 0);
    ibo->release();

    shader->disableAttributeArray("geometry");

}
