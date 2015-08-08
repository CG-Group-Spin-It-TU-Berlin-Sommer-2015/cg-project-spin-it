#include "basicoctree.h"

using namespace std;
using namespace octree;

BasicOctree::BasicOctree():
isDirty(true),
mesh(NULL),
startDepth(6),
maxDepth(6),
rootNodeIndex(-1)
{

}

void BasicOctree::setMesh(Mesh* mesh)
{
    this->mesh = mesh;
}

void inline BasicOctree::addTriangle(QVector3D* p1,QVector3D* p2,QVector3D* p3,QVector<GLfloat>* buffer)
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

void BasicOctree::quantizeSurface()
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

            p3.setZ(frontBuffer->last());frontBuffer->pop_back();
            p3.setY(frontBuffer->last());frontBuffer->pop_back();
            p3.setX(frontBuffer->last());frontBuffer->pop_back();

            p2.setZ(frontBuffer->last());frontBuffer->pop_back();
            p2.setY(frontBuffer->last());frontBuffer->pop_back();
            p2.setX(frontBuffer->last());frontBuffer->pop_back();

            p1.setZ(frontBuffer->last());frontBuffer->pop_back();
            p1.setY(frontBuffer->last());frontBuffer->pop_back();
            p1.setX(frontBuffer->last());frontBuffer->pop_back();

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

octreeNode* BasicOctree::getLeafNodeByCoordinate(GLint x, GLint y, GLint z)
{
    return getLeafNodeByCoordinate(x,y,z,this->rootNodeIndex);
}

octreeNode* BasicOctree::getLeafNodeByCoordinate(GLint x, GLint y, GLint z, GLint startNodeIndex)
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

octreeNode* BasicOctree::getLeafNodeByCoordinateHelper(GLint x, GLint y, GLint z, GLint nodeIndex)
{
    octreeNode* nodePointer = &this->octreeNodes.data()[nodeIndex];

    if(nodePointer->leaf)
    {
        return nodePointer;
    }

    GLint xarea = (nodePointer->x+(nodePointer->cell_length>>1))<=x?1:0;
    GLint yarea = (nodePointer->y+(nodePointer->cell_length>>1))<=y?1:0;
    GLint zarea = (nodePointer->z+(nodePointer->cell_length>>1))<=z?1:0;

    GLint code = zarea*1+yarea*2+xarea*4;

    switch(code){
        case 0: return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex0);
        case 1: return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex1);
        case 2: return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex2);
        case 3: return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex3);
        case 4: return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex4);
        case 5: return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex5);
        case 6: return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex6);
        case 7: return getLeafNodeByCoordinateHelper(x,y,z,nodePointer->childIndex7);
    }

    return NULL;
}

bool inline BasicOctree::testNeighborNode(GLint x, GLint y, GLint z, octreeNode* nodePointer)
{

    octreeNode* neighborNodePointer = this->getLeafNodeByCoordinate(x,y,z);
    if(neighborNodePointer->isSet && !neighborNodePointer->inside)
    {
        nodePointer->isSet = true;
        nodePointer->inside = false;

        return true;
    }

    return false;
}

GLint BasicOctree::createNode(GLint x,GLint y,GLint z,GLint depth,bool addToInteriors,GLint parentIndex){

    GLfloat xf = x*this->cell_length-(max_g-this->mean.x());
    GLfloat yf = y*this->cell_length-(max_g-this->mean.y());
    GLfloat zf = z*this->cell_length-(max_g-this->mean.z());
    GLfloat cell_length = this->cell_length*(pow(2,this->maxDepth-depth));

    octreeNode node;

    node.shellNode = false;
    node.leaf = true;
    node.isSet = true;
    node.inside = addToInteriors;
    node.parentIndex = parentIndex;
    node.setPoints(xf,yf,zf,cell_length);
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

GLint inline BasicOctree::handleHashItem(GLint vertexIndex,QVector3D point)
{

    if (geometryMap.contains(vertexIndex))
    {
        return geometryMap.value(vertexIndex).index;
    }

    point.setX(point.x()*this->cell_length-(max_g-this->mean.x()));
    point.setY(point.y()*this->cell_length-(max_g-this->mean.y()));
    point.setZ(point.z()*this->cell_length-(max_g-this->mean.z()));

    hashItem item;

    item.vertex = point;
    item.index = geometryMap.size();

    geometryMap.insert(vertexIndex, item);

    return item.index;

}

bool inline BasicOctree::addTriangle(GLint x, GLint y, GLint z)
{
    octreeNode* nodePointer = this->getLeafNodeByCoordinate(x,y,z);

    return nodePointer->shellNode || (nodePointer->isSet && nodePointer->inside && !nodePointer->isVoid);
}

void BasicOctree::createTriangle(GLint x, GLint y, GLint z, GLint code)
{

    QVector3D vec;
    bool flip = false;

    GLint pl = this->plane_length;
    GLint al = this->axis_length;

    GLint index0,index1,index2,index3;
    index0 = index1 = index2 = index3 = -1;

    switch(code){

    case 0:

        vec.setX(x);        vec.setY(y);        vec.setZ(z);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x+1);      vec.setY(y);        vec.setZ(z);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x+1);      vec.setY(y+1);      vec.setZ(z);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x);        vec.setY(y+1);      vec.setZ(z);
        index3 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        flip = true;
        break;

    case 1:

        vec.setX(x);        vec.setY(y);        vec.setZ(z+1);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x+1);      vec.setY(y);        vec.setZ(z+1);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x+1);      vec.setY(y+1);      vec.setZ(z+1);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x);        vec.setY(y+1);      vec.setZ(z+1);
        index3 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        break;

    case 2:

        vec.setX(x);        vec.setY(y);        vec.setZ(z);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x);        vec.setY(y);        vec.setZ(z+1);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x+1);      vec.setY(y);        vec.setZ(z+1);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x+1);      vec.setY(y);        vec.setZ(z);
        index3 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        flip = true;
        break;

    case 3:

        vec.setX(x);        vec.setY(y+1);      vec.setZ(z);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x);        vec.setY(y+1);      vec.setZ(z+1);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x+1);      vec.setY(y+1);      vec.setZ(z+1);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x+1);      vec.setY(y+1);      vec.setZ(z);
        index3 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        break;

    case 4:

        vec.setX(x);        vec.setY(y);        vec.setZ(z);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x);        vec.setY(y);        vec.setZ(z+1);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x);        vec.setY(y+1);      vec.setZ(z+1);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x);        vec.setY(y+1);      vec.setZ(z);
        index3 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);

        break;

    case 5:

        vec.setX(x+1);      vec.setY(y);        vec.setZ(z);
        index0 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x+1);      vec.setY(y);        vec.setZ(z+1);
        index1 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x+1);      vec.setY(y+1);      vec.setZ(z+1);
        index2 = handleHashItem(pl*vec.x()+al*vec.y()+vec.z(),vec);
        vec.setX(x+1);      vec.setY(y+1);      vec.setZ(z);
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

}

void BasicOctree::setInnerNodeIndices()
{

    octreeNode* nodePointer;

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

}

void BasicOctree::setShellNodeIndices()
{

    octreeNode* nodePointer;

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

}

void BasicOctree::createInnerSurface()
{

    geometry.clear();
    indices.clear();
    geometryMap.clear();

    GLint length = innerNodeIndices.length();

    for(int i=0;i<length;i++)
    {
        octreeNode* nodePointer = &this->octreeNodes.data()[innerNodeIndices.at(i)];

        if(!nodePointer->isVoid)
        {
            continue;
        }

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

    QHash<GLint, hashItem>::iterator i;

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

void BasicOctree::setOuterNodes()
{
    octreeNode* nodePointer;

    QHash<GLint, octreeNode*> tempMap1;
    QHash<GLint, octreeNode*> tempMap2;

    QHash<GLint, octreeNode*>* frontMap = &tempMap1;
    QHash<GLint, octreeNode*>* backMap = &tempMap2;

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

        QHash<GLint, octreeNode*>::iterator i;

        for (i = frontMap->begin(); i != frontMap->end(); ++i)
        {

            octreeNode* nodePointer = i.value();
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

void BasicOctree::setInnerNodes()
{

    octreeNode* nodePointer;

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

void BasicOctree::setupVectors()
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

GLint BasicOctree::sortHalf(GLint start,GLint end,GLint coor, GLint prior)
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

void BasicOctree::setupOctree(){

    freeIndicesForAllNodes.clear();
    freeIndicesForInnerNodes.clear();

    this->shellNodeIndices.clear();

    this->rootNodeIndex = setupOctreeHelper(0,0,this->voxels.length(),0,0,0);

    this->voxels.clear();

}

GLint BasicOctree::setupOctreeHelper(GLint depth,GLint start, GLint end, GLint x, GLint y, GLint z){


    GLfloat xf = x*this->cell_length-(max_g-this->mean.x());
    GLfloat yf = y*this->cell_length-(max_g-this->mean.y());
    GLfloat zf = z*this->cell_length-(max_g-this->mean.z());
    GLfloat cell_length = this->cell_length*(pow(2,this->startDepth-depth));


    if(this->startDepth<=depth || end-start<1 ){

        octreeNode node;

        node.shellNode = this->startDepth<=depth && end-start>0;
        node.leaf = true;
        node.setPoints(xf,yf,zf,cell_length);
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

    octreeNode node;

    GLint half_cell_length_int = cell_length_int >> 1;

    node.childIndex0 = setupOctreeHelper(newDepth,start         ,startEndTemp1 ,x,                 y,                 z                  );
    node.childIndex1 = setupOctreeHelper(newDepth,startEndTemp1 ,startEndZ1    ,x,                 y,                 z+half_cell_length_int );
    node.childIndex2 = setupOctreeHelper(newDepth,startEndZ1    ,startEndTemp2 ,x,                 y+half_cell_length_int,z                  );
    node.childIndex3 = setupOctreeHelper(newDepth,startEndTemp2 ,startEndY     ,x,                 y+half_cell_length_int,z+half_cell_length_int );

    node.childIndex4 = setupOctreeHelper(newDepth,startEndY     ,startEndTemp3 ,x+half_cell_length_int,y,                 z                  );
    node.childIndex5 = setupOctreeHelper(newDepth,startEndTemp3 ,startEndZ2    ,x+half_cell_length_int,y,                 z+half_cell_length_int );
    node.childIndex6 = setupOctreeHelper(newDepth,startEndZ2    ,startEndTemp4 ,x+half_cell_length_int,y+half_cell_length_int,z                  );
    node.childIndex7 = setupOctreeHelper(newDepth,startEndTemp4 ,end           ,x+half_cell_length_int,y+half_cell_length_int,z+half_cell_length_int );

    node.shellNode = true;
    node.leaf = false;
    node.setPoints(xf,yf,zf,cell_length);
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

void BasicOctree::setStartDepth(GLint depth){

    GLint minDepthLimit = 3;
    GLint maxDepthLimit = 9;

    depth = depth<maxDepthLimit?depth:maxDepthLimit;
    depth = depth>minDepthLimit?depth:minDepthLimit;
    depth = depth<=maxDepth?depth:maxDepth;

    this->startDepth = depth;
}

void BasicOctree::setMaxDepth(GLint depth){

    GLint minDepthLimit = 3;
    GLint maxDepthLimit = 9;

    depth = depth<maxDepthLimit?depth:maxDepthLimit;
    depth = depth>minDepthLimit?depth:minDepthLimit;
    depth = depth>=startDepth?depth:startDepth;

    this->maxDepth = depth;
}

void BasicOctree::adjustMaxDepth()
{
    GLint diff = this->maxDepth-startDepth;

    this->cell_length = this->cell_length/pow(2,diff);

    this->axis_length = this->axis_length<<diff;
    this->plane_length = this->axis_length*this->axis_length;

    octreeNode* nodePointer;
    for(int i=0;i<this->octreeNodes.length();i++){
        nodePointer = &this->octreeNodes.data()[i];

        nodePointer->x = nodePointer->x<<diff;
        nodePointer->y = nodePointer->y<<diff;
        nodePointer->z = nodePointer->z<<diff;

        nodePointer->cell_length = nodePointer->cell_length<<diff;
    }

}

Mesh* BasicOctree::getMesh(bool flip)
{

    QVector<GLfloat>* geometry = new QVector<GLfloat>();
    QVector<GLint>* indices = new QVector<GLint>();

    geometry->reserve(this->geometry.length());
    indices->reserve(this->indices.length());

    for (int i=0;i<this->geometry.length();i++)
    {
       geometry->push_back(this->geometry.at(i));
    }

    if(flip)
    {
        for (int i=0;i<this->indices.length();i+=3)
        {
            indices->push_back(this->indices.at(i+2));
            indices->push_back(this->indices.at(i+1));
            indices->push_back(this->indices.at(i+0));
        }
    }
    else
    {
        for (int i=0;i<this->indices.length();i++)
        {
            indices->push_back(this->indices.at(i));
        }
    }

    Mesh* mesh = new Mesh(geometry,indices);

    return mesh;
}

Mesh* BasicOctree::getPointMesh()
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

void BasicOctree::render(QGLShaderProgram* shader)
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

        //GLint viewIndex1 = (this->axis_length>>1);
        //GLint viewIndex2 = (this->axis_length>>1)+1;

        for (int i = 0; i < length; i++) {

            octreeNode node = this->octreeNodes.at(i);

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
