#include "basicoctree.h"

using namespace std;
using namespace octree;

const int GEOMETRY_DATA_SIZE    = 3;

BasicOctree::BasicOctree():
isDirty(true),
mesh(NULL),
startMaxDepth(6),
optimizationMaxDepth(6),
basicMaxDepth(6),
rootNodeIndex(-1)
{

}

//-------------------------------------------------- setup parameters

/**
 * @brief BasicOctree::setMesh Set the mesh for the octree
 * @param mesh the considered mesh
 */
void BasicOctree::setMesh(Mesh* mesh)
{
    this->mesh = mesh;
}

/**
 * @brief BasicOctree::setStartDepth Set a new start depth
 * @param depth the start depth
 */
void BasicOctree::setStartMaxDepth(GLint depth)
{
    this->startMaxDepth = depth;
    this->basicMaxDepth = this->startMaxDepth>this->optimizationMaxDepth?this->startMaxDepth:this->optimizationMaxDepth;
}

/**
 * @brief BasicOctree::setOptimizationMaxDepth Set a new maximal depth
 * @param depth the maximal depth
 */
void BasicOctree::setOptimizationMaxDepth(GLint depth)
{
    this->optimizationMaxDepth = depth;
    this->basicMaxDepth = this->startMaxDepth>this->optimizationMaxDepth?this->startMaxDepth:this->optimizationMaxDepth;
}

//-------------------------------------------------- quantizing mesh

/**
 * @brief BasicOctree::addTriangle Set the triangle defined be the three points
 * @param p1 point 1
 * @param p2 point 2
 * @param p3 point 3
 * @param buffer triangle buffer
 */
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

/**
 * @brief BasicOctree::setRawVoxel Set a voxel
 * @param x x coordinate
 * @param y y coordinate
 * @param z z coordinate
 */
void inline BasicOctree::setRawVoxel(GLfloat x,GLfloat y,GLfloat z)
{

    this->rawVoxels.push_back(x);
    this->rawVoxels.push_back(y);
    this->rawVoxels.push_back(z);

}

/**
 * @brief BasicOctree::quantizeSurface Quantize the surface of the mesh
 */
void BasicOctree::quantizeSurface()
{

    if(this->mesh == NULL)
    {
        return;
    }

    mean = mesh->getMean();
    GLfloat x = mean.x();
    GLfloat y = mean.y();
    GLfloat z = mean.z();


    GLfloat* gvd = this->mesh->getGeometry()->data();
    GLint* ivd = this->mesh->getIndices()->data();
    int length = this->mesh->getIndices()->length();

    GLdouble max_g = 0;

    // search for the maximal coordiante
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

    this->rawVoxels.clear();
    this->rawVoxels.reserve( length*pow(4,2) );

    QVector<GLfloat> triangleBuffer1;
    QVector<GLfloat> triangleBuffer2;
    triangleBuffer1.reserve(length*3);
    triangleBuffer2.reserve(length*3);

    QVector<float>* frontBuffer = &triangleBuffer1;
    QVector<float>* backBuffer = &triangleBuffer2;

    GLfloat px = max_g-x;
    GLfloat py = max_g-y;
    GLfloat pz = max_g-z;

    axis_length = pow(2,this->startMaxDepth);
    plane_length = axis_length*axis_length;
    cell_length = (max_g*2)/axis_length;

    // add the triangles to the initial front vector
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

    // create a point cloud for the triangles
    while(frontBuffer->size()>0)
    {

        while(frontBuffer->size()>0)
        {

            // get the points for the next triangle
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
                // triangle is small enough

                // add raw voxel
                this->setRawVoxel(p1.x(),p1.y(),p1.z());
                this->setRawVoxel(p2.x(),p2.y(),p2.z());
                this->setRawVoxel(p3.x(),p3.y(),p3.z());

            }
            else{

                // split triangle into four parts and add them to the back buffer
                QVector3D newp1 = p1+(p2-p1)/2;
                QVector3D newp2 = p1+(p3-p1)/2;
                QVector3D newp3 = p2+(p3-p2)/2;
                this->addTriangle(&p1,&newp2,&newp1,backBuffer);
                this->addTriangle(&p2,&newp1,&newp3,backBuffer);
                this->addTriangle(&p3,&newp3,&newp2,backBuffer);
                this->addTriangle(&newp1,&newp2,&newp3,backBuffer);

            }
        }

        // switch buffers
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

/**
 * @brief BasicOctree::setupVectors Set voxel points for raw voxels
 */
void BasicOctree::setupVectors()
{

    QVector<QVector3D> tempVoxels;
    tempVoxels.reserve(this->rawVoxels.length()/3);

    this->voxels.clear();
    this->voxels.reserve(this->rawVoxels.length()/3);

    GLint length;

    length = this->rawVoxels.length();

    for (GLint i = 0; i < length ; i+=3)
    {

      QVector3D point;

      point.setX(this->rawVoxels.at(i+0));
      point.setY(this->rawVoxels.at(i+1));
      point.setZ(this->rawVoxels.at(i+2));

      voxels.push_back(point);
    }

}

//-------------------------------------------------- helper functions for octree

/**
 * @brief BasicOctree::getLeafNodeByCoordinate Get leaf for the coordinates
 * @param x x coordinate
 * @param y y coordinate
 * @param z z coordinate
 * @return pointer to searched node
 */
octreeNode* BasicOctree::getLeafNodeByCoordinate(GLint x, GLint y, GLint z)
{
    return getLeafNodeByCoordinate(x,y,z,this->rootNodeIndex);
}

/**
 * @brief BasicOctree::getLeafNodeByCoordinate Get leaf for the coordinates starting from specific node
 * @param x x coordinate
 * @param y y coordinate
 * @param z z coordinate
 * @param startNodeIndex index of start node
 * @return pointer to searched node
 */
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

/**
 * @brief BasicOctree::getLeafNodeByCoordinateHelper
 * @param x x coordinate
 * @param y y coordinate
 * @param z z coordinate
 * @param nodeIndex index of node
 * @return pointer to searched node
 */
octreeNode* BasicOctree::getLeafNodeByCoordinateHelper(GLint x, GLint y, GLint z, GLint nodeIndex)
{
    octreeNode* nodePointer = &this->octreeNodes.data()[nodeIndex];

    if(nodePointer->isLeaf)
    {
        return nodePointer;
    }

    GLint xarea = (nodePointer->x+(nodePointer->cell_length>>1))<=x?1:0;
    GLint yarea = (nodePointer->y+(nodePointer->cell_length>>1))<=y?1:0;
    GLint zarea = (nodePointer->z+(nodePointer->cell_length>>1))<=z?1:0;

    // choose next octree node
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

/**
 * @brief BasicOctree::createNode Create a new node
 * @param x x coordinate of octree
 * @param y y coordinate of octree
 * @param z z coordinate of octree
 * @param depth node depth
 * @param addToInteriors should be added to interiors
 * @param parentIndex index of the parent node
 * @return index of new node
 */
GLint BasicOctree::createNode(GLint x,GLint y,GLint z,GLint depth,bool isInside,GLint parentIndex){

    GLfloat xf = x*this->cell_length-(max_g-this->mean.x());
    GLfloat yf = y*this->cell_length-(max_g-this->mean.y());
    GLfloat zf = z*this->cell_length-(max_g-this->mean.z());
    GLfloat cell_length = this->cell_length*(pow(2,this->basicMaxDepth-depth));

    octreeNode node;

    node.isSet = true;

    node.isLeaf = true;
    node.isInside = isInside;
    node.isShell = false;

    node.parentIndex = parentIndex;
    node.setPoints(xf,yf,zf,cell_length);
    node.x = x;
    node.y = y;
    node.z = z;
    node.cell_length = pow(2,this->basicMaxDepth-depth);
    node.nodeDepth = depth;

    // add node to octree node vector
    node.index = this->octreeNodes.length();
    this->octreeNodes.push_back(node);

    this->octreeNodes.data()[node.index] = node;
    return node.index;
}

//-------------------------------------------------- shell mesh calculation

/**
 * @brief BasicOctree::handleHashItem Handle vertice for avoiding multiple set up of a vertex
 * @param vertexIndex unique index of a vertice
 * @param point point in world
 * @return index of the point
 */
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

/**
 * @brief BasicOctree::addTriangle Should triangles added here
 * @param x x coordinate in octree
 * @param y y coordiante in octree
 * @param z z coordinate in octree
 * @return true, triangles should added. false, otherwise
 */
bool inline BasicOctree::addTriangle(GLint x, GLint y, GLint z)
{
    octreeNode* nodePointer = this->getLeafNodeByCoordinate(x,y,z);

    return nodePointer->isShell || (nodePointer->isInside && !nodePointer->isVoid);
}

/**
 * @brief BasicOctree::createTriangle Add vertices and indices for inner and shell border
 * @param x x coordinate in octree
 * @param y y coordinate in octree
 * @param z z coordinate in octree
 * @param code code for side where surface should be created
 */
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

/**
 * @brief BasicOctree::createInnerSurface Calculate vertices and indices for inner shell mesh
 */
void BasicOctree::createInnerSurface()
{

    // get a vector of the indices of all inner nodes
    QVector<GLint> innerLeafIndices;
    this->getInnerLeaves(&innerLeafIndices);
    octreeNode* nodePointer;

    // prepare new calcuation
    geometry.clear();
    indices.clear();
    geometryMap.clear();

    for(int i=0;i<innerLeafIndices.length();i++)
    {
        nodePointer = &this->octreeNodes.data()[innerLeafIndices.at(i)];

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

    // set up vector for vertices
    for (i = geometryMap.begin(); i != geometryMap.end(); ++i)
    {

        QVector3D vec = i.value().vertex;
        geometry.push_back(vec.x());
        geometry.push_back(vec.y());
        geometry.push_back(vec.z());

    }

    GLfloat* data = geometry.data();

    // reverse the order of the vertices values
    for (i = geometryMap.begin(); i != geometryMap.end(); ++i)
    {

        QVector3D vec = i.value().vertex;
        data[i.value().index*3+0] = vec.x();
        data[i.value().index*3+1] = vec.y();
        data[i.value().index*3+2] = vec.z();

    }

    geometryMap.clear();

    innerLeafIndices.clear();

}

/**
 * @brief BasicOctree::getShellMesh Get the mesh of the inner shell
 * @param flip true the normals are flip. false otherwise
 * @return the mesh
 */
Mesh* BasicOctree::getShellMesh(bool flip)
{

    QVector<GLfloat>* geometry = new QVector<GLfloat>();
    QVector<GLint>* indices = new QVector<GLint>();

    geometry->reserve(this->geometry.length());
    indices->reserve(this->indices.length());

    for (int i=0;i<this->geometry.length();i++)
    {
       geometry->push_back(this->geometry.at(i));
    }

    // flip surface
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

//-------------------------------------------------- mark nodes

/**
 * @brief BasicOctree::setOuterNodes Mark outer nodes
 */
void BasicOctree::setOuterNodes()
{
    octreeNode* nodePointer;

    QHash<GLint, octreeNode*> tempMap1;
    QHash<GLint, octreeNode*> tempMap2;

    QHash<GLint, octreeNode*>* frontMap = &tempMap1;
    QHash<GLint, octreeNode*>* backMap = &tempMap2;

    // set border nodes as initial outer nodes
    for(int i=0;i<this->axis_length;i++)
    {
        for(int j=0;j<this->axis_length;j++)
        {


            nodePointer = this->getLeafNodeByCoordinate(i,j,0);
            if(nodePointer !=NULL && !nodePointer->isShell)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }
            nodePointer = this->getLeafNodeByCoordinate(i,j,this->axis_length-1);
            if(nodePointer !=NULL && !nodePointer->isShell)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }
            nodePointer = this->getLeafNodeByCoordinate(i,0,j);
            if(nodePointer !=NULL && !nodePointer->isShell)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }
            nodePointer = this->getLeafNodeByCoordinate(i,this->axis_length-1,j);
            if(nodePointer !=NULL && !nodePointer->isShell)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }
            nodePointer = this->getLeafNodeByCoordinate(0,i,j);
            if(nodePointer !=NULL && !nodePointer->isShell)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }
            nodePointer = this->getLeafNodeByCoordinate(this->axis_length-1,i,j);
            if(nodePointer !=NULL && !nodePointer->isShell)
            {
                frontMap->insert(nodePointer->index,nodePointer);
            }

        }
    }

    // search for outer nodes iteratively
    while(frontMap->size()>0)
    {

        QHash<GLint, octreeNode*>::iterator i;

        for (i = frontMap->begin(); i != frontMap->end(); ++i)
        {

            octreeNode* nodePointer = i.value();
            nodePointer->isSet = true;

            nodePointer->isInside = false;
            nodePointer->isShell = false;
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
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->isShell)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }
                    nodePointer = this->getLeafNodeByCoordinate(x+j,y+k,z+length2);
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->isShell)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }


                    nodePointer = this->getLeafNodeByCoordinate(x+j,y-1,z+k);
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->isShell)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }
                    nodePointer = this->getLeafNodeByCoordinate(x+j,y+length2,z+k);
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->isShell)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }


                    nodePointer = this->getLeafNodeByCoordinate(x-1,y+j,z+k);
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->isShell)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }
                    nodePointer = this->getLeafNodeByCoordinate(x+length2,y+j,z+k);
                    if(nodePointer !=NULL && !nodePointer->isSet && !nodePointer->isShell)
                    {
                        backMap->insert(nodePointer->index,nodePointer);
                    }

                }
            }
        }

        // swap buffers
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

/**
 * @brief BasicOctree::setInnerNodes Mark the inner nodes ( after calculating outer nodes)
 */
void BasicOctree::setInnerNodes()
{

    octreeNode* nodePointer;

    for(int i=1;i<this->octreeNodes.length();i++)
    {

         nodePointer = &this->octreeNodes.data()[i];

         if(!nodePointer->isLeaf || nodePointer->isSet || nodePointer->isShell)
         {
           continue;
         }

         nodePointer->isSet = true;

         nodePointer->isInside = true;
         nodePointer->isShell = false;

    }

}

//-------------------------------------------------- set up octree

/**
 * @brief BasicOctree::sortHalf Sort voxels until half of the array size (for x,y or z coordinate)
 * @param start start index
 * @param end end index
 * @param coor code for coordinate
 * @param prior prior value
 * @return index of the last element of half sorted array
 */
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

/**
 * @brief BasicOctree::setupOctree Create the octree according to set preferences
 */
void BasicOctree::setupOctree(){

    this->octreeNodes.clear();

    this->rootNodeIndex = setupOctreeHelper(0,0,this->voxels.length(),0,0,0);

    // clear obsolete data structures
    this->rawVoxels.clear();
    this->voxels.clear();
    this->isDirty = true;
}

/**
 * @brief BasicOctree::setupOctreeHelper Helper function for creating the octree
 * @param depth current depth
 * @param start start value for range
 * @param end end value for range
 * @param x x coordinate in octree
 * @param y y coordinate in octree
 * @param z z coordinate in octree
 * @return index of created node
 */
GLint BasicOctree::setupOctreeHelper(GLint depth,GLint start, GLint end, GLint x, GLint y, GLint z){

    // calculate values
    GLfloat xf = x*this->cell_length-(max_g-this->mean.x());
    GLfloat yf = y*this->cell_length-(max_g-this->mean.y());
    GLfloat zf = z*this->cell_length-(max_g-this->mean.z());
    GLfloat cell_length = this->cell_length*(pow(2,this->startMaxDepth-depth));

    // if recursion should stop (leaf is reached)
    if(this->startMaxDepth<=depth || end-start<1 ){

        octreeNode node;

        // set node data
        node.isShell = this->startMaxDepth<=depth && end-start>0;
        node.isLeaf = true;

        node.setPoints(xf,yf,zf,cell_length);
        node.x = x;
        node.y = y;
        node.z = z;
        node.cell_length = pow(2,this->startMaxDepth-depth);
        node.index = octreeNodes.length();
        node.nodeDepth = depth;

        octreeNodes.push_back(node);
        return node.index;
    }

    GLint cell_length_int = pow(2,this->startMaxDepth-depth);
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

    // recursion step
    node.childIndex0 = setupOctreeHelper(newDepth,start         ,startEndTemp1 ,x,                 y,                 z                  );
    node.childIndex1 = setupOctreeHelper(newDepth,startEndTemp1 ,startEndZ1    ,x,                 y,                 z+half_cell_length_int );
    node.childIndex2 = setupOctreeHelper(newDepth,startEndZ1    ,startEndTemp2 ,x,                 y+half_cell_length_int,z                  );
    node.childIndex3 = setupOctreeHelper(newDepth,startEndTemp2 ,startEndY     ,x,                 y+half_cell_length_int,z+half_cell_length_int );

    node.childIndex4 = setupOctreeHelper(newDepth,startEndY     ,startEndTemp3 ,x+half_cell_length_int,y,                 z                  );
    node.childIndex5 = setupOctreeHelper(newDepth,startEndTemp3 ,startEndZ2    ,x+half_cell_length_int,y,                 z+half_cell_length_int );
    node.childIndex6 = setupOctreeHelper(newDepth,startEndZ2    ,startEndTemp4 ,x+half_cell_length_int,y+half_cell_length_int,z                  );
    node.childIndex7 = setupOctreeHelper(newDepth,startEndTemp4 ,end           ,x+half_cell_length_int,y+half_cell_length_int,z+half_cell_length_int );

    // set node data
    node.isShell = true;
    node.isLeaf = false;

    node.setPoints(xf,yf,zf,cell_length);
    node.x = x;
    node.y = y;
    node.z = z;
    node.cell_length = cell_length_int;
    node.index = octreeNodes.length();
    node.nodeDepth = depth;

    // set parent index
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

/**
 * @brief BasicOctree::getInnerNodesForNode Get the indices of all inner leaf nodes which are predecessors of a specific node.
 * @param index index of the specific node
 * @param indices a pointer of th vector which get the indices of the found nodes
 */
void BasicOctree::getInnerLeavesForNode(GLint index,QVector<GLint>* indices)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    // break recursion because current node is a leaf
    if(nodePointer->isLeaf)
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
 * @brief BasicOctree::getNodesOfDepth Get the indices of all nodes which lay a specific depth.
 * @param depth the depth of the searched nodes
 * @param indices a pointer of th vector which get the indices of the found nodes
 */
void BasicOctree::getNodesOfDepth(GLint depth,QVector<GLint>* indices)
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
 * @brief BasicOctree::adjustToBasicMaxDepth Adjust the octree to the maximal depth
 */
void BasicOctree::adjustToBasicMaxDepth()
{
    if(this->basicMaxDepth<this->startMaxDepth)
    {

       QVector<GLint> nodeIndices;
       getNodesOfDepth(basicMaxDepth,&nodeIndices);

       for(int i=0;i<nodeIndices.length();i++){
         makeExplicitMergeRoot(nodeIndices.data()[i]);
       }

       nodeIndices.clear();


    }
    else
    {

        GLint diff = this->basicMaxDepth-startMaxDepth;

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

}

/**
 * @brief BasicOctree::propageDownMergeChildHelper
 * @param index
 */
void BasicOctree::propageDownMergeChildHelper(GLint index)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    if(nodePointer->isShell || nodePointer->isInside)
    {
        nodePointer->isMergeChild = true;
        nodePointer->isMergeRoot = false;
    }

    if(!nodePointer->isLeaf)
    {

        propageDownMergeChildHelper(nodePointer->childIndex0);
        propageDownMergeChildHelper(nodePointer->childIndex1);
        propageDownMergeChildHelper(nodePointer->childIndex2);
        propageDownMergeChildHelper(nodePointer->childIndex3);
        propageDownMergeChildHelper(nodePointer->childIndex4);
        propageDownMergeChildHelper(nodePointer->childIndex5);
        propageDownMergeChildHelper(nodePointer->childIndex6);
        propageDownMergeChildHelper(nodePointer->childIndex7);

    }

}

/**
 * @brief BasicOctree::propageDownMergeChild
 * @param index
 */
void BasicOctree::propageDownMergeChild(GLint index)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    if(!nodePointer->isLeaf)
    {

        propageDownMergeChildHelper(nodePointer->childIndex0);
        propageDownMergeChildHelper(nodePointer->childIndex1);
        propageDownMergeChildHelper(nodePointer->childIndex2);
        propageDownMergeChildHelper(nodePointer->childIndex3);
        propageDownMergeChildHelper(nodePointer->childIndex4);
        propageDownMergeChildHelper(nodePointer->childIndex5);
        propageDownMergeChildHelper(nodePointer->childIndex6);
        propageDownMergeChildHelper(nodePointer->childIndex7);

    }

}

/**
 * @brief BasicOctree::makeExplicitMergeRoot
 * @param index
 */
void BasicOctree::makeExplicitMergeRoot(GLint index)
{

    octreeNode* nodePointer = &this->octreeNodes.data()[index];

    if( !(nodePointer->isShell || nodePointer->isInside))
    {
        return;
    }

    QVector<GLint> nodeIndices;
    getInnerLeavesForNode(index,&nodeIndices);

    nodePointer->isIgnored = nodeIndices.length()<=0;

    nodeIndices.clear();

    nodePointer->isMergeChild = false;
    nodePointer->isMergeRoot = true;

    propageDownMergeChild(index);

}

//-------------------------------------------------- draw octree grid

/**
 * @brief BasicOctree::renderOctreeGrid Render the grid of the octree
 * @param shader the shader
 */
void BasicOctree::renderOctreeGrid(QGLShaderProgram* shader)
{

    if(this->octreeNodes.length()<0){
        return;
    }

    GLenum primitive = GL_LINES;

    /* if needed needed variables are set for painting */
    if (isDirty) {
        QVector<GLfloat> buffer_vertices,buffer_colors;

        buffer_vertices.reserve( this->octreeNodes.length() * 8 * 3 * sizeof(GLfloat));
        buffer_colors.reserve( this->octreeNodes.length() * 8 * 3 * sizeof(GLfloat));

        indexBuffer.clear();
        GLint index = 0;

        GLint length = this->octreeNodes.length();

        GLint viewIndex = (this->axis_length>>1);

        for (int i = 0; i < length; i++) {

            octreeNode node = this->octreeNodes.at(i);

            if( !node.isSet || !node.isInside || !node.isLeaf || node.isShell )
            {
                continue;
            }

            if(node.x>=viewIndex)
            {
                continue;
            }

            /* add vertices*/
            buffer_vertices.push_back(node.p0.x());buffer_vertices.push_back(node.p0.y());buffer_vertices.push_back(node.p0.z());
            buffer_vertices.push_back(node.p1.x());buffer_vertices.push_back(node.p1.y());buffer_vertices.push_back(node.p1.z());
            buffer_vertices.push_back(node.p2.x());buffer_vertices.push_back(node.p2.y());buffer_vertices.push_back(node.p2.z());
            buffer_vertices.push_back(node.p3.x());buffer_vertices.push_back(node.p3.y());buffer_vertices.push_back(node.p3.z());

            buffer_vertices.push_back(node.p4.x());buffer_vertices.push_back(node.p4.y());buffer_vertices.push_back(node.p4.z());
            buffer_vertices.push_back(node.p5.x());buffer_vertices.push_back(node.p5.y());buffer_vertices.push_back(node.p5.z());
            buffer_vertices.push_back(node.p6.x());buffer_vertices.push_back(node.p6.y());buffer_vertices.push_back(node.p6.z());
            buffer_vertices.push_back(node.p7.x());buffer_vertices.push_back(node.p7.y());buffer_vertices.push_back(node.p7.z());

            QColor color;

            if(node.beta>1)
            {
                color.setRedF(0);
                color.setGreenF(0);
                color.setBlueF(0);
            }
            else
            if(node.beta<0)
            {
                color.setRedF(1);
                color.setGreenF(1);
                color.setBlueF(1);
            }
            else
            {
                QVector3D fillColor(255/(float)255,255/(float)255,224/(float)255);
                QVector3D voidColor(128/(float)255,128/(float)255,0/(float)255);

                QVector3D col = fillColor+(voidColor-fillColor)*node.beta;

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

            /*
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
            */

            index +=8;
        }

        vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        vbo->create();
        vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        vbo->bind();
        vbo->allocate( buffer_vertices.size() * sizeof(GLfloat) );
        vbo->write(0, buffer_vertices.constData(), buffer_vertices.size() * sizeof(GLfloat));
        vbo->release();

        cbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        cbo->create();
        cbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        cbo->bind();
        cbo->allocate(buffer_colors.size() * 3 * sizeof(GLfloat));
        cbo->write(0, buffer_colors.constData(), buffer_colors.size() * sizeof(GLfloat));
        cbo->release();

        ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        ibo->create();
        ibo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        ibo->bind();
        ibo->allocate(indexBuffer.size() * sizeof(GLint));
        ibo->write(0, indexBuffer.constData(), indexBuffer.size() * sizeof(GLint));
        ibo->release();

        /* create vertex buffer */
        /*
        vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        vbo->create();
        vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        vbo->bind();
        vbo->allocate(index * 3 * sizeof(GLfloat));
        vbo->write(0, buffer.constData(), buffer.size() * sizeof(GLfloat));
        vbo->release();
        */

        /* create index buffer */
        /*
        ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        ibo->create();
        ibo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        ibo->bind();
        ibo->allocate(cubeLineIndices.size() * sizeof(GLint));
        ibo->write(0, cubeLineIndices.constData(), cubeLineIndices.size() * sizeof(GLint));
        ibo->release();
        */

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
    glDrawElements(GL_TRIANGLES, indexBuffer.length(), GL_UNSIGNED_INT, (void*) 0);
    ibo->release();

    shader->disableAttributeArray("geometry");
    shader->disableAttributeArray("color");

    //GLint stride = sizeof(GLfloat) * (3);

    /* use vertex buffer */
    /*
    vbo->bind();
    shader->setAttributeBuffer("geometry", GL_FLOAT, 0, 3, stride);
    shader->enableAttributeArray("geometry");
    vbo->release();
    */
    /* use index buffer */
    /*
    ibo->bind();
    glDrawElements(primitive, cubeLineIndices.length(), GL_UNSIGNED_INT, (void*) 0);
    ibo->release();

    shader->disableAttributeArray("geometry");
    */

}

//-------------------------------------------------- vector getters

/**
 * @brief BasicOctree::getInnerLeaves Get the indices of all inner leaf nodes
 * @param indices a pointer of th vector which get the indices of the found nodes
 */
void BasicOctree::getInnerLeaves(QVector<GLint>* indices)
{

    octreeNode* nodePointer;

    for(int i=0;i<this->octreeNodes.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];

        if(nodePointer->isInside && nodePointer->isLeaf)
        {
           indices->push_back(nodePointer->index);
        }

    }

}

/**
 * @brief BasicOctree::getShellLeaves Get the indices of all shell leaf nodes
 * @param indices a pointer of th vector which get the indices of the found nodes
 */
void BasicOctree::getShellLeaves(QVector<GLint>* indices)
{

    octreeNode* nodePointer;

    for(int i=0;i<this->octreeNodes.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];

        if(nodePointer->isLeaf && nodePointer->isShell)
        {
            indices->push_back(nodePointer->index);
        }

    }

}

/**
 * @brief BasicOctree::getMergeIndices Get the indices of all merge nodes
 * @param indices a pointer of th vector which get the indices of the found nodes
 */
void BasicOctree::getMergeRoots(QVector<GLint>* indices)
{

    octreeNode* nodePointer;

    for(int i=0;i<this->octreeNodes.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];

        if(nodePointer->isMergeRoot)
        {
            indices->push_back(nodePointer->index);
        }

    }

}

/**
 * @brief BasicOctree::getMergeCandidateIndices Get the indices of all nodes which are merge candidates
 * @param indices a pointer of th vector which get the indices of the found nodes
 */
void BasicOctree::getMergeRootCandidates(QVector<GLint>* indices)
{

    octreeNode* nodePointer;

    for(int i=0;i<this->octreeNodes.length();i++)
    {

        nodePointer = &this->octreeNodes.data()[i];

        if(
                !nodePointer->isLeaf &&
                !nodePointer->isMergeRoot &&
                !nodePointer->isMergeChild &&
                !(nodePointer->isInside && nodePointer->isShell))
        {
            indices->push_back(nodePointer->index);
        }

    }

}

void BasicOctree::setDirty()
{
    this->isDirty = true;
}
