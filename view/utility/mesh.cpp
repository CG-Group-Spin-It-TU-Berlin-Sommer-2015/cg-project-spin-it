#include "mesh.h"

using namespace std;

const int GEOMETRY_DATA_SIZE    = 3;
const int NORMAL_DATA_SIZE      = 3;

Mesh::Mesh(QVector<GLfloat>* geometry, QVector<GLshort>* indices)
{
    this->geometry = geometry;
    this->indices = indices;

    this->normals = new QVector<GLfloat>();
    this->normals->reserve(geometry->size() * sizeof(GLfloat));
    for (int i = 0; i < geometry->size(); i++) {
        normals->push_back(0);
    }

    for (int i = 0; i < indices->size(); i += 3) {
        QVector3D n1;
        n1.setX(geometry->at(3 * indices->at(i)) - geometry->at(3 * indices->at(i + 1)));
        n1.setY(geometry->at(3 * indices->at(i) + 1) - geometry->at(3 * indices->at(i + 1) + 1));
        n1.setZ(geometry->at(3 * indices->at(i) + 2) - geometry->at(3 * indices->at(i + 1) + 2));

        QVector3D n2;
        n2.setX(geometry->at(3 * indices->at(i)) - geometry->at(3 * indices->at(i + 2)));
        n2.setY(geometry->at(3 * indices->at(i) + 1) - geometry->at(3 * indices->at(i + 2) + 1));
        n2.setZ(geometry->at(3 * indices->at(i) + 2) - geometry->at(3 * indices->at(i + 2) + 2));

        QVector3D n = QVector3D::crossProduct(n1, n2);

        normals->replace(3 * indices->at(i), normals->at(3 * indices->at(i)) + n.x());
        normals->replace(3 * indices->at(i) + 1, normals->at(3 * indices->at(i) + 1) + n.y());
        normals->replace(3 * indices->at(i) + 2, normals->at(3 * indices->at(i) + 2) + n.z());

        normals->replace(3 * indices->at(i + 1), normals->at(3 * indices->at(i + 1)) + n.x());
        normals->replace(3 * indices->at(i + 1) + 1, normals->at(3 * indices->at(i + 1) + 1) + n.y());
        normals->replace(3 * indices->at(i + 1) + 2, normals->at(3 * indices->at(i + 1) + 2) + n.z());

        normals->replace(3 * indices->at(i + 2), normals->at(3 * indices->at(i + 2)) + n.x());
        normals->replace(3 * indices->at(i + 2) + 1, normals->at(3 * indices->at(i + 2) + 1) + n.y());
        normals->replace(3 * indices->at(i + 2) + 2, normals->at(3 * indices->at(i + 2) + 2) + n.z());
    }

    for (int i = 0; i < geometry->size() / 3; i++) {
        QVector3D normal;
        normal.setX(normals->at(3*i));
        normal.setY(normals->at(3*i + 1));
        normal.setZ(normals->at(3*i + 2));

        normals->replace(3 * i, normal.x() / normal.length());
        normals->replace(3 * i + 1, normal.y() / normal.length());
        normals->replace(3 * i + 2, normal.z() / normal.length());
    }

    this->isDirty = true;
}

Mesh::~Mesh()
{
    delete vbo;
    delete ibo;

    delete geometry;
    delete normals;
    delete indices;
}

Mesh::Mesh(QVector<GLfloat> *geometry, QVector<GLfloat> *normals, QVector<GLshort> *indices)
{
    this->geometry = geometry;
    this->normals = normals;
    this->indices = indices;
    this->isDirty = true;
}

QVector<GLfloat>* Mesh::getGeometry()
{
    return geometry;
}

QVector<GLfloat>* Mesh::getNormals()
{
    return normals;
}

QVector<GLshort>* Mesh::getIndices()
{
    return indices;
}

void Mesh::render(QGLShaderProgram* shader, GLenum primitive)
{
    if (isDirty) {
        QVector<GLfloat> buffer;
        buffer.reserve(geometry->size() * sizeof(GLfloat) + normals->size() * sizeof(GLfloat));
        for (int i = 0; i < geometry->size() / GEOMETRY_DATA_SIZE; i++) {
            buffer.push_back(geometry->at(3*i));
            buffer.push_back(geometry->at(3*i + 1));
            buffer.push_back(geometry->at(3*i + 2));

            buffer.push_back(normals->at(3*i));
            buffer.push_back(normals->at(3*i + 1));
            buffer.push_back(normals->at(3*i + 2));
        }

        vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        vbo->create();
        vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        vbo->bind();
        vbo->allocate(geometry->size() * sizeof(GLfloat) + normals->size() * sizeof(GLfloat));
        vbo->write(0, buffer.constData(), buffer.size() * sizeof(GLfloat));
        vbo->release();

        ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        ibo->create();
        ibo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        ibo->bind();
        ibo->allocate(indices->size() * sizeof(GLshort));
        ibo->write(0, indices->constData(), indices->size() * sizeof(GLshort));
        ibo->release();

        isDirty = false;
    }

    GLint stride = sizeof(GLfloat) * (GEOMETRY_DATA_SIZE + NORMAL_DATA_SIZE);

    vbo->bind();
    shader->setAttributeBuffer("geometry", GL_FLOAT, 0, 3, stride);
    shader->enableAttributeArray("geometry");
    shader->setAttributeBuffer("normal", GL_FLOAT, GEOMETRY_DATA_SIZE * sizeof(GL_FLOAT), 3, stride);
    shader->enableAttributeArray("normal");
    vbo->release();

    ibo->bind();
    glDrawElements(primitive, indices->length(), GL_UNSIGNED_SHORT, (void*) 0);
    ibo->release();

    shader->disableAttributeArray("geometry");
}

QVector3D Mesh::getMean()
{
    QVector3D mean;
    for (int i = 0; i < geometry->size(); i += 3) {
        mean.setX(mean.x() + geometry->at(i));
        mean.setY(mean.y() + geometry->at(i + 1));
        mean.setZ(mean.z() + geometry->at(i + 2));
    }
    mean.setX(mean.x() / (geometry->size() / 3));
    mean.setY(mean.y() / (geometry->size() / 3));
    mean.setZ(mean.z() / (geometry->size() / 3));
    return mean;
}

GLint Mesh::setKDTreeHelper(GLint start, GLint end, GLint searchIndex, QVector<GLint>* searchIndices, QVector<kdNode>* nodes, GLint depth)
{

    GLfloat* gvd = geometry->data();
    GLint* svd = searchIndices->data();

    if(end-start<1){
        return -1;
    }

    if(end-start<2){

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

void Mesh::setKDTree()
{

    QVector<GLfloat>* geometry = this->getGeometry();
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

inline GLfloat Mesh::getDistanceToPlane(GLint axisIndex, GLint index, GLfloat x, GLfloat y, GLfloat z)
{

    GLfloat* gvd = geometry->data();

    GLfloat value = (axisIndex==0?x:(axisIndex==1?y:z));

    GLint index2 = nodes->at(index).index;

    return fabs(gvd[index2*3+axisIndex]-value);
}

inline GLfloat Mesh::getLength(GLint index, GLfloat x, GLfloat y, GLfloat z)
{

    GLfloat* gvd = geometry->data();

    GLint index2 = nodes->at(index).index;

    GLfloat xv = pow(gvd[index2*3+0]-x,2);
    GLfloat yv = pow(gvd[index2*3+1]-y,2);
    GLfloat zv = pow(gvd[index2*3+2]-z,2);

    return sqrt(xv+yv+zv);
}

GLint Mesh::getNearestNeighborHelper1(GLint nodeIndex, float x, float y, float z)
{

    kdNode node = nodes->at(nodeIndex);

    kdNode left;
    if(node.leftChildIndex!=-1){
        left = nodes->at(node.leftChildIndex);
    }
    kdNode right;
    if(node.rightChildIndex!=-1){
        right = nodes->at(node.rightChildIndex);
    }

    GLfloat nodeDistance = getLength(node.index,x,y,z);
    GLfloat leftDistance = node.leftChildIndex==-1?std::numeric_limits<GLfloat>::max():getLength(left.index,x,y,z);
    GLfloat rightDistance = node.rightChildIndex==-1?std::numeric_limits<GLfloat>::max():getLength(right.index,x,y,z);

    if(nodeDistance<leftDistance){
        if(nodeDistance<rightDistance){

            //node closest
            return nodeIndex;
        }
        else{

            //left closest
            return getNearestNeighborHelper1(node.leftChildIndex,x,y,z);
        }
    }
    else{
        if(leftDistance<rightDistance){

            //left closest
            return getNearestNeighborHelper1(node.leftChildIndex,x,y,z);
        }
        else{

            //left closest
            return getNearestNeighborHelper1(node.rightChildIndex,x,y,z);
        }
    }
}

GLint Mesh::getNearestNeighborHelper2(GLint nodeIndex, float x, float y, float z)
{

    kdNode node = nodes->at(nodeIndex);

    kdNode left;
    if(node.leftChildIndex!=-1){
        left = nodes->at(node.leftChildIndex);
    }
    kdNode right;
    if(node.rightChildIndex!=-1){
        right = nodes->at(node.rightChildIndex);
    }

    GLfloat nodeDistance = getLength(node.index,x,y,z);
    GLfloat leftDistance = node.leftChildIndex==-1?std::numeric_limits<GLfloat>::max():getLength(left.index,x,y,z);
    GLfloat rightDistance = node.rightChildIndex==-1?std::numeric_limits<GLfloat>::max():getLength(right.index,x,y,z);

    GLfloat dist = getDistanceToPlane(node.depth%3,node.index,x,y,z);

    GLint newLeftIndex = node.leftChildIndex;
    GLint newRightIndex = node.rightChildIndex;

    if(leftDistance<dist){
        newLeftIndex = getNearestNeighborHelper2(newLeftIndex,x,y,z);
        leftDistance = getLength(newLeftIndex,x,y,z);

    }
    else
    if(rightDistance<dist){
        newRightIndex = getNearestNeighborHelper2(newRightIndex,x,y,z);
        rightDistance = getLength(newRightIndex,x,y,z);
    }
    else{

        if(node.leftChildIndex!=-1){
            newLeftIndex = getNearestNeighborHelper2(newLeftIndex,x,y,z);
            leftDistance = getLength(newLeftIndex,x,y,z);
        }
        if(node.rightChildIndex!=-1){
            newRightIndex = getNearestNeighborHelper2(newRightIndex,x,y,z);
            rightDistance = getLength(newRightIndex,x,y,z);
        }
    }

    if(nodeDistance<leftDistance){
        if(nodeDistance<rightDistance){
            return nodeIndex;
        }
        else{
            return newRightIndex;
        }
    }
    else{
        if(leftDistance<rightDistance){
            return newLeftIndex;
        }
        else{
            return newRightIndex;
        }
    }

}


GLint Mesh::getNearestNeighbor(float x, float y, float z)
{

    GLint nodeIndex = getNearestNeighborHelper2(nodes->length()-1,x,y,z);

    return nodes->at(nodeIndex).index;
}

void Mesh::setOctreeInteriors(GLfloat x,GLfloat y,GLfloat z, GLint maxDepth){

    QVector<GLfloat>* geometry = this->getGeometry();
    QVector<GLshort>* indices = this->getIndices();

    GLfloat* gvd = geometry->data();
    GLshort* ivd = indices->data();

    QVector<triObject>* triObjects = new QVector<triObject>();
    QVector<GLint> triObjectIndices;
    QVector<octreeNode>* nodes = new QVector<octreeNode>();

    int length = indices->length();

    GLfloat xaverage;
    GLfloat yaverage;
    GLfloat zaverage;

    GLdouble max_v = 0;

    GLdouble max_g = 0;

    for (GLint i=0;i<length;i+=3)
    {
        xaverage = gvd[ivd[i]*3+0];
        yaverage = gvd[ivd[i]*3+1];
        zaverage = gvd[ivd[i]*3+2];

        xaverage += gvd[ivd[i+1]*3+0];
        yaverage += gvd[ivd[i+1]*3+1];
        zaverage += gvd[ivd[i+1]*3+2];

        xaverage += gvd[ivd[i+2]*3+0];
        yaverage += gvd[ivd[i+2]*3+1];
        zaverage += gvd[ivd[i+2]*3+2];

        xaverage /=3;
        yaverage /=3;
        zaverage /=3;

        max_v = 0;

        max_v = max(max_v,fabs(gvd[ivd[i]*3+0]-xaverage));
        max_v = max(max_v,fabs(gvd[ivd[i]*3+1]-yaverage));
        max_v = max(max_v,fabs(gvd[ivd[i]*3+2]-zaverage));

        max_v = max(max_v,fabs(gvd[ivd[i+1]*3+0]-xaverage));
        max_v = max(max_v,fabs(gvd[ivd[i+1]*3+1]-yaverage));
        max_v = max(max_v,fabs(gvd[ivd[i+1]*3+2]-zaverage));

        max_v = max(max_v,fabs(gvd[ivd[i+2]*3+0]-xaverage));
        max_v = max(max_v,fabs(gvd[ivd[i+2]*3+1]-yaverage));
        max_v = max(max_v,fabs(gvd[ivd[i+2]*3+2]-zaverage));

        triObject obj;
        obj.index = i;
        obj.x = xaverage;
        obj.y = yaverage;
        obj.z = z;
        obj.halflength = max_v;

        triObjects->push_back(obj);
        triObjectIndices.push_back(i/3);

        max_g = max(max_g,fabs(gvd[ivd[i]*3+0]));
        max_g = max(max_g,fabs(gvd[ivd[i]*3+1]));
        max_g = max(max_g,fabs(gvd[ivd[i]*3+2]));

        max_g = max(max_g,fabs(gvd[ivd[i+1]*3+0]));
        max_g = max(max_g,fabs(gvd[ivd[i+1]*3+1]));
        max_g = max(max_g,fabs(gvd[ivd[i+1]*3+2]));

        max_g = max(max_g,fabs(gvd[ivd[i+2]*3+0]));
        max_g = max(max_g,fabs(gvd[ivd[i+2]*3+1]));
        max_g = max(max_g,fabs(geometry->data()[ivd[i+2]*3+2]));

    }

    max_g += 1;

    setOctreeInteriorsHelper(max_g,x,y,z,nodes,triObjects,&triObjectIndices,0,maxDepth);

    delete triObjects;

}

inline void Mesh::testIntersection(QVector<triObject>* triObjects,QVector<GLint>* newTriObjectIndices,QVector<GLint>* oldTriObjectIndices,GLfloat x,GLfloat y,GLfloat z,GLfloat length){

    int length2 = oldTriObjectIndices->length();


    GLint index;
    triObject obj;

    GLfloat halfLength = length/2;

    GLfloat xdiff,ydiff,zdiff;
    GLfloat distance;

    for (GLint i=0;i<length2;i++)
    {
        /*test intersection of bounding boxes*/

        index = oldTriObjectIndices->at(i);
        obj = triObjects->at(index);
        distance = halfLength+obj.halflength;

        xdiff = fabs(x+halfLength-obj.x);
        ydiff = fabs(y+halfLength-obj.y);
        zdiff = fabs(z+halfLength-obj.z);

        if(xdiff<distance && ydiff<distance && zdiff<distance){
           newTriObjectIndices->push_back(index);
        }

    }

}

inline bool Mesh::testIntersection2(QVector<triObject>* triObjects,QVector<GLint>* triObjectIndices,GLfloat x,GLfloat y,GLfloat z,GLfloat length){

    int length2 = triObjectIndices->length();

    for (GLint i=0;i<length2;i++)
    {
        /*test intersection of cube and triangle*/
    }

    return true;
}

GLint Mesh::setOctreeInteriorsHelper(
        GLfloat length,
        GLfloat x,
        GLfloat y,
        GLfloat z,
        QVector<octreeNode>* nodes,
        QVector<triObject>* triObjects,
        QVector<GLint>* triObjectIndices,
        GLint depth,
        GLint maxDepth)
{

    /*test all tri objects for intersection or whether max depth is reached*/

    bool cut = testIntersection2(triObjects,triObjectIndices,x,y,z,length);

    if(!cut || maxDepth<depth){

        octreeNode obj;

        obj.x=x;
        obj.y=y;
        obj.z=z;

        obj.childIndex0=-1;
        obj.childIndex1=-1;
        obj.childIndex2=-1;
        obj.childIndex3=-1;

        obj.childIndex4=-1;
        obj.childIndex5=-1;
        obj.childIndex6=-1;
        obj.childIndex7=-1;

        obj.depth=depth;
        obj.cut = cut;

        nodes->push_back(obj);
        return nodes->length()-1;
    }


    octreeNode obj;

    obj.x=x;
    obj.y=y;
    obj.z=z;

    GLint newDepth = depth+1;
    GLfloat newLength = length/2;

    QVector<GLint> triObjectIndices0;
    QVector<GLint> triObjectIndices1;
    QVector<GLint> triObjectIndices2;
    QVector<GLint> triObjectIndices3;

    QVector<GLint> triObjectIndices4;
    QVector<GLint> triObjectIndices5;
    QVector<GLint> triObjectIndices6;
    QVector<GLint> triObjectIndices7;

    testIntersection(triObjects,&triObjectIndices0,triObjectIndices,x,y,z,newLength);
    testIntersection(triObjects,&triObjectIndices1,triObjectIndices,x,y,z+newLength,newLength);
    testIntersection(triObjects,&triObjectIndices2,triObjectIndices,x,y+newLength,z,newLength);
    testIntersection(triObjects,&triObjectIndices3,triObjectIndices,x,y+newLength,z+newLength,newLength);

    testIntersection(triObjects,&triObjectIndices4,triObjectIndices,x+newLength,y,z,newLength);
    testIntersection(triObjects,&triObjectIndices5,triObjectIndices,x+newLength,y,z+newLength,newLength);
    testIntersection(triObjects,&triObjectIndices6,triObjectIndices,x+newLength,y+newLength,z,newLength);
    testIntersection(triObjects,&triObjectIndices7,triObjectIndices,x+newLength,y+newLength,z+newLength,newLength);

    obj.childIndex0=setOctreeInteriorsHelper(newLength,x,y,z,nodes,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex1=setOctreeInteriorsHelper(newLength,x,y,z+newLength,nodes,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex2=setOctreeInteriorsHelper(newLength,x,y+newLength,z,nodes,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex3=setOctreeInteriorsHelper(newLength,x,y+newLength,z+newLength,nodes,triObjects,triObjectIndices,newDepth,maxDepth);

    obj.childIndex4=setOctreeInteriorsHelper(newLength,x+newLength,y,z,nodes,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex5=setOctreeInteriorsHelper(newLength,x+newLength,y,z+newLength,nodes,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex6=setOctreeInteriorsHelper(newLength,x+newLength,y+newLength,z,nodes,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex7=setOctreeInteriorsHelper(newLength,x+newLength,y+newLength,z+newLength,nodes,triObjects,triObjectIndices,newDepth,maxDepth);

    obj.depth=depth;

    nodes->push_back(obj);
    return nodes->length()-1;

}
