#include "octree.h"

using namespace std;

Octree::Octree()
{
    /* rotations matrices */
    rotx.setToIdentity();
    rotx.rotate(90,1,0,0);
    roty.setToIdentity();
    roty.rotate(90,0,1,0);

    /* unit vectors */
    xvec.setX(1);
    xvec.setY(0);
    xvec.setZ(0);
    yvec.setX(0);
    yvec.setY(1);
    yvec.setZ(0);
    zvec.setX(0);
    zvec.setY(0);
    zvec.setZ(1);

    /* */
    this->allNodes = NULL;
    this->interiorLeafIndices = NULL;
    this->leafVector = NULL;

    this->freeIndicesBufferForAllNodes = new QVector<GLint>();
    this->freeIndicesBufferForInteriorLeafIndices = new QVector<GLint>();

    this->isDirty = true;
    this->rootIndex = -1;
}

void Octree::setOctreeInteriors(GLint maxDepth, Mesh* outerMesh, Mesh* innerMesh)
{
    setOctreeInteriors(
      outerMesh->getMean().x(),
      outerMesh->getMean().y(),
      outerMesh->getMean().z(),
      maxDepth,
      outerMesh,
      innerMesh);
}

void Octree::setOctreeInteriors(
        GLfloat x,
        GLfloat y,
        GLfloat z,
        GLint maxDepth,
        Mesh* outerMesh,
        Mesh* innerMesh)
{

    this->maxDepth = maxDepth;

    this->outerMesh = outerMesh;
    this->innerMesh = innerMesh;

    GLfloat* gvd_o = this->outerMesh->getGeometry()->data();
    GLint* ivd_o = this->outerMesh->getIndices()->data();
    int length_o = this->outerMesh->getIndices()->length();
    GLfloat* gvd_i = this->innerMesh->getGeometry()->data();
    GLint* ivd_i = this->innerMesh->getIndices()->data();
    int length_i = this->innerMesh->getIndices()->length();

    QVector<triObject> triObjects;
    triObjects.clear();
    triObjects.reserve(length_i);

    QVector<GLint> triObjectIndices;
    triObjectIndices.clear();
    triObjectIndices.reserve(length_i);

    if(this->allNodes != NULL)
    {
        delete this->allNodes;
    }
    if(this->interiorLeafIndices != NULL)
    {
        delete this->interiorLeafIndices;
    }

    this->allNodes = new QVector<octreeNode>();
    this->interiorLeafIndices = new QVector<GLint>();

    GLfloat xaverage, yaverage, zaverage;
    GLdouble max_v = 0;
    GLdouble max_g = 0;

    for (GLint i=0;i<length_i;i+=3)
    {

        // get average of the triangle
        xaverage = gvd_i[ivd_i[i]*3+0];
        yaverage = gvd_i[ivd_i[i]*3+1];
        zaverage = gvd_i[ivd_i[i]*3+2];
        xaverage += gvd_i[ivd_i[i+1]*3+0];
        yaverage += gvd_i[ivd_i[i+1]*3+1];
        zaverage += gvd_i[ivd_i[i+1]*3+2];
        xaverage += gvd_i[ivd_i[i+2]*3+0];
        yaverage += gvd_i[ivd_i[i+2]*3+1];
        zaverage += gvd_i[ivd_i[i+2]*3+2];

        xaverage /=3;yaverage /=3;zaverage /=3;

        max_v = 0;

        // looking for the maximal distance (average of triangle and point axis)
        max_v = max(max_v,fabs(gvd_i[ivd_i[i+0]*3+0]-xaverage));
        max_v = max(max_v,fabs(gvd_i[ivd_i[i+0]*3+1]-yaverage));
        max_v = max(max_v,fabs(gvd_i[ivd_i[i+0]*3+2]-zaverage));
        max_v = max(max_v,fabs(gvd_i[ivd_i[i+1]*3+0]-xaverage));
        max_v = max(max_v,fabs(gvd_i[ivd_i[i+1]*3+1]-yaverage));
        max_v = max(max_v,fabs(gvd_i[ivd_i[i+1]*3+2]-zaverage));
        max_v = max(max_v,fabs(gvd_i[ivd_i[i+2]*3+0]-xaverage));
        max_v = max(max_v,fabs(gvd_i[ivd_i[i+2]*3+1]-yaverage));
        max_v = max(max_v,fabs(gvd_i[ivd_i[i+2]*3+2]-zaverage));

        // set object for the triangle
        triObject obj;
        obj.index = i;
        obj.p.setX(xaverage);
        obj.p.setY(yaverage);
        obj.p.setZ(zaverage);
        obj.halflength = max_v;

        // add tri object and index
        triObjects.push_back(obj);
        triObjectIndices.push_back(i/3);

    }

    for (GLint i=0;i<length_o;i+=3)
    {

        // looking for the maximal distance (average object and point axis)
        max_g = max(max_g,fabs(gvd_o[ivd_o[i+0]*3+0]-x));
        max_g = max(max_g,fabs(gvd_o[ivd_o[i+0]*3+1]-y));
        max_g = max(max_g,fabs(gvd_o[ivd_o[i+0]*3+2]-z));
        max_g = max(max_g,fabs(gvd_o[ivd_o[i+1]*3+0]-x));
        max_g = max(max_g,fabs(gvd_o[ivd_o[i+1]*3+1]-y));
        max_g = max(max_g,fabs(gvd_o[ivd_o[i+1]*3+2]-z));
        max_g = max(max_g,fabs(gvd_o[ivd_o[i+2]*3+0]-x));
        max_g = max(max_g,fabs(gvd_o[ivd_o[i+2]*3+1]-y));
        max_g = max(max_g,fabs(gvd_o[ivd_o[i+2]*3+2]-z));

    }

    x -= max_g;y -= max_g;z -= max_g;
    max_g *= 2;

    setOctreeInteriorsHelper(max_g,x,y,z,&triObjects,&triObjectIndices,1,maxDepth);

    /* decide which leaf nodes are interior cells */
    setLeafVector(maxDepth);
    setBorder(maxDepth);
    traverseLeafVector(maxDepth);

    this->rootIndex = this->allNodes->length()-1;

    return;
}

/**
 * @brief Octree::testIntersection test whether the provided triangles and the provided cube intersect each other
 * @param triObjects triangle objects which are needed for intersection test
 * @param newTriObjectIndices filtered list which of indices of triangle objects
 * (for performance purposes)
 * @param oldTriObjectIndices all indices of regarded triangle objects
 * @param x x-coordinate of the specific cube point
 * @param y y- ...
 * @param z z- ...
 * @param length edge length of the cube
 */
inline void Octree::testIntersection(
        QVector<triObject>* triObjects,
        QVector<GLint>* newTriObjectIndices,
        QVector<GLint>* oldTriObjectIndices,
        GLfloat x,
        GLfloat y,
        GLfloat z,
        GLfloat length)
{

    int length2 = oldTriObjectIndices->length();

    QVector3D p,diff;
    p.setX(x);
    p.setY(y);
    p.setZ(z);

    GLint index;
    triObject obj;

    GLfloat halfLength = length/2;
    GLfloat distance;

    for (GLint i=0;i<length2;i++)
    {
        /*test intersection of bounding boxes*/

        index = oldTriObjectIndices->at(i);
        obj = triObjects->at(index);
        distance = halfLength+obj.halflength;

        diff = p-obj.p;
        diff.setX(diff.x()+halfLength);diff.setY(diff.y()+halfLength);diff.setZ(diff.z()+halfLength);

        if(fabs(diff.x())<distance && fabs(diff.y())<distance && fabs(diff.z())<distance){
           newTriObjectIndices->push_back(index);
        }

    }

}

/**
 * @brief Octree::lineIntersectsSquare test whether the provided line and the provided square intersect each other
 * @param halfLength the half lenght of the edge of the square
 * @param p0 the first point of the line
 * @param p1 the second point of the line
 * @return true if the line and the square intersect each other else false
 */
inline bool Octree::lineIntersectsSquare(GLfloat halfLength, QVector3D p0, QVector3D p1)
{

    QVector3D diff = p1-p0;

    /* test whether any point is inside of the square */
    if(fabs(p0.x())<=halfLength && fabs(p0.y())<=halfLength){return true;}
    if(fabs(p1.x())<=halfLength && fabs(p1.y())<=halfLength){return true;}

    /*
     * lambda is the scale factor for the line
     * if between 0 and 1 then intersection exists
     */
    GLfloat lambda;
    QVector3D testPoint;

    if(diff.x()!=0)
    {

        lambda = (halfLength-p0.x())/diff.x();
        if(0<=lambda && lambda<=1)
        {
            testPoint = p0+diff*lambda;
            if(fabs(testPoint.x())<=halfLength && fabs(testPoint.y())<=halfLength){return true;}
        }

        lambda = (-halfLength-p0.x())/diff.x();
        if(0<=lambda && lambda<=1)
        {
            testPoint = p0+diff*lambda;
            if(fabs(testPoint.x())<=halfLength && fabs(testPoint.y())<=halfLength){return true;}
        }

    }

    if(diff.y()!=0){

        lambda = (halfLength-p0.y())/diff.y();
        if(0<=lambda && lambda<=1)
        {
            testPoint = p0+diff*lambda;
            if(fabs(testPoint.x())<=halfLength && fabs(testPoint.y())<=halfLength){return true;}
        }

        lambda = (-halfLength-p0.y())/diff.y();
        if(0<=lambda && lambda<=1)
        {
            testPoint = p0+diff*lambda;
            if(fabs(testPoint.x())<=halfLength && fabs(testPoint.y())<=halfLength){return true;}
        }

    }

    return false;
}

/**
 * @brief Octree::triangleIntersectsPlane test whether the provided triangle and the provided plane intersect each other
 * @param halfLength the half lenght of the edge of the square
 * @param p first point of the triangle
 * @param n0 vector between first and second point of the triangle
 * @param n1 vector between first and third point of the triangle
 * @return true if the line and the square intersect each other
 */
inline bool Octree::triangleIntersectsPlane(
        GLfloat halfLength,
        QVector3D p,
        QVector3D n0,
        QVector3D n1)
{

    bool c0 = false;
    bool c1 = false;
    bool c2 = false;
    QVector3D cv0,cv1,cv2;

    if(n0.z()==0){
        if(p.z()==0){
            if(n1.z()==0){
                return lineIntersectsSquare(halfLength,p+n0,p) ||
                       lineIntersectsSquare(halfLength,p+n1,p) ||
                       lineIntersectsSquare(halfLength,p+n1,p+n0);
            }
            else{return lineIntersectsSquare(halfLength,p+n0,p);}
        }
    }
    else{

        GLfloat lambda = -p.z()/n0.z();

        if(0 <=lambda && lambda <=1){
            c0 = true;
            cv0 = p+n0*lambda;
        }
    }

    if(n1.z()==0)
    {
        if(p.z()==0){return lineIntersectsSquare(halfLength,p+n1,p);}
    }
    else{

        GLfloat lambda = -p.z()/n1.z();

        if(0 <=lambda && lambda <=1)
        {
            c1 = true;
            cv1 = p+n1*lambda;
        }
    }

    if( (c0&&!c1) || (!c0&&c1))
    {

        QVector3D p1 = p+n0;
        QVector3D n2 = n1-n0;

        GLfloat lambda = -p1.z()/n2.z();

        if(0 <=lambda && lambda <=1)
        {
            c2 = true;
            cv2 = p1+n2*lambda;
        }
    }

    // test intersection points for square test
    if(c0&&c1){return lineIntersectsSquare(halfLength,cv0,cv1);}
    if(c0&&c2){return lineIntersectsSquare(halfLength,cv0,cv2);}
    if(c1&&c2){return lineIntersectsSquare(halfLength,cv1,cv2);}

    return false;
}

/**
 * @brief Octree::triangleIntersectsCube test whether the provided triangle and the provided cube intersect each other
 * @param q middle point of the cube
 * @param halfLength half length of the edge the cube
 * @param p0 first point of the triangle
 * @param p1 second point of the triangle
 * @param p2 third point of the triangle
 * @return true if the triangle and the cube intersect each other else false
 */
inline bool Octree::triangleIntersectsCube(
        QVector3D q,
        GLfloat halfLength,
        QVector3D p0,
        QVector3D p1,
        QVector3D p2)
{

    QVector3D n0,n1,n0_1,n0_2,n1_1,n1_2;
    QVector3D vec0,vec1,vec2;

    n0 = p1-p0;
    n1 = p2-p0;
    n0_1 = this->roty.map(n0);
    n1_1 = this->roty.map(n1);
    n0_2 = this->rotx.map(n0);
    n1_2 = this->rotx.map(n1);

    vec0 = this->roty.map(((p0-q)+this->xvec*halfLength)); // pos x plane
    vec1 = this->rotx.map(((p0-q)+this->yvec*halfLength)); // pos y plane
    vec2 = ((p0-q)+this->zvec*halfLength); // pos z plane

    // test x-plane (negative)
    bool t1 = triangleIntersectsPlane(halfLength,vec0,n0_1,n1_1);
    if(t1){return true;}

    // test y-plane (negative)
    bool t2 = triangleIntersectsPlane(halfLength,vec1,n0_2,n1_2);
    if(t2){return true;}

    // test z-plane (negative)
    bool t3 = triangleIntersectsPlane(halfLength,vec2,n0,n1);
    if(t3){return true;}

    // neg x plane
    vec0 = this->roty.map(((p0-q)-this->xvec*halfLength));
    // neg y plane
    vec1 = this->rotx.map(((p0-q)-this->yvec*halfLength));
    // neg z plane
    vec2 = ((p0-q)-this->zvec*halfLength);

    // test x-plane (positive)
    bool t4 = triangleIntersectsPlane(halfLength,vec0,n0_1,n1_1);
    if(t4){return true;}

    // test y-plane (positive)
    bool t5 = triangleIntersectsPlane(halfLength,vec1,n0_2,n1_2);
    if(t5){return true;}

    // test z-plane (positive)
    bool t6 = triangleIntersectsPlane(halfLength,vec2,n0,n1);
    if(t6){return true;}

    return false;
}

/**
 * @brief Octree::testIntersection2 test intersection (2)
 * @param triObjects all triangle object
 * @param triObjectIndices indices for all triangle object which are needed for testing intersection
 * @param x x-coordinate of the specific point of the cube
 * @param y y- ...
 * @param z z- ...
 * @param length edge length of the cube
 * @return true if the cube and any triangle intersect each other else false
 */
inline bool Octree::testIntersection2(
        QVector<triObject>* triObjects,
        QVector<GLint>* triObjectIndices,
        GLfloat x,
        GLfloat y,
        GLfloat z,
        GLfloat length)
{

    GLfloat* gvd = this->innerMesh->getGeometry()->data();
    GLint* ivd = this->outerMesh->getIndices()->data();

    int length2 = triObjectIndices->length();

    GLfloat halfLength = length/2;
    GLfloat distance = halfLength;

    // get middle point
    QVector3D middle;
    middle.setX(x+halfLength);
    middle.setY(y+halfLength);
    middle.setZ(z+halfLength);

    GLint i0,i1,i2;
    QVector3D p0,p1,p2,diff;

    for (GLint i=0;i<length2;i++)
    {
        triObject obj = triObjects->at(triObjectIndices->at(i));

        i0 = ivd[obj.index+0]*3;
        i1 = ivd[obj.index+1]*3;
        i2 = ivd[obj.index+2]*3;

        // get points
        p0.setX( gvd[i0+0]);p0.setY( gvd[i0+1]);p0.setZ( gvd[i0+2]);
        p1.setX( gvd[i1+0]);p1.setY( gvd[i1+1]);p1.setZ( gvd[i1+2]);
        p2.setX( gvd[i2+0]);p2.setY( gvd[i2+1]);p2.setZ( gvd[i2+2]);

        // test whether both cubes do not intersect each other
        diff = p0-middle;
        if(fabs(diff.x())<distance && fabs(diff.y())<distance && fabs(diff.z())<distance){
            return true;
        }
        diff = p1-middle;
        if(fabs(diff.x())<distance && fabs(diff.y())<distance && fabs(diff.z())<distance){
            return true;
        }
        diff = p2-middle;
        if(fabs(diff.x())<distance && fabs(diff.y())<distance && fabs(diff.z())<distance){
            return true;
        }

    }

    for (GLint i=0;i<length2;i++)
    {
        triObject obj = triObjects->at(triObjectIndices->at(i));

        i0 = ivd[obj.index+0]*3;
        i1 = ivd[obj.index+1]*3;
        i2 = ivd[obj.index+2]*3;

        // get points
        p0.setX( gvd[i0+0]);p0.setY( gvd[i0+1]);p0.setZ( gvd[i0+2]);
        p1.setX( gvd[i1+0]);p1.setY( gvd[i1+1]);p1.setZ( gvd[i1+2]);
        p2.setX( gvd[i2+0]);p2.setY( gvd[i2+1]);p2.setZ( gvd[i2+2]);

        // test whether both cubes do not intersect each other
        diff = p0-middle;
        if(fabs(diff.x())<distance && fabs(diff.y())<distance && fabs(diff.z())<distance){
            return true;
        }
        diff = p1-middle;
        if(fabs(diff.x())<distance && fabs(diff.y())<distance && fabs(diff.z())<distance){
            return true;
        }
        diff = p2-middle;
        if(fabs(diff.x())<distance && fabs(diff.y())<distance && fabs(diff.z())<distance){
            return true;
        }

        /*test intersection of cube and triangle*/
        if(triangleIntersectsCube(middle,halfLength,p0,p1,p2)){
            return true;
        }

    }

    return false;
}

/**
 * @brief Octree::setOctreeInteriorsHelper helper method
 * @param length length of the current considered cube
 * @param x x-coordinate of the specific point
 * @param y y- ...
 * @param z z- ...
 * @param triObjects all triangle object
 * @param triObjectIndices indices for all triangle object which are needed for testing intersection
 * @param depth current depth
 * @param maxDepth maximum depth
 * @return the index of the created node in allNoded
 */
GLint Octree::setOctreeInteriorsHelper(
        GLfloat length,
        GLfloat x,
        GLfloat y,
        GLfloat z,
        QVector<triObject>* triObjects,
        QVector<GLint>* triObjectIndices,
        GLint depth,
        GLint maxDepth)
{

    if(maxDepth<depth){
        int i=0;
    }

    /*test all tri objects for intersection or whether max depth is reached*/
    bool intersect = testIntersection2(triObjects,triObjectIndices,x,y,z,length);

    if(!intersect || maxDepth<depth){

        octreeNode obj;

        obj.leaf = true;

        // set the positions for the corner points fo the cube
        obj.p0.setX(x);obj.p0.setY(y);obj.p0.setZ(z);
        obj.p1.setX(x);obj.p1.setY(y);obj.p1.setZ(z+length);
        obj.p2.setX(x);obj.p2.setY(y+length);obj.p2.setZ(z);
        obj.p3.setX(x);obj.p3.setY(y+length);obj.p3.setZ(z+length);

        obj.p4.setX(x+length);obj.p4.setY(y);obj.p4.setZ(z);
        obj.p5.setX(x+length);obj.p5.setY(y);obj.p5.setZ(z+length);
        obj.p6.setX(x+length);obj.p6.setY(y+length);obj.p6.setZ(z);
        obj.p7.setX(x+length);obj.p7.setY(y+length);obj.p7.setZ(z+length);


        // set flag for no sub cubes
        obj.childIndex0=-1;obj.childIndex1=-1;obj.childIndex2=-1;obj.childIndex3=-1;
        obj.childIndex4=-1;obj.childIndex5=-1;obj.childIndex6=-1;obj.childIndex7=-1;

        obj.parentIndex = -1;

        obj.nodeDepth = depth;

        obj.intersect = intersect;
        obj.index = allNodes->length();
        obj.interiorIndex = -1;
        obj.interior = false;
        obj.set = false;

        this->allNodes->push_back(obj);

        GLint index = obj.index;

        return index;
    }

    octreeNode obj;

    // set the positions for the corner points fo the cube
    obj.p0.setX(x);obj.p0.setY(y);obj.p0.setZ(z);
    obj.p1.setX(x);obj.p1.setY(y);obj.p1.setZ(z+length);
    obj.p2.setX(x);obj.p2.setY(y+length);obj.p2.setZ(z);
    obj.p3.setX(x);obj.p3.setY(y+length);obj.p3.setZ(z+length);

    obj.p4.setX(x+length);obj.p4.setY(y);obj.p4.setZ(z);
    obj.p5.setX(x+length);obj.p5.setY(y);obj.p5.setZ(z+length);
    obj.p6.setX(x+length);obj.p6.setY(y+length);obj.p6.setZ(z);
    obj.p7.setX(x+length);obj.p7.setY(y+length);obj.p7.setZ(z+length);


    GLint newDepth = depth+1;
    GLfloat newLength = length/2;

    // create tri objects for the new sub cubes
    QVector<GLint> triObjectIndices0;QVector<GLint> triObjectIndices1;
    QVector<GLint> triObjectIndices2;QVector<GLint> triObjectIndices3;
    QVector<GLint> triObjectIndices4;QVector<GLint> triObjectIndices5;
    QVector<GLint> triObjectIndices6;QVector<GLint> triObjectIndices7;


    // make a fast test for intersection and get return all candidates for complete test
    testIntersection(triObjects,&triObjectIndices0,triObjectIndices,x,y,z,newLength);
    testIntersection(triObjects,&triObjectIndices1,triObjectIndices,x,y,z+newLength,newLength);
    testIntersection(triObjects,&triObjectIndices2,triObjectIndices,x,y+newLength,z,newLength);
    testIntersection(triObjects,&triObjectIndices3,triObjectIndices,x,y+newLength,z+newLength,newLength);

    testIntersection(triObjects,&triObjectIndices4,triObjectIndices,x+newLength,y,z,newLength);
    testIntersection(triObjects,&triObjectIndices5,triObjectIndices,x+newLength,y,z+newLength,newLength);
    testIntersection(triObjects,&triObjectIndices6,triObjectIndices,x+newLength,y+newLength,z,newLength);
    testIntersection(triObjects,&triObjectIndices7,triObjectIndices,x+newLength,y+newLength,z+newLength,newLength);

    obj.leaf = false;

    // recursive step (eight new sub cubes)
    obj.childIndex0=setOctreeInteriorsHelper(newLength,x,y,z,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex1=setOctreeInteriorsHelper(newLength,x,y,z+newLength,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex2=setOctreeInteriorsHelper(newLength,x,y+newLength,z,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex3=setOctreeInteriorsHelper(newLength,x,y+newLength,z+newLength,triObjects,triObjectIndices,newDepth,maxDepth);

    obj.childIndex4=setOctreeInteriorsHelper(newLength,x+newLength,y,z,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex5=setOctreeInteriorsHelper(newLength,x+newLength,y,z+newLength,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex6=setOctreeInteriorsHelper(newLength,x+newLength,y+newLength,z,triObjects,triObjectIndices,newDepth,maxDepth);
    obj.childIndex7=setOctreeInteriorsHelper(newLength,x+newLength,y+newLength,z+newLength,triObjects,triObjectIndices,newDepth,maxDepth);

    obj.nodeDepth = depth;

    obj.intersect = true;
    obj.index = allNodes->length();
    obj.interiorIndex = -1;
    obj.interior = false;
    obj.set = false;

    allNodes->push_back(obj);

    GLint index = obj.index;

    /* set parent index for sub cubes */
    allNodes->data()[obj.childIndex0].parentIndex = index;
    allNodes->data()[obj.childIndex1].parentIndex = index;
    allNodes->data()[obj.childIndex2].parentIndex = index;
    allNodes->data()[obj.childIndex3].parentIndex = index;

    allNodes->data()[obj.childIndex4].parentIndex = index;
    allNodes->data()[obj.childIndex5].parentIndex = index;
    allNodes->data()[obj.childIndex6].parentIndex = index;
    allNodes->data()[obj.childIndex7].parentIndex = index;

    return index;

}

/**
 * @brief Octree::setLeafVectorHelper helper method
 * @param maxDepth maximum depth
 * @param currentDepth current depth
 * @param index index of the node
 * @param x x-coordinate of cube (index)
 * @param y y- ...
 * @param z z- ...
 */
void Octree::setLeafVectorHelper(
        GLint maxDepth,
        GLint currentDepth,
        GLint index,
        GLint x,
        GLint y,
        GLint z)
{
    octreeNode node = this->allNodes->at(index);

    GLint axisSize = 2<<(maxDepth-1);
    GLint currentAxisSize = axisSize>>currentDepth;

    GLint planeSize = pow(axisSize,2);

    /* if leaf node the set indices into the leaf vector */
    if(node.leaf)
    {

        for(int i=0;i<currentAxisSize;i++)
        {
            for(int j=0;j<currentAxisSize;j++)
            {
                for(int k=0;k<currentAxisSize;k++)
                {
                    this->leafVector->data()[planeSize*(i+x)+axisSize*(j+y)+(k+z)] = node.index;
                }
            }
        }

        return;
    }

    GLint nextAxisSize = currentAxisSize>>1;

    currentDepth++;

    /* recursion with all child nodes */
    setLeafVectorHelper(maxDepth,currentDepth,node.childIndex0,x,y,z);
    setLeafVectorHelper(maxDepth,currentDepth,node.childIndex1,x,y,z+nextAxisSize);
    setLeafVectorHelper(maxDepth,currentDepth,node.childIndex2,x,y+nextAxisSize,z);
    setLeafVectorHelper(maxDepth,currentDepth,node.childIndex3,x,y+nextAxisSize,z+nextAxisSize);

    setLeafVectorHelper(maxDepth,currentDepth,node.childIndex4,x+nextAxisSize,y,z);
    setLeafVectorHelper(maxDepth,currentDepth,node.childIndex5,x+nextAxisSize,y,z+nextAxisSize);
    setLeafVectorHelper(maxDepth,currentDepth,node.childIndex6,x+nextAxisSize,y+nextAxisSize,z);
    setLeafVectorHelper(maxDepth,currentDepth,node.childIndex7,x+nextAxisSize,y+nextAxisSize,z+nextAxisSize);

}

void Octree::setLeafVector(GLint maxDepth)
{
    this->leafVector = new QVector<GLint>();

    GLint size = pow(8,maxDepth);

    this->leafVector->reserve(size);

    for(int i=0;i<size;i++)
    {
        this->leafVector->push_back(-1);
    }

    setLeafVectorHelper(maxDepth,0,this->allNodes->length()-1,0,0,0);

}

/**
 * @brief Octree::setBorder mark border cubes which do not intersect the surface
 * @param maxDepth maximum depth which will be considered
 */
void Octree::setBorder(GLint maxDepth)
{

    GLint axisSize = 2<<(maxDepth-1);
    GLint axisSizeS = axisSize-1;
    GLint planeSize = pow(axisSize,2);

    int startIndex = 0;
    int endIndex = axisSizeS;

    octreeNode node;

    // travers over all leaf cubes
    for(int i=0;i<axisSize;i++)
    {
        for(int j=0;j<axisSize;j++)
        {

            /* if cube does not intersect then cube is marked as border cube */

            node = this->allNodes->at(this->leafVector->at(planeSize*startIndex+axisSize*i+j));
            if(!node.intersect)
            {
                node.interior = false;
                node.set = true;
                this->allNodes->data()[this->leafVector->at(planeSize*startIndex+axisSize*i+j)] = node;
            }
            node = this->allNodes->at(this->leafVector->at(planeSize*endIndex+axisSize*i+j));
            if(!node.intersect)
            {
                node.interior = false;
                node.set = true;
                this->allNodes->data()[this->leafVector->at(planeSize*endIndex+axisSize*i+j)] = node;
            }

            node = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*startIndex+j));
            if(!node.intersect)
            {
                node.interior = false;
                node.set = true;
                this->allNodes->data()[this->leafVector->at(planeSize*i+axisSize*startIndex+j)] = node;
            }
            node = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*endIndex+j));
            if(!node.intersect)
            {
                node.interior = false;
                node.set = true;
                this->allNodes->data()[this->leafVector->at(planeSize*i+axisSize*endIndex+j)] = node;
            }

            node = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*j+startIndex));
            if(!node.intersect)
            {
                node.interior = false;
                node.set = true;
                this->allNodes->data()[this->leafVector->at(planeSize*i+axisSize*j+startIndex)] = node;
            }
            node = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*j+endIndex));
            if(!node.intersect)
            {
                node.interior = false;
                node.set = true;
                this->allNodes->data()[this->leafVector->at(planeSize*i+axisSize*j+endIndex)] = node;
            }

        }
    }


}

/**
 * @brief Octree::traverseLeafVector mark all border cubes recursively
 * @param maxDepth maximum depth which will be considered
 */
void Octree::traverseLeafVector(GLint maxDepth)
{

    bool loopIsDirty;

    GLint axisSize = 2<<(maxDepth-1);
    GLint axisSizeS = axisSize-1;
    GLint planeSize = pow(axisSize,2);

    /* do this until no new outer cell is found */
    do{

        loopIsDirty = false;

        for(int i=1;i<axisSizeS;i++)
        {
            for(int j=1;j<axisSizeS;j++)
            {
                for(int k=1;k<axisSizeS;k++)
                {

                    octreeNode currentNode = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*j+k));

                    if(currentNode.intersect || currentNode.set){continue;}

                    /* the neighbor cell */
                    octreeNode node0 = this->allNodes->at(this->leafVector->at(planeSize*(i-1)+axisSize*j+k));
                    octreeNode node1 = this->allNodes->at(this->leafVector->at(planeSize*(i+1)+axisSize*j+k));
                    octreeNode node2 = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*(j-1)+k));
                    octreeNode node3 = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*(j+1)+k));
                    octreeNode node4 = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*j+(k-1)));
                    octreeNode node5 = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*j+(k+1)));



                    /* if there is one outer neighbor cell then this one is also an outer cell */
                    if( (node0.set && !node0.interior) ||
                        (node1.set && !node1.interior) ||
                        (node2.set && !node2.interior) ||
                        (node3.set && !node3.interior) ||
                        (node4.set && !node4.interior) ||
                        (node5.set && !node5.interior))
                    {
                         currentNode.set = true;
                         currentNode.interior = false;

                         this->allNodes->data()[this->leafVector->at(planeSize*i+axisSize*j+k)] = currentNode;

                         loopIsDirty = true;
                    }
                }
            }
        }

    }
    while(loopIsDirty);

    axisSizeS = axisSize;

    /* set all nodes which are not set already as interior cells */
    for(int i=0;i<axisSizeS;i++)
    {
        for(int j=0;j<axisSizeS;j++)
        {
            for(int k=0;k<axisSizeS;k++)
            {

                octreeNode currentNode = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*j+k));

                if(currentNode.intersect || currentNode.set){continue;}

                currentNode.set = true;
                currentNode.interior = true;
                currentNode.interiorIndex = this->interiorLeafIndices->length();

                this->allNodes->data()[this->leafVector->at(planeSize*i+axisSize*j+k)] = currentNode;

                this->interiorLeafIndices->push_back(currentNode.index);
            }
        }
    }

}

/**
 * @brief Octree::handleHashItem handle hash item to avoid setting an existing point
 * @param vertexIndex index of leaf cube
 * @param point point which will be stored
 * @return the index of the point in the hash map
 */
GLint inline Octree::handleHashItem(GLint vertexIndex,QVector3D point)
{

    if (geometryMap->contains(vertexIndex))
    {
        return geometryMap->value(vertexIndex).index;
    }

    hashItem item;

    item.vertex = point;
    item.index = geometryMap->size();

    geometryMap->insert(vertexIndex, item);

    return item.index;

}

bool Octree::allChildrenAreLeaves(GLint nodeIndex)
{
    if(nodeIndex<0)
    {
        return false;
    }

    octreeNode* node = this->getOctreeNode(nodeIndex);

    return
            isLeaf(node->childIndex0) &&
            isLeaf(node->childIndex1) &&
            isLeaf(node->childIndex2) &&
            isLeaf(node->childIndex3) &&
            isLeaf(node->childIndex4) &&
            isLeaf(node->childIndex5) &&
            isLeaf(node->childIndex6) &&
            isLeaf(node->childIndex7);
}

bool inline Octree::isLeaf(GLint nodeIndex)
{
    if(nodeIndex<0)
    {
        return false;
    }

    octreeNode* node = this->getOctreeNode(nodeIndex);

    return node->childIndex0<0;
}


GLint Octree::getRootID()
{
    return this->rootIndex;
}

void Octree::getAllInnerNodeIDsOfDepth(QVector<GLint>* vec,GLint depth)
{
    if(this->rootIndex<0)
    {
        return;
    }

    octreeNode* node = this->getOctreeNode(this->rootIndex);

    if(node->nodeDepth == depth && (node->intersect || node->interior) )
    {
        vec->push_back(node->index);
        return;
    }

    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex0);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex1);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex2);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex3);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex4);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex5);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex6);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex7);
}

void Octree::getAllInnerNodeLeafIDs(QVector<GLint>* vec)
{
    getAllInnerNodeLeafIDsOfNode(vec,this->rootIndex);
}

void Octree::getAllInnerNodeLeafIDsOfNode(QVector<GLint>* vec,GLint nodeIndex)
{

    if(nodeIndex<0)
    {
        return;
    }

    octreeNode* node = this->getOctreeNode(nodeIndex);

    if(node->interior && isLeaf(node->index))
    {
        vec->push_back(nodeIndex);
    }

    /* */
    getAllInnerNodeLeafIDsOfNode(vec,node->childIndex0);
    getAllInnerNodeLeafIDsOfNode(vec,node->childIndex1);
    getAllInnerNodeLeafIDsOfNode(vec,node->childIndex2);
    getAllInnerNodeLeafIDsOfNode(vec,node->childIndex3);
    getAllInnerNodeLeafIDsOfNode(vec,node->childIndex4);
    getAllInnerNodeLeafIDsOfNode(vec,node->childIndex5);
    getAllInnerNodeLeafIDsOfNode(vec,node->childIndex6);
    getAllInnerNodeLeafIDsOfNode(vec,node->childIndex7);

}

/**
 * @brief Octree::getAllInnerNodeIDsOfDepthHelper helper method
 * @param vec container for collected indices
 * @param depth searching depth
 * @param nodeIndex index of current considered node
 */
void Octree::getAllInnerNodeIDsOfDepthHelper(QVector<GLint>* vec,GLint depth,GLint nodeIndex)
{

    if(nodeIndex<0){
        return;
    }

    octreeNode* node = this->getOctreeNode(nodeIndex);

    if(node->nodeDepth == depth && (node->intersect || node->interior) )
    {
        vec->push_back(nodeIndex);
        return;
    }

    /* */
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex0);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex1);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex2);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex3);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex4);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex5);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex6);
    getAllInnerNodeIDsOfDepthHelper(vec,depth,node->childIndex7);

}

octreeNode* Octree::getOctreeNode(GLint index)
{
    return &this->allNodes->data()[index];
}

/**
 * @brief Octree::addSurface TODO
 * @param inner
 * @param code
 * @param i
 * @param j
 * @param k
 * @param axisSize
 * @param planeSize
 */
void Octree::addSurface(
        octreeNode inner,
        GLint code,
        GLint i,
        GLint j,
        GLint k,
        GLint axisSize,
        GLint planeSize)
{

    GLint index0;
    GLint index1;
    GLint index2;
    GLint index3;

    /* TODO get correct point */
    QVector3D vec;

    switch(code){

    case 0:

        index0 = handleHashItem(planeSize*i+axisSize*j+k,vec);
        index1 = handleHashItem(planeSize*i+axisSize*j+k,vec);
        index2 = handleHashItem(planeSize*i+axisSize*j+k,vec);
        index3 = handleHashItem(planeSize*i+axisSize*j+k,vec);

        indices->push_back(index0);
        indices->push_back(index1);
        indices->push_back(index2);
        indices->push_back(index1);
        indices->push_back(index3);
        indices->push_back(index2);

        break;

    case 1:

        /*
        index0 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index1 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index2 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index3 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        */

        break;

    case 2:

        /*
        index0 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index1 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index2 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index3 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        */

        break;

    case 3:

        /*
        index0 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index1 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index2 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index3 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        */

        break;

    case 4:

        /*
        index0 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index1 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index2 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index3 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        */

        break;

    case 5:

        /*
        index0 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index1 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index2 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        index3 = handleHashItem(planeSize*i+axisSize*j+k,NULL);
        */

        break;
    }

    /*
    indices->push_back(index0);
    indices->push_back(index1);
    indices->push_back(index2);
    indices->push_back(index1);
    indices->push_back(index3);
    indices->push_back(index2);
    */

}

void Octree::setVoidSurface()
{

    GLint axisSize = 2<<(maxDepth-1);
    GLint planeSize = pow(axisSize,2);

    GLint verticesAxisSize = (2<<(maxDepth-1))+1;
    GLint verticesPlaneSize = pow(axisSize,2);

    geometry = new QVector<GLfloat>();
    geometryMap = new QHash<GLint, hashItem>();
    indices = new QVector<GLint>();

    /* set all nodes which are not set already as interior cells */
    for(int i=1;i<axisSize-1;i++)
    {
        for(int j=1;j<axisSize-1;j++)
        {
            for(int k=1;k<axisSize-1;k++)
            {

                octreeNode currentNode = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*j+k));

                if(!currentNode.interior){continue;}

                /* the neighbor cell */
                octreeNode node0 = this->allNodes->at(this->leafVector->at(planeSize*(i-1)+axisSize*j+k));
                octreeNode node1 = this->allNodes->at(this->leafVector->at(planeSize*(i+1)+axisSize*j+k));
                octreeNode node2 = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*(j-1)+k));
                octreeNode node3 = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*(j+1)+k));
                octreeNode node4 = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*j+(k-1)));
                octreeNode node5 = this->allNodes->at(this->leafVector->at(planeSize*i+axisSize*j+(k+1)));

                if(!node0.interior){addSurface(currentNode,0,i,j,k,verticesAxisSize,verticesPlaneSize);}
                if(!node1.interior){addSurface(currentNode,1,i,j,k,verticesAxisSize,verticesPlaneSize);}
                if(!node2.interior){addSurface(currentNode,2,i,j,k,verticesAxisSize,verticesPlaneSize);}
                if(!node3.interior){addSurface(currentNode,3,i,j,k,verticesAxisSize,verticesPlaneSize);}
                if(!node4.interior){addSurface(currentNode,4,i,j,k,verticesAxisSize,verticesPlaneSize);}
                if(!node5.interior){addSurface(currentNode,5,i,j,k,verticesAxisSize,verticesPlaneSize);}

            }
        }
    }

}

void Octree::merge(octreeNode* parent)
{

    if(!this->allChildrenAreLeaves(parent->index))
    {
        return;
    }

    /* set the indices as free for avoiding too many empty holes*/
    this->freeIndicesBufferForAllNodes->push_back(parent->childIndex0);
    this->freeIndicesBufferForAllNodes->push_back(parent->childIndex1);
    this->freeIndicesBufferForAllNodes->push_back(parent->childIndex2);
    this->freeIndicesBufferForAllNodes->push_back(parent->childIndex3);

    this->freeIndicesBufferForAllNodes->push_back(parent->childIndex4);
    this->freeIndicesBufferForAllNodes->push_back(parent->childIndex5);
    this->freeIndicesBufferForAllNodes->push_back(parent->childIndex6);
    this->freeIndicesBufferForAllNodes->push_back(parent->childIndex7);

    /* ---------------------------- */

    octreeNode node;

    /* the children will be removed from interiorNodeLeafIndices*/
    node = this->allNodes->data()[parent->childIndex0];
    this->interiorLeafIndices->data()[node.interiorIndex] = -1;
    this->freeIndicesBufferForInteriorLeafIndices->push_back(node.interiorIndex);

    node = this->allNodes->data()[parent->childIndex1];
    this->interiorLeafIndices->data()[node.interiorIndex] = -1;
    this->freeIndicesBufferForInteriorLeafIndices->push_back(node.interiorIndex);

    node = this->allNodes->data()[parent->childIndex2];
    this->interiorLeafIndices->data()[node.interiorIndex] = -1;
    this->freeIndicesBufferForInteriorLeafIndices->push_back(node.interiorIndex);

    node = this->allNodes->data()[parent->childIndex3];
    this->interiorLeafIndices->data()[node.interiorIndex] = -1;
    this->freeIndicesBufferForInteriorLeafIndices->push_back(node.interiorIndex);

    node = this->allNodes->data()[parent->childIndex4];
    this->interiorLeafIndices->data()[node.interiorIndex] = -1;
    this->freeIndicesBufferForInteriorLeafIndices->push_back(node.interiorIndex);

    node = this->allNodes->data()[parent->childIndex5];
    this->interiorLeafIndices->data()[node.interiorIndex] = -1;
    this->freeIndicesBufferForInteriorLeafIndices->push_back(node.interiorIndex);

    node = this->allNodes->data()[parent->childIndex6];
    this->interiorLeafIndices->data()[node.interiorIndex] = -1;
    this->freeIndicesBufferForInteriorLeafIndices->push_back(node.interiorIndex);

    node = this->allNodes->data()[parent->childIndex7];
    this->interiorLeafIndices->data()[node.interiorIndex] = -1;
    this->freeIndicesBufferForInteriorLeafIndices->push_back(node.interiorIndex);

    /* delete the references */
    parent->childIndex0=-1;parent->childIndex1=-1;parent->childIndex2=-1;parent->childIndex3=-1;
    parent->childIndex4=-1;parent->childIndex5=-1;parent->childIndex6=-1;parent->childIndex7=-1;

    if(this->freeIndicesBufferForInteriorLeafIndices != NULL && !this->freeIndicesBufferForInteriorLeafIndices->empty())
    {
       parent->interiorIndex = this->freeIndicesBufferForInteriorLeafIndices->last();
       this->freeIndicesBufferForInteriorLeafIndices->pop_back();
       this->interiorLeafIndices->data()[parent->interiorIndex] = parent->index;
    }
    else
    {
       parent->interiorIndex = this->interiorLeafIndices->length();
       this->interiorLeafIndices->push_back((parent->index));
    }

}

void Octree::merge(GLint parentIndex)
{
    octreeNode parent = this->allNodes->data()[parentIndex];
    merge(&parent);
    this->allNodes->data()[parentIndex] = parent;
}

void Octree::split(octreeNode* node)
{

    if(!this->isLeaf(node->index)){
        return;
    }

    GLfloat newLength = (node->p0-node->p1).length()/2;

    /* the the position of the node (corner point) */
    GLfloat x = node->p0.x();
    GLfloat y = node->p0.y();
    GLfloat z = node->p0.z();

    GLint newDepth = node->nodeDepth+1;

    bool addToInterios = node->interior;

    /* create sub cubes */
    node->childIndex0=createNode(newLength,x,y,z,newDepth,addToInterios);
    node->childIndex1=createNode(newLength,x,y,z+newLength,newDepth,addToInterios);
    node->childIndex2=createNode(newLength,x,y+newLength,z,newDepth,addToInterios);
    node->childIndex3=createNode(newLength,x,y+newLength,z+newLength,newDepth,addToInterios);
    node->childIndex4=createNode(newLength,x+newLength,y,z,newDepth,addToInterios);
    node->childIndex5=createNode(newLength,x+newLength,y,z+newLength,newDepth,addToInterios);
    node->childIndex6=createNode(newLength,x+newLength,y+newLength,z,newDepth,addToInterios);
    node->childIndex7=createNode(newLength,x+newLength,y+newLength,z+newLength,newDepth,addToInterios);

    if(node->interior)
    {
        this->interiorLeafIndices->data()[node->interiorIndex] = -1;
        this->freeIndicesBufferForInteriorLeafIndices->push_back(node->interiorIndex);
        node->interiorIndex = -1;
    }

    GLint index = node->index;

    /* set parent index */
    this->allNodes->data()[node->childIndex0].parentIndex = index;
    this->allNodes->data()[node->childIndex1].parentIndex = index;
    this->allNodes->data()[node->childIndex2].parentIndex = index;
    this->allNodes->data()[node->childIndex3].parentIndex = index;

    this->allNodes->data()[node->childIndex4].parentIndex = index;
    this->allNodes->data()[node->childIndex5].parentIndex = index;
    this->allNodes->data()[node->childIndex6].parentIndex = index;
    this->allNodes->data()[node->childIndex7].parentIndex = index;

}

void Octree::split(GLint nodeIndex)
{
    octreeNode node = this->allNodes->data()[nodeIndex];
    split(&node);
    this->allNodes->data()[nodeIndex] = node;
}

/**
 * @brief Octree::createNode helper method for creating and adding a new octree node (cube)
 * @param length edge length of the cube
 * @param x x-coordinate of the basic vertex
 * @param y y-coordinate ...
 * @param z z-coordinate ...
 * @param nodeDepth depth of the node in the octree
 * @param addToInteriors if true the node will be added to interiorNodeLeafIndices
 * @return index of the created octree node (cube)
 */
GLint Octree::createNode(
        GLfloat length,
        GLfloat x,
        GLfloat y,
        GLfloat z,
        GLint nodeDepth,
        bool addToInteriors)
{

    octreeNode obj;

    // set the positions for the corner points fo the cube
    obj.p0.setX(x);obj.p0.setY(y);obj.p0.setZ(z);
    obj.p1.setX(x);obj.p1.setY(y);obj.p1.setZ(z+length);
    obj.p2.setX(x);obj.p2.setY(y+length);obj.p2.setZ(z);
    obj.p3.setX(x);obj.p3.setY(y+length);obj.p3.setZ(z+length);
    obj.p4.setX(x+length);obj.p4.setY(y);obj.p4.setZ(z);
    obj.p5.setX(x+length);obj.p5.setY(y);obj.p5.setZ(z+length);
    obj.p6.setX(x+length);obj.p6.setY(y+length);obj.p6.setZ(z);
    obj.p7.setX(x+length);obj.p7.setY(y+length);obj.p7.setZ(z+length);

    // set flag for no sub cubes
    obj.childIndex0=-1;obj.childIndex1=-1;obj.childIndex2=-1;obj.childIndex3=-1;
    obj.childIndex4=-1;obj.childIndex5=-1;obj.childIndex6=-1;obj.childIndex7=-1;

    obj.parentIndex = -1;

    obj.nodeDepth = nodeDepth;

    obj.intersect = false;

    obj.interior = addToInteriors;

    /* add node in the data structures (avoid waste of storage by using voids in the vectors) */
    if(this->freeIndicesBufferForAllNodes != NULL && !this->freeIndicesBufferForAllNodes->empty())
    {
        obj.index = this->freeIndicesBufferForAllNodes->last();
        this->freeIndicesBufferForAllNodes->pop_back();
        this->allNodes->data()[obj.index] = obj;
    }
    else
    {
        obj.index = this->allNodes->length();
        this->allNodes->push_back(obj);
    }

    /* add to interiors if parent is an interior */
    if(addToInteriors)
    {
        if(this->freeIndicesBufferForInteriorLeafIndices != NULL && !this->freeIndicesBufferForInteriorLeafIndices->empty())
        {
            obj.interiorIndex = this->freeIndicesBufferForInteriorLeafIndices->last();
            this->freeIndicesBufferForInteriorLeafIndices->pop_back();
            this->interiorLeafIndices->data()[obj.interiorIndex] = obj.index;
        }
        else
        {
            obj.interiorIndex = this->interiorLeafIndices->length();
            this->interiorLeafIndices->push_back((obj.index));
        }
    }

    /* is needed because of the interior index */
    this->allNodes->data()[obj.index] = obj;

    GLint index = obj.index;

    return index;
}

void Octree::render(QGLShaderProgram* shader)
{

    if(this->interiorLeafIndices == NULL || this->interiorLeafIndices->length()<1){
        return;
    }

    GLenum primitive = GL_LINES;

    QVector<GLint>* indices = this->interiorLeafIndices;

    /* if needed needed variables are set for painting */
    if (isDirty) {
        QVector<GLfloat> buffer;

        buffer.reserve(indices->length() * 8 * 3 * sizeof(GLfloat));

        cubeLineIndices.clear();
        GLint index = 0;

        for (int i = 0; i < indices->length(); i++) {

            if(indices->at(i)<0)
            {
                continue;
            }

            octreeNode node = this->allNodes->at(indices->at(i));

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
        vbo->allocate(indices->length() * 8 * 3 * sizeof(GLfloat));
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
