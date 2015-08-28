#include "meshbooleanfunctions.h"

Mesh* booleanHelper(Mesh* inputmesh1, Mesh* inputmesh2,GLint type)
{

    int length = 0;

    // set cork mesh for the first mesh
    CorkTriMesh mesh1;
    mesh1.n_triangles = inputmesh1->getIndices()->length()/3;
    mesh1.n_vertices = inputmesh1->getGeometry()->length()/3;

    length = inputmesh1->getIndices()->length();
    mesh1.triangles = new uint[length];
    for(int i=0;i<length;i++)
    {
        mesh1.triangles[i] = inputmesh1->getIndices()->data()[i];
    }

    mesh1.vertices = inputmesh1->getGeometry()->data();

    // set cork mesh for the second mesh
    CorkTriMesh mesh2;
    mesh2.n_triangles = (uint)inputmesh2->getIndices()->length()/3;
    mesh2.n_vertices = inputmesh2->getGeometry()->length()/3;

    length = inputmesh2->getIndices()->length();
    mesh2.triangles = new uint[length];
    for(int i=0;i<length;i++)
    {
        mesh2.triangles[i] = inputmesh2->getIndices()->data()[i];
    }

    mesh2.vertices = inputmesh2->getGeometry()->data();

    CorkTriMesh result;

    // execute the boolean operation
    switch(type){

    case UNION_BOOLEAN:
        computeUnion(mesh1,mesh2,&result);
        break;

    case DIFFERENCE_BOOLEAN:
        computeDifference(mesh1,mesh2,&result);
        break;

    default:
        delete mesh1.triangles;
        delete mesh2.triangles;
        return NULL;
    }

    delete mesh1.triangles;
    delete mesh2.triangles;

    // create the new mesh for the result

    QVector<GLfloat>* geometry = new QVector<GLfloat>();
    QVector<GLint>* indices = new QVector<GLint>();

    for(unsigned int i=0;i<result.n_triangles*3;i+=3)
    {
        indices->push_back(result.triangles[i+0]);
        indices->push_back(result.triangles[i+1]);
        indices->push_back(result.triangles[i+2]);

    }

    for(unsigned int i=0;i<result.n_vertices*3;i+=3)
    {
        geometry->push_back(result.vertices[i+0]);
        geometry->push_back(result.vertices[i+1]);
        geometry->push_back(result.vertices[i+2]);

    }

    Mesh* resultMesh = new Mesh(geometry,indices);

    return resultMesh;
}

/**
 * @brief booleanUnion Calculate the union of two meshes
 * @param inputmesh1 the first mesh
 * @param inputmesh2 the second mesh
 * @return the union of the first and second mesh
 */
Mesh* booleanUnion(Mesh* inputmesh1, Mesh* inputmesh2)
{
    return booleanHelper(inputmesh1,inputmesh2,UNION_BOOLEAN);
}

/**
 * @brief booleanDifference Calculate the difference of two meshes
 * @param inputmesh1 the first mesh
 * @param inputmesh2 the second mesh
 * @return the union of the first and second mesh
 */
Mesh* booleanDifference(Mesh* inputmesh1, Mesh* inputmesh2)
{
    return booleanHelper(inputmesh1,inputmesh2,DIFFERENCE_BOOLEAN);
}
