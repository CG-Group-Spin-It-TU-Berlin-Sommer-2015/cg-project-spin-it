#include "extendedmeshmerger.h"

Mesh* booleanUnion(Mesh* inputmesh1, Mesh* inputmesh2)
{

    int length = 0;

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

    CorkTriMesh mesh3;
    mesh3.n_triangles = (uint)inputmesh2->getIndices()->length()/3;
    mesh3.n_vertices = inputmesh2->getGeometry()->length()/3;

    length = inputmesh2->getIndices()->length();
    mesh3.triangles = new uint[length];
    for(int i=0;i<length;i++)
    {
        mesh3.triangles[i] = inputmesh2->getIndices()->data()[i];
    }

    mesh3.vertices = inputmesh2->getGeometry()->data();

    CorkTriMesh result;

    computeUnion(mesh1,mesh3,&result);

    delete mesh1.triangles;
    delete mesh3.triangles;

    QVector<GLfloat>* geometry = new QVector<GLfloat>();
    QVector<GLint>* indices = new QVector<GLint>();

    for(int i=0;i<result.n_triangles*3;i+=3)
    {
        indices->push_back(result.triangles[i+0]);
        indices->push_back(result.triangles[i+1]);
        indices->push_back(result.triangles[i+2]);

    }

    for(int i=0;i<result.n_vertices*3;i+=3)
    {
        geometry->push_back(result.vertices[i+0]);
        geometry->push_back(result.vertices[i+1]);
        geometry->push_back(result.vertices[i+2]);

    }

    Mesh* resultMesh = new Mesh(geometry,indices);

    return resultMesh;
}
