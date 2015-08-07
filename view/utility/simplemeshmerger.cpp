#include "simplemeshmerger.h"

Mesh* mergeMeshes(Mesh* mesh1, Mesh* mesh2)
{

    GLint geoMesh1Length = mesh1->getGeometry()->length();
    GLint geoMesh2Length = mesh2->getGeometry()->length();

    GLfloat* geo1 = mesh1->getGeometry()->data();
    GLfloat* geo2 = mesh2->getGeometry()->data();

    QVector<GLfloat>* geometry = new QVector<GLfloat>();
    geometry->reserve(geoMesh1Length+geoMesh2Length);

    for(int i=0;i<geoMesh1Length;i++)
    {
        geometry->push_back(geo1[i]);
    }

    for(int i=0;i<geoMesh2Length;i++)
    {
        geometry->push_back(geo2[i]);
    }

    GLint indicesMesh1Length = mesh1->getIndices()->length();
    GLint indicesMesh2Length = mesh2->getIndices()->length();

    QVector<GLint>* indices = new QVector<GLint>();
    indices->reserve(indicesMesh1Length+indicesMesh2Length);

    GLint* indi1 = mesh1->getIndices()->data();
    GLint* indi2 = mesh2->getIndices()->data();

    GLint amountOfPointsInMesh1 = geoMesh1Length/3;

    for(int i=0;i<indicesMesh1Length;i++)
    {
        indices->push_back(indi1[i]);
    }

    for(int i=0;i<indicesMesh2Length;i++)
    {
        indices->push_back(indi2[i]+amountOfPointsInMesh1);
    }

    return new Mesh(geometry,indices);
}

