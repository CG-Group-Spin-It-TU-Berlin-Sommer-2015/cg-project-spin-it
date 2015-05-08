#include "mesh.h"

Mesh::Mesh(char *file_name)
{

}

Mesh::Mesh(int num_geometry, int num_normals, int num_indices)
{
    this->geometry[3 * num_geometry];
    this->normals[3 * num_normals];
    this->indices[num_indices];
}

float *Mesh::getGeometry()
{
    return geometry;
}

float *Mesh::getNormals()
{
    return normals;
}

short *Mesh::getIndices()
{
    return indices;
}

void Mesh::draw()
{

}
