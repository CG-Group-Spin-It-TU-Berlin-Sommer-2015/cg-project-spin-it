#ifndef MESH_H
#define MESH_H

class Mesh
{
private:
    float *geometry;
    float *normals;
    short *indices;

public:
    Mesh(char *file_name);
    Mesh(int n_geo, int n_nor, int n_ind);
    float *getGeometry();
    float *getNormals();
    short *getIndices();
    void draw();
};

#endif // MESH_H
