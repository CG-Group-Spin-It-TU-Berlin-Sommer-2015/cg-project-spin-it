#ifndef MESH_H
#define MESH_H

#include <QtOpenGL>
#include <QGLFunctions>

using namespace std;

class Mesh
{
private:
    bool isDirty = true;

    GLuint* vbo;
    GLuint* ibo;

    float* geometry;
    float* normals;
    short* indices;

public:
    Mesh(char* file_name);
    Mesh(int n_geo, int n_nor, int n_ind);
    float* getGeometry();
    float* getNormals();
    short* getIndices();
    void render(GLuint program, GLenum primitive);
};

#endif // MESH_H
