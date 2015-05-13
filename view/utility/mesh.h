#ifndef MESH_H
#define MESH_H

#include <QtOpenGL>
#include <QGLFunctions>

class Mesh
{
private:
    bool isDirty;

    GLuint* vbo;
    GLuint* ibo;

    float* geometry;
    float* normals;
    short* indices;

    const int GEOMETRY_DATA_SIZE    = 3;
    const int COLOR_DATA_SIZE       = 4;
    const int NORMAL_DATA_SIZE      = 3;

public:
    Mesh(float* geometry, short* indices);
    float* getGeometry();
    float* getNormals();
    short* getIndices();
    void render(GLuint program, GLenum primitive);
};

#endif // MESH_H
