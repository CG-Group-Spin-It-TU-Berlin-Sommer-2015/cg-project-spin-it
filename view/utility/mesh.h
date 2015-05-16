#ifndef MESH_H
#define MESH_H

#include <iostream>

#include <QtOpenGL>
#include <QGLFunctions>

class Mesh
{
private:
    bool isDirty;

    GLuint* vbo;
    GLuint* ibo;

    std::vector<float>* geometry;
    std::vector<float>* normals;
    std::vector<short>* indices;

    const int GEOMETRY_DATA_SIZE    = 3;
    const int COLOR_DATA_SIZE       = 4;
    const int NORMAL_DATA_SIZE      = 3;

public:
    Mesh(std::vector<float>* geometry, std::vector<short>* indices);
    std::vector<float>* getGeometry();
    std::vector<float>* getNormals();
    std::vector<short>* getIndices();
    void render(GLuint program, GLenum primitive);
};

#endif // MESH_H
