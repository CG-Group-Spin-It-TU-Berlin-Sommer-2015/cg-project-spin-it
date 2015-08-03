#ifndef MESH_H
#define MESH_H

#include <iostream>

#include <QtOpenGL>
#include <QGLFunctions>
#include <QOpenGLBuffer>

class Mesh
{
private:
    bool isDirty;

    QOpenGLBuffer* vbo;
    QOpenGLBuffer* ibo;

    QVector<GLfloat>* geometry;
    QVector<GLfloat>* normals;
    QVector<GLint>* indices;

public:
    Mesh(QVector<GLfloat>* geometry, QVector<GLint>* indices);
    Mesh(QVector<GLfloat>* geometry, QVector<GLfloat>* normals, QVector<GLint>* indices);
    ~Mesh();
    QVector<GLfloat>* getGeometry();
    QVector<GLfloat>* getNormals();
    QVector<GLint>* getIndices();


    void render(QGLShaderProgram* shader, GLenum primitive);

    Mesh* copy();
    void tranlateInNormalDirection(GLfloat magnitude);

    QVector3D getMean();
};

#endif // MESH_H
