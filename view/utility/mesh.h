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
    QVector<GLshort>* indices;

public:
    Mesh(QVector<GLfloat>* geometry, QVector<GLshort>* indices);
    Mesh(QVector<GLfloat>* geometry, QVector<GLfloat>* normals, QVector<GLshort>* indices);
    ~Mesh();
    QVector<GLfloat>* getGeometry();
    QVector<GLfloat>* getNormals();
    QVector<GLshort>* getIndices();
    void render(QGLShaderProgram* shader, GLenum primitive);

    QVector3D getMean();
};

#endif // MESH_H
