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
    QVector<GLfloat>* vertex_normals;
    QVector<GLfloat>* surface_normals;
    QVector<GLint>* indices;

public:
    Mesh(QVector<GLfloat>* geometry, QVector<GLint>* indices);
    Mesh(QVector<GLfloat>* geometry, QVector<GLfloat>* normals, QVector<GLint>* indices);
    Mesh(QVector<GLfloat>* geometry, QVector<GLfloat>* vertex_normals, QVector<GLfloat>* surface_normals, QVector<GLint>* indices);
    ~Mesh();
    QVector<GLfloat>* getGeometry();
    QVector<GLfloat>* getSurfaceNormals();
    QVector<GLfloat>* getVertexNormals();
    QVector<GLint>* getIndices();


    void render(QGLShaderProgram* shader, GLenum primitive);

    QVector3D getLowestPoint();
    QVector3D getMean();
    QVector3D getMiddle();
    GLfloat getMaxDistance2Middle();

    Mesh* copy();
    void transform(QMatrix4x4 matrix);
    void swapYZ();
    void updateNormals();

};

#endif // MESH_H
