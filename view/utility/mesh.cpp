#include "mesh.h"

using namespace std;

const int GEOMETRY_DATA_SIZE    = 3;
const int NORMAL_DATA_SIZE      = 3;

Mesh::Mesh(QVector<GLfloat>* geometry, QVector<GLshort>* indices)
{
    this->geometry = geometry;
    this->indices = indices;

    this->normals = new QVector<GLfloat>();
    this->normals->reserve(geometry->size() * sizeof(GLfloat));
    for (uint i = 0; i < geometry->size(); i++) {
        normals->push_back(0);
    }

    int count[geometry->size() / 3];
    for (uint i = 0; i < indices->size(); i += 3) {
        count[indices->at(i)] += 1;
        count[indices->at(i + 1)] += 1;
        count[indices->at(i + 2)] += 1;

        QVector3D n1;
        n1.setX(geometry->at(3 * indices->at(i)) - geometry->at(3 * indices->at(i + 1)));
        n1.setY(geometry->at(3 * indices->at(i) + 1) - geometry->at(3 * indices->at(i + 1) + 1));
        n1.setZ(geometry->at(3 * indices->at(i) + 2) - geometry->at(3 * indices->at(i + 1) + 2));

        QVector3D n2;
        n2.setX(geometry->at(3 * indices->at(i)) - geometry->at(3 * indices->at(i + 2)));
        n2.setY(geometry->at(3 * indices->at(i) + 1) - geometry->at(3 * indices->at(i + 2) + 1));
        n2.setZ(geometry->at(3 * indices->at(i) + 2) - geometry->at(3 * indices->at(i + 2) + 2));

        QVector3D n = QVector3D::normal(n1, n2);

        normals->replace(3 * indices->at(i), normals->at(3 * indices->at(i)) + n.x());
        normals->replace(3 * indices->at(i) + 1, normals->at(3 * indices->at(i) + 1) + n.y());
        normals->replace(3 * indices->at(i) + 2, normals->at(3 * indices->at(i) + 2) + n.z());

        normals->replace(3 * indices->at(i + 1), normals->at(3 * indices->at(i + 1)) + n.x());
        normals->replace(3 * indices->at(i + 1) + 1, normals->at(3 * indices->at(i + 1) + 1) + n.y());
        normals->replace(3 * indices->at(i + 1) + 2, normals->at(3 * indices->at(i + 1) + 2) + n.z());

        normals->replace(3 * indices->at(i + 2), normals->at(3 * indices->at(i + 2)) + n.x());
        normals->replace(3 * indices->at(i + 2) + 1, normals->at(3 * indices->at(i + 2) + 1) + n.y());
        normals->replace(3 * indices->at(i + 2) + 2, normals->at(3 * indices->at(i + 2) + 2) + n.z());
    }

    for (uint i = 0; i < geometry->size() / 3; i++) {
        normals->replace(i, normals->at(i) / count[i]);
        normals->replace(i + 1, normals->at(i + 1) / count[i]);
        normals->replace(i + 2, normals->at(i + 2) / count[i]);
    }
}

QVector<GLfloat>* Mesh::getGeometry()
{
    return geometry;
}

QVector<GLfloat>* Mesh::getNormals()
{
    return normals;
}

QVector<GLshort>* Mesh::getIndices()
{
    return indices;
}

void Mesh::render(QGLShaderProgram* shader, GLenum primitive)
{
    if (isDirty) {
        QVector<GLfloat> buffer;
        buffer.reserve(geometry->size() * sizeof(GLfloat) + normals->size() * sizeof(GLfloat));
        for (uint i = 0; i < geometry->size() / GEOMETRY_DATA_SIZE; i++) {
            buffer.push_back(geometry->at(3*i));
            buffer.push_back(geometry->at(3*i + 1));
            buffer.push_back(geometry->at(3*i + 2));

            buffer.push_back(normals->at(3*i));
            buffer.push_back(normals->at(3*i + 1));
            buffer.push_back(normals->at(3*i + 2));
        }

        vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        vbo->create();
        vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        vbo->bind();
        vbo->allocate(geometry->size() * sizeof(GLfloat) + normals->size() * sizeof(GLfloat));
        vbo->write(0, buffer.constData(), buffer.size() * sizeof(GLfloat));
        vbo->release();

        ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        ibo->create();
        ibo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        ibo->bind();
        ibo->allocate(indices->size() * sizeof(GLshort));
        ibo->write(0, indices->constData(), indices->size() * sizeof(GLshort));
        ibo->release();

        isDirty = false;
    }

    GLint stride = sizeof(GLfloat) * (GEOMETRY_DATA_SIZE + NORMAL_DATA_SIZE);

    vbo->bind();
    shader->setAttributeBuffer("geometry", GL_FLOAT, 0, 3, stride);
    shader->enableAttributeArray("geometry");
    shader->setAttributeBuffer("normal", GL_FLOAT, GEOMETRY_DATA_SIZE * sizeof(GL_FLOAT), 3, stride);
    shader->enableAttributeArray("normal");
    vbo->release();

    ibo->bind();
    glDrawElements(primitive, indices->length(), GL_UNSIGNED_SHORT, 0);
    ibo->release();

    shader->disableAttributeArray("geometry");
}

QVector3D Mesh::getMean()
{
    QVector3D mean;
    for (uint i = 0; i < geometry->size(); i += 3) {
        mean.setX(mean.x() + geometry->at(i));
        mean.setY(mean.y() + geometry->at(i + 1));
        mean.setZ(mean.z() + geometry->at(i + 2));
    }
    mean.setX(mean.x() / (geometry->size() / 3));
    mean.setY(mean.y() / (geometry->size() / 3));
    mean.setZ(mean.z() / (geometry->size() / 3));
    return mean;
}
