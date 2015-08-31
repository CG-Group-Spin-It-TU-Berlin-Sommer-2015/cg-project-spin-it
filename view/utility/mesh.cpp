#include "mesh.h"

using namespace std;

const int GEOMETRY_DATA_SIZE    = 3;
const int NORMAL_DATA_SIZE      = 3;

Mesh::Mesh(QVector<GLfloat>* geometry, QVector<GLint>* indices)
{
    this->geometry = geometry;
    this->indices = indices;

    surface_normals = new QVector<GLfloat>();
    surface_normals->reserve(indices->size());

    vertex_normals = new QVector<GLfloat>();
    vertex_normals->reserve(3 * indices->size());
    for (int i = 0; i < 3 * indices->size(); i++) {
        vertex_normals->push_back(0);
    }

    for (int i = 0; i < indices->size(); i += 3) {
        QVector3D n1;
        n1.setX(geometry->at(3 * indices->at(i)) - geometry->at(3 * indices->at(i + 1)));
        n1.setY(geometry->at(3 * indices->at(i) + 1) - geometry->at(3 * indices->at(i + 1) + 1));
        n1.setZ(geometry->at(3 * indices->at(i) + 2) - geometry->at(3 * indices->at(i + 1) + 2));

        QVector3D n2;
        n2.setX(geometry->at(3 * indices->at(i)) - geometry->at(3 * indices->at(i + 2)));
        n2.setY(geometry->at(3 * indices->at(i) + 1) - geometry->at(3 * indices->at(i + 2) + 1));
        n2.setZ(geometry->at(3 * indices->at(i) + 2) - geometry->at(3 * indices->at(i + 2) + 2));

        QVector3D n = QVector3D::crossProduct(n1, n2);

        surface_normals->push_back(n.x());
        surface_normals->push_back(n.y());
        surface_normals->push_back(n.z());

        vertex_normals->replace(3 * indices->at(i), vertex_normals->at(3 * indices->at(i)) + n.x());
        vertex_normals->replace(3 * indices->at(i) + 1, vertex_normals->at(3 * indices->at(i) + 1) + n.y());
        vertex_normals->replace(3 * indices->at(i) + 2, vertex_normals->at(3 * indices->at(i) + 2) + n.z());

        vertex_normals->replace(3 * indices->at(i + 1), vertex_normals->at(3 * indices->at(i + 1)) + n.x());
        vertex_normals->replace(3 * indices->at(i + 1) + 1, vertex_normals->at(3 * indices->at(i + 1) + 1) + n.y());
        vertex_normals->replace(3 * indices->at(i + 1) + 2, vertex_normals->at(3 * indices->at(i + 1) + 2) + n.z());

        vertex_normals->replace(3 * indices->at(i + 2), vertex_normals->at(3 * indices->at(i + 2)) + n.x());
        vertex_normals->replace(3 * indices->at(i + 2) + 1, vertex_normals->at(3 * indices->at(i + 2) + 1) + n.y());
        vertex_normals->replace(3 * indices->at(i + 2) + 2, vertex_normals->at(3 * indices->at(i + 2) + 2) + n.z());
    }

    for (int i = 0; i < geometry->size() / 3; i++) {
        QVector3D normal;
        normal.setX(vertex_normals->at(3*i));
        normal.setY(vertex_normals->at(3*i + 1));
        normal.setZ(vertex_normals->at(3*i + 2));

        vertex_normals->replace(3 * i, normal.x() / normal.length());
        vertex_normals->replace(3 * i + 1, normal.y() / normal.length());
        vertex_normals->replace(3 * i + 2, normal.z() / normal.length());
    }

    this->isDirty = true;
}

Mesh::Mesh(QVector<GLfloat> *geometry, QVector<GLfloat>* vertex_normals, QVector<GLint>* indices)
{
    this->geometry = geometry;
    this->vertex_normals = vertex_normals;
    this->surface_normals = new QVector<GLfloat>();
    this->indices = indices;

    this->isDirty = true;
}

Mesh::~Mesh()
{
    geometry->clear();
    delete geometry;

    vertex_normals->clear();
    delete vertex_normals;

    surface_normals->clear();
    delete surface_normals;

    indices->clear();
    delete indices;
}

QVector<GLfloat>* Mesh::getGeometry()
{
    return geometry;
}


QVector<GLfloat>* Mesh::getSurfaceNormals()
{
    return surface_normals;
}

QVector<GLfloat>* Mesh::getVertexNormals()
{
    return vertex_normals;
}

QVector<GLint>* Mesh::getIndices()
{
    return indices;
}

void Mesh::render(QGLShaderProgram* shader, GLenum primitive)
{
    if (isDirty) {
        QVector<GLfloat> buffer;
        buffer.reserve(geometry->size() * sizeof(GLfloat) + vertex_normals->size() * sizeof(GLfloat));
        for (int i = 0; i < geometry->size() / GEOMETRY_DATA_SIZE; i++) {
            buffer.push_back(geometry->at(3*i));
            buffer.push_back(geometry->at(3*i + 1));
            buffer.push_back(geometry->at(3*i + 2));

            buffer.push_back(vertex_normals->at(3*i));
            buffer.push_back(vertex_normals->at(3*i + 1));
            buffer.push_back(vertex_normals->at(3*i + 2));
        }

        vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        vbo->create();
        vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        vbo->bind();
        vbo->allocate(geometry->size() * sizeof(GLfloat) + vertex_normals->size() * sizeof(GLfloat));
        vbo->write(0, buffer.constData(), buffer.size() * sizeof(GLfloat));
        vbo->release();

        ibo = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        ibo->create();
        ibo->setUsagePattern(QOpenGLBuffer::StaticDraw);
        ibo->bind();
        ibo->allocate(indices->size() * sizeof(GLint));
        ibo->write(0, indices->constData(), indices->size() * sizeof(GLint));
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
    glDrawElements(primitive, indices->length(), GL_UNSIGNED_INT, (void*) 0);
    ibo->release();

    shader->disableAttributeArray("geometry");
}

/**
 * @brief Mesh::getMiddle Get the middle point according to minimal and maximal coordinates
 * @return the middle of the mesh
 */
QVector3D Mesh::getMiddle()
{
    GLfloat x_min, x_max, y_min, y_max, z_min, z_max;

    x_min = std::numeric_limits<float>::max();
    y_min = std::numeric_limits<float>::max();
    z_min = std::numeric_limits<float>::max();

    x_max = std::numeric_limits<float>::min();
    y_max = std::numeric_limits<float>::min();
    z_max = std::numeric_limits<float>::min();

    for (int i = 0; i < getGeometry()->size(); i += 3) {
        if (getGeometry()->at(i) > x_max) {
            x_max = getGeometry()->at(i);
        }
        if (getGeometry()->at(i) < x_min) {
            x_min = getGeometry()->at(i);
        }

        if (getGeometry()->at(i + 1) > y_max) {
            y_max = getGeometry()->at(i + 1);
        }
        if (getGeometry()->at(i + 1) < y_min) {
            y_min = getGeometry()->at(i + 1);
        }

        if (getGeometry()->at(i + 2) > z_max) {
            z_max = getGeometry()->at(i + 2);
        }
        if (getGeometry()->at(i + 2) < z_min) {
            z_min = getGeometry()->at(i + 2);
        }
    }

    QVector3D middle;

    middle.setX((x_max + x_min) / 2);
    middle.setY((y_max + y_min) / 2);
    middle.setZ((z_max + z_min) / 2);

    return middle;
}

/**
 * @brief Mesh::getMaxDistance2Middle Get the maximal distance according to the middle point
 * @return the maximal distance
 */
GLfloat Mesh::getMaxDistance2Middle()
{
   QVector3D middle = getMiddle();

   GLfloat max = 0;

   for (int i = 0; i < getGeometry()->size(); i += 3) {

       GLfloat xt = fabs(getGeometry()->at(i + 0)-middle.x());
       max = xt>max?xt:max;

       GLfloat yt = fabs(getGeometry()->at(i + 1)-middle.y());
       max = yt>max?yt:max;

       GLfloat zt = fabs(getGeometry()->at(i + 2)-middle.z());
       max = zt>max?zt:max;

   }

   return max;
}

QVector3D Mesh::getMean()
{
    QVector3D mean;
    for (int i = 0; i < geometry->size(); i += 3) {
        mean.setX(mean.x() + geometry->at(i));
        mean.setY(mean.y() + geometry->at(i + 1));
        mean.setZ(mean.z() + geometry->at(i + 2));
    }
    mean.setX(mean.x() / (geometry->size() / 3));
    mean.setY(mean.y() / (geometry->size() / 3));
    mean.setZ(mean.z() / (geometry->size() / 3));
    return mean;
}

/**
 * @brief Mesh::copy Make a copy of the mesh
 * @return the copy of the mesh
 */
Mesh* Mesh::copy()
{

    QVector<GLfloat>* newGeometry = new QVector<GLfloat>();
    for (int i = 0; i < geometry->size(); i+=3) {

        newGeometry->push_back(geometry->at(i+0));
        newGeometry->push_back(geometry->at(i+1));
        newGeometry->push_back(geometry->at(i+2));
    }

    QVector<GLint>* newIndices = new QVector<GLint>();
    for (int i = 0; i < indices->size(); i++) {
        newIndices->push_back(indices->at(i));
    }

    return new Mesh(newGeometry,newIndices);

}

/**
 * @brief Mesh::transform Transform the vertices of the mesh
 * @param matrix the transformation matrix
 */
void Mesh::transform(QMatrix4x4 matrix)
{

    QVector4D vec,tempVec;
    vec.setW(1);

    GLfloat* data = geometry->data();

    for (int i = 0; i < geometry->size(); i+=3) {

        vec.setX(data[i+0]);
        vec.setY(data[i+1]);
        vec.setZ(data[i+2]);

        tempVec = matrix*vec;

        data[i+0] = tempVec.x();
        data[i+1] = tempVec.y();
        data[i+2] = tempVec.z();

    }

    updateNormals();
}

/**
 * @brief Mesh::swapYZ Swap the y and z coordinate
 */
void Mesh::swapYZ()
{

    QVector4D vec;

    GLfloat* data = geometry->data();

    for (int i = 0; i < geometry->size(); i+=3) {

        vec.setX(data[i+0]);
        vec.setY(data[i+1]);
        vec.setZ(data[i+2]);

        data[i+0] = vec.x();
        data[i+1] = vec.z();
        data[i+2] = vec.y();

    }

    updateNormals();

}

void Mesh::updateNormals()
{

    surface_normals->clear();
    surface_normals->reserve(indices->size());

    vertex_normals->clear();
    vertex_normals->reserve(3 * indices->size());
    for (int i = 0; i < 3 * indices->size(); i++) {
        vertex_normals->push_back(0);
    }

    for (int i = 0; i < indices->size(); i += 3) {
        QVector3D n1;
        n1.setX(geometry->at(3 * indices->at(i)) - geometry->at(3 * indices->at(i + 1)));
        n1.setY(geometry->at(3 * indices->at(i) + 1) - geometry->at(3 * indices->at(i + 1) + 1));
        n1.setZ(geometry->at(3 * indices->at(i) + 2) - geometry->at(3 * indices->at(i + 1) + 2));

        QVector3D n2;
        n2.setX(geometry->at(3 * indices->at(i)) - geometry->at(3 * indices->at(i + 2)));
        n2.setY(geometry->at(3 * indices->at(i) + 1) - geometry->at(3 * indices->at(i + 2) + 1));
        n2.setZ(geometry->at(3 * indices->at(i) + 2) - geometry->at(3 * indices->at(i + 2) + 2));

        QVector3D n = QVector3D::crossProduct(n1, n2);

        surface_normals->push_back(-n.x());
        surface_normals->push_back(-n.y());
        surface_normals->push_back(-n.z());

        vertex_normals->replace(3 * indices->at(i), vertex_normals->at(3 * indices->at(i)) + n.x());
        vertex_normals->replace(3 * indices->at(i) + 1, vertex_normals->at(3 * indices->at(i) + 1) + n.y());
        vertex_normals->replace(3 * indices->at(i) + 2, vertex_normals->at(3 * indices->at(i) + 2) + n.z());

        vertex_normals->replace(3 * indices->at(i + 1), vertex_normals->at(3 * indices->at(i + 1)) + n.x());
        vertex_normals->replace(3 * indices->at(i + 1) + 1, vertex_normals->at(3 * indices->at(i + 1) + 1) + n.y());
        vertex_normals->replace(3 * indices->at(i + 1) + 2, vertex_normals->at(3 * indices->at(i + 1) + 2) + n.z());

        vertex_normals->replace(3 * indices->at(i + 2), vertex_normals->at(3 * indices->at(i + 2)) + n.x());
        vertex_normals->replace(3 * indices->at(i + 2) + 1, vertex_normals->at(3 * indices->at(i + 2) + 1) + n.y());
        vertex_normals->replace(3 * indices->at(i + 2) + 2, vertex_normals->at(3 * indices->at(i + 2) + 2) + n.z());
    }

    for (int i = 0; i < geometry->size() / 3; i++) {
        QVector3D normal;
        normal.setX(vertex_normals->at(3*i));
        normal.setY(vertex_normals->at(3*i + 1));
        normal.setZ(vertex_normals->at(3*i + 2));

        vertex_normals->replace(3 * i, normal.x() / normal.length());
        vertex_normals->replace(3 * i + 1, normal.y() / normal.length());
        vertex_normals->replace(3 * i + 2, normal.z() / normal.length());
    }

    this->isDirty = true;

}
