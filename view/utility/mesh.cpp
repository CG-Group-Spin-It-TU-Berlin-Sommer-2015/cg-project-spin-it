#include "mesh.h"

using namespace std;

Mesh::Mesh(std::vector<float>* geometry, std::vector<short>* indices)
{
    this->vbo = new GLuint[2];

    this->geometry = geometry;
    this->indices = indices;

    //TODO: Normalen berechnen
}

std::vector<float>* Mesh::getGeometry()
{
    return geometry;
}

std::vector<float>* Mesh::getNormals()
{
    return normals;
}

std::vector<short>* Mesh::getIndices()
{
    return indices;
}

void Mesh::render(GLuint program, GLenum primitive)
{
    QGLFunctions* qf = new QGLFunctions();
    if (isDirty) {
        vector<float>* fb = new vector<float>();
        fb->reserve(geometry->size() + 4* geometry->size() / 3);
        for (uint i = 0; i < geometry->size(); i += 3) {
            fb->push_back(geometry->at(i));
            fb->push_back(geometry->at(i + 1));
            fb->push_back(geometry->at(i + 2));
            fb->push_back((float) 100/255);
            fb->push_back((float) 100/255);
            fb->push_back((float) 100/255);
            fb->push_back((float) 100/255);
            //fb->push_back(normals[i]);
            //fb->push_back(normals[i + 1]);
        }
        qf->glGenBuffers(1, vbo);
        qf->glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
        qf->glBufferData(GL_ARRAY_BUFFER, fb->capacity() * sizeof(float), fb, GL_STATIC_DRAW);
        qf->glBindBuffer(GL_ARRAY_BUFFER, 0);

        vector<short>* sb = new vector<short>();
        sb->reserve(indices->size());
        for (uint i = 0; i < indices->size(); i++) {
            sb->push_back(indices->at(i));
        }
        qf->glGenBuffers(1, ibo);
        qf->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo[0]);
        qf->glBufferData(GL_ELEMENT_ARRAY_BUFFER, sb->capacity() * sizeof(short), sb, GL_STATIC_DRAW);
        qf->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        isDirty = false;
    }

    qf->glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    int position_handle = qf->glGetAttribLocation(program, "a_position");
    qf->glEnableVertexAttribArray(position_handle);
    qf->glVertexAttribPointer(position_handle, 3, GL_FLOAT, false, 24, (char *) NULL);

    int normal_handle = qf->glGetAttribLocation(program, "a_normal");
    qf->glEnableVertexAttribArray(normal_handle);
    qf->glVertexAttribPointer(normal_handle, 3, GL_FLOAT, false, 24, ((char *)NULL + 12));

    int count = sizeof(indices);
    qf->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo[0]);
    glDrawElements(primitive, count, GL_UNSIGNED_SHORT, 0);
    delete qf;
}
