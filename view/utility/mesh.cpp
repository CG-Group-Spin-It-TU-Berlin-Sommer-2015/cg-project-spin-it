#include "mesh.h"



Mesh::Mesh(char* file_name)
{

}

Mesh::Mesh(int num_geometry, int num_normals, int num_indices)
{
    this->geometry = new float[3 * num_geometry];
    this->normals = new float[3 * num_normals];
    this->indices = new short[num_indices];
}

float* Mesh::getGeometry()
{
    return geometry;
}

float* Mesh::getNormals()
{
    return normals;
}

short* Mesh::getIndices()
{
    return indices;
}

void Mesh::render(GLuint program, GLenum primitive)
{
    QGLFunctions* qf = new QGLFunctions();
    if (isDirty) {
        vector<float>* fb = new vector<float>();
        fb->reserve(sizeof(geometry) + sizeof(normals));
        for (uint i = 0; i < sizeof(geometry) / 3; i++) {
            fb->push_back(geometry[i]);
            fb->push_back(geometry[i + 1]);
            fb->push_back(geometry[i + 2]);
            fb->push_back(normals[i]);
            fb->push_back(normals[i + 1]);
        }
        qf->glGenBuffers(1, vbo);
        qf->glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
        qf->glBufferData(GL_ARRAY_BUFFER, fb->capacity() * sizeof(float), fb, GL_STATIC_DRAW);
        qf->glBindBuffer(GL_ARRAY_BUFFER, 0);

        vector<short>* sb = new vector<short>();
        sb->reserve(sizeof(indices));
        for (uint i = 0; i < sizeof(indices); i++) {
            sb->push_back(indices[i]);
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
