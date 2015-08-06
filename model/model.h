#ifndef MODEL_H
#define MODEL_H

#include <stdio.h>

#include "view/utility/mesh.h"

using namespace std;

class Model
{
public:
    static float p;
    static QVector3D cp;

private:
    static Mesh* mesh;
    static float* mesh_volumne;

public:
    static void initialize(Mesh *mesh);
    static void hollow();

private:
    static float* calculateVolumne(Mesh* mesh, float p);
    static float* slqp(float* b);
    static float spinability(float w_1, float w_2, float* volume);
};

#endif // MODEL_H
