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
    static float* mesh_volumne;

public:
    static void initialize(Mesh* mesh);

private:
    static float spinability(float w_1, float w_2, float* volume);
    static float calculateMass(float* volumne);
    static float* calculateVolumne(Mesh* mesh, float p);
};

#endif // MODEL_H
