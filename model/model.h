#ifndef MODEL_H
#define MODEL_H

#include "view/utility/mesh.h"

class Model
{
private:
    float density;
    float* mesh_volumne;

public:
    Model();
    initialize(Mesh mesh);

private:
    float spinability(float w_1, float w_2, float* volume);
    float calculateMass();
    float* calculateVolumne(Mesh mesh, float density);

};

#endif // MODEL_H
