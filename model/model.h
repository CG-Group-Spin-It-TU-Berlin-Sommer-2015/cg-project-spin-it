#ifndef MODEL_H
#define MODEL_H

#include "view/utility/mesh.h"

class Model
{
private:
    float mesh_volumne;
    float inner_volumne;

public:
    Model();

private:
    float* calculateVolumne(Mesh mesh);

};

#endif // MODEL_H
