#ifndef EXTENDEDMESHMERGER_H
#define EXTENDEDMESHMERGER_H

#include "view/utility/mesh.h"

#include <cork.h>

Mesh* booleanUnion(Mesh* inputmesh1, Mesh* inputmesh2);

Mesh* booleanDifference(Mesh* inputmesh1, Mesh* inputmesh2);


#endif // EXTENDEDMESHMERGER_H
