#ifndef EXTENDEDMESHMERGER_H
#define EXTENDEDMESHMERGER_H

#include "view/utility/mesh.h"

#include <cork.h>

#define UNION_BOOLEAN 0
#define DIFFERENCE_BOOLEAN 1

Mesh* booleanUnion(Mesh* inputmesh1, Mesh* inputmesh2);
Mesh* booleanDifference(Mesh* inputmesh1, Mesh* inputmesh2);

#endif // EXTENDEDMESHMERGER_H
