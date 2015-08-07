#ifndef MESHWRITER_H
#define MESHWRITER_H

#include <iostream>
#include <stdlib.h>
#include <string>

#include <QFile>
#include <QTextStream>

#include "view/utility/mesh.h"

bool writeMeshFromObjFile (std::string file_name, Mesh* mesh);

#endif // MESHWRITER_H
