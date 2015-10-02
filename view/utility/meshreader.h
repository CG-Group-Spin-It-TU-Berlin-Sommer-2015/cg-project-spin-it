#ifndef OBJREADER_H
#define OBJREADER_H

#include <iostream>
#include <stdlib.h>
#include <string>

#include <QFile>
#include <QTextStream>

#include "mesh/mesh.h"

Mesh* readMeshFromObjFile (std::string file_name);

#endif // OBJREADER_H
