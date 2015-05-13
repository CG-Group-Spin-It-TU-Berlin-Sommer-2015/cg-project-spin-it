#ifndef OBJREADER_H
#define OBJREADER_H

#include <string>

#include <QFile>
#include <QTextStream>

#include "view/utility/mesh.h"

using namespace std;

Mesh* readMeshFromObjFile (string file_name);

#endif // OBJREADER_H
