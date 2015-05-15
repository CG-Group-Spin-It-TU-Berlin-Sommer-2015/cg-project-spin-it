#include "meshreader.h"

using namespace std;

/**
 * @brief readMeshFromObjFile Loads a Mesh from an .obj file
 * @param file_name name of obj file
 * @return Mesh object from obj. file if successful, null else
 */
Mesh* readMeshFromObjFile (string file_name)
{
    QFile* file = new QFile((":/obj/"+ file_name + ".obj").c_str());
    if (!file->open(QIODevice::ReadOnly | QIODevice::Text))
        return 0;

    vector<float> geometry;
    vector<short> indices;

    /*
    char* token;
    QTextStream* in = new QTextStream(file);
    while (!in->atEnd()) {
        string line = in->readLine().toStdString();
        char c_line[line.length()];
        strcpy(c_line, line.c_str());
        token = strtok(c_line, " ");
        if (strcmp(token, "v") == 1) {
            geometry.push_back(atof(strtok(NULL, " ")));
            geometry.push_back(atof(strtok(NULL, " ")));
            geometry.push_back(atof(strtok(NULL, " ")));
        }
        if (strcmp(token, "f") == 1) {
            indices.push_back(atoi(strtok(NULL, " ")));
            indices.push_back(atoi(strtok(NULL, " ")));
            indices.push_back(atoi(strtok(NULL, " ")));
        }
    }
    file->close();

    Mesh* mesh = new Mesh(geometry.data(), indices.data());
    return mesh;
    */

    return 0;
}
