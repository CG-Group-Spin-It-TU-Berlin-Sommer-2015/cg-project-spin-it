#include "meshreader.h"

using namespace std;

/**
 * @brief readMeshFromObjFileDirectory Loads a Mesh from an .obj file from a particular directory
 * @param file_name name of obj file
 * @return Mesh object from obj. file if successful, null else
 */
Mesh* readMeshFromObjFileDirectory (string file_name)
{
    return readMeshFromObjFile(":/obj/"+ file_name + ".obj");
}

/**
 * @brief readMeshFromObjFile Loads a Mesh from an .obj file
 * @param file_name name and path of obj file
 * @return Mesh object from obj. file if successful, null else
 */
Mesh* readMeshFromObjFile (string file_name)
{
    QFile* file = new QFile(file_name.c_str());
    if (!file->open(QIODevice::ReadOnly | QIODevice::Text))
        return 0;

    QVector<float>* geometry = new QVector<float>;
    QVector<short>* indices = new QVector<short>;

    QTextStream* in = new QTextStream(file);
    while (!in->atEnd()) {
        QString line = in->readLine();
        QStringList token = line.split(" ");
        if (strcmp(token.at(0).toStdString().c_str(), "v") == 0) {
            geometry->push_back(token.at(1).toFloat());
            geometry->push_back(token.at(2).toFloat());
            geometry->push_back(token.at(3).toFloat());
        }
        if (strcmp(token.at(0).toStdString().c_str(), "f") == 0) {
            indices->push_back(token.at(1).toShort() - 1);
            indices->push_back(token.at(2).toShort() - 1);
            indices->push_back(token.at(3).toShort() - 1);
        }
    }
    file->close();

    return new Mesh(geometry, indices);
}
