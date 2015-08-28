#include "meshwriter.h"

using namespace std;

/**
 * @brief writeMeshFromObjFile Write the mesh to a specific file
 * @param file_name the file with the path
 * @param mesh the mesh
 * @return True if the process was successful. False otherwise.
 */
bool writeMeshFromObjFile (string file_name, Mesh* mesh)
{

    QFile file(file_name.c_str());
    if (!file.open(QIODevice::WriteOnly))
        return false;

    QTextStream outStream(&file);

    GLfloat* geometry = mesh->getGeometry()->data();
    GLint* indices = mesh->getIndices()->data();

    GLint geometryLength = mesh->getGeometry()->length();
    GLint indicesLength = mesh->getIndices()->length();

    // write the vertices
    for(int i=0;i<geometryLength/3;i++)
    {
        outStream << "v";
        outStream << " " << geometry[i*3+0];
        outStream << " " << geometry[i*3+1];
        outStream << " " << geometry[i*3+2];
        outStream << "\n";
    }

    // write the indices
    for(int i=0;i<indicesLength/3;i++)
    {
        outStream << "f";
        outStream << " " << (indices[i*3+0]+1);
        outStream << " " << (indices[i*3+1]+1);
        outStream << " " << (indices[i*3+2]+1);
        outStream << "\n";
    }

    file.close();

    return true;

}
