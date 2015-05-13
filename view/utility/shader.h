#ifndef SHADER_H
#define SHADER_H

#include <iostream>
#include <string>

#include <QFile>
#include <QTextStream>
#include <QtOpenGL>
#include <QOpenGLFunctions>

using namespace std;

GLuint loadShader(string shader_name);

#endif // SHADER_H
