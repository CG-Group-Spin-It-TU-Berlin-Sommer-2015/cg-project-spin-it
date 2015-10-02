#ifndef SHADER_H
#define SHADER_H

#include <iostream>
#include <string>

#include <QFile>
#include <QTextStream>
#include <QtOpenGL>
#include <QOpenGLFunctions>

GLuint loadShader(std::string shader_name);

#endif // SHADER_H
