#include <iostream>
#include <fstream>
#include <string>

#include <QFile>
#include <QTextStream>
#include <QtOpenGL>
#include <QOpenGLFunctions>

using namespace std;

int initShader()
{
    string code = "";
    QFile file (":/shader/simple.vsh");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return 0;

    QTextStream in(&file);
    while (!in.atEnd()) {

        QString line = in.readLine();
        code = code + line.toStdString() + "\n";
    }
    file.close();


    QOpenGLFunctions *qf = new QOpenGLFunctions(QOpenGLContext::currentContext());
    GLuint vs = qf->glCreateShader(GL_VERTEX_SHADER);
    const char *c_str = code.c_str();
    qf->glShaderSource(vs, 1, &c_str, NULL);
    qf->glCompileShader(vs);
    //qf->glGetShaderInfoLog(vs, NULL, NULL, &e_message);
    /*code = "";
    File *file = fopen("shader.fs", "r");
    String code = "";
    while (file >> s) {
        code = code + s;
    }
    int fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, code);
    glCompileShader(fragment_shader);
    Log.log(glGetShaderInfoLog(fragment_shader));

    int program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    Log.log(glGetProgramInfoLog(program));
    glUseProgram(program);

    return program;*/
}
