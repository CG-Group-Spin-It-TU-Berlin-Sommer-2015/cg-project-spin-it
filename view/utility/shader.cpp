#include "shader.h"

using namespace std;

/**
 * @brief initShader
 * @return the compiled shader program or 0 if the shader file does not exist
 */
GLuint loadShader(string shader_name)
{
    QOpenGLFunctions* qf = new QOpenGLFunctions(QOpenGLContext::currentContext());
    char* error;

    QFile* file = new QFile((":/shader/" + shader_name + ".vsh").c_str());
    if (!file->open(QIODevice::ReadOnly | QIODevice::Text))
        return 0;

    string code = "";
    QTextStream* in = new QTextStream(file);
    while (!in->atEnd()) {
        QString line = in->readLine();
        code = code + line.toStdString() + "\n";
    }
    file->close();

    GLuint vs = qf->glCreateShader(GL_VERTEX_SHADER);
    const char* v_str = code.c_str();
    qf->glShaderSource(vs, 1, &v_str, NULL);
    qf->glCompileShader(vs);
    qf->glGetShaderInfoLog(vs, NULL, NULL, error);
    cout << error << "\n";

    code = "";

    delete file;
    delete in;
    file = new QFile((":/shader/" + shader_name + ".fsh").c_str());
    if (!file->open(QIODevice::ReadOnly | QIODevice::Text))
        return 0;

    in = new QTextStream(file);
    while (!in->atEnd()) {
        QString line = in->readLine();
        code = code + line.toStdString() + "\n";
    }
    file->close();

    GLuint fs = qf->glCreateShader(GL_FRAGMENT_SHADER);
    const char *f_str = code.c_str();
    qf->glShaderSource(fs, 1, &f_str, NULL);
    qf->glCompileShader(fs);
    qf->glGetShaderInfoLog(fs, NULL, NULL, error);
    cout << error << "\n";

    GLuint program = qf->glCreateProgram();
    qf->glAttachShader(program, vs);
    qf->glAttachShader(program, fs);
    qf->glLinkProgram(program);
    qf->glGetProgramInfoLog(program, NULL, NULL, error);
    cout << error << "\n";
    qf->glUseProgram(program);

    return program;
}
