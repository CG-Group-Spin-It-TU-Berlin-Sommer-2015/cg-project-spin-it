#include "glwidget.h"

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{

}

GLWidget::~GLWidget()
{
}

void GLWidget::initializeGL()
{
    //program = loadShader("simple");
    object = readMeshFromObjFile("test");

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    qglClearColor(Qt::gray);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    object->render(program, GL_TRIANGLES);
}

void GLWidget::resizeGL(int width, int height)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-3, 3, -3, 3, 3, -3);
    //glMatrixMode(GL_MODELVIEW);
    //glLoadIdentity();
    //glTranslatef(0,0,-3);
    //glRotatef(60,1,1,1);
}

