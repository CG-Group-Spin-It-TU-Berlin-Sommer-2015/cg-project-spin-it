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
    program = loadShader("simple");
    object = readMeshFromObjFile("test");

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    qglClearColor(Qt::gray);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glBegin( GL_QUADS ); // Wir bauen den Würfel aus Quadraten (Quads) auf

      glColor3f(1, 0,   0  );   // Ab jetzt werden alle gezeichneten Punkte rot
        glVertex3f( 1,  1, -1);
        glVertex3f( 1, -1, -1);
        glVertex3f(-1, -1, -1);
        glVertex3f(-1,  1, -1);

      glColor3f(0, 1,   0  );   // Ab jetzt werden alle gezeichneten Punkte grün
        glVertex3f( 1,  1,  1);
        glVertex3f(-1,  1,  1);
        glVertex3f(-1, -1,  1);
        glVertex3f( 1, -1,  1);

      glColor3f(0, 0,   1  );
        glVertex3f( 1,  1, -1);
        glVertex3f( 1,  1,  1);
        glVertex3f( 1, -1,  1);
        glVertex3f( 1, -1, -1);

      glColor3f(1, 1,   0  );
        glVertex3f( 1, -1, -1);
        glVertex3f( 1, -1,  1);
        glVertex3f(-1, -1,  1);
        glVertex3f(-1, -1, -1);

      glColor3f(0, 0.5, 1  );
        glVertex3f(-1, -1, -1);
        glVertex3f(-1, -1,  1);
        glVertex3f(-1,  1,  1);
        glVertex3f(-1,  1, -1);

      glColor3f(1, 0.1, 0.8);
        glVertex3f( 1,  1,  1);
        glVertex3f( 1,  1, -1);
        glVertex3f(-1,  1, -1);
        glVertex3f(-1,  1,  1);

    glEnd();
}

void GLWidget::resizeGL(int width, int height)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-3, 3, -3, 3, -3, 3);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0,0,-3);
    glRotatef(60,1,1,1);
}

