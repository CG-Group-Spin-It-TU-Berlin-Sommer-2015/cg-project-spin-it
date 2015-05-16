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
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    qglClearColor(Qt::gray);

    shader = new QGLShaderProgram();
    shader->addShaderFromSourceFile(QGLShader::Vertex, ":/shader/simple.vsh");
    shader->addShaderFromSourceFile(QGLShader::Fragment, ":/shader/simple.fsh");
    shader->link();

    camera_position.setX(0);
    camera_position.setY(0);
    camera_position.setZ(1);

    camera_direction.setX(0);
    camera_direction.setY(0);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(5);
    camera_up.setZ(0);

    object = readMeshFromObjFile("test");
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    view_matrix.setToIdentity();
    view_matrix.lookAt(camera_position, camera_direction, camera_up);
    model_matrix.setToIdentity();
    QVector3D mean = object->getMean();
    model_matrix.translate(-mean.x(), -mean.y(), -mean.z());
    model_matrix.rotate(rot_x, 0.0, 1.0, 0.0);
    model_matrix.rotate(rot_y, 1.0, 0.0, 0.0);
    model_matrix.translate(mean.x(), mean.y(), mean.z());

    shader->bind();
    shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
    shader->setUniformValue("color", QColor(Qt::green));

    object->render(shader, GL_TRIANGLES);

    shader->release();
}

void GLWidget::resizeGL(int width, int height)
{
    projection_matrix.setToIdentity();
    projection_matrix.ortho(-10, 10, -10, 10, 10, -10);

    glViewport(0,0, width, height);
}

void GLWidget::mouseMoveEvent(QMouseEvent *ev)
{
    if (mouse_pressed) {
        rot_x = ev->globalX() % 360;
        rot_y = ev->globalY() % 360;
        this->updateGL();
    }
}

void GLWidget::mousePressEvent(QMouseEvent *ev)
{
    mouse_pressed = true;
}

void GLWidget::leaveEvent(QMouseEvent *ev)
{
    mouse_pressed = false;
}



