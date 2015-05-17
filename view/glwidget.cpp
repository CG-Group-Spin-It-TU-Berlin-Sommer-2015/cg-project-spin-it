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

    camera_position.setX(1);
    camera_position.setY(-0.75);
    camera_position.setZ(1);

    camera_direction.setX(0);
    camera_direction.setY(0);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(5);
    camera_up.setZ(0);

    object = readMeshFromObjFile("test");

    QVector<GLfloat>* geometry = new QVector<GLfloat>();
    QVector<GLshort>* indices = new QVector<GLshort>();

    int idx = 0;
    for (float x = -3; x <= 3; x += 0.5) {
        for (float z = -3; z <= 3; z += 0.5) {
            geometry->push_back(x);
            geometry->push_back(0);
            geometry->push_back(z);

            if (z < 3) {
                indices->push_back(idx);
                indices->push_back(idx + 1);
            }
            if (x < 3) {
                indices->push_back(idx);
                indices->push_back(idx + 13);
            }
            idx += 1;
        }
    }

    grid = new Mesh(geometry, indices);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    view_matrix.setToIdentity();
    view_matrix.lookAt(camera_position, camera_direction, camera_up);
    view_matrix.rotate(rot_cam_phi, 0.0, 1.0, 0.0);
    view_matrix.rotate(rot_cam_psy, 1.0, 0.0, 0.0);

    shader->bind();
    model_matrix.setToIdentity();
    QVector3D mean = object->getMean();
    model_matrix.translate(-mean.x(), -mean.y(), -mean.z());
    model_matrix.rotate(rot_obj_phi, 0.0, 1.0, 0.0);
    model_matrix.rotate(rot_obj_psy, 1.0, 0.0, 0.0);
    model_matrix.translate(mean.x(), mean.y(), mean.z());

    shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
    shader->setUniformValue("color", QColor(Qt::green));
    object->render(shader, GL_TRIANGLES);

    model_matrix.setToIdentity();
    model_matrix.translate(0.0, -2.0, 0.0);
    shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
    shader->setUniformValue("color", QColor(Qt::black));
    grid->render(shader, GL_LINES);


    shader->release();
}

void GLWidget::resizeGL(int width, int height)
{
    projection_matrix.setToIdentity();
    projection_matrix.ortho(-6, 6, -6, 6, 6, -6);

    glViewport(0,0, width, height);
}

void GLWidget::mouseMoveEvent(QMouseEvent *ev)
{
    if (left_pressed) {
        rot_obj_phi = ev->globalX() % 360;
        rot_obj_psy = ev->globalY() % 360;
        this->updateGL();
    }
    if (right_pressed) {
        rot_cam_phi = ev->globalX() % 360;
        rot_cam_psy = ev->globalY() % 360;
        this->updateGL();
    }
}

void GLWidget::mousePressEvent(QMouseEvent *ev)
{
    if (ev->button() == Qt::LeftButton) {
        left_pressed = true;
        return;
    }
    if (ev->button() == Qt::RightButton) {
        right_pressed = true;
        return;
    }
}

void GLWidget::mouseReleaseEvent(QMouseEvent *ev)
{
    left_pressed = false;
    right_pressed = false;
}



