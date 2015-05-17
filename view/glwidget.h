#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QEvent>
#include <QMouseEvent>
#include <QGLWidget>
#include <QtOpenGL>
#include <QGLShader>
#include <QGLShaderProgram>

#include "utility/mesh.h"
#include "utility/meshreader.h"
#include "utility/shader.h"


class GLWidget : public QGLWidget
{
    Q_OBJECT
private:
    QGLShaderProgram* shader;

    QMatrix4x4 projection_matrix;
    QMatrix4x4 model_matrix;
    QMatrix4x4 view_matrix;

    QVector3D camera_position;
    QVector3D camera_direction;
    QVector3D camera_up;

    Mesh* object;
    Mesh* grid;

    bool left_pressed = false;
    bool right_pressed = false;

    int rot_obj_phi = 0;
    int rot_obj_psy = 0;

    int rot_cam_phi = 0;
    int rot_cam_psy = 0;

public:
    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    void mouseMoveEvent(QMouseEvent* ev);
    void mousePressEvent(QMouseEvent* ev);
    void mouseReleaseEvent(QMouseEvent* ev);

};

#endif // GLWIDGET_H
