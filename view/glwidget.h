#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QEvent>
#include <QMouseEvent>
#include <QGLWidget>
#include <QtOpenGL>
#include <QGLShader>
#include <QGLShaderProgram>
#include <QString>
#include <QFileDialog>

#include "model/model.h"
#include "utility/mesh.h"
#include "utility/meshreader.h"
#include "utility/meshwriter.h"
#include "utility/simplemeshmerger.h"
#include "utility/extendedmeshmerger.h"
#include "utility/shader.h"
#include "utility/basicoctree.h"
#include "utility/extendedoctree.h"

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

    QVector4D ambient_light;
    QVector4D diffuse_light;
    QVector4D direction_light;

    Mesh* object;
    Mesh* objectShell;

    Mesh* grid;
    Mesh* rot_axis;

    bool left_pressed;
    bool right_pressed;
    bool middle_pressed;

    int rot_obj_phi;
    int rot_obj_psy;

    int rot_cam_phi;

    GLfloat trans_x;
    GLfloat trans_z;

    QPoint mouse_pos;

    bool showOuterSurface;
    bool showInnerSurface;
    bool showGrid;

public:
    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();

public slots:
    void loadNewMesh();
    void makeItSpin();
    void showOnlyOuterSurface();
    void showOnlyInnerSurface();
    void showOnlyOctreeGrid();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    void mouseMoveEvent(QMouseEvent* ev);
    void mousePressEvent(QMouseEvent* ev);
    void mouseReleaseEvent(QMouseEvent* ev);

};

#endif // GLWIDGET_H
