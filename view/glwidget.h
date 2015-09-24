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
#include "mesh/mesh.h"
#include "utility/meshreader.h"
#include "utility/meshwriter.h"

#include "utility/shader.h"

#include "mesh/meshOperations/simplemeshmerger.h"
#include "mesh/meshOperations/meshbooleanfunctions.h"
#include "optimization/octree/extendedoctree.h"
#include "optimization/betaOptimization/betaoptimization.h"

#define TOP_WITH_AXIS_MODE       (topOptimized && !tippeTopOptimized &&  addSpinAxisToTop)
#define TOP_WITHOUT_AXIS_MODE    (topOptimized && !tippeTopOptimized && !addSpinAxisToTop)
#define TIPPE_TOP_MODE               (topOptimized &&  tippeTopOptimized)
#define YOYO_MODE                   !topOptimized

#define TRANSLATION_TAB 0
#define MODEL_TAB       1
#define HOLLOWING_TAB   2

#define TRANSLATION_VIEW_XY         0
#define TRANSLATION_VIEW_Z          1
#define ROTATION_SCALE_VIEW         2
#define TRANSLATION_VIEW_DEFAULT    3

#define TRANSLATION_XY_RATIO    40.0f
#define TRANSLATION_Z_RATIO     30.0f
#define SCALE_RATIO 30.0f;

class GLWidget : public QGLWidget
{
    Q_OBJECT
private:

    int viewState;

    QGLShaderProgram* shader;
    QGLShaderProgram* colorshader;

    QMatrix4x4 projection_matrix;
    QMatrix4x4 model_matrix;

    QMatrix4x4 last_object_model_matrix;
    QMatrix4x4 last_rotation_axis_model_matrix;

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
    Mesh* half_sphere;
    Mesh* yoyo_center;

    bool left_pressed;
    bool right_pressed;
    bool middle_pressed;

    int rot_obj_phi;
    int rot_obj_psy;

    int rot_cam_phi;

    QVector3D alignPos;
    QVector3D trans;

    GLfloat scaleFactor;
    GLfloat scale_xyz;

    QPoint mouse_pos;

    bool showOuterSurface;
    bool showInnerSurface;
    bool showGrid;

    GLint startMaximalDepth;
    GLint optimizationMaximalDepth;
    GLint shellExtensionValue;

    GLfloat lowest_y_rot_axis;
    GLfloat lowest_y_half_sphere;

    bool topOptimized;
    bool tippeTopOptimized;
    bool addSpinAxisToTop;

    bool rebuildOctree;

    bool octreeIsDirty;
    bool viewIsDirty;

public:
    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();

public slots:

    void loadNewMesh();
    void makeItSpin();

    void showOnlyOuterSurface();
    void showOnlyInnerSurface();
    void showOnlyOctreeGrid();

    void setView(int);
    void setViewXY();
    void setViewZ();
    void setViewRotationScale();

    void resetXY();
    void resetZ();
    void resetRotationScale();
    void resetAll();

    void setStartDepthValue(int);
    void setMaximumDepthValue(int);
    void setShellExtensionValue(int);

    void calculateOctree();
    void saveMesh();

    void setYoyo();
    void setTop();
    void setAddAxis(bool);
    void setTippeTop(bool);

    void setCWeight(double);
    void setIWeight(double);
    void setLWeight(double);

signals:

    void setStartDepthSpinBoxValue(int);
    void setMaximumDepthSpinBoxValue(int);
    void setShellExtensionSpinBoxValue(int);

    void modelLoaded(bool);

    void yoyoIsSet(bool);
    void topWithoutAxisIsSet(bool);
    void topWithAxisIsSet(bool);
    void tippeTopIsSet(bool);
    void resetTranformationWidget();

    void activateAddAxisCheckBox(bool);
    void activateTippeTopCheckBox(bool);

    void shellIsSet(bool);
    void shellIsNotSet(bool);

    void activateViewControlWidget(bool);
    void deactivateViewControlWidget(bool);

    void setAddAxisCheckBox(bool);

    void setViewBack(bool);

    void setCWeightSpinBox(double);
    void setIWeightSpinBox(double);
    void setLWeightSpinBox(double);

    void activateCSpinBox(bool);
    void deactivateCSpinBox(bool);

protected:

    void updateView();

    void setViewDefault();

    void loadInitialMesh();

    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    void mouseMoveEvent(QMouseEvent* ev);
    void mousePressEvent(QMouseEvent* ev);
    void mouseReleaseEvent(QMouseEvent* ev);

    void saveMeshAsTippeTop(QString fileName);
    void saveMeshAsTop(QString fileName);    
    void saveMeshAsYoyo(QString fileName);

    void resetGLWidget();

    void createGrid();
    void createGridForYoyo();


};

#endif // GLWIDGET_H
