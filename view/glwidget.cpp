#include "glwidget.h"

#define DEFAULT_SCALE_FACTOR 1.66238999
#define Y_DEFAULT_VALUE 4.f

#define USE_OPTIMIZATION

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{   

    showOuterSurface = true;
    showInnerSurface = false;
    showGrid = false;
    viewState = TRANSLATION_VIEW_XY;

    object = NULL; 

    middle.setX(0);
    middle.setY(0);
    middle.setZ(0);
    scaleFactor = 1;

    topOptimized = true;
    tippeTopOptimized = false;

}

GLWidget::~GLWidget()
{
    delete shader;
    delete colorshader;

    delete object;
    delete grid;
    delete rot_axis;
    delete half_sphere;
}

/**
 * @brief GLWidget::resetGLWidget
 */
void GLWidget::resetGLWidget()
{

    this->rot_obj_phi = 0.f;
    this->rot_obj_psy = 0.f;
    this->rot_cam_phi = 0.f;

    this->trans_x = 0.f;
    this->trans_y = Y_DEFAULT_VALUE;
    this->trans_z = 0.f;
    this->scale_xyz = 1.0f;

    this->startMaximalDepth = 6;
    this->optimizationMaximalDepth = 3;
    emit setStartDepthSpinBoxValue(this->startMaximalDepth);
    emit setMaximumDepthSpinBoxValue(this->optimizationMaximalDepth);

    shellExtensionValue = 1;
    emit setShellExtensionSpinBoxValue(shellExtensionValue);

    emit shellIsNotSet();
    emit shellIsNotSet(true);

}

/**
 * @brief GLWidget::createGrid
 */
void GLWidget::createGrid()
{

    QVector<GLfloat>* geometry = new QVector<GLfloat>();
    QVector<GLfloat>* normals = new QVector<GLfloat>();
    QVector<GLint>* indices = new QVector<GLint>();

    int idx = 0;
    for (float x = -3; x <= 3; x += 0.5) {
        for (float z = -3; z <= 3; z += 0.5) {
            geometry->push_back(x);
            geometry->push_back(0);
            geometry->push_back(z);

            normals->push_back(0);
            normals->push_back(1);
            normals->push_back(0);

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

    grid = new Mesh(geometry, normals, indices);

}

void GLWidget::initializeGL()
{

    glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    resetGLWidget();

    qglClearColor(QColor(205, 205, 255));

    colorshader = new QGLShaderProgram();
    colorshader->addShaderFromSourceFile(QGLShader::Vertex, ":/shader/colorshader.vsh");
    colorshader->addShaderFromSourceFile(QGLShader::Fragment, ":/shader/colorshader.fsh");
    colorshader->link();

    shader = new QGLShaderProgram();
    shader->addShaderFromSourceFile(QGLShader::Vertex, ":/shader/simple.vsh");
    shader->addShaderFromSourceFile(QGLShader::Fragment, ":/shader/simple.fsh");
    shader->link();

    camera_position.setX(0);
    camera_position.setY(10);
    camera_position.setZ(-10);
    camera_direction.setX(0);
    camera_direction.setY(4);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(1);
    camera_up.setZ(0);

    ambient_light.setX(0.75);
    ambient_light.setY(0.75);
    ambient_light.setZ(0.75);
    ambient_light.setW(1);

    diffuse_light.setX(0.75);
    diffuse_light.setY(0.75);
    diffuse_light.setZ(0.75);
    diffuse_light.setW(1);

    // load helper meshes
    rot_axis = readMeshFromObjFileDirectory("rot_axis_bold");
    half_sphere = readMeshFromObjFileDirectory("tippe_top_object");
    yoyo_center = readMeshFromObjFileDirectory("yoyo_center_object");

    QMatrix4x4 mat;

    // align rotation axis to touch point
    QVector3D lowestPointRotAxis = rot_axis->getLowestPoint();
    mat.setToIdentity();
    mat.translate(-lowestPointRotAxis);
    rot_axis->transform(mat);

    // align half sphere to touch point
    QVector3D lowestPointHalfSphere = half_sphere->getLowestPoint();
    mat.setToIdentity();
    mat.translate(-lowestPointHalfSphere);
    half_sphere->transform(mat);

    // align yoyo center to origin
    QVector3D lowestPointYoyoCenter = yoyo_center->getLowestPoint();
    lowestPointYoyoCenter.setY(lowestPointYoyoCenter.y()-4.0);
    mat.setToIdentity();
    mat.translate(-lowestPointYoyoCenter);
    yoyo_center->transform(mat);

    createGrid();

    last_object_model_matrix.setToIdentity();

    /* test loading start */
    loadInitialMesh();

    model_matrix.setToIdentity();

    model_matrix.translate(-trans_x,trans_y,trans_z);
    model_matrix.translate(-middle.x(), -middle.y(), -middle.z());

    model_matrix.rotate(rot_obj_phi, 0.0, 1.0, 0.0);
    model_matrix.rotate(rot_obj_psy, 1.0, 0.0, 0.0);

    model_matrix.scale(scale_xyz);
    model_matrix.scale(DEFAULT_SCALE_FACTOR/scaleFactor);

    last_object_model_matrix = model_matrix;

    calculateOctree();
    /* test loading end */

}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    shader->bind();

    view_matrix.setToIdentity();
    view_matrix.lookAt(camera_position, camera_direction, camera_up);

    if(this->viewState == TRANSLATION_VIEW_DEFAULT)
    {
        view_matrix.rotate(-rot_cam_phi, 0.0, 1.0, 0.0);
    }

    direction_light.setX(camera_direction.normalized().x() - camera_position.normalized().x());
    direction_light.setY(camera_direction.normalized().y() - camera_position.normalized().y());
    direction_light.setZ(camera_direction.normalized().z() - camera_position.normalized().z());
    direction_light.setW(0);

    shader->setUniformValue("direction_light", direction_light);
    shader->setUniformValue("ambient_light", ambient_light);
    shader->setUniformValue("diffuse_light", diffuse_light);

    model_matrix.setToIdentity();

    model_matrix.translate(-trans_x,trans_y,-trans_z);
    model_matrix.translate(-middle.x(), -middle.y(), -middle.z());

    model_matrix.rotate(-rot_obj_phi, 0.0, 1.0, 0.0);
    model_matrix.rotate(rot_obj_psy, 1.0, 0.0, 0.0);

    model_matrix.scale(scale_xyz);
    model_matrix.scale(DEFAULT_SCALE_FACTOR/scaleFactor);

    shader->setUniformValue("nMatrix", view_matrix * model_matrix);
    shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);

    if(showOuterSurface && object != NULL)
    {
        shader->setUniformValue("color", QColor(115, 115, 85));
        object->render(shader, GL_TRIANGLES);
    }

    last_object_model_matrix = model_matrix;

    model_matrix.setToIdentity();

    shader->setUniformValue("nMatrix", view_matrix * model_matrix);
    shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);

    #ifdef USE_OPTIMIZATION

    if(showInnerSurface && BetaOptimization::shellMesh != NULL)
    {
        shader->setUniformValue("color", QColor(115, 115, 85));
        BetaOptimization::shellMesh->render(shader, GL_TRIANGLES);
    }

    #else

    if(showInnerSurface && Model::shellMesh != NULL)
    {
        shader->setUniformValue("color", QColor(115, 115, 85));
        Model::shellMesh->render(shader, GL_TRIANGLES);
    }

    #endif

    shader->release();
    colorshader->bind();

    colorshader->setUniformValue("nMatrix", view_matrix * model_matrix);
    colorshader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);

    #ifdef USE_OPTIMIZATION

    if(showGrid && BetaOptimization::shellMesh != NULL)
    {
        colorshader->setUniformValue("color", QColor(Qt::blue));
        BetaOptimization::octree.renderOctreeGrid(colorshader);
    }

    #else

    if(showGrid && Model::shellMesh != NULL)
    {
        colorshader->setUniformValue("color", QColor(Qt::blue));
        Model::octree->renderOctreeGrid(colorshader);
    }

    #endif

    colorshader->release();
    shader->bind();

    if(showOuterSurface)
    {
    if(topOptimized && tippeTopOptimized)
    {
        // draw half sphere
        model_matrix.setToIdentity();
        shader->setUniformValue("nMatrix", view_matrix * model_matrix);
        shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
        shader->setUniformValue("color", QColor(Qt::red));
        half_sphere->render(shader, GL_TRIANGLES);
    }
    else
    if(topOptimized)
    {
        // draw rotation axis
        model_matrix.setToIdentity();
        shader->setUniformValue("nMatrix", view_matrix * model_matrix);
        shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
        shader->setUniformValue("color", QColor(Qt::red));
        rot_axis->render(shader, GL_TRIANGLES);

    }
    else
    {
        // draw yoyo area
        model_matrix.setToIdentity();
        shader->setUniformValue("nMatrix", view_matrix * model_matrix);
        shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
        shader->setUniformValue("color", QColor(Qt::blue));
        yoyo_center->render(shader, GL_TRIANGLES);

    }
    }

    // draw grid
    model_matrix.setToIdentity();
    model_matrix.translate(0.0, 0.0, 0.0);
    shader->setUniformValue("nMatrix", view_matrix * model_matrix);
    shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
    shader->setUniformValue("color", QColor(Qt::black));
    grid->render(shader, GL_LINES);

    shader->release();

}

void GLWidget::resizeGL(int width, int height)
{
    projection_matrix.setToIdentity();
    projection_matrix.perspective(60,1,5,200);

    glViewport(0,0, width, height);
}

void GLWidget::mouseMoveEvent(QMouseEvent *ev)
{
    (void)ev;

    if (left_pressed) {


        if(this->viewState != TRANSLATION_VIEW_DEFAULT)
        {
            // if xy translation should be set
            if(this->viewState == TRANSLATION_VIEW_XY){
                this->trans_x += (mouse_pos.x() - ev->pos().x())/TRANSLATION_XY_RATIO;
                this->trans_z += (mouse_pos.y() - ev->pos().y())/TRANSLATION_XY_RATIO;
            }
            else
            // if z translation should be set
            if(this->viewState == TRANSLATION_VIEW_Z)
            {
                this->trans_y += (mouse_pos.y() - ev->pos().y())/TRANSLATION_Z_RATIO;
            }
            else
            // if rotation and scale should be set
            if(this->viewState == ROTATION_SCALE_VIEW)
            {
               if (mouse_pos.x() > ev->pos().x()) {
                   rot_obj_phi += (mouse_pos.x() - ev->pos().x());
               }
               if (mouse_pos.x() < ev->pos().x()) {
                   rot_obj_phi -= (ev->pos().x() - mouse_pos.x());
               }
               rot_obj_phi %= 360;

               if (mouse_pos.y() > ev->pos().y()) {
                   rot_obj_psy += (mouse_pos.y() - ev->pos().y());
               }
               if (mouse_pos.y() < ev->pos().y()) {
                   rot_obj_psy -= (ev->pos().y() - mouse_pos.y());
               }
               rot_obj_psy %= 360;

            }

            mouse_pos = ev->pos();
            this->updateGL();
            return;
        }

        if (mouse_pos.x() > ev->pos().x()) {
            rot_cam_phi += (mouse_pos.x() - ev->pos().x());
        }
        if (mouse_pos.x() < ev->pos().x()) {
            rot_cam_phi -= (ev->pos().x() - mouse_pos.x());
        }
        rot_cam_phi %= 360;

        mouse_pos = ev->pos();
        this->updateGL();
        return;

    }
    if (right_pressed) {

        // if rotation and scale should be set
        if(this->viewState == ROTATION_SCALE_VIEW)
        {
            scale_xyz += (ev->pos().y() - mouse_pos.y())/SCALE_RATIO;
            scale_xyz = scale_xyz > 0.1f ? scale_xyz:0.1f;
            scale_xyz = scale_xyz < 3.0f ? scale_xyz:3.0f;

            mouse_pos = ev->pos();
            this->updateGL();
            return;
        }

        if(this->viewState != TRANSLATION_VIEW_DEFAULT)
        {
            mouse_pos = ev->pos();
            return;
        }

        mouse_pos = ev->pos();
        return;

    }
    if (middle_pressed) {

        float scale = 0.05f;

        trans_x += (mouse_pos.x() - ev->pos().x())*scale;
        trans_z += (mouse_pos.y() - ev->pos().y())*scale;

        mouse_pos = ev->pos();
        this->updateGL();
    }
}

void GLWidget::mousePressEvent(QMouseEvent *ev)
{
    (void)ev;

    if (ev->button() == Qt::LeftButton) {
        mouse_pos = ev->pos();
        left_pressed = true;
        return;
    }
    if (ev->button() == Qt::RightButton) {
        mouse_pos = ev->pos();
        right_pressed = true;
        return;
    }
    if (ev->button() == Qt::MiddleButton) {
        mouse_pos = ev->pos();
        middle_pressed = true;
        return;
    }
}

void GLWidget::mouseReleaseEvent(QMouseEvent *ev)
{
    (void)ev;

    left_pressed = false;
    right_pressed = false;
    middle_pressed = false;
}

void GLWidget::loadInitialMesh()
{

    Mesh* newObject = readMeshFromObjFileDirectory("cube");

    if(newObject==NULL)
    {
        return;
    }

    if(object!=NULL)
    {
        delete object;
    }

    object = newObject;

    resetGLWidget();

    middle = object->getMiddle();
    scaleFactor = object->getMaxDistance2Middle();

    //trans_y = 2.f;

    emit modelLoaded(true);

    this->updateGL();

}

void GLWidget::loadNewMesh()
{
    QString s = QFileDialog::getOpenFileName(this,tr("Open Mesh"), "../", tr("Meshes (*.obj)"));
    Mesh* newObject = readMeshFromObjFile(s.toStdString());

    if(newObject==NULL)
    {
        return;
    }

    if(object!=NULL)
    {
        delete object;
    }

    object = newObject;

    resetGLWidget();

    middle = object->getMiddle();
    scaleFactor = object->getMaxDistance2Middle();

    emit modelLoaded(true);

    this->updateGL();
}

void GLWidget::makeItSpin()
{

    if(rebuildOctree)
    {
        this->calculateOctree();
    }

    rebuildOctree = true;

    #ifdef USE_OPTIMIZATION

    BetaOptimization::mesh->swapYZ();
    BetaOptimization::optimizeBetasBottomUp(OPTIMIZATION_TYPE_TOP);
    BetaOptimization::optimizeBetasWithSplitAndMerge(OPTIMIZATION_TYPE_TOP);
    //BetaOptimization::optimizeBetas(OPTIMIZATION_TYPE_TOP);
    BetaOptimization::mesh->swapYZ();

    //BetaOptimization::testSimpleSplitAndMerge();
    //BetaOptimization::testSplitAndMerge();

    #else

    Model::mesh->swapYZ();
    Model::hollow();
    Model::mesh->swapYZ();

    #endif

    this->updateGL();
}

/* ----------------------------------------------- */
/* slots for controlling painting of the different models */

void GLWidget::showOnlyOuterSurface()
{

    showOuterSurface = true;
    showInnerSurface = false;
    showGrid = false;

    this->updateGL();

}

void GLWidget::showOnlyInnerSurface()
{

    showOuterSurface = false;
    showInnerSurface = true;
    showGrid = false;

    this->updateGL();

}

void GLWidget::showOnlyOctreeGrid()
{

    showOuterSurface = false;
    showInnerSurface = false;
    showGrid = true;

    this->updateGL();

}

/* ----------------------------------------------- */

void GLWidget::setView(int index)
{

    switch(index)
    {
        case TRANSLATION_TAB:
            emit this->setViewXYSignal();
        break;
        case MODEL_TAB:
            this->setViewDefault();
        break;
        case HOLLOWING_TAB:
            this->setViewDefault();
        break;

    }

}

void GLWidget::setViewXY()
{

    // bird eye view

    camera_position.setX(0);
    camera_position.setY(15);
    camera_position.setZ(0);
    camera_direction.setX(0);
    camera_direction.setY(0);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(0);
    camera_up.setZ(-1);

    this->viewState = TRANSLATION_VIEW_XY;

    this->updateGL();

}

void GLWidget::setViewZ()
{
    // view from the front

    camera_position.setX(0);
    camera_position.setY(4);
    camera_position.setZ(-10);
    camera_direction.setX(0);
    camera_direction.setY(4);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(1);
    camera_up.setZ(0);

    this->viewState = TRANSLATION_VIEW_Z;

    this->updateGL();

}

void GLWidget::setViewRotationScale()
{

    camera_position.setX(0);
    camera_position.setY(10);
    camera_position.setZ(-10);
    camera_direction.setX(0);
    camera_direction.setY(4);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(1);
    camera_up.setZ(0);

    this->viewState = ROTATION_SCALE_VIEW;

    this->updateGL();

}

void GLWidget::setViewDefault()
{

    camera_position.setX(0);
    camera_position.setY(10);
    camera_position.setZ(-10);
    camera_direction.setX(0);
    camera_direction.setY(4);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(1);
    camera_up.setZ(0);

    this->viewState = TRANSLATION_VIEW_DEFAULT;

    this->updateGL();

}

void GLWidget::resetXY()
{
    this->trans_x = 0;
    this->trans_z = 0;
    this->updateGL();
}

void GLWidget::resetZ()
{
    this->trans_y = Y_DEFAULT_VALUE;
    this->updateGL();
}

void GLWidget::resetRotationScale()
{
    this->rot_obj_phi = 0;
    this->rot_obj_psy = 0;
    this->scale_xyz = 1;
    this->updateGL();
}

void GLWidget::resetAll()
{

    this->trans_x = 0;
    this->trans_y = Y_DEFAULT_VALUE;
    this->trans_z = 0;
    this->rot_obj_phi = 0;
    this->rot_obj_psy = 0;
    this->scale_xyz = 1;
    this->updateGL();
}

void GLWidget::setStartDepthValue(int value)
{
    this->startMaximalDepth = value;
}

void GLWidget::setMaximumDepthValue(int value)
{
    this->optimizationMaximalDepth = value;
}

void GLWidget::setShellExtensionValue(int value)
{
    this->shellExtensionValue = value;
}

/**
 * @brief GLWidget::calculateOctree Calculate the octree
 */
void GLWidget::calculateOctree()
{

    Mesh* newModifiedMesh = object->copy();
    newModifiedMesh->transform(this->last_object_model_matrix);

    // calculate the difference for the yoyo
    if(topOptimized && tippeTopOptimized)
    {
        Mesh* tempMesh = booleanUnion(newModifiedMesh,half_sphere);
        delete newModifiedMesh;
        newModifiedMesh = tempMesh;
    }
    if(topOptimized)
    {
        Mesh* tempMesh = booleanUnion(newModifiedMesh,rot_axis);
        delete newModifiedMesh;
        newModifiedMesh = tempMesh;
    }

    #ifdef USE_OPTIMIZATION

    BetaOptimization::initializeOctree(
                newModifiedMesh,
                this->startMaximalDepth,
                this->optimizationMaximalDepth,
                this->shellExtensionValue);

    #else

    Model::initializeOctree(
                newModifiedMesh,
                this->startMaximalDepth,
                this->optimizationMaximalDepth,
                this->shellExtensionValue);

    #endif

    rebuildOctree = false;

    emit shellIsSet(true);

    this->updateGL();

}

void GLWidget::saveMesh()
{

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Mesh"),"spinIt.obj",tr("Meshes (*.obj)"));

    if(fileName.isEmpty())
    {
        return;
    }

    if(topOptimized && tippeTopOptimized)
    {
        saveMeshAsTippeTop(fileName);
    }
    else
    if(topOptimized)
    {
        saveMeshAsTop(fileName);
    }
    else
    {
        saveMeshAsYoyo(fileName);
    }
}

/**
 * @brief GLWidget::saveMeshAsTippeTop Save object as tippe top
 * @param fileName name and path of file
 */
void GLWidget::saveMeshAsTippeTop(QString fileName)
{

    #ifdef USE_OPTIMIZATION

    Mesh* shell = BetaOptimization::octree.getShellMesh(true);
    Mesh* mesh1 = booleanUnion(BetaOptimization::mesh,half_sphere);
    Mesh* mesh2 = mergeMeshes(mesh1,shell);
    writeMeshFromObjFile(fileName.toStdString(),mesh2);

    #else

    Mesh* shell = Model::octree->getShellMesh(true);
    Mesh* mesh1 = booleanUnion(Model::mesh,half_sphere);
    Mesh* mesh2 = mergeMeshes(mesh1,shell);
    writeMeshFromObjFile(fileName.toStdString(),mesh2);

    #endif

    delete shell;
    delete mesh1;
    delete mesh2;
}

/**
 * @brief GLWidget::saveMeshAsTop Save object as top
 * @param fileName name and path of file
 */
void GLWidget::saveMeshAsTop(QString fileName)
{

    #ifdef USE_OPTIMIZATION

    Mesh* shell = BetaOptimization::octree.getShellMesh(true);
    Mesh* mesh1 = booleanUnion(BetaOptimization::mesh,rot_axis);
    Mesh* mesh2 = mergeMeshes(mesh1,shell);
    writeMeshFromObjFile(fileName.toStdString(),mesh2);

    #else

    Mesh* shell = Model::octree->getShellMesh(true);
    Mesh* mesh1 = booleanUnion(Model::mesh,rot_axis);
    Mesh* mesh2 = mergeMeshes(mesh1,shell);
    writeMeshFromObjFile(fileName.toStdString(),mesh2);

    #endif

    delete shell;
    delete mesh1;
    delete mesh2;
}

/**
 * @brief GLWidget::saveMeshAsYoyo Save object as yoyo
 * @param fileName name and path of file
 */
void GLWidget::saveMeshAsYoyo(QString fileName)
{

    #ifdef USE_OPTIMIZATION

    Mesh* shell = BetaOptimization::octree.getShellMesh(true);
    Mesh* mesh = mergeMeshes(BetaOptimization::mesh,shell);
    writeMeshFromObjFile(fileName.toStdString(),mesh);

    #else

    Mesh* shell = Model::octree->getShellMesh(true);
    Mesh* mesh = mergeMeshes(Model::mesh,shell);
    writeMeshFromObjFile(fileName.toStdString(),mesh);

    #endif

    delete shell;
    delete mesh;
}

void GLWidget::setTippeTop(bool tippeTop)
{
    this->tippeTopOptimized = tippeTop;
    this->updateGL();
}

void GLWidget::setYoyo()
{
    this->topOptimized = false;
    this->updateGL();
}

void GLWidget::setTop()
{
    this->topOptimized = true;
    this->updateGL();
}
