#include "glwidget.h"

#define SCALE_FACTOR_TOP_WITHOUT_AXIS 3.0f
#define SCALE_FACTOR_TOP_WITH_AXIS 1.5f
#define SCALE_FACTOR_TIPPE_TOP 1.5f
#define SCALE_FACTOR_YOYO 3.f

#define Y_DEFAULT_VALUE 4.f

#define TEST_BO_2

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent),

    viewState(TRANSLATION_VIEW_XY),

    object(NULL),

    scaleFactor(1),

    showOuterSurface(true),
    showInnerSurface(false),
    showGrid(false),

    topOptimized(true),
    tippeTopOptimized(false),
    addSpinAxisToTop(true),
    octreeIsDirty(false),
    viewIsDirty(false)

{   

    alignPos = QVector3D(0,0,0);
    trans = QVector3D(0,0,0);

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

    trans = QVector3D(0,0,0);

    this->scale_xyz = 1.0f;

    this->startMaximalDepth = 7;
    this->optimizationMaximalDepth = 3;
    this->shellExtensionValue = 1;

    rebuildOctree = true;


    if(YOYO_MODE)
    {
        deactivateCSpinBox(true);
        setCWeightSpinBox(0);
        setIWeightSpinBox(BetaOptimization::gamma_i_yoyo);
        setLWeightSpinBox(BetaOptimization::gamma_l_yoyo);
    }
    else
    {
        activateCSpinBox(true);
        setCWeightSpinBox(BetaOptimization::gamma_c_top);
        setIWeightSpinBox(BetaOptimization::gamma_i_top);
        setLWeightSpinBox(BetaOptimization::gamma_l_top);
    }


    emit setStartDepthSpinBoxValue(this->startMaximalDepth);
    emit setMaximumDepthSpinBoxValue(this->optimizationMaximalDepth);
    emit setShellExtensionSpinBoxValue(shellExtensionValue);

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

/**
 * @brief GLWidget::initializeGL
 */
void GLWidget::initializeGL()
{

    glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    resetGLWidget();

    qglClearColor(QColor(205, 205, 255));

    colorshader = new QGLShaderProgram();
    colorshader->addShaderFromSourceFile(QGLShader::Vertex, ":/shader/shaders/colorshader.vsh");
    colorshader->addShaderFromSourceFile(QGLShader::Fragment, ":/shader/shaders/colorshader.fsh");
    colorshader->link();

    shader = new QGLShaderProgram();
    shader->addShaderFromSourceFile(QGLShader::Vertex, ":/shader/shaders/simple.vsh");
    shader->addShaderFromSourceFile(QGLShader::Fragment, ":/shader/shaders/simple.fsh");
    shader->link();

    setViewDefault();

    ambient_light.setX(0.75);
    ambient_light.setY(0.75);
    ambient_light.setZ(0.75);
    ambient_light.setW(1);

    diffuse_light.setX(0.75);
    diffuse_light.setY(0.75);
    diffuse_light.setZ(0.75);
    diffuse_light.setW(1);

    // load helper meshes
    rot_axis = readMeshFromObjFile(":/obj/spinIt_models/rot_axis.obj");
    half_sphere = readMeshFromObjFile(":/obj/spinIt_models/tippe_top_object.obj");
    yoyo_center = readMeshFromObjFile(":/obj/spinIt_models/yoyo_center_object.obj");

    setAddAxisCheckBox(this->addSpinAxisToTop);

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

    createGrid();

    last_object_model_matrix.setToIdentity();

    /* test loading start */

    loadInitialMesh();

    model_matrix.setToIdentity();

    model_matrix.translate(trans);
    model_matrix.rotate(-rot_obj_phi, 0.0, 1.0, 0.0);
    model_matrix.rotate(rot_obj_psy, 1.0, 0.0, 0.0);
    model_matrix.scale(scale_xyz);

    GLfloat defaultScaleFactor = 1.f;

    defaultScaleFactor = YOYO_MODE ? SCALE_FACTOR_YOYO:defaultScaleFactor;
    defaultScaleFactor = TIPPE_TOP_MODE ? SCALE_FACTOR_TIPPE_TOP:defaultScaleFactor;
    defaultScaleFactor = TOP_WITH_AXIS_MODE ? SCALE_FACTOR_TOP_WITH_AXIS:defaultScaleFactor;
    defaultScaleFactor = TOP_WITHOUT_AXIS_MODE ? SCALE_FACTOR_TOP_WITHOUT_AXIS:defaultScaleFactor;

    model_matrix.scale(defaultScaleFactor/scaleFactor);

    model_matrix.translate(-alignPos);

    last_object_model_matrix = model_matrix;

    calculateOctree();
    /* test loading end */

}

/**
 * @brief GLWidget::paintGL
 */
void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

    bool transformMode =
        viewState == TRANSLATION_VIEW_XY ||
        viewState == TRANSLATION_VIEW_Z ||
        viewState == ROTATION_SCALE_VIEW;

    shader->bind();

    shader->setUniformValue("direction_light", direction_light);
    shader->setUniformValue("ambient_light", ambient_light);
    shader->setUniformValue("diffuse_light", diffuse_light);

    model_matrix.setToIdentity();

    model_matrix.translate(trans);
    model_matrix.rotate(-rot_obj_phi, 0.0, 1.0, 0.0);
    model_matrix.rotate(rot_obj_psy, 1.0, 0.0, 0.0);
    model_matrix.scale(scale_xyz);


    GLfloat defaultScaleFactor = 1.f;

    defaultScaleFactor = YOYO_MODE ? SCALE_FACTOR_YOYO:defaultScaleFactor;
    defaultScaleFactor = TIPPE_TOP_MODE ? SCALE_FACTOR_TIPPE_TOP:defaultScaleFactor;
    defaultScaleFactor = TOP_WITH_AXIS_MODE ? SCALE_FACTOR_TOP_WITH_AXIS:defaultScaleFactor;
    defaultScaleFactor = TOP_WITHOUT_AXIS_MODE ? SCALE_FACTOR_TOP_WITHOUT_AXIS:defaultScaleFactor;

    model_matrix.scale(defaultScaleFactor/scaleFactor);

    model_matrix.translate(-alignPos);

    if((transformMode || showOuterSurface) && object != NULL)
    {

        shader->setUniformValue("nMatrix", view_matrix * model_matrix);
        shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);

        shader->setUniformValue("color", QColor(115, 115, 85));
        object->render(shader, GL_TRIANGLES);

        last_object_model_matrix = model_matrix;

    }

    model_matrix.setToIdentity();

    if(!transformMode && showInnerSurface && BetaOptimization::shellMesh != NULL)
    {
        model_matrix.rotate(BetaOptimization::align_phi,QVector3D(0,1,0));

        shader->setUniformValue("nMatrix", view_matrix * model_matrix);
        shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
        shader->setUniformValue("color", QColor(115, 115, 85));
        BetaOptimization::shellMesh->render(shader, GL_TRIANGLES);
    }
    else
    if(!transformMode && showGrid)
    {

        shader->release();
        colorshader->bind();

        model_matrix.rotate(BetaOptimization::align_phi,QVector3D(0,1,0));

        colorshader->setUniformValue("direction_light", direction_light);
        colorshader->setUniformValue("ambient_light", ambient_light);
        colorshader->setUniformValue("diffuse_light", diffuse_light);

        colorshader->setUniformValue("nMatrix", view_matrix * model_matrix);
        colorshader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
        colorshader->setUniformValue("color", QColor(Qt::blue));
        BetaOptimization::octree.renderOctreeGrid(colorshader);

        colorshader->release();
        shader->bind();

    }

    // helper object
    if(showOuterSurface || transformMode)
    {

        model_matrix.setToIdentity();
        shader->setUniformValue("nMatrix", view_matrix * model_matrix);
        shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
        shader->setUniformValue("color", QColor(Qt::red));

        if(TIPPE_TOP_MODE)
        {
            half_sphere->render(shader, GL_TRIANGLES);
        }
        else
        if(TOP_WITH_AXIS_MODE)
        {
            rot_axis->render(shader, GL_TRIANGLES);
        }
        else
        if(YOYO_MODE)
        {
            shader->setUniformValue("color", QColor(Qt::blue));
            yoyo_center->render(shader, GL_TRIANGLES);
        }
    }

    // grid
    if(topOptimized)
    {
        model_matrix.setToIdentity();
        shader->setUniformValue("nMatrix", view_matrix * model_matrix);
        shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
        shader->setUniformValue("color", QColor(Qt::black));
        grid->render(shader, GL_LINES);
    }

    shader->release();

}

/**
 * @brief GLWidget::resizeGL
 * @param width
 * @param height
 */
void GLWidget::resizeGL(int width, int height)
{
    projection_matrix.setToIdentity();
    projection_matrix.perspective(30,(float)width/(float)height,5,200);

    glViewport(0,0, width, height);
}

/**
 * @brief GLWidget::mouseMoveEvent
 * @param ev
 */
void GLWidget::mouseMoveEvent(QMouseEvent *ev)
{
    (void)ev;

    if (left_pressed) {


        if(this->viewState != TRANSLATION_VIEW_DEFAULT)
        {
            // if xy translation should be set
            if(this->viewState == TRANSLATION_VIEW_XY){

                GLfloat trans_x = trans.x();
                GLfloat trans_z = trans.z();

                trans_x -= (mouse_pos.x() - ev->pos().x())/TRANSLATION_XY_RATIO;
                trans_z -= (mouse_pos.y() - ev->pos().y())/TRANSLATION_XY_RATIO;

                trans.setX(trans_x);
                trans.setZ(trans_z);

                octreeIsDirty = true;
            }
            else
            // if z translation should be set
            if(this->viewState == TRANSLATION_VIEW_Z)
            {
                GLfloat trans_y = trans.y();

                trans_y += (mouse_pos.y() - ev->pos().y())/TRANSLATION_Z_RATIO;

                trans.setY(trans_y);

                octreeIsDirty = true;
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

               octreeIsDirty = true;
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

            octreeIsDirty = true;

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

}

/**
 * @brief GLWidget::mousePressEvent
 * @param ev
 */
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

/**
 * @brief GLWidget::mouseReleaseEvent
 * @param ev
 */
void GLWidget::mouseReleaseEvent(QMouseEvent *ev)
{
    (void)ev;

    left_pressed = false;
    right_pressed = false;
    middle_pressed = false;
}

/**
 * @brief GLWidget::loadInitialMesh
 */
void GLWidget::loadInitialMesh()
{

    Mesh* newObject = readMeshFromObjFile(":/obj/test_models/standard_objs/cube.obj");

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

    scaleFactor = object->getMaxDistance2Middle();

    emit modelLoaded(true);
    emit setViewBack(true);
    emit deactivateViewControlWidget(true);

    if(YOYO_MODE)
    {
        alignPos = QVector3D(0,0,0);
        trans = QVector3D(0,0,0);
    }
    else
    if(TOP_WITH_AXIS_MODE || TIPPE_TOP_MODE)
    {
        alignPos = object->getMiddle();
        trans = QVector3D(0,3,0);
    }
    else
    if(TOP_WITHOUT_AXIS_MODE)
    {
        alignPos = object->getLowestPoint();
        trans = QVector3D(0,0,0);
    }

    this->rot_obj_phi = 0;
    this->rot_obj_psy = 0;

    rebuildOctree = true;

    updateView();

}

/**
 * @brief GLWidget::loadNewMesh
 */
void GLWidget::loadNewMesh()
{
    QString s = QFileDialog::getOpenFileName(this,tr("Open Mesh"), "../", tr("Meshes (*.obj)"));
    Mesh* newObject = readMeshFromObjFile(s.toStdString());

    if(newObject==NULL)
    {
        return;
    }

    if(newObject->getGeometry()->size()<1 || newObject->getIndices()->size()<1)
    {
        delete newObject;
        return;
    }

    if(object!=NULL)
    {
        delete object;
    }

    object = newObject;

    resetGLWidget();

    scaleFactor = object->getMaxDistance2Middle();

    emit modelLoaded(true);
    emit setViewBack(true);
    emit deactivateViewControlWidget(true);

    if(YOYO_MODE)
    {
        alignPos = QVector3D(0,0,0);
        trans = QVector3D(0,0,0);
    }
    else
    if(TOP_WITH_AXIS_MODE || TIPPE_TOP_MODE)
    {
        alignPos = QVector3D(0,0,0);
        //deactivated
        //alignPos = object->getMiddle();
        trans = QVector3D(0,3,0);
    }
    else
    if(TOP_WITHOUT_AXIS_MODE)
    {
        alignPos = object->getLowestPoint();
        trans = QVector3D(0,0,0);
    }

    this->rot_obj_phi = 0;
    this->rot_obj_psy = 0;

    rebuildOctree = true;

    updateView();

}

/**
 * @brief GLWidget::makeItSpin
 */
void GLWidget::makeItSpin()
{

    if(rebuildOctree || octreeIsDirty)
    {
        this->calculateOctree();
    }

    rebuildOctree = true;

    if(TOP_WITHOUT_AXIS_MODE || TOP_WITH_AXIS_MODE)
    {

        BetaOptimization::doTopOptimization();

    }
    else
    if(TIPPE_TOP_MODE)
    {

        BetaOptimization::doTippeTopOptimization();

    }
    else
    if(YOYO_MODE)
    {

        BetaOptimization::doYoyoOptimization();

    }

    this->updateGL();
}

/* ----------------------------------------------- */
/* slots for controlling painting of the different models */

/**
 * @brief GLWidget::showOnlyOuterSurface
 */
void GLWidget::showOnlyOuterSurface()
{

    showOuterSurface = true;
    showInnerSurface = false;
    showGrid = false;

    this->updateGL();

}

/**
 * @brief GLWidget::showOnlyInnerSurface
 */
void GLWidget::showOnlyInnerSurface()
{

    showOuterSurface = false;
    showInnerSurface = true;
    showGrid = false;

    this->updateGL();

}

/**
 * @brief GLWidget::showOnlyOctreeGrid
 */
void GLWidget::showOnlyOctreeGrid()
{

    showOuterSurface = false;
    showInnerSurface = false;
    showGrid = true;

    this->updateGL();

}

/* ----------------------------------------------- */

/**
 * @brief GLWidget::setView
 * @param index
 */
void GLWidget::setView(int index)
{

    switch(index)
    {
        case TRANSLATION_TAB:

            if(TOP_WITH_AXIS_MODE || TIPPE_TOP_MODE)
            {
                deactivateViewControlWidget(true);
            }
            viewIsDirty = true;

            emit resetTranformationWidget();
        break;
        case MODEL_TAB:
            activateViewControlWidget(true);
            if(viewIsDirty)
            {

                viewIsDirty = false;

                rot_cam_phi = 0;
                emit setViewBack(true);
            }

            if(octreeIsDirty)
            {
                emit deactivateViewControlWidget(true);
                emit shellIsNotSet(true);
            }

            this->setViewDefault();
        break;
        case HOLLOWING_TAB:
            activateViewControlWidget(true);
            if(viewIsDirty)
            {

                viewIsDirty = false;

                rot_cam_phi = 0;
                emit setViewBack(true);
            }

            if(octreeIsDirty)
            {
                emit deactivateViewControlWidget(true);
                emit shellIsNotSet(true);
            }

            this->setViewDefault();
        break;

    }

}



/**
 * @brief GLWidget::calculateOctree Calculate the octree
 */
void GLWidget::calculateOctree()
{

    Mesh* newModifiedMesh = object->copy();
    newModifiedMesh->transform(this->last_object_model_matrix);

    if(TIPPE_TOP_MODE)
    {
        Mesh* tempMesh = booleanUnion(newModifiedMesh,half_sphere);
        delete newModifiedMesh;
        newModifiedMesh = tempMesh;
    }

    BetaOptimization::initializeOctree(
                newModifiedMesh,
                this->startMaximalDepth,
                this->optimizationMaximalDepth,
                this->shellExtensionValue);

    rebuildOctree = false;
    octreeIsDirty=false;

    emit activateViewControlWidget(true);
    emit shellIsSet(true);

    this->updateGL();

}

/**
 * @brief GLWidget::saveMesh
 */
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

    if(octreeIsDirty)
    {

        Mesh* modifiedMesh = object->copy();
        modifiedMesh->transform(this->last_object_model_matrix);
        Mesh* mesh = booleanUnion(modifiedMesh,half_sphere);
        writeMeshFromObjFile(fileName.toStdString(),mesh);
        delete mesh;
        delete modifiedMesh;

    }
    else
    {

        Mesh* shell = BetaOptimization::getShellMesh(true);
        Mesh* mesh = mergeMeshes(BetaOptimization::mesh,shell);
        writeMeshFromObjFile(fileName.toStdString(),mesh);

        delete shell;
        delete mesh;

    }

}

/**
 * @brief GLWidget::saveMeshAsTop Save object as top
 * @param fileName name and path of file
 */
void GLWidget::saveMeshAsTop(QString fileName)
{

    if(octreeIsDirty)
    {
        Mesh* mesh = object->copy();

        mesh->transform(this->last_object_model_matrix);

        if(TOP_WITH_AXIS_MODE)
        {
            mesh = booleanUnion(mesh,rot_axis);
        }

        writeMeshFromObjFile(fileName.toStdString(),mesh);

        delete mesh;

    }
    else
    {

        Mesh* shell = BetaOptimization::getShellMesh(true);
        Mesh* objMesh = BetaOptimization::mesh;

        if(TOP_WITH_AXIS_MODE)
        {
            objMesh = booleanUnion(objMesh,rot_axis);
        }

        Mesh* mesh = mergeMeshes(objMesh,shell);
        writeMeshFromObjFile(fileName.toStdString(),mesh);

        if(TOP_WITH_AXIS_MODE)
        {
            delete objMesh;
        }

        delete shell;
        delete mesh;

    }

}

/**
 * @brief GLWidget::saveMeshAsYoyo Save object as yoyo
 * @param fileName name and path of file
 */
void GLWidget::saveMeshAsYoyo(QString fileName)
{

    if(octreeIsDirty)
    {

        Mesh* modifiedMesh = object->copy();
        modifiedMesh->transform(this->last_object_model_matrix);
        writeMeshFromObjFile(fileName.toStdString(),modifiedMesh);
        delete modifiedMesh;

    }
    else
    {

        Mesh* shell = BetaOptimization::getShellMesh(true);
        Mesh* mesh = mergeMeshes(BetaOptimization::mesh,shell);
        writeMeshFromObjFile(fileName.toStdString(),mesh);

        delete shell;
        delete mesh;

    }

}

/**
 * @brief GLWidget::setYoyo
 */
void GLWidget::setYoyo()
{
    this->topOptimized = false;
    octreeIsDirty=true;

    emit setViewBack(true);
    emit deactivateViewControlWidget(true);
    emit shellIsNotSet(true);

    updateView();
}

/**
 * @brief GLWidget::setTop
 */
void GLWidget::setTop()
{
    this->topOptimized = true;
    octreeIsDirty=true;

    emit setViewBack(true);
    emit deactivateViewControlWidget(true);
    emit shellIsNotSet(true);

    updateView();
}

/**
 * @brief GLWidget::setTippeTop
 * @param tippeTop
 */
void GLWidget::setTippeTop(bool tippeTop)
{
    this->tippeTopOptimized = tippeTop;
    octreeIsDirty=true;

    emit setViewBack(true);
    emit deactivateViewControlWidget(true);
    emit shellIsNotSet(true);

    updateView();
}

/**
 * @brief GLWidget::setAddAxis
 * @param addAxis
 */
void GLWidget::setAddAxis(bool addAxis)
{
    this->addSpinAxisToTop = addAxis;
    octreeIsDirty=true;

    emit setViewBack(true);
    emit deactivateViewControlWidget(true);
    emit shellIsNotSet(true);

    updateView();
}

/**
 * @brief GLWidget::updateView
 */
void GLWidget::updateView()
{

    if(YOYO_MODE)
    {
        emit yoyoIsSet(true);
    }
    else
    if(TOP_WITH_AXIS_MODE)
    {
        emit activateAddAxisCheckBox(true);
        emit activateTippeTopCheckBox(true);
        emit topWithAxisIsSet(true);
    }
    else
    if(TOP_WITHOUT_AXIS_MODE)
    {
        emit activateAddAxisCheckBox(true);
        emit activateTippeTopCheckBox(true);
        emit topWithoutAxisIsSet(true);
    }
    else
    if(TIPPE_TOP_MODE)
    {
        emit activateTippeTopCheckBox(true);
        emit tippeTopIsSet(true);
    }

    if(object != NULL)
    {
        if(YOYO_MODE)
        {
            alignPos = QVector3D(0,0,0);
            trans = QVector3D(0,0,0);
        }
        else
        if(TOP_WITH_AXIS_MODE || TIPPE_TOP_MODE)
        {
            alignPos = QVector3D(0,0,0);
            //deactivated
            //alignPos = object->getMiddle();
            trans = QVector3D(0,3,0);
        }
        else
        if(TOP_WITHOUT_AXIS_MODE)
        {
            alignPos = object->getLowestPoint();
            trans = QVector3D(0,0,0);
        }

        this->rot_obj_phi = 0;
        this->rot_obj_psy = 0;
    }

    if(YOYO_MODE)
    {
        deactivateCSpinBox(true);
        setCWeightSpinBox(0);
        setIWeightSpinBox(BetaOptimization::gamma_i_yoyo);
        setLWeightSpinBox(BetaOptimization::gamma_l_yoyo);
    }
    else
    if(TIPPE_TOP_MODE)
    {
        deactivateCSpinBox(true);
        setCWeightSpinBox(0);
        setIWeightSpinBox(BetaOptimization::gamma_i_tippe_top);
        setLWeightSpinBox(BetaOptimization::gamma_l_tippe_top);
    }
    else
    {
        activateCSpinBox(true);
        setCWeightSpinBox(BetaOptimization::gamma_c_top);
        setIWeightSpinBox(BetaOptimization::gamma_i_top);
        setLWeightSpinBox(BetaOptimization::gamma_l_top);
    }

    setViewDefault();
}

/**
 * @brief GLWidget::setViewXY
 */
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

/**
 * @brief GLWidget::setViewZ
 */
void GLWidget::setViewZ()
{
    // view from the front

    camera_position.setX(0);
    camera_position.setY(4);
    camera_position.setZ(-15);
    camera_direction.setX(0);
    camera_direction.setY(3);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(1);
    camera_up.setZ(0);

    this->viewState = TRANSLATION_VIEW_Z;

    this->updateGL();

}

/**
 * @brief GLWidget::setViewRotationScale
 */
void GLWidget::setViewRotationScale()
{

    camera_position.setX(0);
    camera_position.setY(10);
    camera_position.setZ(-15);
    camera_direction.setX(0);
    camera_direction.setY(3);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(1);
    camera_up.setZ(0);

    this->viewState = ROTATION_SCALE_VIEW;

    this->updateGL();

}

/**
 * @brief GLWidget::setViewDefault
 */
void GLWidget::setViewDefault()
{

    if(YOYO_MODE)
    {

        camera_position.setX(0);
        camera_position.setY(7);
        camera_position.setZ(-15);
        camera_direction.setX(0);
        camera_direction.setY(0);
        camera_direction.setZ(0);

    }
    else
    {

        camera_position.setX(0);
        camera_position.setY(10);
        camera_position.setZ(-15);
        camera_direction.setX(0);
        camera_direction.setY(3);
        camera_direction.setZ(0);

    }

    camera_up.setX(0);
    camera_up.setY(1);
    camera_up.setZ(0);

    this->viewState = TRANSLATION_VIEW_DEFAULT;

    this->updateGL();

}

/**
 * @brief GLWidget::resetXY
 */
void GLWidget::resetXY()
{
    trans.setX(0);
    trans.setZ(0);

    octreeIsDirty = true;
    this->updateGL();
}

/**
 * @brief GLWidget::resetZ
 */
void GLWidget::resetZ()
{
    trans.setY(3);

    octreeIsDirty = true;
    this->updateGL();
}

/**
 * @brief GLWidget::resetRotationScale
 */
void GLWidget::resetRotationScale()
{
    this->rot_obj_phi = 0;
    this->rot_obj_psy = 0;
    this->scale_xyz = 1;

    octreeIsDirty = true;
    this->updateGL();
}

/**
 * @brief GLWidget::resetAll
 */
void GLWidget::resetAll()
{

    trans = QVector3D(0,3,0);
    this->rot_obj_phi = 0;
    this->rot_obj_psy = 0;
    this->scale_xyz = 1;
    this->updateGL();
}

/**
 * @brief GLWidget::setStartDepthValue
 * @param value
 */
void GLWidget::setStartDepthValue(int value)
{
    this->startMaximalDepth = value;
    rebuildOctree = true;

}

/**
 * @brief GLWidget::setMaximumDepthValue
 * @param value
 */
void GLWidget::setMaximumDepthValue(int value)
{
    this->optimizationMaximalDepth = value;
    rebuildOctree = true;

}

/**
 * @brief GLWidget::setShellExtensionValue
 * @param value
 */
void GLWidget::setShellExtensionValue(int value)
{
    this->shellExtensionValue = value;
    rebuildOctree = true;

}


void GLWidget::setCWeight(double value)
{
    BetaOptimization::gamma_c_top=value;
}

void GLWidget::setIWeight(double value)
{
    if(YOYO_MODE)
    {
        BetaOptimization::gamma_i_yoyo=value;
    }
    else
    {
        if(TIPPE_TOP_MODE)
        {
            BetaOptimization::gamma_i_tippe_top=value;
        }
        else
        {
            BetaOptimization::gamma_i_top=value;
        }
    }
}

void GLWidget::setLWeight(double value)
{
    if(YOYO_MODE)
    {
        BetaOptimization::gamma_l_yoyo=value;
    }
    else
    {
        if(TIPPE_TOP_MODE)
        {
            BetaOptimization::gamma_l_tippe_top=value;
        }
        else
        {
            BetaOptimization::gamma_l_top=value;
        }
    }
}

