#include "glwidget.h"

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{   

    showOuterSurface = true;
    showInnerSurface = false;
    showGrid = false;
    viewState = TRANSLATION_VIEW_XY;

}

GLWidget::~GLWidget()
{
    delete shader;

    delete object;
    delete grid;
    delete rot_axis;
}

void GLWidget::initializeGL()
{

    shellExtensionValue = 0;
    emit setShellExtensionSpinBoxValue(shellExtensionValue);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    this->rot_obj_phi = 0;
    this->rot_obj_psy = 0;
    this->rot_cam_phi = 0;
    this->trans_x = 0.f;
    this->trans_y = 0.f;
    this->trans_z = 0.f;
    this->scale_xyz = 1.0f;

    this->startDepth = 2;
    this->maximumDepth = 2;
    emit setStartDepthSpinBoxValue(this->startDepth);
    emit setMaximumDepthSpinBoxValue(this->maximumDepth);

    qglClearColor(QColor(205, 205, 255));

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

    rot_axis = readMeshFromObjFileDirectory("rot_axis");

    object = readMeshFromObjFileDirectory("test");
    emit modelLoaded(true);


    //Model::initialize(object);


    // test octree

    setView(TRANSLATION_TAB);

    // write the final mesh into a file
    /*
    octree.createInnerSurface();
    Mesh* objectShell_flipped = octree.getMesh(true);
    Mesh* merge1 = booleanUnion(object,rot_axis);
    Mesh* merge2 = mergeMeshes(merge1,objectShell_flipped);
    writeMeshFromObjFile("test.obj",merge2);
    */


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

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    shader->bind();

    view_matrix.setToIdentity();
    view_matrix.lookAt(camera_position, camera_direction, camera_up);

    if(this->viewState == TRANSLATION_VIEW_DEFAULT)
    {
        view_matrix.rotate(rot_cam_phi, 0.0, 1.0, 0.0);
    }

    direction_light.setX(camera_direction.normalized().x() - camera_position.normalized().x());
    direction_light.setY(camera_direction.normalized().y() - camera_position.normalized().y());
    direction_light.setZ(camera_direction.normalized().z() - camera_position.normalized().z());
    direction_light.setW(0);

    shader->setUniformValue("direction_light", direction_light);
    shader->setUniformValue("ambient_light", ambient_light);
    shader->setUniformValue("diffuse_light", diffuse_light);

    model_matrix.setToIdentity();
    GLfloat x_min, x_max, y_min, y_max, z_min, z_max;

    /*
    for (int i = 0; i < object->getGeometry()->size(); i += 3) {
        if (object->getGeometry()->at(i) > x_max) {
            x_max = object->getGeometry()->at(i);
        }
        if (object->getGeometry()->at(i) < x_min) {
            x_min = object->getGeometry()->at(i);
        }

        if (object->getGeometry()->at(i + 1) > y_max) {
            y_max = object->getGeometry()->at(i + 1);
        }
        if (object->getGeometry()->at(i + 1) < y_min) {
            y_min = object->getGeometry()->at(i + 1);
        }

        if (object->getGeometry()->at(i + 2) > z_max) {
            z_max = object->getGeometry()->at(i + 2);
        }
        if (object->getGeometry()->at(i + 2) < z_min) {
            z_min = object->getGeometry()->at(i + 2);
        }
    }
    */

    x_min = 0;
    x_max = 0;
    y_min = 0;
    y_max = 0;
    z_min = 0;
    z_max = 0;


    model_matrix.translate(-trans_x,trans_y,trans_z);

    model_matrix.translate(-(x_max + x_min) / 2, -(y_max + y_min) / 2, -(z_max + z_min) / 2);

    model_matrix.rotate(rot_obj_phi, 0.0, 1.0, 0.0);
    model_matrix.rotate(rot_obj_psy, 1.0, 0.0, 0.0);

    QMatrix4x4 model_matrix_temp = model_matrix;

    model_matrix.scale(scale_xyz);

    shader->setUniformValue("nMatrix", view_matrix * model_matrix);
    shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);

    if(showOuterSurface)
    {
        shader->setUniformValue("color", QColor(115, 115, 85));
        object->render(shader, GL_TRIANGLES);
    }

    model_matrix = model_matrix_temp;

    shader->setUniformValue("nMatrix", view_matrix * model_matrix);
    shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);

    if(showInnerSurface)
    {
        shader->setUniformValue("color", QColor(115, 115, 85));
        //objectShell->render(shader, GL_TRIANGLES);
    }

    if(showGrid)
    {
        shader->setUniformValue("color", QColor(Qt::black));
        //octree.render(shader);
    }

    model_matrix.setToIdentity();
    GLfloat lowest_y = 0.0;
    for (int i = 1; i < rot_axis->getGeometry()->size(); i += 3) {
        if (rot_axis->getGeometry()->at(i) < lowest_y) {
            lowest_y = rot_axis->getGeometry()->at(i);
        }
    }
    model_matrix.translate(0.0, -2 - lowest_y, 0.0);
    shader->setUniformValue("nMatrix", view_matrix * model_matrix);
    shader->setUniformValue("mvpMatrix", projection_matrix * view_matrix * model_matrix);
    shader->setUniformValue("color", QColor(Qt::red));
    rot_axis->render(shader, GL_TRIANGLES);

    model_matrix.setToIdentity();
    model_matrix.translate(0.0, -2.0, 0.0);
    shader->setUniformValue("nMatrix", view_matrix * model_matrix);
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

        if(this->viewState != TRANSLATION_VIEW_DEFAULT)
        {
            if(this->viewState == TRANSLATION_VIEW_XY){
                this->trans_x += (mouse_pos.x() - ev->pos().x())/TRANSLATION_XY_RATIO;
                this->trans_z += (mouse_pos.y() - ev->pos().y())/TRANSLATION_XY_RATIO;
            }
            else
            if(this->viewState == TRANSLATION_VIEW_Z)
            {
                this->trans_y += (mouse_pos.y() - ev->pos().y())/TRANSLATION_Z_RATIO;
            }
            else
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
    left_pressed = false;
    right_pressed = false;
    middle_pressed = false;
}

void GLWidget::loadNewMesh()
{
    QString s = QFileDialog::getOpenFileName(this,tr("Open Mesh"), "../", tr("Meshes (*.obj)"));
    Mesh* object = readMeshFromObjFile(s.toStdString());
    Model::initialize(object);

    if(object==0)
    {
        return;
    }

    this->object = object;

    this->rot_obj_phi = 0;
    this->rot_obj_psy = 0;
    this->rot_cam_phi = 0;
    this->trans_x = 0;
    this->trans_z = 0;

    this->repaint();
}

void GLWidget::makeItSpin()
{
    Model::hollow();
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

    camera_position.setX(0);
    camera_position.setY(-1);
    camera_position.setZ(0);
    camera_direction.setX(0);
    camera_direction.setY(0);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(0);
    camera_up.setZ(1);

    this->viewState = TRANSLATION_VIEW_XY;

    this->updateGL();

}

void GLWidget::setViewZ()
{
    camera_position.setX(0);
    camera_position.setY(0);
    camera_position.setZ(1);
    camera_direction.setX(0);
    camera_direction.setY(0);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(1);
    camera_up.setZ(0);

    this->viewState = TRANSLATION_VIEW_Z;

    this->updateGL();

}

void GLWidget::setViewRotationScale()
{
    camera_position.setX(1);
    camera_position.setY(-0.75);
    camera_position.setZ(1);
    camera_direction.setX(0);
    camera_direction.setY(0);
    camera_direction.setZ(0);

    camera_up.setX(0);
    camera_up.setY(1);
    camera_up.setZ(0);

    this->viewState = ROTATION_SCALE_VIEW;

    this->updateGL();

}

void GLWidget::setViewDefault()
{
    camera_position.setX(1);
    camera_position.setY(-0.75);
    camera_position.setZ(1);
    camera_direction.setX(0);
    camera_direction.setY(0);
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
    this->trans_y = 0;
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
    this->trans_y = 0;
    this->trans_z = 0;
    this->rot_obj_phi = 0;
    this->rot_obj_psy = 0;
    this->scale_xyz = 1;
    this->updateGL();
}

void GLWidget::setStartDepthValue(int value)
{

    this->startDepth = value;
    if(this->maximumDepth<this->startDepth)
    {
       this->maximumDepth = this->startDepth;
       emit setMaximumDepthSpinBoxValue(this->maximumDepth);
    }

}

void GLWidget::setMaximumDepthValue(int value)
{
    this->maximumDepth = value;
    if(this->maximumDepth<this->startDepth)
    {
       this->startDepth = this->maximumDepth;
       emit setStartDepthSpinBoxValue(this->startDepth);
    }
}

void GLWidget::setShellExtensionValue(int value)
{
    this->shellExtensionValue = value;
}

void GLWidget::calculateOctree()
{

}
