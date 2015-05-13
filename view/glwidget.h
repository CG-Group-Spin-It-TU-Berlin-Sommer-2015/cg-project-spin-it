#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QtOpenGL>

#include "utility/mesh.h"
#include "utility/meshreader.h"
#include "utility/shader.h"


class GLWidget : public QGLWidget
{
    Q_OBJECT
private:
    GLuint program;
    Mesh* object;

public:
    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

signals:

public slots:

};

#endif // GLWIDGET_H
