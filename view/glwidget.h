#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>

#include "utility/shader.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT
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
