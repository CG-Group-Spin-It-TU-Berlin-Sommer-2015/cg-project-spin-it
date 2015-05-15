#ifndef NewGLWidget_H
#define NewGLWidget_H

#include <QGLWidget>
#include <QMouseEvent>

#include "utility/Model.h"

class NewGLWidget : public QGLWidget
{

	Q_OBJECT

public:
    explicit NewGLWidget();
    ~NewGLWidget();

    QSize sizeHint() const;

	void openModel(char* meshPath);
	void saveModel(char* meshPath);

	bool modelLoaded();

	bool isClicked;
	float scaleValue;

	void unmarkMarkAllVertices();
	void setMarkMode(bool isMergeMode);
	void switchGrabMode();
	void changeScrollScaleValue(float stepValue);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

private:
    int xRot;
    int yRot;
    int zRot;
    QPoint lastPos;
	QPointF windowPos;
	QPoint firstPos;
	QPoint secondPos;
    QColor qtPurple;

	Model* model;
	void paintModel(Model* model);
	void paintVertices(Model* model);

    void paintSpinAxis();
	void transformPoint(QPoint* i, QPointF* o);
	void grabMarkedPoints(int x, int y);

	void markVerticesByRect(float left, float right, float top, float bottom);

	float scrollScaleValue;

	void drawRectangle(QPoint* pos1, QPoint* pos2);
	void drawCross(QPointF* pos);

	bool isMergeMode;
	bool isGrabMode;

	GLfloat modelMatrix[16];

public slots:
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);

signals:
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);
};

#endif // NewGLWidget_H
