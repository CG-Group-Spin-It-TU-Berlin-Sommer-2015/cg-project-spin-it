#include "newglwidget.h"

#include <glm/glm.hpp>

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

// constants for window and frostum width and height
#define GL_WINDOW_WIDTH 800.0
#define GL_WINDOW_HEIGHT 500.0
#define FROSTUM_WIDTH 8.0
#define FROSTUM_HEIGHT 6.0

NewGLWidget::NewGLWidget()
{
	
    xRot = .0;
    yRot = .0;
    zRot = .0;

    xModelRot = .0;
    yModelRot = .0;
    zModelRot = .0;

	isClicked = false;
	scaleValue = 1.0;

    qtPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);

	lastPos = QPoint(0,0);
	windowPos = QPointF(0,0);

	firstPos = QPoint(0,0);
	secondPos = QPoint(0,0);

    model = NULL;

	isMergeMode = false;
	isGrabMode = false;

	scrollScaleValue = 1.0;
}

NewGLWidget::~NewGLWidget()
{

	if(model != NULL)
	{
		delete model;
	}

}

/*
 * initialize Open GL
 */
void NewGLWidget::initializeGL()
{
    qglClearColor(qtPurple.dark());

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
	glShadeModel(GL_SMOOTH);
    
	glEnable(GL_MULTISAMPLE);
    
	glEnable(GL_NORMALIZE);

	float pos0[] = {1.0,5.0,7.0,1.0};
	float diffuse0[] = {1.0,1.0,1.0,1.0};
	float ambient0[] = {0.0,0.0,0.0,1.0};
	float specular0[] = {1.0,1.0,1.0,1.0};

	glLightfv(GL_LIGHT0, GL_POSITION, pos0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular0);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
    glEnable ( GL_COLOR_MATERIAL );

}

/*
 * open model from file referd to meshPath
 */
void NewGLWidget::openModel(char* meshPath)
{

	if(model != NULL)
	{
		delete model;
	}

	// initiate and load initial and modified model
	model = new Model();
	model->loadModel(meshPath);

	xRot = .0;
    yRot = .0;
    zRot = .0;
    xModelRot = .0;
    yModelRot = .0;
    zModelRot = .0;
	scrollScaleValue = 1.0;

	this->isGrabMode = false;
	this->isClicked = false;

	if(!model->meshLoaded)
	{
		return;
	}

	// set scale value for the view
	scaleValue = model->maxCoor;

}

/*
 * make 4by4 matrix
 */
inline glm::mat4 makeMat4(GLfloat* arr)
{
	glm::vec4 vec1(arr[ 0], arr[ 1], arr[ 2], arr[ 3]);
	glm::vec4 vec2(arr[ 4], arr[ 5], arr[ 6], arr[ 7]);
	glm::vec4 vec3(arr[ 8], arr[ 9], arr[10], arr[11]);
	glm::vec4 vec4(arr[12], arr[13], arr[14], arr[15]);

	return glm::mat4(vec1,vec2,vec3,vec4);
}

/*
 * make homogenous point
 */
inline glm::vec4 makePointVec4(float* arr,int index)
{
	return glm::vec4(arr[index], arr[index+1], arr[index+2], 1);
}

/*
 * make homogenous vector
 */
inline glm::vec4 makeVecVec4(float* arr,int index)
{
	return glm::vec4(arr[index], arr[index+1], arr[index+2], 0);
}

/*
 * mark vertices by rect using value for left, right, top and bottom value
 */
void NewGLWidget::markVerticesByRect(float left, float right, float top, float bottom)
{

	// test whether modified model is set and loaded
	if(this->model == NULL || !this->model->meshLoaded)
	{
		return;
	}

	GLfloat projectMatrix[16]; 
	glGetFloatv(GL_PROJECTION_MATRIX, projectMatrix);

	glm::mat4 mat1,mat2,mat3;
	glm::vec4 vec_in,vec_out;

	// calculate matrix
	mat1 = makeMat4(modelMatrix);
	mat2 = makeMat4(projectMatrix);
	mat3 = mat2*mat1;

	// set vector for new marked vertices
	std::vector<int>* newMarkedVertices = new std::vector<int>();
	newMarkedVertices->reserve(this->model->numVertices);

	// check for each vertex whether he is marked
	for(int i=0;i<this->model->numVertices;i++)
	{	
		vec_in = makePointVec4(this->model->vertices,i*3);
		vec_out = mat3*vec_in;
		vec_out = vec_out/vec_out.data[3];

		if(left>vec_out[0] || right<vec_out[0])
		{	
			continue;
		}
		if(bottom>vec_out[1] || top<vec_out[1])
		{
			continue;
		}

		newMarkedVertices->push_back(i);
	}

	if(!this->isMergeMode)
	{
		this->model->unmarkAllVertices();
	}

	this->model->markVertices(newMarkedVertices);

	delete newMarkedVertices;

}

/*
 * unmark or mark all vertices (if all vertices are marked then unmark all vertices, otherwise mark all vertices)
 */
void NewGLWidget::unmarkMarkAllVertices()
{

	if(isGrabMode)
	{
		return;
	}

	// test whether modified model is set and loaded
	if(this->model == NULL || !this->model->meshLoaded)
	{
		return;
	}

	bool allVerticesMarked = this->model->markedVertices->size() == this->model->numVertices;

	if(!allVerticesMarked)
	{
		// not all vertices are marked

		std::vector<int>* newMarkedVertices = new std::vector<int>();

		newMarkedVertices->reserve(this->model->numVertices);

		for(int i=0;i<this->model->numVertices;i++)
		{
			newMarkedVertices->push_back(i);
		}
		this->model->markVertices(newMarkedVertices);
		
		delete newMarkedVertices;
	}
	else
	{
		// all vertices are marked

		this->model->unmarkAllVertices();
	}

}

/*
 * TODO: description
 */
void NewGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if(model == NULL || !model->meshLoaded)
	{
		return;
	}

	glLoadIdentity();
	
	// draw cross and rectangle
	if(isClicked && !isGrabMode)
	{
		drawCross(&windowPos);
		drawRectangle(&firstPos,&secondPos);
	}

	glTranslatef(0.0, 0.0, -10.0);

	// rotate
	glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
	glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
	glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

    glPushMatrix();

    glRotatef(xModelRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yModelRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zModelRot / 16.0, 0.0, 0.0, 1.0);

	float normedScaleValue = 4.0/scaleValue;

	// scale
	glScalef(normedScaleValue,normedScaleValue,normedScaleValue);
	glScalef(scrollScaleValue,scrollScaleValue,scrollScaleValue);

	// paint model and vertices of the model 
    paintModel(model);
    paintVertices(model);

    glPopMatrix();

	paintSpinAxis();

	glFlush();

	glGetFloatv(GL_MODELVIEW_MATRIX, modelMatrix);
}

/*
 * draw rectangle defined by points pos1 and pos2
 */
void NewGLWidget::drawRectangle(QPoint* pos1, QPoint* pos2)
{
	QPointF pos1t;
	QPointF pos2t;

	// get screen points from world points
	transformPoint(pos1, &pos1t);
	transformPoint(pos2, &pos2t);

	float x1,y1,x2,y2;

	x1 = pos1t.x();
	y1 = pos1t.y();
	x2 = pos2t.x();
	y2 = pos2t.y();

	glPushMatrix();
	glTranslatef(0, 0, -4.0);
	glScalef(-1.0,-1.0,0.0);

	glDisable(GL_LIGHTING);

    glLineWidth(1);
	glColor3f(1.0,1.0,1.0);

	// draw rectangle
	glBegin(GL_LINES);
	glVertex2f(x1, y1);
	glVertex2f(x1, y2);
	glVertex2f(x1, y2);
	glVertex2f(x2, y2);
	glVertex2f(x2, y2);
	glVertex2f(x2, y1);
	glVertex2f(x2, y1);
	glVertex2f(x1, y1);
	glEnd();

	glPopMatrix();

	glEnable(GL_LIGHTING);
}

/*
 * draw cross centered around point pos
 */
void NewGLWidget::drawCross(QPointF* pos)
{
	glPushMatrix();
	glTranslatef(-pos->x(), -pos->y(), -4.0);

	glDisable(GL_LIGHTING);

    glLineWidth(1);
	glColor3f(1.0,1.0,1.0);

	// draw cross
	glBegin(GL_LINES);
	glVertex2f(0.0f, 0.0f);
	glVertex2f(1.0f, 0.0f);
	glVertex2f(0.0f, 0.0f);
	glVertex2f(-1.0f, 0.0f);
	glVertex2f(0.0f, 0.0f);
	glVertex2f(0.0f, 1.0f);
	glVertex2f(0.0f, 0.0f);
	glVertex2f(0.0f, -1.0f);
	glEnd();

	glPopMatrix();

	glEnable(GL_LIGHTING);
}

/*
 * paint model model
 */
void NewGLWidget::paintModel(Model* model)
{

	glColor3f(1.0,1.0,1.0);

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	// draw model by using vertices and normals
	glVertexPointer(3,GL_FLOAT,0,model->vertices);
	glNormalPointer(GL_FLOAT,0,model->normals);
    glDrawElements(GL_TRIANGLES,model->numIndices*3,GL_UNSIGNED_INT,model->indices);
    
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

}

/*
 * paint vertices of model model
 */
void NewGLWidget::paintVertices(Model* model)
{

	glDisable(GL_LIGHTING);

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	if(this->isGrabMode)
	{
		glPointSize(6);
	}
	else
	{
		glPointSize(3);
	}
	
	// draw vertices by using vertices and colors
	glVertexPointer(3,GL_FLOAT,0,model->vertices);
	glColorPointer(3,GL_FLOAT,0,model->colors);
	glDrawElements(GL_POINTS,model->numVertices,GL_UNSIGNED_INT,model->counterIndices);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	glEnable(GL_LIGHTING);
}

/*
 * paint spin axis
 */
void NewGLWidget::paintSpinAxis()
{
	glDisable(GL_LIGHTING);
	
    float value = 1000.0f;
	
	glLineWidth(2.5);
	
	glColor3f(1.0, 0.0, 0.0);
	
	glBegin(GL_LINES);
	glVertex3f(0.0, -value, 0.0);
	glVertex3f(0.0, +value, 0.0);
	glEnd();

	glEnable(GL_LIGHTING);
}

/*
 * resize window with size width and height
 */
void NewGLWidget::resizeGL(int width, int height)
{
	width = GL_WINDOW_WIDTH;
	height = GL_WINDOW_HEIGHT;

    int side = qMin(width, height);
    glViewport(0 , 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

	glFrustum(-FROSTUM_WIDTH, +FROSTUM_WIDTH, -FROSTUM_HEIGHT, +FROSTUM_HEIGHT, 4.0, 100.0);
	glMatrixMode(GL_MODELVIEW);

}

/*
 * callback for mouse press event
 */
void NewGLWidget::mousePressEvent(QMouseEvent *event)
{
	lastPos = event->pos();

    return;

    if ( event->buttons() & Qt::RightButton)
	{
		isClicked = true;
		
		// set first and second point of the rectangle
		firstPos = event->pos();
		secondPos = event->pos();

		transformPoint(&lastPos, &windowPos);
		
		updateGL();

	}	
}

/*
 * callback for mouse release event
 */
void NewGLWidget::mouseReleaseEvent(QMouseEvent *event)
{

    return;

    if ( event->button() == Qt::RightButton) {

		if(!isClicked)
		{
			return;
		}

		isClicked = false;

		if(isGrabMode)
		{
			updateGL();
			return;
		}

		float right,left,top,bottom;
		QPointF p1,p2;

		// transform first and second world points to screen points
		transformPoint(&firstPos,&p1);
		transformPoint(&secondPos,&p2);

		float scalar;

		// order values from left to right (there is a negation below)
		if(p1.x()>p2.x())
		{
			left = p1.x();
			right = p2.x();
		}
		else
		{
			right = p1.x();
			left = p2.x();
		}

		scalar = -FROSTUM_WIDTH;

		left /=scalar;
		right /=scalar;

		// order values from bottom to top (there is a negation below)
		if(p1.y()>p2.y())
		{
			bottom = p1.y();
			top = p2.y();
		}
		else
		{
			top = p1.y();
			bottom = p2.y();
		}
		
		scalar = -FROSTUM_HEIGHT;

		bottom /=scalar;
		top /=scalar;

		// test which vertices should be marked
		markVerticesByRect(left,right,top,bottom);

		updateGL();

	}
}

/*
 * callback for mouse move event
 */
void NewGLWidget::mouseMoveEvent(QMouseEvent *event)
{

    if(isGrabMode)
	{

		// get vector for grabing marked vertices
		int dx = event->x() - lastPos.x();
		int dy = event->y() - lastPos.y();
		
		grabMarkedPoints(dx, dy);

		lastPos = event->pos();

		updateGL();
		
		return;
	}

    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

	// rotate model
    if (event->buttons() & Qt::RightButton) {
        setXRotation(xRot + 8 * dy);
        setYRotation(yRot + 8 * dx);
    }

    // rotate model
    if (event->buttons() & Qt::LeftButton) {
        setXModelRotation(xModelRot + 8 * dy);
        setYModelRotation(yModelRot + 8 * dx);
    }
	
	lastPos = event->pos();
	secondPos = event->pos();

	transformPoint(&lastPos, &windowPos);

	updateGL();
}

/*
 * transform a point accoring to point i and return it by point o
 */
void NewGLWidget::transformPoint(QPoint* i, QPointF* o)
{
	float w = GL_WINDOW_WIDTH;
	float h = GL_WINDOW_HEIGHT;

	float x = (i->x()/w-0.5)*(FROSTUM_WIDTH*2);
	float y = (i->y()/h-0.5)*(FROSTUM_HEIGHT*2);

	x *=-1;
	o->setX(x);
	o->setY(y);
}

/*
 * grab marked points about x and y
 */
void NewGLWidget::grabMarkedPoints(int x, int y)
{

    float vecScaleValue = 0.05f;

	glm::mat4 mat,mat_inv;
	glm::vec4 vec_x,vec_y,vec_out;

	// calculate inverse matrix
	mat = makeMat4(modelMatrix);
	mat_inv = glm::inverse(mat);

	float v1[] = {1,0,0};
	float v2[] = {0,1,0};

	// get homogenous vectors for x-axis and y-axis
	vec_x = makeVecVec4(v1,0);
	vec_y = makeVecVec4(v2,0);

	vec_x = mat_inv*vec_x;
	vec_y = mat_inv*vec_y;

	// norm and scale vectors for x-axis and y-axis
	vec_x /= vec_x.length();
	vec_y /= vec_y.length();
	vec_x *=vecScaleValue*x;
	vec_y *=vecScaleValue*y*-1;

	vec_out = vec_x+vec_y;

	this->model->grabMarkedVertices(&vec_out);
	this->model->makeVerticesConsistent();

}

/*
 * set hint
 */
QSize NewGLWidget::sizeHint() const
{
    return QSize(GL_WINDOW_WIDTH, GL_WINDOW_HEIGHT);
}

/*
 * limited angle between 0 and 360 * 16
 */
static void qNormalizeAngle(int &angle)
{
    while (angle < .0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

/*
 * set x rotation
 */
void NewGLWidget::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != xRot) {
        xRot = angle;
        emit xRotationChanged(angle);
    }
}

/*
 * set y rotation
 */
void NewGLWidget::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != yRot) {
        yRot = angle;
        emit yRotationChanged(angle);
    }
}

/*
 * set z rotation
 */
void NewGLWidget::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != zRot) {
        zRot = angle;
        emit zRotationChanged(angle);
    }
}

/*
 * set x rotation
 */
void NewGLWidget::setXModelRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != xModelRot) {
        xModelRot = angle;
        emit xModelRotationChanged(angle);
    }
}

/*
 * set y rotation
 */
void NewGLWidget::setYModelRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != yModelRot) {
        yModelRot = angle;
        emit yModelRotationChanged(angle);
    }
}

/*
 * set z rotation
 */
void NewGLWidget::setZModelRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != zModelRot) {
        zModelRot = angle;
        emit zModelRotationChanged(angle);
    }
}

/*
* set mark mode
*/
void NewGLWidget::setMarkMode(bool isMergeMode)
{
	this->isMergeMode = isMergeMode;
}

/*
 * change scroll scale value
 */
void NewGLWidget::changeScrollScaleValue(float stepValue)
{
	this->scrollScaleValue += stepValue;

	if(this->scrollScaleValue<0.5)
	{
		this->scrollScaleValue = 0.5;
	}
}

/*
 * switch grap mode
 */
void NewGLWidget::switchGrabMode()
{

	if(model == NULL || !model->meshLoaded)
	{
		return;
	}

	if(this->model->markedVerticesMap->empty()){
		return;
	}

	this->isGrabMode = !this->isGrabMode;

	if(isGrabMode)
	{
		isClicked = false;

	}

	if(isGrabMode)
	{
		this->model->setMarkedVerticesForGrabMode();
	}
	else
	{
		this->model->setMarkedVerticesForDefaultMode();
	}

	updateGL();

}

/*
 * save model from file refered to meshPath
 */
void NewGLWidget::saveModel(char* meshPath){

	if(model != NULL)
	{

	}

}

/*
 * test whether a model is loaded
 */
bool NewGLWidget::modelLoaded(){


	if(model == NULL || !model->meshLoaded)
	{
		return false;
	}

	return true;

}
