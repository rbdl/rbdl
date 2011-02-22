/****************************************************************************
**
** Copyright (C) 2009 Nokia Corporation and/or its subsidiary(-ies).
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL$
** Commercial Usage
** Licensees holding valid Qt Commercial licenses may use this file in
** accordance with the Qt Commercial License Agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Nokia.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Nokia gives you certain
** additional rights. These rights are described in the Nokia Qt LGPL
** Exception version 1.0, included in the file LGPL_EXCEPTION.txt in this
** package.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3.0 as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU General Public License version 3.0 requirements will be
** met: http://www.gnu.org/copyleft/gpl.html.
**
** If you are unsure which license is appropriate for your use, please
** contact the sales department at http://www.qtsoftware.com/contact.
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QtGui>
#include <QtOpenGL>
#include <QDebug>

#include <algorithm>
#include <iostream>
#include <cmath>

#include <sys/time.h>
#include <ctime>

#include <assert.h>
#include "glwidget.h"
#include <GL/glu.h>

#include "modelstate.h"
#include "modeldrawing.h"
#include "glprimitives.h"

using namespace std;

static bool update_simulation = false;

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
	poi.setX(0.);
	poi.setY(-0.4);
	poi.setZ(0.);

	eye.setX(4.);
	eye.setY(1.);
	eye.setZ(4.);

	updateSphericalCoordinates();

	/*
	qDebug () << r;
	qDebug () << theta;
	qDebug () << phi;
	*/

	model_init();

	delta_time_sec = -1.;

	setFocusPolicy(Qt::StrongFocus);
	setMouseTracking(true);
}

GLWidget::~GLWidget() {
	makeCurrent();
	glprimitives_destroy();
}

void GLWidget::update_timer() {
	struct timeval clock_value;
	gettimeofday (&clock_value, NULL);

	unsigned int clock_time = clock_value.tv_sec * 1000. + clock_value.tv_usec * 1.0e-3;

	if (delta_time_sec < 0.) {
		delta_time_sec = 0.;
		application_time_msec = 0.;
		first_frame_msec = clock_time;

		return;
	}

	unsigned int last_frame_time_msec = application_time_msec;
	application_time_msec = clock_time - first_frame_msec;

	delta_time_sec = static_cast<double>(application_time_msec - last_frame_time_msec) * 1.0e-3;
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(400, 400);
}

void GLWidget::initializeGL()
{
//	qDebug() << "initializeGL() called";
	glClearColor (0.3, 0.3, 0.3, 1.);
	glClearDepth (1.);
	glEnable (GL_DEPTH_TEST);
	glDepthFunc (GL_LESS);
	glEnable (GL_CULL_FACE);

	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();

	// initialize the glprimitives (cubes, etc.)
	glprimitives_init();

	// initialize lights
	GLfloat light_ka[] = { 0.2f, 0.2f, 0.2f, 1.0f};
	GLfloat light_kd[] = { 0.7f, 0.7f, 0.7f, 1.0f};
	GLfloat light_ks[] = { 1.0f, 1.0f, 1.0f, 1.0f};

	glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ka);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_kd);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_ks);

	GLfloat light_pos[4] = {20.0f, 20.0f, 20.0f, 1.0f};
	glLightfv (GL_LIGHT0, GL_POSITION, light_pos);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}

void GLWidget::updateSphericalCoordinates() {
	QVector3D los = poi - eye;
	r = los.length();
	theta = acos (-los.y() / r);
	phi = atan (los.z() / los.x());
}

void GLWidget::updateCamera() {
	// update the camera
	float s_theta, c_theta, s_phi, c_phi;
	s_theta = sin (theta);
	c_theta = cos (theta);
	s_phi = sin (phi);
	c_phi = cos (phi);

	eye.setX(r * s_theta * c_phi);
	eye.setY(r * c_theta);
	eye.setZ(r * s_theta * s_phi);

	eye += poi;

	QVector3D right (-s_phi, 0., c_phi);

	QVector3D eye_normalized (eye);
	eye_normalized.normalize();

	up = QVector3D::crossProduct (right, eye_normalized);

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt (eye.x(), eye.y(), eye.z(),
			poi.x(), poi.y(), poi.z(),
			up.x(), up.y(), up.z());
}

void GLWidget::paintGL() {
	update_timer();
	glClearColor (0.3, 0.3, 0.3, 1.);

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();

	updateCamera();

	if (update_simulation)
		model_update (delta_time_sec);

//	draw_model (model_get());
}

void GLWidget::resizeGL(int width, int height)
{
//	qDebug() << "resizing to" << width << "x" << height;

	if (height == 0)
		height = 1;

	if (width == 0)
		width = 1;

	glViewport (0, 0, width, height);

	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();

	float fov = 45;
	gluPerspective (fov, (GLfloat) width / (GLfloat) height, 0.005, 200);

	glMatrixMode (GL_MODELVIEW);
}

void GLWidget::keyPressEvent(QKeyEvent* event) {
	if (event->key() == Qt::Key_Return) {
		if (!update_simulation)
			update_simulation = true;
		else 
			update_simulation = false;
	}
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
 	lastMousePos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastMousePos.x();
	int dy = event->y() - lastMousePos.y();

	if (event->buttons().testFlag(Qt::LeftButton)) {
		// rotate
		phi += 0.01 * dx;
		theta -= 0.01 * dy;

		theta = std::max(theta, 0.01f);
		theta = std::min(theta, static_cast<float>(M_PI * 0.99));
	} else if (event->buttons().testFlag(Qt::MiddleButton)) {
		// move
		QVector3D eye_normalized (eye);
		eye_normalized.normalize();
		QVector3D right = QVector3D::crossProduct (up, eye_normalized) * -1.;
		poi += right * dx * 0.01 + up * dy * 0.01;
		eye += right * dx * 0.01 + up * dy * 0.01;
	} else if (event->buttons().testFlag(Qt::RightButton)) {
		// zoom
		r += 0.05 * dy;
		r = std::max (0.01f, r);
	}

	lastMousePos = event->pos();

	updateGL();
}

