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

#include <assert.h>
#include "glwidget.h"
#include <GL/glu.h>

using namespace std;

GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
	poi.setX(0.);
	poi.setY(0.);
	poi.setZ(0.);

	eye.setX(2.);
	eye.setY(1.);
	eye.setZ(2.);

	QVector3D los = poi - eye;
	r = los.length();
	theta = acos (-los.y() / r);
	phi = atan (los.z() / los.x());

	/*
	qDebug () << r;
	qDebug () << theta;
	qDebug () << phi;
	*/

	setFocusPolicy(Qt::StrongFocus);
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
}

void GLWidget::updateCamera() {
	// update the camera
	poi.setX(0.);
	poi.setY(0.);
	poi.setZ(0.);

	float s_theta, c_theta, s_phi, c_phi;
	s_theta = sin (theta);
	c_theta = cos (theta);
	s_phi = sin (phi);
	c_phi = cos (phi);

	eye.setX(r * s_theta * c_phi);
	eye.setY(r * c_theta);
	eye.setZ(r * s_theta * s_phi);

	up.setX(0.);
	up.setY(1.);
	up.setZ(0.);

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt (eye.x(), eye.y(), eye.z(),
			poi.x(), poi.y(), poi.z(),
			up.x(), up.y(), up.z());
}

void GLWidget::paintGL() {
	glClearColor (0.3, 0.3, 0.3, 1.);

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();

	updateCamera();

	glLineWidth (1.);
	glBegin (GL_LINES);
	glColor3f (1., 0., 0.);
	glVertex3f (0., 0., 0.);
	glVertex3f (1., 0., 0.);
	glColor3f (0., 1., 0.);
	glVertex3f (0., 0., 0.);
	glVertex3f (0., 1., 0.);
	glColor3f (0., 0., 1.);
	glVertex3f (0., 0., 0.);
	glVertex3f (0., 0., 1.);
	glEnd();

	swapBuffers();
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

void GLWidget::mousePressEvent(QMouseEvent *event)
{
 	lastMousePos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastMousePos.x();
	int dy = event->y() - lastMousePos.y();

	phi += 0.01 * dx;
	theta -= 0.01 * dy;

	theta = std::max(theta, 0.01f);
	theta = std::min(theta, static_cast<float>(M_PI * 0.99));

	lastMousePos = event->pos();

	updateGL();
}

