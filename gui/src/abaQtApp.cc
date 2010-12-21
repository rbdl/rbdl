#include <QtGui> 

#include "glwidget.h" 
#include "abaQtApp.h"

#include <assert.h>
#include <iostream>

using namespace std;

abaQtApp::abaQtApp(QWidget *parent)
{
	timer = new QTimer (this);

	setupUi(this); // this sets up GUI

	timer->setSingleShot(false);
	timer->start(20);

	// the timer is used to continously redraw the OpenGL widget
	connect (timer, SIGNAL(timeout()), glWidget, SLOT(updateGL()));
	
	connect (actionQuit, SIGNAL( triggered() ), qApp, SLOT( quit() ));
}
