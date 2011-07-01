#include <QApplication>

#include "rbdlQtApp.h"
#include "glwidget.h"

#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	rbdlQtApp *main_window = new rbdlQtApp;

	main_window->show();
	return app.exec();
}
