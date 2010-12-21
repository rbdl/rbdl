#include <QApplication>

#include "abaQtApp.h"
#include "glwidget.h"

#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	abaQtApp *main_window = new abaQtApp;

	main_window->show();
	return app.exec();
}
