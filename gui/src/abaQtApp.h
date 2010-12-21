#ifndef ABAMAINWINDOW_H
#define ABAMAINWINDOW_H

#include <QTimer>
#include "ui_abaMainWindow.h"

class abaQtApp : public QMainWindow, public Ui::abaMainWindow
{
    Q_OBJECT
 
public:
    abaQtApp(QWidget *parent = 0);

protected:
		QTimer *timer;
};
 
#endif
