#ifndef RBDLMAINWINDOW_H
#define RBDLMAINWINDOW_H

#include <QTimer>
#include "ui_rbdlMainWindow.h"

class rbdlQtApp : public QMainWindow, public Ui::rbdlMainWindow
{
    Q_OBJECT
 
public:
    rbdlQtApp(QWidget *parent = 0);

protected:
		QTimer *timer;
};
 
#endif
