#include <QApplication>
#include "EyesWidget.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    RobotEyes w;
    w.resize(400, 200);
    w.show();
    return app.exec();
}
