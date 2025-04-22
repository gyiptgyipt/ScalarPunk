#pragma once

#include <QWidget>
#include <QTimer>
#include <QPointF>

class RobotEyes : public QWidget {
    Q_OBJECT

public:
    RobotEyes(QWidget *parent = nullptr);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QTimer blinkTimer;
    QTimer updateTimer;

    bool isBlinking = false;
    float blinkProgress = 0.0f;
    QPointF pupilOffset;

    void startBlink();
    void updateAnimation();
};
