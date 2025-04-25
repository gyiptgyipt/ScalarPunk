// #pragma once

// #include <QWidget>
// #include <QTimer>
// #include <QPointF>

// class RobotEyes : public QWidget {
//     Q_OBJECT

// public:
//     RobotEyes(QWidget *parent = nullptr);

// protected:
//     void paintEvent(QPaintEvent *event) override;

// private:
//     QTimer blinkTimer;
//     QTimer updateTimer;

//     bool isBlinking = false;
//     float blinkProgress = 0.0f;
//     QPointF pupilOffset;

//     void startBlink();
//     void updateAnimation();
// };


#ifndef EYESWIDGET_H
#define EYESWIDGET_H

#include <QWidget>
#include <QTimer>
#include <QPointF>

class RobotEyes : public QWidget {
    Q_OBJECT

public:
    RobotEyes(QWidget *parent = nullptr);

protected:
    void paintEvent(QPaintEvent *event) override;

private slots:
    void updateAnimation();
    void startBlink();
    void runHappyEyes();
    void lookLeft();
    void lookRight();
    void wakeUp();
    void goToSleep();

private:
    QTimer updateTimer;
    QTimer blinkTimer;
    QTimer actionTimer;

    float blinkProgress = 0.0f;
    bool isBlinking = false;
    bool isSleeping = false;
    bool smileMode = false;

    float eyeOffset = 0.0f;
    float eyeSquash = 0.0f;  // negative for left shrink, positive for right shrink

    float zzzY = 1.0f; //sleeping parameter text height(class ထဲ ၀င်ပြင်ရန်)
    float zzzOpacity = 1.0f;




    // QPointF pupilOffset;
    // float pupilWiggleRadius = 3.0f;

    int animationIndex = 0;
    QList<std::function<void()>> animations;

    void nextAnimation();
};

#endif // EYESWIDGET_H
