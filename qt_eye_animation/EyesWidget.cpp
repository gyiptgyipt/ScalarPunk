#include "EyesWidget.h"
#include <QPainter>
#include <QRandomGenerator>
#include <QtMath>

RobotEyes::RobotEyes(QWidget *parent) : QWidget(parent) {
    connect(&blinkTimer, &QTimer::timeout, this, &RobotEyes::startBlink);
    blinkTimer.start(3000 + QRandomGenerator::global()->bounded(2000));

    connect(&updateTimer, &QTimer::timeout, this, &RobotEyes::updateAnimation);
    updateTimer.start(16); // ~60 FPS
}

void RobotEyes::startBlink() {
    isBlinking = true;
    blinkProgress = 0.0f;
}

void RobotEyes::updateAnimation() {
    if (isBlinking) {
        blinkProgress += 0.05f;
        if (blinkProgress >= 1.0f) {
            isBlinking = false;
            blinkTimer.start(3000 + QRandomGenerator::global()->bounded(2000));
        }
    }

    // Randomly move pupil around slightly
    float angle = QRandomGenerator::global()->bounded(360.0f);
    float radius = QRandomGenerator::global()->bounded(3.0f);
    pupilOffset = QPointF(qCos(qDegreesToRadians(angle)) * radius,
                          qSin(qDegreesToRadians(angle)) * radius);

    update();
}

void RobotEyes::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    QRectF leftEye(50, 50, 100, 100);
    QRectF rightEye(250, 50, 100, 100);

    // Eyelid (blink)
    float blink = isBlinking ? qMin(1.0f, blinkProgress * 2.0f) : 0.0f;

    auto drawEye = [&](const QRectF &rect) {
        p.setBrush(Qt::white);
        p.drawEllipse(rect);

        QPointF center = rect.center();
        QPointF pupilCenter = center + pupilOffset;

        // Pupil
        p.setBrush(Qt::black);
        p.drawEllipse(pupilCenter, 10, 10);

        // Blink effect
        if (blink > 0.0f) {
            p.setBrush(QColor(30, 30, 30));
            QRectF lid(rect.left(), rect.top(), rect.width(), rect.height() * blink);
            p.drawRect(lid);
        }
    };

    drawEye(leftEye);
    drawEye(rightEye);
}
