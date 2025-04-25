
#include "EyesWidget.h"
#include <QPainter>
#include <QRandomGenerator>
#include <QtMath>

RobotEyes::RobotEyes(QWidget *parent) : QWidget(parent) {
    setFixedSize(400, 200);

    connect(&updateTimer, &QTimer::timeout, this, &RobotEyes::updateAnimation);
    updateTimer.start(16);  // ~60 FPS

    connect(&blinkTimer, &QTimer::timeout, this, &RobotEyes::startBlink);
    blinkTimer.start(3000 + QRandomGenerator::global()->bounded(2000));

    connect(&actionTimer, &QTimer::timeout, this, &RobotEyes::nextAnimation);
    actionTimer.start(3000); // Switch animation every 3s

    animations = {
        [&]() { isSleeping = false; blinkProgress = 0; },        // Wakeup
        [&]() { startBlink(); },                                 // Blink
        [&]() { runSaccade(); },                                 // Saccade
        [&]() { runHappyEyes(); },                               // Happy eyes
        [&]() { isSleeping = true; }                             // Sleep
    };
}

void RobotEyes::startBlink() {
    isBlinking = true;
    blinkProgress = 0.0f;
}

void RobotEyes::runHappyEyes() {
    happyMode = true;
    QTimer::singleShot(1000, [this]() {
        happyMode = false;
    });
}

void RobotEyes::runSaccade() {
    float angle = QRandomGenerator::global()->bounded(360.0f);
    float radius = 10.0f;
    pupilOffset = QPointF(qCos(qDegreesToRadians(angle)) * radius,
                          qSin(qDegreesToRadians(angle)) * radius);

    QTimer::singleShot(200, [this]() {
        pupilOffset = QPointF(0, 0);
    });
}

void RobotEyes::updateAnimation() {
    if (isBlinking) {
        blinkProgress += 0.05f;
        if (blinkProgress >= 1.0f) {
            isBlinking = false;
            blinkTimer.start(3000 + QRandomGenerator::global()->bounded(2000));
        }
    }

    if (!isSleeping && !isBlinking) {
        // Subtle movement (pupil wiggle)
        float angle = QRandomGenerator::global()->bounded(360.0f);
        float radius = QRandomGenerator::global()->bounded(pupilWiggleRadius);
        pupilOffset = QPointF(qCos(qDegreesToRadians(angle)) * radius,
                              qSin(qDegreesToRadians(angle)) * radius);
    }

    update();
}

void RobotEyes::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Set background to black
    p.fillRect(rect(), Qt::black);

    QRectF leftEye(50, 50, 100, 100);
    QRectF rightEye(250, 50, 100, 100);

    float blinkAmount = isSleeping ? 1.0f : (isBlinking ? qMin(1.0f, blinkProgress * 2.0f) : 0.0f);

    auto drawEye = [&](const QRectF &rect) {
        p.setBrush(Qt::white);
        p.setPen(Qt::black);
        p.drawRect(rect);

        QPointF center = rect.center();
        QPointF pupilCenter = center + pupilOffset;

        // Draw gold rectangular pupil
        QSizeF pupilSize(20, 20);  // you can tweak the size here
        QRectF pupilRect(pupilCenter - QPointF(pupilSize.width() / 2, pupilSize.height() / 2), pupilSize);
        p.setBrush(QColor(255, 215, 0));  // gold color
        p.setPen(Qt::NoPen);
        p.drawRect(pupilRect);

        // Blink effect
        if (blinkAmount > 0.0f) {
            QRectF lid(rect.left(), rect.top(), rect.width(), rect.height() * blinkAmount);
            p.setBrush(QColor(30, 30, 30));
            p.setPen(Qt::NoPen);
            p.drawRect(lid);
        }

        // Happy overlay
        if (happyMode) {
            QPointF left = rect.bottomLeft();
            QPointF right = rect.bottomRight();
            QPointF tip = QPointF(rect.center().x(), rect.bottom() + 20);
            p.setBrush(Qt::black);
            QPolygonF poly;
            poly << left << right << tip;
            p.drawPolygon(poly);
        }
    };

    drawEye(leftEye);
    drawEye(rightEye);
}


void RobotEyes::nextAnimation() {
    animations[animationIndex % animations.size()]();
    animationIndex++;
}
