#include "EyesWidget.h"
#include <QPainter>
#include <QRandomGenerator>
#include <QtMath>

RobotEyes::RobotEyes(QWidget *parent) : QWidget(parent) {
    setFixedSize(800, 400);

    connect(&updateTimer, &QTimer::timeout, this, &RobotEyes::updateAnimation);
    updateTimer.start(16);  // ~60 FPS

    connect(&blinkTimer, &QTimer::timeout, this, &RobotEyes::startBlink);
    blinkTimer.start(3000 + QRandomGenerator::global()->bounded(2000));

    connect(&actionTimer, &QTimer::timeout, this, &RobotEyes::nextAnimation);
    actionTimer.start(3000); // Switch animation every 3s

    animations = {
        // [&]() { isSleeping = false; blinkProgress = 0; },  // Wakeup
        // [&]() { startBlink(); },                           // Blink
        // [&]() { runHappyEyes(); },                         // Happy eyes
        // [&]() { lookLeft(); },
        // [&]() { lookRight(); },
        // [&]() { isSleeping = true; }                       // Sleep

        [&]() { wakeUp(); },              // Wakeup
        [&]() { startBlink(); },          // Blink
        [&]() { runHappyEyes(); },        // Happy eyes
        [&]() { lookLeft(); },
        [&]() { lookRight(); },
        [&]() { Angry(); } ,
        [&]() { Charging(); } ,
        [&]() { goToSleep(); }  
    };
}


void RobotEyes::updateAnimation() {
    if (isBlinking) {
    blinkProgress += 0.05f;
    if (blinkProgress >= 1.0f) {
        isBlinking = false;

        if (allowBlinking) {
            int delay = 3000 + QRandomGenerator::global()->bounded(3000);  // 3–6 sec
            blinkTimer.start(delay);
        }
    }
}



    // Animate "Zzz" rising effect
    if (isSleeping) {
        zzzY -= 0.5f;             // Move upward
        zzzOpacity -= 0.005f;      // Fade out

        // Reset loop when it fades away
        if (zzzOpacity <= 0.0f) {
            zzzY = 0.0f;
            zzzOpacity = 1.0f;
        }
    }

    update();
}

void RobotEyes::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // Set background to black
    p.fillRect(rect(), Qt::black);

    // Dynamic sizing based on widget size
    int w = width();
    int h = height();

float eyeWidth = w * 0.25f;
float eyeHeight = h * 0.5f;
float spacing = w * 0.05f;
float squashFactor = 0.15f; // adjust for how much to stretch/shrink

float leftScale, rightScale;

    if (eyeSquash == -1) {
        leftScale = 1.0f - squashFactor;
    } else if (eyeSquash == 1) {
        leftScale = 1.0f + squashFactor;
    } else {
        leftScale = 1.0f;
    }

    if (eyeSquash == 1) {
        rightScale = 1.0f - squashFactor;
    } else if (eyeSquash == -1) {
        rightScale = 1.0f + squashFactor;
    } else {
        rightScale = 1.0f;
    }


float leftEyeWidth = eyeWidth * leftScale;
float rightEyeWidth = eyeWidth * rightScale;

// Recompute spacing to center everything nicely
float totalWidth = leftEyeWidth + rightEyeWidth + spacing;
float leftX = (w - totalWidth) / 2.0f + eyeOffset;
float topY = (h - eyeHeight) / 2.0f;

QRectF leftEye(leftX, topY, leftEyeWidth, eyeHeight);
QRectF rightEye(leftX + leftEyeWidth + spacing, topY, rightEyeWidth, eyeHeight);


    // float blinkAmount = isSleeping ? 1.0f : (isBlinking ? qMin(1.0f, blinkProgress * 2.0f) : 0.0f);
    float blinkAmount = 0.0f;

    if (isSleeping) {
        blinkAmount = 1.0f;
        QFont font = p.font();
        font.setPointSize(32);
        font.setBold(true);
        p.setFont(font);

        p.setPen(QColor(255, 255, 255, static_cast<int>(zzzOpacity * 255)));
        QString zzzText = "zzz";
        float textWidth = p.fontMetrics().horizontalAdvance(zzzText);

        QPointF zzzPos(width() / 1.5f - textWidth / 2.0f, topY + 30 + zzzY);
        p.drawText(zzzPos, zzzText);
    } else if (isBlinking) {
        float t = blinkProgress;
        blinkAmount = (t < 0.5f) ? (t * 2.0f) : ((1.0f - t) * 2.0f);
    }

    

auto drawEye = [&](const QRectF &rect, const QColor &eyeColor) {
    p.setBrush(eyeColor);
    p.setPen(Qt::black);
    p.drawRoundedRect(rect, 60, 60);
    
    // Blink effect
    if (blinkAmount > 0.0f) {
        QRectF lid(rect.left(), rect.top(), rect.width(), (rect.height() - 10) * blinkAmount);
        p.setBrush(Qt::black);
        p.setPen(Qt::NoPen);
        p.drawRoundedRect(lid, 30, 30);
    }

    if (isSmiling) {
        QRectF mouthRect(
            rect.left(),
            rect.bottom() - rect.height() * 0.4,
            rect.width(),
            rect.height() * 0.4
        );
        p.setBrush(Qt::black);
        p.drawRect(mouthRect);
    }
};

// Now you can safely call drawEye anywhere below:
if (isAngry) {
    drawEye(leftEye, QColor(200, 0, 0));  // red
    drawEye(rightEye, QColor(200, 0, 0));

    // angry brow
    p.setBrush(Qt::black);
    QPointF topLeft(rect().left() + rect().width(), rect().top() + rect().height() * 0.2);
    QPointF topRight(rect().right() - rect().width(), rect().top() + rect().height() * 0.2);
    QPointF bottomMid(rect().center().x(), rect().top() + rect().height() * 0.5);
    QPolygonF angryBrow;
    angryBrow << topLeft << topRight << bottomMid;
    p.drawPolygon(angryBrow);
    

    // emoji scaling
    angryScale += (angryGrowing ? 0.02f : -0.02f);
    angryGrowing = (angryScale >= 1.4f) ? false : (angryScale <= 1.0f) ? true : angryGrowing;

    QFont font("Segoe UI Emoji");
    font.setPointSizeF(24 * angryScale);
    p.setFont(font);
    p.setPen(Qt::red);
    QString emoji = "💢";
    QPointF emojiPos(
        leftEye.left() - p.fontMetrics().horizontalAdvance(emoji),
        leftEye.top()
    );
    p.drawText(emojiPos, emoji);

} else if (isCharging) {
    drawEye(leftEye, QColor(0, 255, 0));  // green
    drawEye(rightEye, QColor(0, 255, 0));

    // emoji scaling
    chargingScale += (chargingUp ? 0.015f : -0.015f);
    chargingScale = qBound(1.0f, chargingScale, 1.3f);
    chargingUp = (chargingScale >= 1.3f) ? false : (chargingScale <= 1.0f) ? true : chargingUp;

    QFont font("Segoe UI Emoji");
    font.setPointSizeF(24 * chargingScale);
    p.setFont(font);
    p.setPen(QColor(255, 255, 0, 230));
    QString emoji = "⚡️";
    QPointF emojiPos(
        rect().center().x() - p.fontMetrics().horizontalAdvance(emoji) / 2.0,
        rect().top() + rect().height() * 0.1
    );
    p.drawText(emojiPos, emoji);

} else {
    drawEye(leftEye, QColor(255, 215, 0));  // gold
    drawEye(rightEye, QColor(255, 215, 0));
}



}


void RobotEyes::lookLeft() {

    isSmiling = false;

    eyeOffset = -100;
    eyeSquash = qBound(-1.0f, eyeOffset / 100.0f, 1.0f);

    QTimer::singleShot(2000, [this]() {
        eyeOffset = 0;
        eyeSquash = 0;
    });
}

void RobotEyes::lookRight() {

    isSmiling = false;
    
    eyeOffset = 100;
    eyeSquash = qBound(-1.0f, eyeOffset / 100.0f, 1.0f);

    QTimer::singleShot(2000, [this]() {
        eyeOffset = 0;
        eyeSquash = 0;
    });
}


void RobotEyes::goToSleep() {
    isSleeping = true;
    isAngry = false;
    isCharging = false;
    isSmiling = false;

    allowBlinking = false;

}

void RobotEyes::wakeUp() {
    isSleeping = false;
    isAngry = false;
    isCharging = false;
    isSmiling = false;
   
    allowBlinking = true;
    
}

void RobotEyes::Angry() {
    isAngry = true;
    isSleeping = false;
    isCharging = false;
   
    isSmiling = false;
    allowBlinking = true;
}

void RobotEyes::Charging() {
    isCharging = true;
    isSleeping = false;
    isAngry    = false;
     
    isSmiling = true;
    allowBlinking = true;
   
}

void RobotEyes::startBlink() {
    if (!isBlinking && allowBlinking) {
        isBlinking = true;
        blinkProgress = 0.0f;
    }
}

void RobotEyes::runHappyEyes() {
    isSmiling = true;
    // allowBlinking = true;
    
}


void RobotEyes::nextAnimation() {
    animations[animationIndex % animations.size()]();
    animationIndex++;
    
}
