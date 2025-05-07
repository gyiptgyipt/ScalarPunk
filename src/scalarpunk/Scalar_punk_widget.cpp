#include "Scalar_punk_widget.h"

#include <QPainter>
#include <QRectF>
#include <QPen>
#include <QBrush>
#include <QPainterPath>



#include <QRandomGenerator>
#include <QtMath>



RobotEyes::RobotEyes(QWidget *parent) : QWidget(parent) {
    setFixedSize(800, 400);  // default 800 400

    setWindowFlags(Qt::FramelessWindowHint | Qt::Window);


    connect(&updateTimer, &QTimer::timeout, this, &RobotEyes::updateAnimation);
    updateTimer.start(16);  // ~60 FPS

    connect(&blinkTimer, &QTimer::timeout, this, &RobotEyes::startBlink);
    blinkTimer.start(3000 + QRandomGenerator::global()->bounded(2000));

    
    driftTimer.setInterval(2000);  // <- Adjust speed here: 100 = fast, 500 = slow
        connect(&driftTimer, &QTimer::timeout, this, [this]() {                  
            if (movement_eyes) {
                int randomX = QRandomGenerator::global()->bounded( width() * (-1)  , width());
                int randomY = QRandomGenerator::global()->bounded( height() * (-1) , height());
                eyeDriftX = randomX * 0.1;                 // widget dimension ရဲ့ 10 ပုံ တစ်ပုံ
                eyeDriftY = randomY * 0.1;
            } else {
                eyeDriftX = 0;
                eyeDriftY = 0;
            }
        });
        driftTimer.start();


    wakeUp();



    // connect(&actionTimer, &QTimer::timeout, this, &RobotEyes::nextAnimation);
    // actionTimer.start(3000); // Switch animation every 3s

    animations = {
        // [&]() { isSleeping = false; blinkProgress = 0; },  // Wakeup
        // [&]() { startBlink(); },                           // Blink
        // [&]() { Smile(); },                         // Happy eyes
        // [&]() { lookLeft(); },
        // [&]() { lookRight(); },
        // [&]() { isSleeping = true; }                       // Sleep

        [&]() { wakeUp(); },              // Wakeup
        [&]() { startBlink(); },          // Blink
        [&]() { Smile(); },        // smile eyes
        [&]() { lookLeft(); },
        [&]() { lookRight(); },
        [&]() { Angry(); } ,
        [&]() { Charging(); } ,
        [&]() { Happy(); } ,
        [&]() { Cry(); } ,
        [&]() { goToSleep(); }  
    };


   rclcpp::init(0, nullptr);  // Initialize ROS2

    // Ensure ROS 2 node initialization
    node = rclcpp::Node::make_shared("robot_eyes_node");
    
    //topic
    // emotion_subscriber = node->create_subscription<std_msgs::msg::String>(
    //     "/robot_emotion", 10,
    //     [this](const std_msgs::msg::String::SharedPtr msg) {
    //         QString emotion = QString::fromStdString(msg->data);
    //         if (emotion == "happy") {
    //             Smile();
    //         } else if (emotion == "angry") {
    //             Angry();
    //         } else if (emotion == "charging") {
    //             Charging();
    //         } else if (emotion == "sleep") {
    //             goToSleep();
    //         } else if (emotion == "wakeup") {
    //             wakeUp();
    //         } else if (emotion == "look_left") {
    //             lookLeft();
    //         } else if (emotion == "look_right") {
    //             lookRight();
    //         }
    //     }
    // );

    //srv
    emotion_service = node->create_service<scalarpunk_interfaces::srv::EmotionCommand>(
    "/robot_emotion_command",
    [this](const std::shared_ptr<scalarpunk_interfaces::srv::EmotionCommand::Request> request,
           std::shared_ptr<scalarpunk_interfaces::srv::EmotionCommand::Response> response) {
        QString emotion = QString::fromStdString(request->emotion);

        if (emotion == "smile") {
            Smile();
            response->status = "success";
        } else if (emotion == "angry") {
            Angry();
            response->status = "success";
        } else if (emotion == "charging") {
            Charging();
            response->status = "success";  // For example
        } else if (emotion == "sleep") {
            goToSleep();
            response->status = "success";
        } else if (emotion == "wakeup") {
            wakeUp();
            response->status = "success";
        } else if (emotion == "look_left") {
            lookLeft();
            response->status = "success";
        } else if (emotion == "look_right") {
            lookRight();
            response->status = "success";
        } else if (emotion == "happy") {
            Happy();
            response->status = "success";
        } else if (emotion == "cry") {
            Cry();
            response->status = "success";
        }    else {
            response->status = "fail";
        }
    }
);
    // Create a thread to spin ROS node in the background
    ros_spin_thread = std::thread([this]() {
        rclcpp::spin(node);
    });


}


RobotEyes::~RobotEyes() {
    rclcpp::shutdown();  // Properly shut down ROS
    if (ros_spin_thread.joinable()) {
        ros_spin_thread.join();  // Wait for the thread to finish
    }
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
        zzzY -= 1.0f;             // Move upward
        zzzOpacity -= 0.005f;      // Fade out

        // Reset loop when it fades away
        if (zzzOpacity <= 0.0f) {
            zzzY = 0.0f;
            zzzOpacity = 1.0f;
        }
    }


    if (isHappy) {
    happyShakePhase += 0.5f; // Controls speed of shake
    if (happyShakePhase > 2 * M_PI) {
        happyShakePhase -= 2 * M_PI;
    }
}


    if (isCrying) {
    tearOffset += 0.5;
    if (tearOffset > 80) tearOffset = 0;
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
        font.setPointSize(leftEye.height() * 0.2);
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


    QRectF movedRect = rect.translated(eyeDriftX, eyeDriftY);

    qreal radiusX = rect.width() / 3.0;
    qreal radiusY = rect.height() / 3.0;

    p.drawRoundedRect(movedRect, radiusX, radiusY);

    // Blink effect
    if (blinkAmount > 0.0f) {
        QRectF lid(movedRect.left(), movedRect.top(), movedRect.width(), (movedRect.height() - 10) * blinkAmount);
        p.setBrush(Qt::black);
        p.setPen(Qt::NoPen);
        p.drawRoundedRect(lid, 30, 30);
    }

    if (isSmiling) {
        QRectF mouthRect(
            movedRect.left(),
            movedRect.bottom() - movedRect.height() * 0.4,
            movedRect.width(),
            movedRect.height() * 0.4
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

} else if (isHappy) {
    qreal shakeX = qSin(happyShakePhase) * 5.0f; // amplitude of shake      

    // Apply shake offset
    leftEye.translate(shakeX, 0);
    rightEye.translate(shakeX, 0);

    drawEye(leftEye, QColor(255, 215, 0));  // gold
    drawEye(rightEye, QColor(255, 215, 0)); // gold

    // Draw happy eye "smile" circles under each eye
    qreal circleRadius = leftEye.height() * 2.0;

    QRectF leftCircle(
        leftEye.center().x() - circleRadius,
        leftEye.bottom() - circleRadius * 0.3,
        circleRadius * 2,
        circleRadius * 2
    );

    QRectF rightCircle(
        rightEye.center().x() - circleRadius,
        rightEye.bottom() - circleRadius * 0.3,
        circleRadius * 2,
        circleRadius * 2
    );

    p.setBrush(Qt::black);
    p.setPen(Qt::NoPen);
    p.drawEllipse(leftCircle);
    p.drawEllipse(rightCircle);

    // // Blush under/next to each eye (round & subtle)  //ပါးနီ  //မှတ်ချက် ပျော်နေတဲ့ပုံ မပေါက်၊  ကြောက်နေတဲ့ပုံ ပေါက် xD
    // QColor blushColor(255, 105, 180, 70); // Barbie pink with softer transparency

    // p.setBrush(blushColor);
    // p.setPen(Qt::NoPen);

    // qreal blushRadius = leftEye.width() * 0.25;
    // qreal blushYOffset = leftEye.height() * 0.2;
    // qreal blushXOffset = leftEye.width() * 0.5;

    // // Left blush (circle)
    // QRectF leftBlush(
    //     leftEye.center().x() - blushXOffset - blushRadius,
    //     leftEye.bottom() - blushRadius - blushYOffset,
    //     blushRadius * 2,
    //     blushRadius * 2
    // );

    // // Right blush (circle)
    // QRectF rightBlush(
    //     rightEye.center().x() + blushXOffset - blushRadius,
    //     rightEye.bottom() - blushRadius - blushYOffset,
    //     blushRadius * 2,
    //     blushRadius * 2
    // );

    // p.drawEllipse(leftBlush);
    // p.drawEllipse(rightBlush);

        // Anime-style blush: three slanted lines (45 degrees) for each cheek
    QColor blushColor(255, 105, 180, 120); // Barbie pink with soft transparency
    QPen blushPen(blushColor, 4, Qt::SolidLine, Qt::RoundCap);  // <- thicker
    p.setPen(blushPen);
    
    int lineLength = leftEye.width() * 0.15;
    int lineSpacing = leftEye.width() * 0.07;
    
    QPointF leftBlushCenter(
        leftEye.left() - leftEye.width() * 0.2,
        leftEye.bottom() - leftEye.height() * 0.2
    );
    
    QPointF rightBlushCenter(
        rightEye.right() + rightEye.width() * 0.2,
        rightEye.bottom() - rightEye.height() * 0.2
    );
    
    // Left blush lines
    for (int i = 0; i <= 2; ++i) {
        QPointF start(leftBlushCenter.x() + i * lineSpacing, leftBlushCenter.y());
        QPointF end(start.x() + lineLength * 0.7, start.y() + lineLength * 0.7);
        p.drawLine(start, end);
    }
    
    // Right blush lines (mirrored)
    for (int i = 0; i <= 2; ++i) {
        QPointF start(rightBlushCenter.x() + i * lineSpacing, rightBlushCenter.y());
        QPointF end(start.x() - lineLength * 0.7, start.y() + lineLength * 0.7);
        p.drawLine(start, end);
    }
  
}   //end happy

else if (isCrying) {
   
   // Eye base (yellow rectangle)
    drawEye(leftEye, QColor(255, 215, 0));   // gold
    drawEye(rightEye, QColor(255, 215, 0));

    // Black "closed eyelid" triangle overlay
    p.setBrush(Qt::black);
    p.setPen(Qt::NoPen);

    // Left eye black triangle (top-left cover)
    QPolygonF leftEyelid;
    leftEyelid << QPointF(leftEye.left(), leftEye.top())
               << QPointF(leftEye.right(), leftEye.top())
               << QPointF(leftEye.left(), leftEye.bottom() - leftEye.height() * 0.6);
    p.drawPolygon(leftEyelid);

    // Right eye black triangle (top-right cover)
    QPolygonF rightEyelid;
    rightEyelid << QPointF(rightEye.right(), rightEye.top())
                << QPointF(rightEye.left(), rightEye.top())
                << QPointF(rightEye.right(), rightEye.bottom() - rightEye.height() * 0.6);
    p.drawPolygon(rightEyelid);
   
    QFont emojiFont("Segoe UI Emoji");
    emojiFont.setPointSizeF(leftEye.height() * 0.2);  // or use rect.height() * 0.08
    p.setFont(emojiFont);
    p.setPen(QColor(100, 160, 255));  // Light blue

    QString tearEmoji = "💧";

    QPointF leftTearStart(leftEye.center().x()  , leftEye.bottom() + 5 + tearOffset);         // pos ချိန်ရန်
    QPointF rightTearStart(rightEye.center().x() , rightEye.bottom() + 5 + tearOffset);

    p.drawText(leftTearStart, tearEmoji);
    p.drawText(rightTearStart, tearEmoji);

    // Frowning brow
    p.setPen(QPen(Qt::black, 3));
    QPointF frownLeft(leftEye.left(), leftEye.top() - 20);
    QPointF frownRight(rightEye.right(), rightEye.top() - 20);
    QPointF frownCenter(rect().center().x(), leftEye.top() + 5);
    QPolygonF frown;
    frown << frownLeft << frownCenter << frownRight;
    p.drawPolyline(frown);
}




    else {
    drawEye(leftEye, QColor(255, 215, 0));  // gold
    drawEye(rightEye, QColor(255, 215, 0));
}



}


// true , false parameter တွေပြန်စစ်ရန်

void RobotEyes::lookLeft() {

    isCharging = false;
    isSleeping = false;
    isAngry    = false;

    isCrying = false;
    isHappy = false;
    isSmiling = true;

    allowBlinking = true;

    eyeOffset = -100;
    eyeSquash = qBound(-1.0f, eyeOffset / 100.0f, 1.0f);

   
}

void RobotEyes::lookRight() {

    isCharging = false;
    isSleeping = false;
    isAngry    = false;

    isCrying = false;
    isHappy = false;
    isSmiling = true;

    allowBlinking = true;
    
    eyeOffset = 100;
    eyeSquash = qBound(-1.0f, eyeOffset / 100.0f, 1.0f);

    
}


void RobotEyes::goToSleep() {
    isSleeping = true;
    isAngry = false;
    isCharging = false;

    isSmiling = false;
    isHappy = false;
    isCrying = false;

    allowBlinking = false;

    movement_eyes = false;
    eyeOffset = 0;
    eyeSquash = 0;
    

}

void RobotEyes::wakeUp() {
    isSleeping = false;
    isAngry = false;
    isCharging = false;

    isSmiling = false;
    isCrying = false;
    isHappy = false;
   
    allowBlinking = true;

    movement_eyes = true;
    eyeOffset = 0;
    eyeSquash = 0;
    
}

void RobotEyes::Angry() {
    isAngry = true;
    isSleeping = false;
    isCharging = false;
   
    isSmiling = false;
    isCrying = false;
    isHappy = false;

    allowBlinking = true;

    movement_eyes = false;
    eyeOffset = 0;
    eyeSquash = 0;
}

void RobotEyes::Charging() {
    isCharging = true;
    isSleeping = false;
    isAngry    = false;
     
    isSmiling = false;
    isHappy = false;
    isCrying = false;

    allowBlinking = true;

    movement_eyes = true;
    eyeOffset = 0;
    eyeSquash = 0;
   
}

void RobotEyes::startBlink() {
    if (!isBlinking && allowBlinking) {
        isBlinking = true;
        blinkProgress = 0.0f;
    }
}

void RobotEyes::Smile() {
    isCharging = false;
    isSleeping = false;
    isAngry    = false;
    isCrying = false;
     
    isSmiling = true;
    isHappy = false;

    allowBlinking = true;

    movement_eyes = false;
    eyeOffset = 0;
    eyeSquash = 0;
    
}

void RobotEyes::Happy(){
    isCharging = false;
    isSleeping = false;
    isAngry   = false;

    isSmiling = false;
    isCrying = false;
    isHappy = true;

    allowBlinking = true;

    movement_eyes = false;
    eyeOffset = 0;
    eyeSquash = 0;
}

void RobotEyes::Cry(){

    isCharging = false;
    isSleeping = false;
    isAngry   = false;

    isSmiling = false;
    isHappy = false;
    isCrying = true;

    allowBlinking = true;

    movement_eyes = false;
    eyeOffset = 0;
    eyeSquash = 0;

}


void RobotEyes::mousePressEvent(QMouseEvent *event) {   //click to exit
    Q_UNUSED(event);
    close();  // or qApp->quit(); to exit the whole app
}



void RobotEyes::nextAnimation() {
    animations[animationIndex % animations.size()]();
    animationIndex++;
    
}
