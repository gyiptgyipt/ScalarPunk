#ifndef Scalar_punk_widget_H
#define Scalar_punk_widget_H

#include <QWidget>
#include <QTimer>
#include <QPointF>


#include <vector>
#include <functional>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "scalarpunk_interfaces/srv/emotion_command.hpp"


class RobotEyes : public QWidget {
    Q_OBJECT

public:
    RobotEyes(QWidget *parent = nullptr);
    ~RobotEyes(); 


protected:
    void paintEvent(QPaintEvent *event) override;

private slots:
    void updateAnimation();
    void startBlink();
    void Smile();
    void lookLeft();
    void lookRight();
    void wakeUp();
    void goToSleep();
    void Angry();
    void Charging();
    void Happy();
    void Cry();
private:
    QTimer updateTimer;
    QTimer blinkTimer;
    QTimer actionTimer;

    
    bool isBlinking = true;
    bool allowBlinking = true;
    bool isSleeping = false;
    bool isSmiling = false;
    bool isAngry = false;
    bool isCharging = false;
    bool isHappy = false;
   

    float blinkProgress = 0.0f;
    
    float eyeOffset = 0.0f;
    float eyeSquash = 0.0f;  // negative for left shrink, positive for right shrink

    float zzzY = 1.0f; //sleeping parameter text height(class ထဲ ၀င်ပြင်ရန်)
    float zzzOpacity = 1.0f;

    float angryScale = 1.0f; //angry position ချိန်ရန် (၀င်ပြင်)
    bool angryGrowing = false;

    float chargingScale = 0.015f;
    bool chargingUp = false;

    float happyShakePhase = 0.0f;

    bool isCrying = false;
    float tearOffset = 0.0f;



    QTimer driftTimer;  // moment_eyes
    qreal eyeDriftX = 0;
    qreal eyeDriftY = 0;
    bool movement_eyes = false;





    // QPointF pupilOffset;
    // float pupilWiggleRadius = 3.0f;

    int animationIndex = 0;
    QList<std::function<void()>> animations;



    // ROS2
    rclcpp::Node::SharedPtr node;
    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emotion_subscriber;
    rclcpp::Service<scalarpunk_interfaces::srv::EmotionCommand>::SharedPtr emotion_service;
    std::thread ros_spin_thread;

    void nextAnimation();
};

#endif // Scalar_punk_widget_H
