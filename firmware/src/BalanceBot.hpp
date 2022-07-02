#ifndef BALANCEBOT_HPP
#define BALANCEBOT_HPP

#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <MPU6050_light.h>
#include <PID_v1.h>
#include <Wire.h>

enum BalancebotState
{
    Initializing = 0,
    Error = 1,
    Balancing = 2,
};

class BalancebotConfig
{
public:
    BalancebotConfig(double anglePidP, double anglePidI, double anglePidD, double positionPidP, double positionPidI, double positionPidD)
        : anglePidP(anglePidP), anglePidI(anglePidI), anglePidD(anglePidD), positionPidP(positionPidP), positionPidI(positionPidI), positionPidD(positionPidD) {}

    double anglePidP;
    double anglePidI;
    double anglePidD;

    double positionPidP;
    double positionPidI;
    double positionPidD;
};

class BalancebotOdometry
{
public:
    BalancebotOdometry(float leftDistance, float rightDistance)
        : leftDistance(leftDistance), rightDistance(rightDistance), averageDistance((leftDistance + rightDistance) / 2.0f) {}

    float leftDistance;
    float rightDistance;
    float averageDistance;
};

class Balancebot
{
public:
    Balancebot(BalancebotConfig config);
    ~Balancebot();

    void setup();
    void loop(unsigned long dt, unsigned long currentTime);

private:
    String getStateName(BalancebotState state);
    BalancebotOdometry getOdometry();

    void setMotorVelocities(float velocityLeft, float velocityRight);

    void setState(BalancebotState newState);
    void setError(String error);

    void onEnterBalacingState();

    void loopBalacingState(unsigned long dt, unsigned long currentTime);

    // dependencies
    HardwareSerial &odriveSerial;
    ODriveArduino odrive;
    MPU6050 mpu;
    PID anglePid;
    PID positionPid;

    // configuration
    BalancebotConfig config;

    // runtime state
    BalancebotState state = BalancebotState::Initializing;
    String error = "";

    // angle pid controller
    double anglePidSetpoint = 0.0;
    double anglePidInput = 0.0;
    double anglePidOutput = 0.0;

    // position pid controller
    double positionPidSetpoint = 0.0;
    double positionPidInput = 0.0;
    double positionPidOutput = 0.0;
};

#endif // BALANCEBOT_HPP