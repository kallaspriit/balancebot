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
    BalancebotConfig(double pidP, double pidI, double pidD)
        : pidP(pidP), pidI(pidI), pidD(pidD) {}

    double pidP;
    double pidI;
    double pidD;
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

    // configuration
    BalancebotConfig config;

    // runtime state
    BalancebotState state = BalancebotState::Initializing;
    String error = "";

    // pid controller
    double anglePidSetpoint = 0.0;
    double anglePidInput = 0.0;
    double anglePidOutput = 0.0;
};

#endif // BALANCEBOT_HPP