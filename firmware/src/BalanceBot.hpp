#ifndef BALANCEBOT_HPP
#define BALANCEBOT_HPP

#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <MPU6050_light.h>
#include <PID_v1.h>
#include <Wire.h>
#include <ArduinoBLE.h>

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
    void setupPins();
    void setupMpu();
    void setupOdrive();
    void setupBluetooth();

    void onEnterBalacingState();

    void loopBluetooth(unsigned long dt, unsigned long currentTime, int loopIndex);
    void loopRobot(unsigned long dt, unsigned long currentTime, int loopIndex);
    void loopRobotBalacing(unsigned long dt, unsigned long currentTime, int loopIndex);

    String getStateName(BalancebotState state);
    BalancebotOdometry getOdometry();

    void setMotorVelocities(float velocityLeft, float velocityRight);

    void setState(BalancebotState newState);
    void setError(String error);

    // dependencies
    HardwareSerial &odriveSerial;
    ODriveArduino odrive;
    MPU6050 mpu;
    PID anglePid;
    PID positionPid;

    // bluetooth low energy controls
    BLEService statusService;
    BLEFloatCharacteristic angleCharacteristic;

    // configuration
    BalancebotConfig config;

    // runtime state
    BalancebotState state = BalancebotState::Initializing;
    String error = "";
    float angle = 0.0f;
    bool wasBluetoothConnected = false;

    // angle pid controller
    double anglePidSetpoint = 0.0;
    double anglePidInput = 0.0;
    double anglePidOutput = 0.0;

    // position pid controller
    double positionPidSetpoint = 0.0;
    double positionPidInput = 0.0;
    double positionPidOutput = 0.0;

    // timing
    int loopIndex = 0;
    unsigned long lastAngleReportedTime = 0;

    // constants
    unsigned long REPORT_ANGLE_INTERVAL_MS = 100;
};

#endif // BALANCEBOT_HPP