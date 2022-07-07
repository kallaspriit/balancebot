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
    Initializing,
    Error,
    Balancing
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
    void loop(unsigned long dt, unsigned long currentTimeUs);

private:
    void setupPins();
    void setupMpu();
    void setupBluetooth();
    void setupOdrive();

    void onEnterBalacingState();
    //
    void loopBluetooth(unsigned long dt, unsigned long currentTimeUs, int loopIndex);
    void loopRobot(unsigned long dt, unsigned long currentTimeUs, int loopIndex);
    void loopRobotBalacing(unsigned long dt, unsigned long currentTimeUs, int loopIndex);

    String getStateName(BalancebotState state);
    BalancebotOdometry getOdometry();
    void setMotorVelocities(float velocityLeft, float velocityRight);
    void setState(BalancebotState newState);
    void setError(String error);
    void reportAllState();
    void updatePositionHoldStartPosition();

    // dependencies
    HardwareSerial &odriveSerial;
    ODriveArduino odrive;
    MPU6050 mpu;
    PID anglePid;
    PID positionPid;

    // bluetooth low energy controls
    BLEService statusService;
    BLEService controlService;
    BLEFloatCharacteristic angleCharacteristic;
    BLEIntCharacteristic targetSpeedCharacteristic;
    BLEIntCharacteristic targetRotationCharacteristic;
    BLEBoolCharacteristic usePositionHoldCharacteristic;
    BLEBoolCharacteristic isEnabledCharacteristic;

    // configuration
    BalancebotConfig config;

    // runtime state
    BalancebotState state = BalancebotState::Initializing;
    String error = "";
    bool wasBluetoothConnected = false;
    float angle = 0.0f;
    float positionHoldStartPosition = 0.0f;
    float targetDistance = 0.0f;
    // TODO: make configurable via characteristic?
    float idleTargetAngle = 0.0;

    // remote control state
    int targetSpeed = 0;
    int targetRotation = 0;
    bool usePositionHold = false;
    bool isEnabled = true;

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
    unsigned long lastAngleReportedTimeUs = 0;
    unsigned long lastRobotFallenOverTimeUs = 0;

    // constants
    static const unsigned long REPORT_ANGLE_INTERVAL_US = 100 * 1000;
    static const unsigned long ROBOT_FALLEN_OVER_RECENTLY_TIME_US = 1000 * 1000;
    static constexpr float SPEED_DISTANCE_MULTIPLIER = 2.0f;
};

#endif // BALANCEBOT_HPP