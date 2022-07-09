#ifndef BALANCEBOT_HPP
#define BALANCEBOT_HPP

#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <MPU6050_light.h>
#include <PID_v1.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <sbus.h>

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
    BalancebotOdometry(float leftPosition, float rightPosition)
        : leftPosition(leftPosition), rightPosition(rightPosition), position((leftPosition + rightPosition) / 2.0f) {}

    void update(float newLeftPosition, float newRightPosition)
    {
        leftPosition = newLeftPosition;
        rightPosition = newRightPosition;
        position = (leftPosition + rightPosition) / 2.0f;
    }

    float leftPosition;
    float rightPosition;
    float position;
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
    void setupReceiver();

    void onEnterBalacingState();

    void loopBluetooth(unsigned long dt, unsigned long currentTimeUs, int loopIndex);
    void loopReceiver(unsigned long dt, unsigned long currentTimeUs, int loopIndex);
    void loopRobot(unsigned long dt, unsigned long currentTimeUs, int loopIndex);
    void loopRobotBalacing(unsigned long dt, unsigned long currentTimeUs, int loopIndex);

    String getStateName(BalancebotState state);
    BalancebotOdometry getOdometry();
    void setMotorVelocities(float velocityLeft, float velocityRight);
    void setState(BalancebotState newState);
    void setError(String error);
    void reportAllState();
    void updatePositionHoldStartPosition();
    void calibrateImu();

    // dependencies
    HardwareSerial &odriveSerial;
    ODriveArduino odrive;
    MPU6050 mpu;
    PID anglePid;
    PID positionPid;
    UART rxSerial;
    bfs::SbusRx rx;

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
    // BalancebotOdometry odometry;
    String error = "";
    bool wasBluetoothConnected = false;
    bool isReceiverConnected = false;
    bool wasRobotFallenOver = false;
    float angle = 0.0f;
    float positionHoldStartPosition = 0.0f;
    float lastPosition = 0.0f;
    float targetPosition = 0.0f;
    // TODO: make configurable via characteristic?
    float idleTargetAngle = 0.0;
    float motorVelocityLeft = 0.0f;
    float motorVelocityRight = 0.0f;
    std::array<int16_t, bfs::SbusRx::NUM_CH()> rxData;

    // remote control state
    int targetSpeed = 0;
    int targetRotation = 0;
    bool usePositionHold = false;
    bool isEnabled = true;
    bool useDebug = false;

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
    unsigned long lastImuCalibrationTimeUs = 0;

    // constants
    static const int PIN_RX_SERIAL_TX = 4;
    static const int PIN_RX_SERIAL_RX = 3;
    static const unsigned long REPORT_ANGLE_INTERVAL_US = 100 * 1000;
    static const unsigned long IMU_CALIBRATION_THROTTLE_US = 3000 * 1000;
    static const unsigned long ROBOT_FALLEN_OVER_RECENTLY_TIME_US = 1000 * 1000;
    static const unsigned long US_IN_SECOND = 1000 * 1000;
    static constexpr float SPEED_DISTANCE_MULTIPLIER = 2.0f;
    static constexpr float ROTATION_VELOCITY_MULTIPLIER = 0.2f;
};

#endif // BALANCEBOT_HPP