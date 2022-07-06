#include <Arduino.h>

#include "Balancebot.hpp"
#include "stream_operators.hpp"

Balancebot::Balancebot(BalancebotConfig config)
    : odriveSerial(Serial1),
      odrive(odriveSerial),
      mpu(Wire),
      anglePid(PID(&anglePidInput, &anglePidOutput, &anglePidSetpoint, config.anglePidP, config.anglePidI, config.anglePidD, DIRECT)),
      positionPid(PID(&positionPidInput, &positionPidOutput, &positionPidSetpoint, config.positionPidP, config.positionPidI, config.positionPidD, DIRECT)),
      statusService("19B10000-E8F2-537E-4F6C-D104768A1214"),
      controlService("40760000-8912-48af-9146-b057c465d970"),
      angleCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify),
      targetSpeedCharacteristic("40760001-8912-48af-9146-b057c465d970", BLERead | BLENotify | BLEWriteWithoutResponse),
      config(config)
{
    // TODO: configure pid parameters and make configurable from outside
    anglePid.SetMode(AUTOMATIC);
    anglePid.SetOutputLimits(-20, 20);
    anglePid.SetSampleTime(10);

    positionPid.SetMode(AUTOMATIC);
    // positionPid.SetOutputLimits(-8, 8);
    positionPid.SetOutputLimits(-4, 4);
    positionPid.SetSampleTime(10);
}

Balancebot::~Balancebot()
{
}

void Balancebot::setup()
{
    // setup everything
    setupPins();
    setupMpu();
    setupOdrive();
    setupBluetooth();

    // go to balancing state
    setState(BalancebotState::Balancing);
}

void Balancebot::setupPins()
{
    pinMode(LED_BUILTIN, OUTPUT);
}

void Balancebot::setupMpu()
{
    // setup I2C bus
    Wire.begin();

    Serial << "Searching for MPU6050..";

    // attempt to open I2C connection to the IMU
    if (mpu.begin() != 0)
    {
        Serial << " failed!" << '\n';

        setError("Setting up MPU6050 IMU failed");

        return;
    }

    Serial << " done!" << '\n';

    Serial << "Calculating IMU offsets, keep still.. ";

    // calculate IMU offsets
    delay(1000);
    mpu.calcOffsets();

    Serial << " done!" << '\n';
}

void Balancebot::setupOdrive()
{
    Serial << "Setting up odrive..";

    // setup odrive serial
    odriveSerial.begin(115200);

    // TODO: configure axes

    Serial << " done!" << '\n';
}

void Balancebot::setupBluetooth()
{
    Serial << "Setting up bluetooth low energy..";

    // start bluetooth low energy stack
    if (!BLE.begin())
    {
        Serial << "starting bluetooth low energy module failed!" << '\n';

        return;
    }

    // set name
    BLE.setLocalName("Balancebot");

    // add advertised services
    BLE.setAdvertisedService(statusService);
    BLE.setAdvertisedService(controlService);

    // add characteristics
    statusService.addCharacteristic(angleCharacteristic);
    statusService.addCharacteristic(targetSpeedCharacteristic);

    // add the services
    BLE.addService(statusService);
    BLE.addService(controlService);

    // write initial characteristic values
    angleCharacteristic.writeValue(0.0f);
    targetSpeedCharacteristic.writeValue(0);

    // start advertising
    BLE.advertise();

    Serial << " done!" << '\n';
}

void Balancebot::loop(unsigned long dt, unsigned long currentTime)
{
    loopBluetooth(dt, currentTime, loopIndex);
    loopRobot(dt, currentTime, loopIndex);

    loopIndex++;
}

void Balancebot::loopBluetooth(unsigned long dt, unsigned long currentTime, int loopIndex)
{
    bool isBluetoothConnected = BLE.connected();

    // check for change of bluetooth connection state
    if (isBluetoothConnected != wasBluetoothConnected)
    {
        digitalWrite(LED_BUILTIN, isBluetoothConnected);

        wasBluetoothConnected = isBluetoothConnected;

        Serial << "Bluetooth " << (isBluetoothConnected ? "connected" : "disconnected") << '\n';
    }

    // nothing to do if not connected
    if (!isBluetoothConnected)
    {
        return;
    }

    // poll bluetooth for updates
    // BLE.poll();

    // attempt to get central device
    BLEDevice central = BLE.central();

    if (central)
    {
        // Serial << "Connected to central: " << central.address() << '\n';

        // handle target speed updates
        if (targetSpeedCharacteristic.written())
        {
            targetSpeed = targetSpeedCharacteristic.value();

            targetSpeedCharacteristic.writeValue(targetSpeed);

            Serial << "Got new target speed of " << targetSpeed << '\n';
        }

        // calculate time since last reported angle via ble
        unsigned long timeSinceReportedAngle = currentTime - lastAngleReportedTime;

        // report robot angle at certain interval
        if (timeSinceReportedAngle > REPORT_ANGLE_INTERVAL_MS)
        {
            angleCharacteristic.writeValue(angle);

            // Serial << "Reported angle of " << angle << " degrees\n";

            lastAngleReportedTime = currentTime;
        }
    }
}

void Balancebot::loopRobot(unsigned long dt, unsigned long currentTime, int loopIndex)
{
    // handle states
    switch (state)
    {
    case BalancebotState::Balancing:
        loopRobotBalacing(dt, currentTime, loopIndex);
        break;

    default:
        // ignore other states
        break;
    }
}

void Balancebot::loopRobotBalacing(unsigned long dt, unsigned long currentTime, int loopIndex)
{
    // TODO: get target distance from remote controller
    double targetDistance = 0.0;

    // get robot position and averageDistance travelled from encoders
    BalancebotOdometry odometry = getOdometry();

    // calculate position holding PID controller
    positionPidInput = -odometry.averageDistance;
    positionPidSetpoint = targetDistance;
    positionPid.Compute();
    double positionPidTargetAngle = positionPidOutput;

    // don't use position hold at the beginning
    // would require storing target offset at first position hold
    // if (currentTime < 30) {
    //     positionPidTargetAngle = 0.0;
    // }

    // update the inertial measurement unit
    mpu.update();

    // get robot angle and calculate limited velocity
    angle = mpu.getAngleY();

    // TODO: get from configuration / potentiometer
    double idleTargetAngle = 0.0;
    double speedTargetAngle = (double)targetSpeed;

    // calculate angle holding PID controller
    anglePidInput = angle;
    // TODO: restore position pid
    // anglePidSetpoint = idleTargetAngle + speedTargetAngle + positionPidTargetAngle;
    anglePidSetpoint = idleTargetAngle + speedTargetAngle;
    anglePid.Compute();
    double targetVelocity = anglePidOutput;

    // stop motors if angle gets too big
    if (abs(angle) > 20.0f)
    {
        targetVelocity = 0.0f;
    }

    // TODO: implement steering

    // set motor velocities
    setMotorVelocities(targetVelocity, targetVelocity);

    // Serial << "Setpoint: " << anglePidSetpoint << ", real: " << angle << ", dt: " << dt << '\n';

    // log for serial plotter
    // Serial << (currentTime / 1000.0f) << '\t' << angle << '\t' << anglePidSetpoint << '\t' << targetVelocity << '\t' << odometry.leftDistance << '\t' << odometry.rightDistance << '\t' << odometry.averageDistance << '\t' << dt << '\n';
}

void Balancebot::setState(BalancebotState newState)
{
    if (newState == state)
    {
        Serial << "Requested to transition to state " << getStateName(newState) << " but the robot is already in given state" << '\n';

        return;
    }

    BalancebotState oldState = state;
    state = newState;

    // handle entering states
    switch (state)
    {
    case BalancebotState::Balancing:
        onEnterBalacingState();

    default:
        // ignore
        break;
    }

    Serial << "Transitioned from " << getStateName(oldState) << " to " << getStateName(newState) << "state\n";
}

void Balancebot::setError(String newError)
{
    error = newError;

    setState(BalancebotState::Error);
}

void Balancebot::onEnterBalacingState()
{
    // use velocity control with velocity ramp
    odriveSerial << "w axis0.controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';
    odriveSerial << "w axis0.controller.config.input_mode " << INPUT_MODE_VEL_RAMP << '\n';
}

void Balancebot::setMotorVelocities(float velocityLeft, float velocityRight)
{
    // M0 is right, M1 is left and inverted
    odrive.SetVelocity(0, velocityRight);
    odrive.SetVelocity(1, -velocityLeft);
}

String Balancebot::getStateName(BalancebotState state)
{
    switch (state)
    {
    case BalancebotState::Initializing:
        return "Initializing";

    case BalancebotState::Error:
        return "Error";

    case BalancebotState::Balancing:
        return "Balancing";

    default:
        return "Unknown";
    }
}

BalancebotOdometry Balancebot::getOdometry()
{
    // get motor position estimates
    odriveSerial << "r axis" << 0 << ".encoder.pos_estimate << \n";
    float rightDistance = odrive.readFloat();
    odriveSerial << "r axis" << 1 << ".encoder.pos_estimate << \n";
    float leftDistance = odrive.readFloat() * -1;

    // TODO: calculate real distances using wheel diameter
    return BalancebotOdometry(leftDistance, rightDistance);
}